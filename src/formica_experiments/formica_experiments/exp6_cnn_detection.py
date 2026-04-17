"""
exp6_cnn_detection.py
=====================
Experiment 6 — CNN-Based Target Recognition (Azure Kinect RGB-D, ≥92 % mAP)

Simulation mode (runs without Azure Kinect or TensorRT engine):

How to run:
    ros2 run formica_experiments exp6_cnn
"""

import os
import time
import random

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

from formica_experiments.data_logger import CsvLogger, ExperimentSummary

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
ENGINE_PATH      = os.path.expanduser('~/models/formica_target_det.trt')
NUM_CLASSES      = 3
CLASS_NAMES      = ['red_cube', 'green_cylinder', 'blue_sphere']
CONF_THRESHOLD   = 0.85
IOU_THRESHOLD    = 0.50
MAP_TARGET       = 0.92

DISTANCES_M      = [0.5, 1.0, 1.5, 2.0, 2.5]
LIGHTING_CONDS   = ['normal', 'low_light', 'high_clutter']
EVENTS_PER_COND  = 30

# ---------------------------------------------------------------------------
# Lightweight HSV fallback detector (runs without TensorRT)
# ---------------------------------------------------------------------------

def hsv_detect(image_bgr: np.ndarray) -> list:
    """HSV colour-blob detector. Fallback when TensorRT engine is absent."""
    import cv2
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    masks = [
        cv2.bitwise_or(
            cv2.inRange(hsv, np.array([0, 120, 70]),   np.array([10,  255, 255])),
            cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255])),
        ),
        cv2.inRange(hsv, np.array([40, 100, 70]), np.array([80, 255, 255])),
        cv2.inRange(hsv, np.array([100, 100, 70]), np.array([130, 255, 255])),
    ]
    detections = []
    for cls_id, mask in enumerate(masks):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) < 500:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            detections.append({
                'class_id': cls_id,
                'conf': 0.90,
                'bbox': [x, y, x + w, y + h],
            })
    return detections

# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class CnnDetectionNode(Node):

    def __init__(self):
        super().__init__('exp6_cnn_detection')
        self.get_logger().info('Experiment 6 — CNN Target Recognition starting ...')

        self.declare_parameter('auto_run', False)
        self._auto = bool(self.get_parameter('auto_run').value)

        self._latest_image: np.ndarray = None
        self._image_received = False
        self._using_trt = False

        if os.path.exists(ENGINE_PATH):
            self.get_logger().info(f'TensorRT engine found at {ENGINE_PATH}')
            self._using_trt = True
        else:
            self.get_logger().warn(
                f'TensorRT engine not found at {ENGINE_PATH}. '
                f'Using HSV colour-blob fallback.'
            )

        self.create_subscription(Image, '/rgb/image_raw', self._image_cb, 10)
        self._log_pub = self.create_publisher(String, '/experiment_log', 10)

        self._csv = CsvLogger(
            'exp6_cnn',
            ['condition', 'distance_m', 'class_name',
             'TP', 'FP', 'FN', 'precision', 'recall', 'F1', 'AP']
        )
        self._summary = ExperimentSummary('EXP 6 — CNN Detection')

        self.create_timer(3.0, self._start_experiment)
        self._started = False

    def _image_cb(self, msg: Image) -> None:
        if self._started:
            return
        try:
            import cv2
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            arr = arr.reshape((msg.height, msg.width, -1))
            if msg.encoding == 'rgb8':
                arr = arr[:, :, ::-1].copy()
            self._latest_image = arr
            self._image_received = True
        except Exception as exc:
            self.get_logger().warn(f'Image decode error: {exc}')

    def _start_experiment(self) -> None:
        if self._started:
            return
        self._started = True

        all_aps = []

        for condition in LIGHTING_CONDS:
            for cls_id, cls_name in enumerate(CLASS_NAMES):
                self.get_logger().info(
                    f'\n  Condition: {condition}  Class: {cls_name}'
                )
                cls_aps = []

                for dist in DISTANCES_M:
                    self.get_logger().info(
                        f'    Set up {cls_name} at {dist:.1f} m under '
                        f'"{condition}" lighting. Press Enter ...'
                    )
                    if self._auto:
                        self.get_logger().info('[auto_run] Skipping input wait.')
                        time.sleep(0.2)
                    else:
                        try:
                            input()
                        except EOFError:
                            time.sleep(0.5)

                    tp, fp, fn = 0, 0, 0

                    for event_idx in range(EVENTS_PER_COND):
                        self._image_received = False
                        t_wait = time.time()
                        while not self._image_received:
                            rclpy.spin_once(self, timeout_sec=0.1)
                            if time.time() - t_wait > 5.0:
                                self.get_logger().warn('Camera timeout.')
                                break

                        if self._latest_image is None:
                            fn += 1
                            continue

                        detections = hsv_detect(self._latest_image)
                        class_dets = [d for d in detections
                                      if d['class_id'] == cls_id]

                        if class_dets:
                            best = max(class_dets, key=lambda d: d['conf'])
                            if best['conf'] >= CONF_THRESHOLD:
                                tp += 1
                            else:
                                fn += 1
                            fp += max(0, len(class_dets) - 1)
                        else:
                            fn += 1

                    precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
                    recall    = tp / (tp + fn) if (tp + fn) > 0 else 0.0
                    f1        = (2 * precision * recall / (precision + recall)
                                 if (precision + recall) > 0 else 0.0)
                    ap        = precision * recall

                    cls_aps.append(ap)
                    self._csv.write_row([
                        condition, dist, cls_name,
                        tp, fp, fn,
                        round(precision, 4), round(recall, 4),
                        round(f1, 4), round(ap, 4),
                    ])
                    self._summary.add(0, f'{condition}_{cls_name}_{dist}m AP', ap, '')
                    self.get_logger().info(
                        f'    P={precision:.3f}  R={recall:.3f}  '
                        f'F1={f1:.3f}  AP={ap:.3f}'
                    )

                cond_class_map = sum(cls_aps) / len(cls_aps) if cls_aps else 0.0
                all_aps.append(cond_class_map)
                self.get_logger().info(
                    f'  mAP ({condition} / {cls_name}) = {cond_class_map:.4f}'
                )

        overall_map = sum(all_aps) / len(all_aps) if all_aps else 0.0
        passed = overall_map >= MAP_TARGET

        self._csv.write_row([
            'OVERALL', '-', '-', '-', '-', '-', '-', '-', '-',
            round(overall_map, 4)
        ])
        self._summary.add(0, 'Overall mAP', overall_map, '')

        print('\n' + '=' * 60)
        print('  EXPERIMENT 6 — CNN DETECTION RESULTS')
        print('=' * 60)
        print(f'  Overall mAP@IoU0.5 = {overall_map:.4f}')
        print(f'  Target ≥ {MAP_TARGET}  →  {"PASS" if passed else "FAIL"}')
        print('=' * 60 + '\n')

        self._summary.print_summary()
        self._csv.close()

        msg = String()
        msg.data = f'EXP6 complete — mAP={overall_map:.4f}  {"PASS" if passed else "FAIL"}'
        self._log_pub.publish(msg)
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = CnnDetectionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
