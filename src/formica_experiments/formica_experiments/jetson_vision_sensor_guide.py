#!/usr/bin/env python3
"""
jetson_vision_sensor_guide.py
==============================
ROS 2 node for Jetson: subscribe to camera + Arduino sensors, run vision
(Experiment 6 TRT/HSV *or* Ultralytics YOLO), publish a human-readable scene
summary and optionally emit gentle cmd_vel.

  # COCO YOLOv8n (downloads weights on first run if missing):
  ros2 run formica_experiments jetson_vision_guide --ros-args \
    -p vision_backend:=yolo -p yolo_model:=yolov8n.pt \
    -p image_topic:=/camera/image_raw

  # Original Exp6-style coloured targets:
  ros2 run formica_experiments jetson_vision_guide --ros-args \
    -p vision_backend:=exp6 -p image_topic:=/rgb/image_raw

  # YOLO slow autonomous walk (needs arduino_base on cmd_vel + jetson_camera):
  ros2 run formica_experiments jetson_vision_guide --ros-args \
    -p vision_backend:=yolo -p nav_mode:=wander -p enable_cmd_vel:=true \
    -p image_topic:=/camera/image_raw -p forward_speed:=0.1
"""

from __future__ import annotations

import math
import time
from typing import Any, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray, String

from formica_experiments.exp6_cnn_detection import (
    CLASS_NAMES,
    CONF_THRESHOLD,
    ENGINE_PATH,
    TRTDetector,
)

try:
    from ultralytics import YOLO as UltralyticsYOLO
except Exception:  # pragma: no cover
    UltralyticsYOLO = None


def _describe_detections(
    dets: List[dict], frame_w: int, empty_msg: str
) -> tuple[str, Optional[float]]:
    """Return short text and optional steer hint (-1..1, negative = turn right)."""
    if not dets:
        return empty_msg, None
    parts: List[str] = []
    steer: Optional[float] = None
    for d in sorted(dets, key=lambda x: -float(x['conf']))[:5]:
        name = str(d['label'])
        x1, _, x2, _ = d['bbox']
        cx = 0.5 * (float(x1) + float(x2))
        rel = (cx - 0.5 * frame_w) / max(float(frame_w), 1.0)
        side = 'centre'
        if rel < -0.15:
            side = 'left of centre'
        elif rel > 0.15:
            side = 'right of centre'
        parts.append(f'{name} ({100.0 * float(d["conf"]):.0f}% confident, {side})')
        if steer is None:
            steer = max(-1.0, min(1.0, rel * 2.5))
    return '; '.join(parts), steer


def _exp6_to_generic(dets: List[dict]) -> List[dict]:
    out: List[dict] = []
    for d in dets:
        cid = int(d['class_id'])
        label = CLASS_NAMES[cid] if cid < len(CLASS_NAMES) else str(cid)
        out.append({'label': label, 'conf': float(d['conf']), 'bbox': d['bbox']})
    return out


def _yolo_infer(
    model: Any,
    frame_bgr: np.ndarray,
    conf: float,
    imgsz: int,
    device: str,
    max_det: int,
    class_ids: Optional[List[int]],
) -> List[dict]:
    kwargs = {
        'conf': conf,
        'imgsz': imgsz,
        'verbose': False,
        'max_det': max_det,
    }
    if device:
        kwargs['device'] = device
    if class_ids is not None:
        kwargs['classes'] = class_ids
    results = model.predict(frame_bgr, **kwargs)
    if not results:
        return []
    r0 = results[0]
    names = getattr(r0, 'names', None) or {}
    boxes = getattr(r0, 'boxes', None)
    if boxes is None or len(boxes) == 0:
        return []
    out: List[dict] = []
    for box in boxes:
        cls_id = int(box.cls.item())
        label = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else str(cls_id)
        out.append(
            {
                'label': label,
                'cls_id': cls_id,
                'conf': float(box.conf.item()),
                'bbox': [float(x) for x in box.xyxy[0].tolist()],
            }
        )
    return out


# COCO IDs often in front of a small robot (same spirit as tethered_navigation camera_obstacle_detector).
_DEFAULT_YOLO_OBSTACLE_IDS = frozenset({0, 1, 2, 3, 5, 7, 15, 16, 56, 57})


def _parse_obstacle_ids(s: str) -> frozenset[int]:
    if not str(s).strip():
        return _DEFAULT_YOLO_OBSTACLE_IDS
    out = set()
    for part in str(s).split(','):
        part = part.strip()
        if part:
            out.add(int(part))
    return frozenset(out) if out else _DEFAULT_YOLO_OBSTACLE_IDS


def _yolo_avoidance_steer_linear(
    dets: List[dict],
    frame_w: int,
    frame_h: int,
    obstacle_ids: frozenset[int],
    roi_y_min_frac: float,
    min_area_ratio: float,
    block_area_ratio: float,
) -> tuple[Optional[float], float, str]:
    """
    Wander-style navigation: steer away from obstacle-like detections in the lower
    (forward) part of the image. Returns (angular_hint, linear_scale, reason_tag).
    angular_hint sign matches exp6/approach convention (positive = turn left).
    """
    fw = max(int(frame_w), 1)
    fh = max(int(frame_h), 1)
    frame_area = float(fw * fh)
    best_threat = 0.0
    best_cx = 0.0
    best_label = ''
    best_area_ratio = 0.0

    y_cut = fh * roi_y_min_frac

    for d in dets:
        cid = d.get('cls_id')
        if cid is None or int(cid) not in obstacle_ids:
            continue
        x1, y1, x2, y2 = d['bbox']
        x1, y1, x2, y2 = float(x1), float(y1), float(x2), float(y2)
        if y2 < y_cut:
            continue
        bw = max(0.0, x2 - x1)
        bh = max(0.0, y2 - y1)
        area_ratio = (bw * bh) / frame_area
        if area_ratio < min_area_ratio:
            continue
        cx = 0.5 * (x1 + x2)
        threat = float(d['conf']) * min(1.0, area_ratio / max(min_area_ratio, 1e-6))
        if threat > best_threat:
            best_threat = threat
            best_cx = cx
            best_label = str(d['label'])
            best_area_ratio = area_ratio

    if best_threat <= 0.0:
        return None, 1.0, 'clear'

    rel = (best_cx - 0.5 * fw) / (0.5 * fw)
    steer_away = max(-1.0, min(1.0, -rel * 2.8))

    if best_area_ratio >= block_area_ratio:
        return steer_away, 0.0, f'block:{best_label}'

    linear_scale = max(0.15, 1.0 - min(1.0, best_threat * 1.2))
    return steer_away, linear_scale, f'avoid:{best_label}'


class JetsonVisionSensorGuide(Node):
    def __init__(self) -> None:
        super().__init__('jetson_vision_sensor_guide')

        self.declare_parameter('image_topic', '/rgb/image_raw')
        self.declare_parameter('vision_backend', 'exp6')  # 'exp6' | 'yolo'
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('yolo_conf', 0.35)
        self.declare_parameter('yolo_imgsz', 640)
        self.declare_parameter('yolo_device', '')  # e.g. '0' for first CUDA GPU; empty = auto
        self.declare_parameter('yolo_max_det', 10)
        self.declare_parameter('yolo_class_ids', '')  # empty = all COCO classes; else "0,1,2"
        self.declare_parameter('engine_path', ENGINE_PATH)
        self.declare_parameter('conf_threshold', CONF_THRESHOLD)
        self.declare_parameter('describe_rate_hz', 0.5)
        self.declare_parameter('enable_cmd_vel', False)
        self.declare_parameter('forward_speed', 0.12)
        self.declare_parameter('turn_gain', 0.35)
        self.declare_parameter('stop_distance_cm', 22.0)
        # YOLO + cmd_vel: 'wander' avoids COCO obstacles in the lower image; 'approach'
        # steers toward the strongest detection (good for exp6 coloured targets).
        self.declare_parameter('nav_mode', 'auto')
        self.declare_parameter('yolo_obstacle_class_ids', '')
        self.declare_parameter('yolo_roi_y_min_frac', 0.38)
        self.declare_parameter('yolo_min_area_ratio', 0.035)
        self.declare_parameter('yolo_block_area_ratio', 0.18)

        topic = str(self.get_parameter('image_topic').value)
        backend = str(self.get_parameter('vision_backend').value).strip().lower()
        engine = str(self.get_parameter('engine_path').value)
        conf = float(self.get_parameter('conf_threshold').value)
        rate = float(self.get_parameter('describe_rate_hz').value)
        self._enable_motion = bool(self.get_parameter('enable_cmd_vel').value)
        self._v = float(self.get_parameter('forward_speed').value)
        self._turn_gain = float(self.get_parameter('turn_gain').value)
        self._stop_cm = float(self.get_parameter('stop_distance_cm').value)

        nav_raw = str(self.get_parameter('nav_mode').value).strip().lower()

        self._backend = 'yolo' if backend == 'yolo' else 'exp6'
        self._yolo_model = None
        self._detector: Optional[TRTDetector] = None
        self._yolo_conf = float(self.get_parameter('yolo_conf').value)
        self._yolo_imgsz = int(self.get_parameter('yolo_imgsz').value)
        self._yolo_device = str(self.get_parameter('yolo_device').value).strip()
        self._yolo_max_det = int(self.get_parameter('yolo_max_det').value)
        raw_cls = str(self.get_parameter('yolo_class_ids').value).strip()
        self._yolo_class_ids: Optional[List[int]] = None
        if raw_cls:
            self._yolo_class_ids = [int(x.strip()) for x in raw_cls.split(',') if x.strip()]

        if self._backend == 'yolo':
            if UltralyticsYOLO is None:
                self.get_logger().error(
                    'vision_backend=yolo but ultralytics is not installed; '
                    'falling back to exp6.'
                )
                self._backend = 'exp6'
            else:
                yolo_path = str(self.get_parameter('yolo_model').value)
                try:
                    self._yolo_model = UltralyticsYOLO(yolo_path)
                    self.get_logger().info(f'YOLO vision_backend: loaded {yolo_path}')
                except Exception as exc:
                    self.get_logger().error(f'YOLO load failed ({exc}); falling back to exp6.')
                    self._backend = 'exp6'
                    self._yolo_model = None

        if self._backend == 'exp6':
            self._detector = TRTDetector(engine, conf)

        if nav_raw == 'auto':
            self._nav_mode = 'wander' if self._backend == 'yolo' else 'approach'
        else:
            self._nav_mode = nav_raw if nav_raw in ('wander', 'approach') else 'wander'

        self._yolo_obstacle_ids = _parse_obstacle_ids(
            str(self.get_parameter('yolo_obstacle_class_ids').value)
        )
        self._yolo_roi_y_min = float(self.get_parameter('yolo_roi_y_min_frac').value)
        self._yolo_min_area = float(self.get_parameter('yolo_min_area_ratio').value)
        self._yolo_block_area = float(self.get_parameter('yolo_block_area_ratio').value)

        self._vision_empty_msg = (
            'no COCO objects above threshold'
            if self._backend == 'yolo'
            else 'no coloured target (red / green / blue) detected'
        )
        self._latest: Optional[np.ndarray] = None
        self._w = 0
        self._h = 0
        self._dist_cm: Optional[float] = None
        self._line: Optional[List[float]] = None
        self._logged_first_image = False
        self._last_no_frame_log_mono = 0.0

        self.create_subscription(Image, topic, self._img_cb, 10)
        self.create_subscription(Float32, 'sensor/distance', self._dist_cb, 10)
        self.create_subscription(Float32MultiArray, '/line_sensors', self._line_cb, 10)

        self._desc_pub = self.create_publisher(String, '/robot_scene_description', 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        period = max(0.2, 1.0 / max(rate, 0.05))
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f'Listening on image={topic!r}, backend={self._backend}, '
            f'nav_mode={self._nav_mode}, motion={"ON" if self._enable_motion else "OFF"}'
        )
        self.get_logger().info(
            f'If nothing logs after this: no {topic!r} publisher (start jetson_camera '
            f'or match image_topic). Same ROS_DOMAIN_ID on all nodes. '
            f'Scene text: ros2 topic echo /robot_scene_description'
        )

    def _img_cb(self, msg: Image) -> None:
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            arr = arr.reshape((msg.height, msg.width, -1))
            if msg.encoding == 'rgb8':
                arr = arr[:, :, ::-1].copy()
            self._latest = arr
            self._w = msg.width
            self._h = msg.height
            if not self._logged_first_image:
                self._logged_first_image = True
                self.get_logger().info(
                    f'First camera frame: {msg.width}x{msg.height} {msg.encoding}'
                )
        except Exception as exc:
            self.get_logger().warn(f'Image decode failed: {exc}')

    def _dist_cb(self, msg: Float32) -> None:
        self._dist_cm = float(msg.data)

    def _line_cb(self, msg: Float32MultiArray) -> None:
        self._line = list(msg.data)

    def _tick(self) -> None:
        if self._latest is None:
            text = (
                'Waiting for camera frames. Start jetson_camera or remap image_topic '
                'to your RGB topic.'
            )
            self._publish_desc(text)
            now = time.monotonic()
            if now - self._last_no_frame_log_mono >= 5.0:
                self._last_no_frame_log_mono = now
                self.get_logger().warn(
                    'Still no camera Image messages — cmd_vel held at zero. '
                    'Run: ros2 run formica_experiments jetson_camera --ros-args '
                    '-p topic_name:=<your image_topic>, or ros2 topic hz <image_topic>.'
                )
            if self._enable_motion:
                self._cmd_pub.publish(Twist())
            return

        if self._backend == 'yolo' and self._yolo_model is not None:
            gen = _yolo_infer(
                self._yolo_model,
                self._latest,
                self._yolo_conf,
                self._yolo_imgsz,
                self._yolo_device,
                self._yolo_max_det,
                self._yolo_class_ids,
            )
        else:
            assert self._detector is not None
            gen = _exp6_to_generic(self._detector.infer(self._latest))

        vision_txt, steer_hint = _describe_detections(
            gen, self._w, self._vision_empty_msg
        )

        fh = int(self._latest.shape[0]) if self._latest is not None else self._h
        fw = int(self._latest.shape[1]) if self._latest is not None else self._w

        motion_steer: Optional[float] = None
        linear_scale = 1.0
        nav_reason = ''
        if self._enable_motion:
            if self._backend == 'yolo' and self._nav_mode == 'wander':
                motion_steer, linear_scale, nav_reason = _yolo_avoidance_steer_linear(
                    gen,
                    fw,
                    fh,
                    self._yolo_obstacle_ids,
                    self._yolo_roi_y_min,
                    self._yolo_min_area,
                    self._yolo_block_area,
                )
            else:
                motion_steer = steer_hint
                linear_scale = 1.0
                nav_reason = 'approach' if steer_hint is not None else 'straight'

        if self._line is not None and len(self._line) >= 4:
            line_txt = (
                f'line sensors (reflectance scaled): '
                f'{self._line[0]:.0f}, {self._line[1]:.0f}, '
                f'{self._line[2]:.0f}, {self._line[3]:.0f}'
            )
        else:
            line_txt = 'line sensors: not available yet (is arduino_base running?)'

        if self._dist_cm is not None:
            dist_txt = f'ultrasonic distance ≈ {self._dist_cm:.1f} cm'
            too_close = self._dist_cm < self._stop_cm
        else:
            dist_txt = 'ultrasonic: no reading yet'
            too_close = False

        motion_txt = 'Motors: idle (enable_cmd_vel is false).'
        tw = Twist()
        if self._enable_motion:
            if too_close:
                motion_txt = (
                    f'Motors: stopped — obstacle closer than {self._stop_cm:.0f} cm.'
                )
            else:
                tag = f' [{nav_reason}]' if nav_reason else ''
                motion_txt = f'Motors: autonomous crawl ({self._nav_mode}){tag}.'
                tw.linear.x = self._v * linear_scale
                if motion_steer is not None:
                    tw.angular.z = float(motion_steer) * self._turn_gain
                tw.linear.x *= max(0.2, 1.0 - min(1.0, abs(tw.angular.z)))

        paragraph = (
            f'Robot perception — Vision ({self._backend}): {vision_txt}. {line_txt}. {dist_txt}. '
            f'{motion_txt}'
        )
        self.get_logger().info(paragraph)
        self._publish_desc(paragraph)
        if self._enable_motion:
            if not math.isfinite(tw.linear.x):
                tw.linear.x = 0.0
            if not math.isfinite(tw.angular.z):
                tw.angular.z = 0.0
            self._cmd_pub.publish(tw)

    def _publish_desc(self, text: str) -> None:
        m = String()
        m.data = text
        self._desc_pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JetsonVisionSensorGuide()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if getattr(node, '_enable_motion', False):
            stop = Twist()
            try:
                node._cmd_pub.publish(stop)
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
