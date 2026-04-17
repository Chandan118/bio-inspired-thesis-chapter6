#!/usr/bin/env python3
"""
exp6_cnn_detection.py - ROS 1 Version
======================================
Experiment 6 - CNN Target Detection and Classification with TensorRT.

ROS 1 Topics:
  - /rgb/image_raw               (sensor_msgs/Image)
  - /cnn_detections              (vision_msgs/Detection2DArray)
  - /cnn/class_confidence        (std_msgs/Float32MultiArray)

Run:
    rosrun formica_experiments exp6_cnn_detection.py
"""

import csv
import os
import threading
import time

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

try:
    from cv_bridge import CvBridge
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


class CNNDetectionNode(object):
    def __init__(self):
        rospy.init_node("exp6_cnn_detection", anonymous=True)

        self._lock = threading.Lock()
        self._detections = []
        self._confidence_scores = []
        self._frame_count = 0
        self._start_time = None

        self._cb_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self._cb_thread.start()

        if CV2_AVAILABLE:
            self._bridge = CvBridge()

        rospy.Subscriber("/rgb/image_raw", Image, self._on_image, queue_size=1)
        rospy.Subscriber("/cnn_detections", rospy.AnyMsg, self._on_detections, queue_size=5)
        rospy.Subscriber("/cnn/class_confidence", Float32MultiArray, self._on_confidence, queue_size=5)

        self._start_time = rospy.Time.now().to_sec()
        rospy.loginfo("Experiment 6: Starting CNN detection.")

    def _ros_spin(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()

    def _on_image(self, msg):
        with self._lock:
            self._frame_count += 1

    def _on_detections(self, msg):
        with self._lock:
            self._detections.append({
                "t": rospy.Time.now().to_sec(),
                "count": msg._connection_header.get('count', 1),
            })

    def _on_confidence(self, msg):
        with self._lock:
            self._confidence_scores.extend(msg.data)

    def run(self):
        ts = timestamped_filename("exp6_cnn")
        logger = CsvLogger("exp6_cnn", ["timestamp", "detection_count", "avg_confidence"])

        rate = rospy.Rate(1.0)
        duration_s = rospy.get_param("~duration_s", 120.0)
        end_time = rospy.Time.now().to_sec() + duration_s

        while rospy.Time.now().to_sec() < end_time and not rospy.is_shutdown():
            with self._lock:
                det_count = len(self._detections)
                avg_conf = statistics.mean(self._confidence_scores) if self._confidence_scores else 0.0
                logger.write_row([rospy.Time.now().to_sec(), det_count, avg_conf])
            rate.sleep()

        logger.close()
        return self._compute_summary()

    def _compute_summary(self):
        results = {
            "total_frames": self._frame_count,
            "total_detections": len(self._detections),
            "avg_confidence": statistics.mean(self._confidence_scores) if self._confidence_scores else 0.0,
            "fps": self._frame_count / (rospy.Time.now().to_sec() - self._start_time),
        }
        rospy.loginfo("CNN Detection Summary: %s", results)
        return results


def main():
    if not CV2_AVAILABLE:
        rospy.logerr("OpenCV not available. Install: pip install opencv-python")
        return
    node = CNNDetectionNode()
    node.run()


if __name__ == "__main__":
    main()
