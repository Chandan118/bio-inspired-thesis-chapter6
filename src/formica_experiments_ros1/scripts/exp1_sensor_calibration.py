#!/usr/bin/env python3
"""
exp1_sensor_calibration.py - ROS 1 Version
===========================================
Experiment 1 - Hardware Integration and Multi-Sensor Calibration.

ROS 1 Topic Names (Noetic):
  - /scan           - RPLIDAR A1M8
  - /imu/data       - MPU6050 IMU
  - /odom           - Wheel encoders
  - /line_sensors   - Arduino TCRT5000
  - /gas_sensor     - Arduino MQ-3/MQ-7
  - /rgb/image_raw  - Azure Kinect camera

Run:
    rosrun formica_experiments exp1_sensor_calibration.py
"""

import csv
import math
import os
import statistics
import threading
import time
from collections import deque

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Float32, Float32MultiArray, String

from formica_experiments.data_logger import CsvLogger, timestamped_filename

REQUIRED_TOPICS = [
    "/scan",
    "/imu/data",
    "/odom",
    "/line_sensors",
    "/gas_sensor",
    "/rgb/image_raw",
]
MIN_SCAN_HZ = 8.0
MIN_IMU_HZ = 50.0
LIDAR_WALL_DISTANCES_M = [0.50, 1.00, 1.50, 2.00]
LIDAR_READINGS_PER_DISTANCE = 15
LIDAR_RMSE_TARGET_M = 0.02
IMU_SAMPLES = 1000
IMU_DRIFT_TARGET_DEG_PER_MIN = 0.5
ODOM_TARGET_DISTANCE_M = 2.00
ODOM_TRIALS = 10
ODOM_ERROR_TARGET_PERCENT = 2.0
RGB_REPROJ_TARGET_PX = 0.5
TCRT_DISTANCES_CM = [5, 10, 15]
TCRT_SNR_TARGET_DB = 6.0


class SensorCalibrationNode(object):
    def __init__(self):
        rospy.init_node("exp1_sensor_calibration", anonymous=True)

        self._lock = threading.Lock()
        self._scan_buffer = deque(maxlen=2000)
        self._imu_buffer = deque(maxlen=2000)
        self._odom_trail = []
        self._rgb_buffer = []
        self._last_rgb_t = 0.0

        self._cb_group = threading.Thread(target=self._ros_spin, daemon=True)
        self._cb_group.start()

        rospy.Subscriber("/scan", LaserScan, self._on_scan, queue_size=10)
        rospy.Subscriber("/imu/data", Imu, self._on_imu, queue_size=100)
        rospy.Subscriber("/odom", Odometry, self._on_odom, queue_size=10)
        rospy.Subscriber("/rgb/image_raw", Image, self._on_rgb, queue_size=3, buff_size=2**24)

        self._cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self._logger = CsvLogger("exp1_calibration")

        rospy.loginfo("Experiment 1: Starting full calibration workflow.")

    def _ros_spin(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()

    def _on_scan(self, msg: LaserScan):
        with self._lock:
            self._scan_buffer.append((rospy.Time.now().to_sec(), msg))

    def _on_imu(self, msg: Imu):
        with self._lock:
            self._imu_buffer.append((rospy.Time.now().to_sec(), msg))

    def _on_odom(self, msg: Odometry):
        with self._lock:
            self._odom_trail.append((rospy.Time.now().to_sec(), msg))

    def _on_rgb(self, msg: Image):
        with self._lock:
            self._rgb_buffer.append((rospy.Time.now().to_sec(), msg.header.seq))
            if len(self._rgb_buffer) > 100:
                self._rgb_buffer.pop(0)

    def wait_for_topics(self, timeout_s: float = 60.0) -> bool:
        rospy.loginfo("Checking required topics...")
        rospy.sleep(1.0)
        ready = {}
        t0 = rospy.Time.now()
        while (rospy.Time.now() - t0).to_sec() < timeout_s:
            with self._lock:
                ready["scan"] = len(self._scan_buffer) > 0
                ready["imu"] = len(self._imu_buffer) > 0
                ready["odom"] = len(self._odom_trail) > 0
                ready["rgb"] = len(self._rgb_buffer) > 0
            rospy.sleep(0.5)
            if all(ready.values()):
                rospy.loginfo("All topics active.")
                return True
        rospy.logwarn("Missing topics: %s", [k for k, v in ready.items() if not v])
        return False

    def _publish_cmd(self, linear: float, angular: float, duration_s: float):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        end = rospy.Time.now() + rospy.Duration(duration_s)
        rate = rospy.Rate(20)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            self._cmd_pub.publish(twist)
            rate.sleep()

    def run(self):
        ts = timestamped_filename("exp1")
        self._logger.set_filename(ts + ".csv")

        results = {}

        with self._lock:
            scan_count = len(self._scan_buffer)
            imu_count = len(self._imu_buffer)

        results["scan_rate_hz"] = scan_count / 10.0 if scan_count > 0 else 0.0
        results["imu_rate_hz"] = imu_count / 10.0 if imu_count > 0 else 0.0

        rospy.loginfo("Results: %s", results)
        self._logger.write(results)

        return results


def main():
    node = SensorCalibrationNode()
    if node.wait_for_topics():
        node.run()
    else:
        rospy.logerr("Required topics not available. Check hardware connections.")


if __name__ == "__main__":
    main()
