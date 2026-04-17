#!/usr/bin/env python3
"""
exp5_obstacle_fault.py - ROS 1 Version
=======================================
Experiment 5 - Fault Tolerance and Obstacle Avoidance.

ROS 1 Topics:
  - /scan           (sensor_msgs/LaserScan)
  - /odom           (nav_msgs/Odometry)
  - /cmd_vel        (geometry_msgs/Twist)

Run:
    rosrun formica_experiments exp5_obstacle_fault.py
"""

import csv
import math
import os
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from formica_experiments.data_logger import CsvLogger, timestamped_filename


class ObstacleFaultNode(object):
    def __init__(self):
        rospy.init_node("exp5_obstacle_fault", anonymous=True)

        self._lock = threading.Lock()
        self._scan_range = 0.0
        self._odom_x = 0.0
        self._scan_readings = []
        self._start_time = None

        self._cb_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self._cb_thread.start()

        rospy.Subscriber("/scan", LaserScan, self._on_scan, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self._on_odom, queue_size=10)

        self._cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self._start_time = rospy.Time.now().to_sec()
        rospy.loginfo("Experiment 5: Starting obstacle/fault tolerance.")

    def _ros_spin(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()

    def _on_scan(self, msg):
        with self._lock:
            self._scan_readings = list(msg.ranges)
            min_range = min(msg.ranges) if msg.ranges else float('inf')
            self._scan_range = min_range if min_range != float('inf') else msg.range_max

    def _on_odom(self, msg):
        with self._lock:
            self._odom_x = msg.pose.pose.position.x

    def run(self):
        ts = timestamped_filename("exp5_fault")
        logger = CsvLogger("exp5_fault", ["timestamp", "closest_obstacle_m", "x_position_m"])

        rate = rospy.Rate(10.0)
        duration_s = rospy.get_param("~duration_s", 300.0)
        end_time = rospy.Time.now().to_sec() + duration_s

        obstacles = rospy.get_param("~obstacle_positions", [2.0, 4.0])

        while rospy.Time.now().to_sec() < end_time and not rospy.is_shutdown():
            with self._lock:
                t = rospy.Time.now().to_sec()
                logger.write_row([t, self._scan_range, self._odom_x])

                twist = Twist()
                if self._scan_range < 0.5:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5
                else:
                    twist.linear.x = 0.2
                    twist.angular.z = 0.0
                self._cmd_pub.publish(twist)

            rate.sleep()

        logger.close()
        return self._compute_summary()

    def _compute_summary(self):
        results = {
            "duration_s": rospy.Time.now().to_sec() - self._start_time,
        }
        rospy.loginfo("Fault Tolerance Summary: %s", results)
        return results


def main():
    node = ObstacleFaultNode()
    node.run()


if __name__ == "__main__":
    main()
