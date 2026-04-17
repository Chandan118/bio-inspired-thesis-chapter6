#!/usr/bin/env python3
"""
exp7_pheromone_trail.py - ROS 1 Version
========================================
Experiment 7 - Pheromone Trail Following and Ethanol Detection.

ROS 1 Topics:
  - /line_sensors                (std_msgs/Float32MultiArray)
  - /gas_sensor                  (std_msgs/Float32) - MQ-3 ethanol
  - /cmd_vel                     (geometry_msgs/Twist)
  - /imu/data                    (sensor_msgs/Imu)

Run:
    rosrun formica_experiments exp7_pheromone_trail.py
"""

import csv
import math
import os
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float32MultiArray

from formica_experiments.data_logger import CsvLogger, timestamped_filename


class PheromoneTrailNode(object):
    def __init__(self):
        rospy.init_node("exp7_pheromone_trail", anonymous=True)

        self._lock = threading.Lock()
        self._tcrts = [0.0] * 5
        self._ethanol_ppm = 0.0
        self._yaw = 0.0
        self._start_time = None

        self._cb_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self._cb_thread.start()

        rospy.Subscriber("/line_sensors", Float32MultiArray, self._on_line, queue_size=5)
        rospy.Subscriber("/gas_sensor", Float32, self._on_gas, queue_size=5)
        rospy.Subscriber("/imu/data", Imu, self._on_imu, queue_size=10)

        self._cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self._start_time = rospy.Time.now().to_sec()
        rospy.loginfo("Experiment 7: Starting pheromone trail following.")

    def _ros_spin(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()

    def _on_line(self, msg):
        with self._lock:
            self._tcrts = list(msg.data[:5])

    def _on_gas(self, msg):
        with self._lock:
            self._ethanol_ppm = msg.data

    def _on_imu(self, msg):
        with self._lock:
            q = msg.orientation
            self._yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z)
            )

    def run(self):
        ts = timestamped_filename("exp7_pheromone")
        logger = CsvLogger("exp7_pheromone", ["timestamp", "tcrts_0", "tcrts_1", "tcrts_2",
                                               "tcrts_3", "tcrts_4", "ethanol_ppm", "yaw_rad"])

        rate = rospy.Rate(10.0)
        duration_s = rospy.get_param("~duration_s", 180.0)
        end_time = rospy.Time.now().to_sec() + duration_s

        while rospy.Time.now().to_sec() < end_time and not rospy.is_shutdown():
            with self._lock:
                logger.write_row([
                    rospy.Time.now().to_sec(),
                    self._tcrts[0], self._tcrts[1], self._tcrts[2],
                    self._tcrts[3], self._tcrts[4],
                    self._ethanol_ppm, self._yaw
                ])

                twist = Twist()
                if self._ethanol_ppm > 500:
                    twist.linear.x = 0.1
                else:
                    twist.linear.x = 0.0
                self._cmd_pub.publish(twist)

            rate.sleep()

        logger.close()
        return self._compute_summary()

    def _compute_summary(self):
        results = {
            "duration_s": rospy.Time.now().to_sec() - self._start_time,
        }
        rospy.loginfo("Pheromone Trail Summary: %s", results)
        return results


def main():
    node = PheromoneTrailNode()
    node.run()


if __name__ == "__main__":
    main()
