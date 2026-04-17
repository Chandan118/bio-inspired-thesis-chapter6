#!/usr/bin/env python3
"""
exp2_power_profiling.py - ROS 1 Version
=========================================
Experiment 2 - Power Profiling and Energy Consumption Analysis.

ROS 1 Topics:
  - /ina219/current  (std_msgs/Float32) - Current in mA
  - /ina219/voltage  (std_msgs/Float32) - Voltage in V
  - /odom            (nav_msgs/Odometry) - Wheel odometry

Run:
    rosrun formica_experiments exp2_power_profiling.py
"""

import csv
import math
import os
import statistics
import threading
import time
from collections import deque

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

from formica_experiments.data_logger import CsvLogger, timestamped_filename

SAMPLE_INTERVAL_S = 0.1
POWER_SAMPLES_WINDOW = 50
BATTERY_NOMINAL_V = 11.1
BATTERY_NOMINAL_AH = 5.0


class PowerProfilingNode(object):
    def __init__(self):
        rospy.init_node("exp2_power_profiling", anonymous=True)

        self._lock = threading.Lock()
        self._power_samples = deque(maxlen=POWER_SAMPLES_WINDOW)
        self._voltage_samples = deque(maxlen=POWER_SAMPLES_WINDOW)
        self._current_samples = deque(maxlen=POWER_SAMPLES_WINDOW)
        self._odom_trail = []
        self._start_time = None

        self._cb_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self._cb_thread.start()

        rospy.Subscriber("/ina219/current", Float32, self._on_current, queue_size=10)
        rospy.Subscriber("/ina219/voltage", Float32, self._on_voltage, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self._on_odom, queue_size=5)

        self._start_time = rospy.Time.now().to_sec()
        rospy.loginfo("Experiment 2: Starting power profiling.")

    def _ros_spin(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()

    def _on_current(self, msg):
        with self._lock:
            self._current_samples.append((rospy.Time.now().to_sec(), msg.data))
            if len(self._current_samples) > 0 and self._start_time is None:
                self._start_time = self._current_samples[0][0]

    def _on_voltage(self, msg):
        with self._lock:
            self._voltage_samples.append((rospy.Time.now().to_sec(), msg.data))

    def _on_odom(self, msg):
        with self._lock:
            self._odom_trail.append((rospy.Time.now().to_sec(), msg))

    def run(self):
        rate = rospy.Rate(1.0 / SAMPLE_INTERVAL_S)
        ts = timestamped_filename("exp2_power")
        logger = CsvLogger("exp2_power", ["timestamp", "voltage_v", "current_ma", "power_w"])

        duration_s = rospy.get_param("~duration_s", 60.0)
        end_time = rospy.Time.now().to_sec() + duration_s

        while rospy.Time.now().to_sec() < end_time and not rospy.is_shutdown():
            with self._lock:
                if self._current_samples and self._voltage_samples:
                    t = rospy.Time.now().to_sec()
                    v = self._voltage_samples[-1][1]
                    i = self._current_samples[-1][1]
                    p = v * i / 1000.0  # W
                    logger.write_row([t, v, i, p])
            rate.sleep()

        logger.close()
        return self._compute_summary()

    def _compute_summary(self):
        with self._lock:
            voltages = [v for _, v in self._voltage_samples]
            currents = [i for _, i in self._current_samples]

        if not voltages or not currents:
            rospy.logwarn("No power samples collected.")
            return {}

        avg_v = statistics.mean(voltages)
        avg_i = statistics.mean(currents)
        avg_p = avg_v * avg_i / 1000.0

        energy_wh = 0.0
        if len(self._current_samples) > 1:
            for j in range(1, len(self._current_samples)):
                dt = self._current_samples[j][0] - self._current_samples[j - 1][0]
                energy_wh += self._voltage_samples[j][1] * self._current_samples[j][1] * dt / 3600000.0

        results = {
            "avg_voltage_v": avg_v,
            "avg_current_ma": avg_i,
            "avg_power_w": avg_p,
            "energy_wh": energy_wh,
            "runtime_s": rospy.Time.now().to_sec() - self._start_time,
        }

        rospy.loginfo("Power Summary: %s", results)
        return results


def main():
    node = PowerProfilingNode()
    node.run()


if __name__ == "__main__":
    main()
