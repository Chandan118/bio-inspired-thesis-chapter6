#!/usr/bin/env python3
"""
task2_mq_ethanol_record.py - ROS 1 Version
===========================================
Record MQ-3 ethanol gas sensor data from /gas_sensor topic.

Run:
    rosrun formica_experiments task2_mq_ethanol_record.py --output exp7_mq_data.csv
"""

import csv
import argparse
import time

import rospy
from std_msgs.msg import Float32


class EthanolRecorder(object):
    def __init__(self, output_file):
        rospy.init_node("task2_mq_ethanol_record", anonymous=True)

        self._output_file = output_file
        self._samples = []
        self._start_time = None

        rospy.Subscriber("/gas_sensor", Float32, self._on_gas, queue_size=10)

        rospy.loginfo("Recording MQ-3 ethanol data...")

    def _on_gas(self, msg):
        if self._start_time is None:
            self._start_time = rospy.Time.now().to_sec()
        self._samples.append((rospy.Time.now().to_sec(), msg.data))

    def run(self):
        rate = rospy.Rate(10.0)
        duration_s = rospy.get_param("~duration_s", 180.0)
        end_time = rospy.Time.now().to_sec() + duration_s

        rospy.loginfo("Recording for %.1f seconds...", duration_s)

        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < end_time:
            rate.sleep()

        with open(self._output_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "ethanol_ppm"])
            for t, v in self._samples:
                writer.writerow([t, v])

        rospy.loginfo("Saved %d samples to %s", len(self._samples), self._output_file)


def main():
    parser = argparse.ArgumentParser(description="Record ethanol sensor data")
    parser.add_argument("-o", "--output", required=True, help="Output CSV path")
    args, unknown = parser.parse_known_args()

    recorder = EthanolRecorder(args.output)
    recorder.run()


if __name__ == "__main__":
    main()
