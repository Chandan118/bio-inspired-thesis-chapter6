#!/usr/bin/env python3
"""
exp3_slam_mapping.py - ROS 1 Version
====================================
Experiment 3 - Online SLAM with Landmark Detection.

ROS 1 Topics:
  - /scan           (sensor_msgs/LaserScan)
  - /odom           (nav_msgs/Odometry)
  - /map            (nav_msgs/OccupancyGrid)
  - /particlecloud  (geometry_msgs/PoseArray)

Run:
    rosrun formica_experiments exp3_slam_mapping.py
"""

import math
import os
import threading
import time

import rospy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry

from formica_experiments.data_logger import CsvLogger, timestamped_filename


class SlamMappingNode(object):
    def __init__(self):
        rospy.init_node("exp3_slam_mapping", anonymous=True)

        self._lock = threading.Lock()
        self._map_data = None
        self._robot_pose_trail = []
        self._particle_count = 0

        self._cb_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self._cb_thread.start()

        rospy.Subscriber("/map", OccupancyGrid, self._on_map, queue_size=3)
        rospy.Subscriber("/odom", Odometry, self._on_odom, queue_size=10)
        rospy.Subscriber("/particlecloud", PoseArray, self._on_particles, queue_size=5)

        rospy.loginfo("Experiment 3: Starting SLAM mapping.")

    def _ros_spin(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()

    def _on_map(self, msg):
        with self._lock:
            self._map_data = {
                "width": msg.info.width,
                "height": msg.info.height,
                "resolution": msg.info.resolution,
                "timestamp": msg.header.stamp.to_sec(),
            }

    def _on_odom(self, msg):
        with self._lock:
            self._robot_pose_trail.append({
                "t": msg.header.stamp.to_sec(),
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "yaw": math.atan2(
                    2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z +
                           msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
                    1.0 - 2.0 * (msg.pose.pose.orientation.y ** 2 + msg.pose.pose.orientation.z ** 2)
                ),
            })

    def _on_particles(self, msg):
        with self._lock:
            self._particle_count = len(msg.poses)

    def run(self):
        ts = timestamped_filename("exp3_slam")
        logger = CsvLogger("exp3_slam", ["timestamp", "map_width", "map_height", "particles", "x", "y", "yaw"])

        rate = rospy.Rate(1.0)
        duration_s = rospy.get_param("~duration_s", 300.0)
        end_time = rospy.Time.now().to_sec() + duration_s

        while rospy.Time.now().to_sec() < end_time and not rospy.is_shutdown():
            with self._lock:
                if self._map_data and self._robot_pose_trail:
                    entry = self._robot_pose_trail[-1]
                    logger.write_row([
                        entry["t"],
                        self._map_data["width"],
                        self._map_data["height"],
                        self._particle_count,
                        entry["x"],
                        entry["y"],
                        entry["yaw"],
                    ])
            rate.sleep()

        logger.close()
        return self._compute_summary()

    def _compute_summary(self):
        with self._lock:
            poses = list(self._robot_pose_trail)
            map_info = self._map_data

        results = {
            "total_poses": len(poses),
            "map_width": map_info["width"] if map_info else 0,
            "map_height": map_info["height"] if map_info else 0,
            "map_resolution": map_info["resolution"] if map_info else 0,
        }

        if len(poses) > 1:
            dx = poses[-1]["x"] - poses[0]["x"]
            dy = poses[-1]["y"] - poses[0]["y"]
            results["total_distance_m"] = math.sqrt(dx * dx + dy * dy)

        rospy.loginfo("SLAM Summary: %s", results)
        return results


def main():
    node = SlamMappingNode()
    node.run()


if __name__ == "__main__":
    main()
