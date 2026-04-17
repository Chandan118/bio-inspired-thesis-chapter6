#!/usr/bin/env python3
"""
exp4_maze_navigation.py - ROS 1 Version
========================================
Experiment 4 - Maze Navigation and Path Planning with Nav2.

ROS 1 Topics:
  - /odom           (nav_msgs/Odometry)
  - /cmd_vel        (geometry_msgs/Twist)
  - /move_base_simple/goal  (geometry_msgs/PoseStamped)

Run:
    rosrun formica_experiments exp4_maze_navigation.py
"""

import csv
import math
import os
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid

from formica_experiments.data_logger import CsvLogger, timestamped_filename


class MazeNavigationNode(object):
    def __init__(self):
        rospy.init_node("exp4_maze_navigation", anonymous=True)

        self._lock = threading.Lock()
        self._robot_pose = None
        self._map_data = None
        self._nav_goals = []
        self._start_time = None

        self._cb_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self._cb_thread.start()

        rospy.Subscriber("/odom", Odometry, self._on_odom, queue_size=10)
        rospy.Subscriber("/map", OccupancyGrid, self._on_map, queue_size=5)

        self._cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self._goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        self._start_time = rospy.Time.now().to_sec()
        rospy.loginfo("Experiment 4: Starting maze navigation.")

    def _ros_spin(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()

    def _on_odom(self, msg):
        with self._lock:
            self._robot_pose = {
                "t": msg.header.stamp.to_sec(),
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
            }

    def _on_map(self, msg):
        with self._lock:
            self._map_data = {
                "width": msg.info.width,
                "height": msg.info.height,
                "resolution": msg.info.resolution,
            }

    def send_goal(self, x, y, theta=0.0):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "odom"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self._goal_pub.publish(goal)

    def run(self):
        ts = timestamped_filename("exp4_maze")
        logger = CsvLogger("exp4_maze", ["timestamp", "x", "y", "map_width", "map_height", "goal_x", "goal_y"])

        rate = rospy.Rate(1.0)
        duration_s = rospy.get_param("~duration_s", 300.0)
        end_time = rospy.Time.now().to_sec() + duration_s

        goals = rospy.get_param("~goals", [[1.0, 0.0], [2.0, 0.0], [2.0, 1.0]])
        goal_idx = 0
        last_goal_time = rospy.Time.now().to_sec()

        while rospy.Time.now().to_sec() < end_time and not rospy.is_shutdown():
            with self._lock:
                if self._robot_pose and self._map_data:
                    t = self._robot_pose["t"]
                    x = self._robot_pose["x"]
                    y = self._robot_pose["y"]

                    goal = goals[goal_idx]
                    logger.write_row([t, x, y, self._map_data["width"], self._map_data["height"],
                                     goal[0], goal[1]])

                    if rospy.Time.now().to_sec() - last_goal_time > 30.0:
                        goal_idx = (goal_idx + 1) % len(goals)
                        self.send_goal(goal[0], goal[1])
                        last_goal_time = rospy.Time.now().to_sec()

            rate.sleep()

        logger.close()
        return self._compute_summary()

    def _compute_summary(self):
        with self._lock:
            poses = self._robot_pose_trail if hasattr(self, '_robot_pose_trail') else []

        results = {
            "duration_s": rospy.Time.now().to_sec() - self._start_time,
            "poses_recorded": len(poses),
        }
        rospy.loginfo("Maze Navigation Summary: %s", results)
        return results


def main():
    node = MazeNavigationNode()
    node.run()


if __name__ == "__main__":
    main()
