"""
master_launch.py
================
Master ROS 2 launch file for Chapter 6 experiments.

This file starts the shared data_logger node. Individual experiments are
launched separately via their own ros2 run commands (see README below).

Usage:
    ros2 launch formica_experiments master_launch.py

Then in separate terminals run the desired experiment, for example:
    ros2 run formica_experiments exp1_calibration
    ros2 run formica_experiments exp2_power
    ros2 run formica_experiments exp3_slam
    ros2 run formica_experiments exp4_maze
    ros2 run formica_experiments exp5_fault
    ros2 run formica_experiments exp6_cnn
    ros2 run formica_experiments exp7_pheromone

NOTE: Experiments 3–7 require the following nodes to be launched first:
    ros2 launch rplidar_ros2 rplidar_launch.py
    ros2 launch azure_kinect_ros_driver driver.launch.py
    ros2 launch slam_toolbox online_async_launch.py
    ros2 launch nav2_bringup navigation_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='formica_experiments',
            executable='data_logger',
            name='data_logger',
            output='screen',
        ),
    ])
