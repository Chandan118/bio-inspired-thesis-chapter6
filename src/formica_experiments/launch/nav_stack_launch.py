"""
nav_stack_launch.py
===================
Convenience launch file that brings up the full navigation stack
needed for Experiments 3–5 in Chapter 6.

Starts:
    - RPLIDAR A1 driver
    - Nav2 bringup (AMCL + map_server + navigation)
    - cmd_vel relay (/cmd_vel -> /cmd_vel_nav) so the base actually moves

Prerequisites:
    sudo apt install ros-humble-nav2-bringup
    sudo apt install ros-humble-rplidar-ros

Usage:
    ros2 launch formica_experiments nav_stack_launch.py

Edit the parameter file paths below to match your robot's URDF and
nav2 configuration files.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Package share directories ────────────────────────────────────────────
    nav2_bringup_dir    = get_package_share_directory('nav2_bringup')

    # ── Config paths (place your YAML files here or adjust paths) ────────────
    formica_config_dir = os.path.join(
        get_package_share_directory('formica_experiments'), 'config'
    )
    nav2_params_file = os.path.join(formica_config_dir, 'nav2_params.yaml')

    # Default map file (override via launch argument)
    default_map_yaml = '/home/jetson/master_ws/my_first_map.yaml'
    map_yaml = LaunchConfiguration('map', default=default_map_yaml)

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map_yaml,
            description='Full path to the map YAML to load for AMCL',
        ),

        # ── RPLIDAR A1 ─────────────────────────────────────────────────────────
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
            output='screen',
        ),

        # ── Nav2 Bringup (AMCL + map_server + navigation) ─────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'slam': 'False',
                'map': map_yaml,
                'params_file': nav2_params_file,
                'use_sim_time': 'false',
            }.items(),
        ),

        # ── cmd_vel relay: Nav2 publishes /cmd_vel, base listens /cmd_vel_nav ──
        Node(
            package='formica_experiments',
            executable='cmd_vel_relay',
            name='cmd_vel_relay',
            parameters=[{
                'input_topic': '/cmd_vel',
                'output_topic': '/cmd_vel_nav',
            }],
            output='screen',
        ),

        # ── Robot state publisher (publish TF from URDF) ───────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': '',   # fill with xacro.process_file() if needed
            }],
            output='screen',
        ),

    ])
