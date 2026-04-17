from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    baudrate = LaunchConfiguration('baudrate', default='115200')
    
    return LaunchDescription([
        DeclareLaunchArgument('baudrate', default_value=baudrate),
        Node(
            # namespace='imu',  # REMOVED: putting topics in global namespace for experiment compatibility
            package='lpms_ig1',
            executable='lpms_ig1_node',
            name='lpms_ig1_node',
            output='screen',
            parameters=[{
                "port": "/dev/ttyUSB1",
                "baudrate": 115200 # Pass as integer
            }]
        ),
        Node(
            # namespace='imu',
            package='lpms_ig1',
            executable='quat_to_euler_node',
            name='quat_to_euler_node'
        )
    ])
