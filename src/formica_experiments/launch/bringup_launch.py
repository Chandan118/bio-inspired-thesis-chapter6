from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_port = LaunchConfiguration(
        'lidar_port',
        default='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
    )
    imu_port = LaunchConfiguration(
        'imu_port',
        default='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_IG14859004F004A-if00-port0'
    )
    arduino_port = LaunchConfiguration('arduino_port', default='')
    enable_imu = LaunchConfiguration('enable_imu', default='false')
    enable_camera = LaunchConfiguration('enable_camera', default='false')

    return LaunchDescription([
        LogInfo(msg='Bringing up FormicaBot Hardware Stack...'),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
        ),
        DeclareLaunchArgument(
            'imu_port',
            default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_IG14859004F004A-if00-port0'
        ),
        DeclareLaunchArgument('arduino_port', default_value=''),
        DeclareLaunchArgument('enable_imu', default_value='false'),
        DeclareLaunchArgument('enable_camera', default_value='false'),

        # rplidar_composition + Express scan mode is the most stable on this A1 in local tests.
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            parameters=[{
                'serial_port': lidar_port,
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'angle_compensate': True,
                'scan_mode': 'Express',
            }],
            respawn=True,
            respawn_delay=2.0,
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_footprint_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
            output='screen',
        ),

        Node(
            namespace='imu',
            package='lpms_ig1',
            executable='lpms_ig1_node',
            condition=IfCondition(enable_imu),
            output='screen',
            parameters=[{
                'port': imu_port,
                'baudrate': 115200
            }]
        ),
        Node(
            namespace='imu',
            package='lpms_ig1',
            executable='quat_to_euler_node',
            condition=IfCondition(enable_imu),
            output='screen',
        ),

        # Arduino: publishes /odom, /line_sensors, sensor/distance (ultrasonic cm), /gas_sensor (MQ-135 raw)
        Node(
            package='formica_experiments',
            executable='arduino_base',
            name='arduino_base_node',
            parameters=[{
                'port': arduino_port,
                'baud_rate': 115200
            }],
            output='screen'
        ),

        Node(
            package='jetson_camera_pub',
            executable='jetson_camera_node',
            name='jetson_camera_publisher',
            condition=IfCondition(enable_camera),
            parameters=[{
                'topic_name': '/rgb/image_raw',
                'gstreamer_pipeline': (
                    'v4l2src device=/dev/video0 ! '
                    'image/jpeg, width=1280, height=720, framerate=30/1 ! '
                    'jpegdec ! videoconvert ! '
                    'video/x-raw, format=BGR ! appsink drop=true'
                ),
            }],
            output='screen'
        ),

        Node(
            package='formica_experiments',
            executable='data_logger',
            name='data_logger',
            output='screen'
        ),

        LogInfo(msg="Hardware stack active. Run experiment with 'ros2 run formica_experiments expX_...'")
    ])
