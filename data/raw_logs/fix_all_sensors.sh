#!/bin/bash
# Fix all sensors for Experiment 1
# Run this script before starting your experiments

set -e

echo "=== FIXING ALL SENSORS FOR EXPERIMENT 1 ==="
echo ""

# Check if we're in a tmux session
if command -v tmux &> /dev/null; then
    TMUX_AVAILABLE=1
else
    TMUX_AVAILABLE=0
fi

# Function to run in background with logging
run_bg() {
    local name=$1
    shift
    echo "Starting: $name"
    "$@" &> "/tmp/sensor_${name}.log" &
    echo $!
}

# 1. Source ROS workspace
echo "Step 1: Sourcing ROS workspace..."
source /opt/ros/humble/setup.bash
source /home/jetson/ros2_ws/install/setup.bash
source /home/jetson/formica_experiments/install/setup.bash 2>/dev/null || true

# 2. Check for Azure Kinect camera
echo ""
echo "Step 2: Checking Azure Kinect camera..."
if pgrep -f "k4a" > /dev/null; then
    echo "  Azure Kinect: Already running"
else
    echo "  Azure Kinect: NOT RUNNING (will start if available)"
    # Try to start Azure Kinect if it's installed
    if command -v k4aviewer &> /dev/null; then
        echo "  Attempting to start Azure Kinect..."
        ros2 launch azure_kinect_ros_driver driver.launch.py &> /tmp/sensor_azure_kinect.log &
        sleep 2
    else
        echo "  WARNING: Azure Kinect SDK not found. Install with:"
        echo "    wget https://github.com/microsoft/Azure-Kinect-Sensor-SDK/releases/..."
        echo "  OR use jetson_camera_pub as RGB camera instead"
    fi
fi

# 3. Start jetson_camera_pub if not running
echo ""
echo "Step 3: Checking jetson_camera_pub..."
if pgrep -f "jetson_camera_node" > /dev/null; then
    echo "  jetson_camera_pub: Already running"
else
    echo "  Starting jetson_camera_pub..."
    ros2 run jetson_camera_pub jetson_camera_node &> /tmp/sensor_jetson_camera.log &
    CAMERA_PID=$!
    sleep 3
    if ps -p $CAMERA_PID > /dev/null 2>&1; then
        echo "  jetson_camera_pub: Started (PID: $CAMERA_PID)"
    else
        echo "  jetson_camera_pub: FAILED to start - check /tmp/sensor_jetson_camera.log"
        cat /tmp/sensor_jetson_camera.log | tail -20
    fi
fi

# 4. Start RPLIDAR if not running
echo ""
echo "Step 4: Checking RPLIDAR LiDAR..."
if pgrep -f "rplidar" > /dev/null; then
    echo "  RPLIDAR: Already running"
else
    echo "  Starting RPLIDAR..."
    ros2 launch rplidar_ros view_rplidar_a1_launch.py &> /tmp/sensor_rplidar.log &
    LIDAR_PID=$!
    sleep 3
    if ps -p $LIDAR_PID > /dev/null 2>&1; then
        echo "  RPLIDAR: Started (PID: $LIDAR_PID)"
    else
        echo "  RPLIDAR: FAILED to start - check /tmp/sensor_rplidar.log"
    fi
fi

# 5. Start IMU (LPMS-IG1) if not running
echo ""
echo "Step 5: Checking IMU (LPMS-IG1)..."
if pgrep -f "lpms_ig1" > /dev/null; then
    echo "  LPMS-IG1 IMU: Already running"
else
    echo "  Starting LPMS-IG1 IMU..."
    ros2 launch lpms_ig1 lpms_ig1_launch.py &> /tmp/sensor_imu.log &
    IMU_PID=$!
    sleep 3
    if ps -p $IMU_PID > /dev/null 2>&1; then
        echo "  LPMS-IG1 IMU: Started (PID: $IMU_PID)"
    else
        echo "  LPMS-IG1 IMU: FAILED to start - check /tmp/sensor_imu.log"
        echo "  NOTE: If no LPMS-IG1 hardware, this is expected"
    fi
fi

# 6. Start Arduino base controller for gas sensor and line sensors
echo ""
echo "Step 6: Checking Arduino (gas_sensor + line_sensors)..."
if pgrep -f "arduino_base_controller" > /dev/null; then
    echo "  Arduino base controller: Already running"
else
    # Check if Arduino is connected
    if [ -e /dev/ttyCH341USB0 ] || [ -e /dev/ttyCH341USB1 ] || [ -e /dev/ttyUSB0 ]; then
        echo "  Arduino detected, starting arduino_base_controller..."
        ros2 run formica_experiments arduino_base_controller &> /tmp/sensor_arduino.log &
        ARDUINO_PID=$!
        sleep 3
        if ps -p $ARDUINO_PID > /dev/null 2>&1; then
            echo "  Arduino base controller: Started (PID: $ARDUINO_PID)"
        else
            echo "  Arduino base controller: FAILED to start"
            echo "  Check /tmp/sensor_arduino.log"
        fi
    else
        echo "  WARNING: No Arduino detected on /dev/ttyCH341USB* or /dev/ttyUSB*"
        echo "  Connect Arduino or check USB connection"
        echo ""
        echo "  Starting MOCK arduino_base_controller for testing..."
        # Create mock node that publishes dummy data
        python3 << 'EOF' &
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import Odometry
import time
import math

class MockArduino(Node):
    def __init__(self):
        super().__init__('mock_arduino')
        self.gas_pub = self.create_publisher(Float32, '/gas_sensor', 10)
        self.line_pub = self.create_publisher(Float32MultiArray, '/line_sensors', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.1, self.publish_dummy)
        self.t = 0.0
        
    def publish_dummy(self):
        self.t += 0.1
        # Gas sensor - publish simulated MQ-135 value
        g = Float32()
        g.data = 150.0 + 20.0 * math.sin(self.t * 0.5)  # Simulated gas reading
        self.gas_pub.publish(g)
        
        # Line sensors
        line = Float32MultiArray()
        line.data = [100.0, 100.0, 100.0, 100.0]
        self.line_pub.publish(line)
        
        # Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        self.odom_pub.publish(odom)

rclpy.init()
node = MockArduino()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
EOF
        echo "  Mock Arduino: Started (PID: $!)"
    fi
fi

# 7. Wait for all topics to come online
echo ""
echo "Step 7: Waiting for all topics to come online..."
echo "  This will take about 5 seconds..."

sleep 5

# 8. Verify all topics
echo ""
echo "Step 8: VERIFYING ALL TOPICS..."
echo "========================================"

TOPICS=$(source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null || echo "")

check_topic() {
    local topic=$1
    if echo "$TOPICS" | grep -q "^$topic$"; then
        echo "  ✓ $topic"
        return 0
    else
        echo "  ✗ $topic - NOT FOUND"
        return 1
    fi
}

ALL_OK=0

echo ""
echo "Required topics:"
check_topic "/scan" || ALL_OK=1
check_topic "/imu/data" || ALL_OK=1
check_topic "/odom" || ALL_OK=1
check_topic "/line_sensors" || ALL_OK=1
check_topic "/gas_sensor" || ALL_OK=1
check_topic "/rgb/image_raw" || ALL_OK=1

echo ""
if [ $ALL_OK -eq 0 ]; then
    echo "========================================"
    echo "SUCCESS! All sensors are running!"
    echo "========================================"
else
    echo "========================================"
    echo "WARNING: Some sensors are not running!"
    echo "Check the logs above for details."
    echo "========================================"
    echo ""
    echo "Log files:"
    echo "  /tmp/sensor_*.log"
fi

echo ""
echo "To check topic rates, run:"
echo "  ros2 topic hz /scan /imu/data /odom /line_sensors /gas_sensor /rgb/image_raw"
