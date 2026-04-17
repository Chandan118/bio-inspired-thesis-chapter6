#!/bin/bash
# Startup script for all Experiment 1 sensors
# Run this before starting any experiment

echo "=========================================="
echo "Starting all sensors for Experiment 1..."
echo "=========================================="

# Source ROS workspaces
source /opt/ros/humble/setup.bash
source /home/jetson/ros2_ws/install/setup.bash
source /home/jetson/formica_experiments/install/setup.bash 2>/dev/null || true

# Function to start a node in background
start_node() {
    local name=$1
    shift
    echo "Starting $name..."
    "$@" &>/tmp/${name}.log &
    echo "  PID: $!"
    sleep 1
}

# 1. Start RPLIDAR LiDAR
start_node "rplidar" ros2 launch rplidar_ros rplidar_a1_launch.py

# 2. Start IMU (LPMS-IG1)
start_node "lpms_imu" ros2 launch lpms_ig1 lpms_ig1_launch.py

# 3. Start Arduino base controller (gas sensor + line sensors)
start_node "arduino_base" ros2 run formica_experiments arduino_base

# 4. Start RGB camera
start_node "jetson_camera" ros2 run jetson_camera_pub jetson_camera_node

# Wait for topics to come online
echo ""
echo "Waiting for topics to come online..."
sleep 5

# Show all topics
echo ""
echo "=========================================="
echo "Current ROS topics:"
echo "=========================================="
ros2 topic list

echo ""
echo "=========================================="
echo "Topic verification:"
echo "=========================================="
for topic in /scan /imu/data /odom /line_sensors /gas_sensor /rgb/image_raw; do
    if ros2 topic list | grep -q "^$topic$"; then
        echo "  ✓ $topic"
    else
        echo "  ✗ $topic - NOT FOUND"
    fi
done

echo ""
echo "=========================================="
echo "Sensor startup complete!"
echo "=========================================="
echo ""
echo "Log files are in /tmp/*.log"
echo "To stop all sensors: pkill -f 'rplidar|lpms|arduino_base|jetson_camera'"
