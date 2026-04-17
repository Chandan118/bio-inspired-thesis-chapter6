#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/ros/humble/setup.bash
source /home/jetson/ros2_ws/install/setup.bash
set -u

# Match interactive shells on this robot (see Desktop/info / bashrc); override with export ROS_DOMAIN_ID=...
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"

OUT_DIR="/home/jetson/exp1_logs"
mkdir -p "$OUT_DIR"
RUN_TAG="$(date +%Y%m%d_%H%M%S)"
STATUS_FILE="$OUT_DIR/exp3_status_${RUN_TAG}.txt"
LOG_FILE="$OUT_DIR/exp3_run_${RUN_TAG}.log"

echo "status,starting" > "$STATUS_FILE"
echo "started_at,$(date --iso-8601=seconds)" >> "$STATUS_FILE"

# Use stable LiDAR path and kill stale lidar nodes.
pkill -f "rplidar_node" >/dev/null 2>&1 || true
pkill -f "ros2 launch rplidar_ros" >/dev/null 2>&1 || true

# Same TF chain as formica_experiments/launch/bringup_launch.py — without this,
# scan frame "laser" is not reachable from slam_toolbox base_frame "base_footprint",
# so /map stays all unknown and coverage stays 0%.
ros2 run tf2_ros static_transform_publisher \
  --frame-id base_link --child-frame-id laser \
  > "$OUT_DIR/exp3_tf_base_laser_${RUN_TAG}.log" 2>&1 &
TF_BASE_LASER_PID=$!
ros2 run tf2_ros static_transform_publisher \
  --frame-id base_link --child-frame-id base_footprint \
  > "$OUT_DIR/exp3_tf_base_foot_${RUN_TAG}.log" 2>&1 &
TF_BASE_FOOT_PID=$!

ros2 launch rplidar_ros rplidar_a1_launch.py \
  serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  > "$OUT_DIR/exp3_lidar_${RUN_TAG}.log" 2>&1 &
LIDAR_PID=$!

ros2 launch slam_toolbox online_async_launch.py \
  > "$OUT_DIR/exp3_slamtb_${RUN_TAG}.log" 2>&1 &
SLAM_PID=$!

ros2 launch nav2_bringup navigation_launch.py \
  > "$OUT_DIR/exp3_nav2_${RUN_TAG}.log" 2>&1 &
NAV2_PID=$!

ros2 run formica_experiments arduino_base \
  --ros-args -p port:=/dev/ttyCH341USB0 -p baud_rate:=115200 -p linear_deadband:=0.02 \
  > "$OUT_DIR/exp3_base_${RUN_TAG}.log" 2>&1 &
BASE_PID=$!

echo "tf_base_laser_pid,$TF_BASE_LASER_PID" >> "$STATUS_FILE"
echo "tf_base_foot_pid,$TF_BASE_FOOT_PID" >> "$STATUS_FILE"
echo "lidar_pid,$LIDAR_PID" >> "$STATUS_FILE"
echo "slam_pid,$SLAM_PID" >> "$STATUS_FILE"
echo "nav2_pid,$NAV2_PID" >> "$STATUS_FILE"
echo "base_pid,$BASE_PID" >> "$STATUS_FILE"

# Allow stack startup.
sleep 8

# Sanity: /map should be visible.
TOPICS_SNAPSHOT="/tmp/exp3_topics_${RUN_TAG}.txt"
ros2 topic list > "$TOPICS_SNAPSHOT" 2>/dev/null || true
if ! python3 - <<PY
from pathlib import Path
txt = Path("$TOPICS_SNAPSHOT").read_text() if Path("$TOPICS_SNAPSHOT").exists() else ""
need = {"/map", "/scan", "/odom"}
raise SystemExit(0 if need.issubset(set(txt.split())) else 1)
PY
then
  echo "status,failed_no_map_topic" >> "$STATUS_FILE"
  kill "$TF_BASE_LASER_PID" "$TF_BASE_FOOT_PID" \
    "$LIDAR_PID" "$SLAM_PID" "$NAV2_PID" "$BASE_PID" >/dev/null 2>&1 || true
  exit 1
fi

echo "status,running_exp3" >> "$STATUS_FILE"

# Strict run: 10 trials configured in exp3_slam_mapping.py.
# 36% linear speed cap + obstacle-safe behavior are in exp3_slam_mapping.py.
export PYTHONUNBUFFERED=1
PYTHONPATH="/home/jetson/formica_experiments/install/formica_experiments/lib/python3.10/site-packages:$PYTHONPATH" \
EXP3_FORCE_CMDVEL_AUTONOMY=1 \
EXP3_AUTO_START=1 \
python3 -u - <<'PY' > "$LOG_FILE" 2>&1
import formica_experiments.exp3_slam_mapping as m
m.main()
PY

echo "status,saving_map" >> "$STATUS_FILE"
if ros2 run nav2_map_server map_saver_cli -f /home/jetson/formica_map >/dev/null 2>&1; then
  echo "map_saved,/home/jetson/formica_map" >> "$STATUS_FILE"
else
  echo "map_saved,failed" >> "$STATUS_FILE"
fi

echo "status,completed" >> "$STATUS_FILE"
echo "finished_at,$(date --iso-8601=seconds)" >> "$STATUS_FILE"

kill "$LIDAR_PID" "$SLAM_PID" "$NAV2_PID" >/dev/null 2>&1 || true
kill "$BASE_PID" >/dev/null 2>&1 || true
kill "$TF_BASE_LASER_PID" "$TF_BASE_FOOT_PID" >/dev/null 2>&1 || true
