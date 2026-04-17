#!/usr/bin/env bash
# ROS2 + Jetson camera + AI-style detection (exp6 TRT/HSV) + sensor fusion +
# plain-language /robot_scene_description. Optionally drives the robot.
#
# Usage:
#   # Description only (safe — no cmd_vel):
#   bash /home/jetson/exp1_logs/run_vision_sensor_ai_demo.sh
#
#   # With Arduino sensors + slow motion (careful — real motors):
#   ENABLE_CMD_VEL=1 ARDUINO_PORT=/dev/ttyCH341USB1 \
#     bash /home/jetson/exp1_logs/run_vision_sensor_ai_demo.sh
#
set -e
export PYTHONUNBUFFERED=1

source /opt/ros/humble/setup.bash
source /home/jetson/ros2_ws/install/setup.bash

ros2 daemon start 2>/dev/null || true

IMAGE_TOPIC="${IMAGE_TOPIC:-/camera/image_raw}"
# exp6 = coloured blobs / TensorRT thesis model; yolo = Ultralytics YOLOv8 (COCO)
VISION_BACKEND="${VISION_BACKEND:-yolo}"
YOLO_MODEL="${YOLO_MODEL:-yolov8n.pt}"
# auto | wander | approach — wander avoids COCO obstacles in camera (best for YOLO walk)
NAV_MODE="${NAV_MODE:-auto}"
ENABLE_CMD_VEL="${ENABLE_CMD_VEL:-0}"
RUN_TAG="$(date +%Y%m%d_%H%M%S)"
LOG="/home/jetson/exp1_logs"
mkdir -p "$LOG"

cleanup() {
  [[ -n "${CAM_PID:-}" ]] && kill "$CAM_PID" 2>/dev/null || true
  [[ -n "${BASE_PID:-}" ]] && kill "$BASE_PID" 2>/dev/null || true
}
trap cleanup EXIT

if [[ "$ENABLE_CMD_VEL" == "1" ]]; then
  if [[ -z "${ARDUINO_PORT:-}" ]]; then
    for p in /dev/ttyCH341USB1 /dev/ttyCH341USB0 /dev/ttyACM0 /dev/ttyUSB0; do
      [[ -e "$p" ]] && ARDUINO_PORT="$p" && break
    done
  fi
  if [[ -n "${ARDUINO_PORT:-}" ]]; then
    echo "[demo] arduino_base on ${ARDUINO_PORT}"
    nohup ros2 run formica_experiments arduino_base --ros-args -p "port:=${ARDUINO_PORT}" \
      >"$LOG/vision_demo_arduino_${RUN_TAG}.log" 2>&1 &
    BASE_PID=$!
    sleep 2
  else
    echo "[demo] WARN: no serial port; line/ultrasonic will be missing."
  fi
fi

echo "[demo] jetson_camera → ${IMAGE_TOPIC}"
nohup ros2 run formica_experiments jetson_camera --ros-args \
  -p "topic_name:=${IMAGE_TOPIC}" \
  >"$LOG/vision_demo_camera_${RUN_TAG}.log" 2>&1 &
CAM_PID=$!
sleep 3

CMD_ARGS=(
  ros2 run formica_experiments jetson_vision_guide --ros-args
  -p "image_topic:=${IMAGE_TOPIC}"
  -p "vision_backend:=${VISION_BACKEND}"
  -p "yolo_model:=${YOLO_MODEL}"
  -p "nav_mode:=${NAV_MODE}"
  -p "enable_cmd_vel:=false"
)
if [[ "$ENABLE_CMD_VEL" == "1" ]]; then
  CMD_ARGS=(
    ros2 run formica_experiments jetson_vision_guide --ros-args
    -p "image_topic:=${IMAGE_TOPIC}"
    -p "vision_backend:=${VISION_BACKEND}"
    -p "yolo_model:=${YOLO_MODEL}"
    -p "nav_mode:=${NAV_MODE}"
    -p "enable_cmd_vel:=true"
    -p "forward_speed:=${FORWARD_SPEED:-0.1}"
  )
fi

echo "[demo] jetson_vision_guide (Ctrl+C to stop) — echo /robot_scene_description in another terminal:"
echo "  source /opt/ros/humble/setup.bash && source /home/jetson/ros2_ws/install/setup.bash && ros2 topic echo /robot_scene_description"
"${CMD_ARGS[@]}"
