#!/usr/bin/env bash
# YOLOv8 + camera + ultrasonic/line (arduino_base) — slow autonomous forward motion with
# obstacle avoidance in the image (COCO person, chair, vehicle, …) + stop if ultrasonic
# is below stop_distance_cm.
#
# WARNING: Runs real motors. Use clear floor, low speed, and stay ready to lift the robot
# or pull USB power. Do not run while arduino_motor_control.py holds the same serial port.
#
# Usage:
#   ARDUINO_PORT=/dev/ttyCH341USB1 bash /home/jetson/exp1_logs/run_yolo_autonomous_walk.sh
#
# Tune:
#   FORWARD_SPEED=0.08 YOLO_MODEL=yolov8s.pt bash .../run_yolo_autonomous_walk.sh
#
set -e
export PYTHONUNBUFFERED=1

echo "================================================================"
echo "  YOLO AUTONOMOUS WALK — motors will move. Ctrl+C stops the node"
echo "  (emergency: disconnect USB / kill arduino power)."
echo "================================================================"

export VISION_BACKEND=yolo
export NAV_MODE=wander
export ENABLE_CMD_VEL=1
export FORWARD_SPEED="${FORWARD_SPEED:-0.1}"

exec bash /home/jetson/exp1_logs/run_vision_sensor_ai_demo.sh
