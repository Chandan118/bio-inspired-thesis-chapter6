#!/usr/bin/env bash
# Experiment 2 — full pipeline (thesis protocol):
#   Task 1: I²C 0x40 + /power_monitor @ ~10 Hz
#   Task 2–3: exp2_power cycles TRANSIT(10.2s)→DECISION(5.1s)→STANDBY(1.7s), 1 Hz CSV
#   Task 4: Table 6.2 + Figure 6.3 + exp2_summary (also emitted by exp2_power at end)
#
# Usage:
#   MISSION_DURATION_S=120 bash /home/jetson/exp1_logs/run_exp2_full_pipeline.sh   # 2 min test
#   bash /home/jetson/exp1_logs/run_exp2_full_pipeline.sh                         # 6 h default
#   NO_ARDUINO=1 ...   # skip base (stationary power only)
#   NO_LIDAR=1 ...     # skip LiDAR (no obstacle avoidance during TRANSIT)
set -eo pipefail

LOG="/home/jetson/exp1_logs"
DATA="/home/jetson/formica_experiments/data"
mkdir -p "$LOG" "$DATA"

set +u
source /opt/ros/humble/setup.bash
source /home/jetson/ros2_ws/install/setup.bash
set -u

RUN_TAG="$(date +%Y%m%d_%H%M%S)"
STATUS="$LOG/exp2_hw_check_${RUN_TAG}.txt"
MISSION_DURATION_S="${MISSION_DURATION_S:-21600}"

{
  echo "run_tag,$RUN_TAG"
  echo "mission_duration_s,$MISSION_DURATION_S"
  date --iso-8601=seconds
} >"$STATUS"

echo "[exp2] === Task 1: I²C bus 1 (expect 40 = INA219) ==="
if command -v i2cdetect >/dev/null 2>&1; then
  i2cdetect -y 1 2>&1 | tee -a "$STATUS" || true
  # Row label "40:" alone is not a device; require a cell value (e.g. "40" or "UU").
  if i2cdetect -y 1 2>/dev/null | grep -qE '^40:[^:]*(\s[0-9a-f]{2}|\sUU)'; then
    echo "[exp2] PASS: I²C device at 0x40 (confirm INA219)." | tee -a "$STATUS"
  elif command -v i2cget >/dev/null 2>&1 && i2cget -y 1 0x40 0x00 &>/dev/null; then
    echo "[exp2] PASS: i2cget read OK at 0x40." | tee -a "$STATUS"
  else
    echo "[exp2] WARN: no device at 0x40 — check wiring; Jetson may still use sysfs hwmon." | tee -a "$STATUS"
  fi
else
  echo "[exp2] WARN: i2cdetect not installed (apt install i2c-tools)." | tee -a "$STATUS"
fi

ros2 daemon start 2>/dev/null || true

cleanup() {
  [[ -n "${INA_PID:-}" ]] && kill "$INA_PID" 2>/dev/null || true
  [[ -n "${BASE_PID:-}" ]] && kill "$BASE_PID" 2>/dev/null || true
  [[ -n "${LIDAR_PID:-}" ]] && kill "$LIDAR_PID" 2>/dev/null || true
}
trap cleanup EXIT

echo "[exp2] Starting INA219 publisher (ros2 run formica_experiments ina219_power_monitor) ..."
nohup ros2 run formica_experiments ina219_power_monitor \
  >"$LOG/exp2_ina219_${RUN_TAG}.log" 2>&1 &
INA_PID=$!

for i in $(seq 1 25); do
  if timeout 2 ros2 topic list 2>/dev/null | grep -qx '/power_monitor'; then
    break
  fi
  sleep 1
done
timeout 6 ros2 topic list 2>/dev/null | grep -qx '/power_monitor' || {
  echo "[exp2] ERROR: /power_monitor not found. See $LOG/exp2_ina219_${RUN_TAG}.log"
  exit 1
}

echo "[exp2] Checking /power_monitor rate (expect ≥ 8 Hz) ..."
HZ_OUT="$(timeout 12 ros2 topic hz /power_monitor 2>&1 || true)"
echo "$HZ_OUT" | tee -a "$STATUS"
if echo "$HZ_OUT" | grep -qE 'average rate: [89]\.|average rate: [1-9][0-9]'; then
  echo "[exp2] PASS: topic rate looks like 10 Hz class." | tee -a "$STATUS"
else
  echo "[exp2] WARN: could not confirm 10 Hz from hz output — continue if samples arrive." | tee -a "$STATUS"
fi

if [[ "${NO_ARDUINO:-0}" != "1" ]]; then
  echo "[exp2] Starting arduino_base (cmd_vel for TRANSIT) ..."
  nohup ros2 run formica_experiments arduino_base --ros-args \
    -p port:=/dev/ttyCH341USB0 -p baud_rate:=115200 -p linear_deadband:=0.02 \
    >"$LOG/exp2_base_${RUN_TAG}.log" 2>&1 &
  BASE_PID=$!
fi

if [[ "${NO_LIDAR:-0}" != "1" ]]; then
  LIDAR_DEV="${LIDAR_DEV:-/dev/ttyUSB0}"
  echo "[exp2] Starting LiDAR on $LIDAR_DEV (optional obstacle stop) ..."
  nohup ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:="$LIDAR_DEV" \
    >"$LOG/exp2_lidar_${RUN_TAG}.log" 2>&1 &
  LIDAR_PID=$!
  sleep 5
fi

echo "[exp2] Recording start-of-run bus snapshot (first /power_monitor echo) ..."
timeout 4 ros2 topic echo /power_monitor --once 2>&1 | tee -a "$STATUS" || true

echo "[exp2] === Task 2–3: exp2_power (MISSION_DURATION_S=$MISSION_DURATION_S) ==="
export MISSION_DURATION_S
export EXP2_ALLOW_SYNTHETIC="${EXP2_ALLOW_SYNTHETIC:-0}"
ros2 run formica_experiments exp2_power 2>&1 | tee "$LOG/exp2_power_run_${RUN_TAG}.log"

echo "[exp2] Done. Latest artifacts under $LOG and $DATA"
echo "[exp2]   table6_2_power_profile.csv  Figure6_3_Power_Profile.png  exp2_summary.txt"
