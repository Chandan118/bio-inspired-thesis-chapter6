#!/usr/bin/env bash
# Experiment 7 — pheromone trail (TCRT5000 + MQ-135) full pipeline:
#   Optional: Arduino base for real /odom + sensors (skip with MOCK_ONLY=1)
#   exp7_pheromone (all sub-experiments A–D)
#   exp7_postprocess → figures + table in exp1_logs
#
# Quick dry run (no hardware, 1 trial each):
#   MOCK_ONLY=1 MOCK_TRIALS=1 bash /home/jetson/exp1_logs/run_exp7_full_pipeline.sh
#
# Full thesis protocol (10 trials each, interactive Enter prompts):
#   bash /home/jetson/exp1_logs/run_exp7_full_pipeline.sh
# No pipefail: avoids spurious failures when sort/head/python pipelines signal EPIPE
set -e

LOG="/home/jetson/exp1_logs"
DATA="/home/jetson/formica_experiments/data"
mkdir -p "$LOG" "$DATA"

# So Python print() (CSV logger, summary) interleaves correctly with rclpy logs on stderr
export PYTHONUNBUFFERED=1

set +u
source /opt/ros/humble/setup.bash
source /home/jetson/ros2_ws/install/setup.bash 2>/dev/null || source /home/jetson/formica_experiments/install/setup.bash
set -u

RUN_TAG="$(date +%Y%m%d_%H%M%S)"
MOCK_ONLY="${MOCK_ONLY:-0}"
MOCK_TRIALS="${MOCK_TRIALS:-10}"

cleanup() {
  [[ -n "${BASE_PID:-}" ]] && kill "$BASE_PID" 2>/dev/null || true
}
trap cleanup EXIT

ros2 daemon start 2>/dev/null || true

if [[ "$MOCK_ONLY" != "1" ]]; then
  # Match arduino_motor_control.py: CH341USB1 often wins when two USB serial devices exist
  if [[ -z "${ARDUINO_PORT:-}" ]]; then
    for p in /dev/ttyCH341USB1 /dev/ttyCH341USB0 /dev/ttyACM0 /dev/ttyUSB0; do
      if [[ -e "$p" ]]; then
        ARDUINO_PORT="$p"
        break
      fi
    done
  fi
  echo "[exp7] Starting arduino_base (port=${ARDUINO_PORT:-auto-scan}) ..."
  if [[ -n "${ARDUINO_PORT:-}" ]]; then
    nohup ros2 run formica_experiments arduino_base --ros-args -p "port:=${ARDUINO_PORT}" \
      >"$LOG/exp7_arduino_${RUN_TAG}.log" 2>&1 &
  else
    nohup ros2 run formica_experiments arduino_base \
      >"$LOG/exp7_arduino_${RUN_TAG}.log" 2>&1 &
  fi
  BASE_PID=$!
  sleep 3
  if ! timeout 4 ros2 topic echo /line_sensors --once &>/dev/null; then
    echo "[exp7] WARN: /line_sensors not seen yet — check USB, flash firmware with TCRT on A2–A5, see log $LOG/exp7_arduino_${RUN_TAG}.log"
  fi
fi

EXP_ARGS=(
  -p
  "mock_sensors:=false"
  -p
  "auto_run:=false"
  -p
  "num_straight_trials:=10"
  -p
  "num_curved_trials:=10"
  -p
  "num_snr_trials:=10"
)

if [[ "$MOCK_ONLY" == "1" ]]; then
  EXP_ARGS=(
    -p
    "mock_sensors:=true"
    -p
    "auto_run:=true"
    -p
    "num_straight_trials:=${MOCK_TRIALS}"
    -p
    "num_curved_trials:=${MOCK_TRIALS}"
    -p
    "num_snr_trials:=${MOCK_TRIALS}"
  )
fi

echo "[exp7] Running exp7_pheromone (log: $LOG/exp7_run_${RUN_TAG}.log) ..."
set +e
ros2 run formica_experiments exp7_pheromone --ros-args "${EXP_ARGS[@]}" \
  2>&1 | tee "$LOG/exp7_run_${RUN_TAG}.log"
RC=$?

# Newest exp7 CSV (nullglob: no ls error if empty; avoids set -e abort before postprocess)
CSV=""
shopt -s nullglob
_csv_list=("$DATA"/exp7_pheromone_*.csv)
shopt -u nullglob
if ((${#_csv_list[@]} > 0)); then
  CSV="$(printf '%s\n' "${_csv_list[@]}" | sort -r | head -1)"
fi

PP_RC=0
if [[ -n "$CSV" ]]; then
  echo "[exp7] Post-processing $CSV ..." | tee -a "$LOG/exp7_run_${RUN_TAG}.log"
  python3 /home/jetson/formica_experiments/formica_experiments/exp7_postprocess.py "$CSV" --out-dir "$LOG" \
    2>&1 | tee -a "$LOG/exp7_run_${RUN_TAG}.log"
  PP_RC=${PIPESTATUS[0]}
else
  echo "[exp7] WARN: no exp7 CSV found under $DATA" | tee -a "$LOG/exp7_run_${RUN_TAG}.log"
  PP_RC=1
fi
set -e

# ros2 often returns non-zero after rclpy.shutdown() even when the experiment finished OK
if [[ -z "$CSV" ]]; then
  FINAL="$RC"
elif [[ "$PP_RC" -ne 0 ]]; then
  FINAL="$PP_RC"
else
  FINAL=0
fi

echo "[exp7] Done run_tag=$RUN_TAG exp7_exit=$RC postprocess_exit=$PP_RC final_exit=$FINAL" | tee -a "$LOG/exp7_run_${RUN_TAG}.log"
exit "$FINAL"
