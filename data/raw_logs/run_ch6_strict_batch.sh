#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /home/jetson/ros2_ws/install/setup.bash
set -u

AUTO_YES="${AUTO_YES:-1}"
QUICK="${QUICK:-1}"
RUN_EXP3="${RUN_EXP3:-0}"
RUN_EXP5="${RUN_EXP5:-0}"
RUN_EXP6="${RUN_EXP6:-0}"
RUN_EXP7="${RUN_EXP7:-1}"

echo "[ch6] Starting strict batch helper."
echo "[ch6] This script will pause for manual experiments/actions."

run_step() {
  echo ""
  echo "============================================================"
  echo "[ch6] $1"
  echo "============================================================"
}

ask_continue() {
  if [[ "$AUTO_YES" == "1" ]]; then
    return 0
  fi
  read -r -p "[ch6] Continue? (y/N): " ans
  [[ "${ans:-N}" == "y" || "${ans:-N}" == "Y" ]]
}

run_step "Exp1 sensor calibration"
python3 /home/jetson/exp1_logs/run_exp1_hardware_integration.py || true
echo "[ch6] If RGB-D reprojection was not entered, rerun Exp1 with --rgbd-reprojection <px>."

run_step "Exp2 power profiling quick test (120 s)"
if [[ "$QUICK" == "1" ]]; then
  MISSION_DURATION_S=120 bash /home/jetson/exp1_logs/run_exp2_full_pipeline.sh || true
else
  bash /home/jetson/exp1_logs/run_exp2_full_pipeline.sh || true
fi

run_step "Exp3 strict SLAM mapping"
if [[ "$RUN_EXP3" == "1" ]] && ask_continue; then
  bash /home/jetson/exp1_logs/run_exp3_full_strict.sh || true
else
  echo "[ch6] Skipping Exp3 (set RUN_EXP3=1 to enable)."
fi

run_step "Exp4 maze navigation full protocol"
if ask_continue; then
  if [[ "$QUICK" == "1" ]]; then
    EXP4_NUM_TRIALS=3 EXP4_TRIAL_TIMEOUT_S=120 \
      bash /home/jetson/exp1_logs/run_exp4_stack.sh --with-trials || true
  else
    EXP4_NUM_TRIALS=20 EXP4_TRIAL_TIMEOUT_S=120 \
      bash /home/jetson/exp1_logs/run_exp4_stack.sh --with-trials || true
  fi
fi

run_step "Exp5 dynamic obstacle/fault (manual interaction required)"
if [[ "$RUN_EXP5" == "1" ]] && ask_continue; then
  ros2 run formica_experiments exp5_fault || true
else
  echo "[ch6] Skipping Exp5 (set RUN_EXP5=1 to enable)."
fi

run_step "Exp6 CNN detection (manual object/lighting setup required)"
if [[ "$RUN_EXP6" == "1" ]] && ask_continue; then
  echo "[ch6] Auto-starting camera for Exp6..."
  ros2 run formica_experiments jetson_camera --ros-args -p topic_name:=/rgb/image_raw >/tmp/ch6_exp6_camera.log 2>&1 &
  CAM_PID=$!
  ros2 run formica_experiments exp6_cnn || true
  kill "$CAM_PID" 2>/dev/null || true
else
  echo "[ch6] Skipping Exp6 (set RUN_EXP6=1 to enable)."
fi

run_step "Exp7 pheromone full pipeline"
if [[ "$RUN_EXP7" == "1" ]] && ask_continue; then
  if [[ "$QUICK" == "1" ]]; then
    MOCK_ONLY=1 MOCK_TRIALS=1 bash /home/jetson/exp1_logs/run_exp7_full_pipeline.sh || true
  else
    bash /home/jetson/exp1_logs/run_exp7_full_pipeline.sh || true
  fi
else
  echo "[ch6] Skipping Exp7 (set RUN_EXP7=1 to enable)."
fi

run_step "Rebuild Chapter 6 deliverables"
python3 /home/jetson/chapter6_formica_deliverables/build_chapter6_from_logs.py

echo ""
echo "[ch6] Done. Check:"
echo "  /home/jetson/chapter6_formica_deliverables/meta/PROVENANCE.md"
echo "  /home/jetson/chapter6_formica_deliverables/tables/"
echo "  /home/jetson/chapter6_formica_deliverables/figures/"

