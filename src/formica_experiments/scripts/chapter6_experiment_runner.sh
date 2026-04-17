#!/usr/bin/env bash
# Chapter 6 — Experiments 1–7 (FormicaBot)
#
# Thesis text may say ROS 2 Foxy; this Jetson uses ROS 2 Humble — same ros2 CLI patterns.
#
# What this script does: source workspaces, set ROS_DOMAIN_ID / DDS hints, run hardware check,
# launch bringup variants, or exec the same ros2 run … commands as the detail sheet.
#
# What it cannot do: physical setup (tape measure, ArUco, LED strip, cardboard box, INA219 wiring,
# pressing Enter between trials, 6-hour unattended power run). Use exp7 mock mode only for pipeline tests.
#
# Usage:
#   ./scripts/chapter6_experiment_runner.sh help
#   ./scripts/chapter6_experiment_runner.sh check
#   ./scripts/chapter6_experiment_runner.sh bringup          # lidar + Arduino (default launch args)
#   ./scripts/chapter6_experiment_runner.sh bringup-full       # + IMU + camera (Experiment 1 topics)
#   ./scripts/chapter6_experiment_runner.sh exp1 … exp7
#   ./scripts/chapter6_experiment_runner.sh exp2-quick       # 2 min power loop (not thesis claim)
#   ./scripts/chapter6_experiment_runner.sh exp3-stack        # lidar+SLAM+Nav2+base (see exp1_logs script)
#   ./scripts/chapter6_experiment_runner.sh exp4-diagnose | exp4-stack | exp4-trials
#   ./scripts/chapter6_experiment_runner.sh mapping          # remap + chapter 6 figures (SAVE_B=1)
#
set -eo pipefail

FORMICA_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS2_OVERLAY="${ROS2_OVERLAY:-/home/jetson/ros2_ws/install/setup.bash}"

_source_ros() {
  set +u
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
  # shellcheck source=/dev/null
  source "${FORMICA_WS}/install/setup.bash"
  if [[ -f "$ROS2_OVERLAY" ]]; then
    # shellcheck source=/dev/null
    source "$ROS2_OVERLAY"
  fi
  set -euo pipefail
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
  export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"
  export RCUTILS_LOGGING_BUFFERED_STREAM="${RCUTILS_LOGGING_BUFFERED_STREAM:-0}"
  if [[ -f "$HOME/ros2_ws/config/cyclonedds.xml" ]]; then
    export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$HOME/ros2_ws/config/cyclonedds.xml}"
  fi
}

_export_ports() {
  eval "$("${FORMICA_WS}/scripts/detect_formica_ports.py" --export-bash 2>/dev/null)" || true
}

_help() {
  cat <<'EOF'
Chapter 6 experiment runner — maps thesis tasks to commands on this robot.

Physical prerequisites (you): arena 4×4 m, ArUco positions in exp3 code, map for exp4/5,
INA219 @ 0x40 for exp2 claims, LED strip + ethanol protocol for exp7, checkerboard for RGB cal (exp1).

Commands:
  help              This text.
  check             USB / port quick check (hardware_checker.py).
  bringup           ros2 launch formica_experiments bringup_launch.py (lidar + Arduino).
  bringup-full      + camera; IMU only if FORMICA_IMU_PORT is set and ≠ Arduino (same USB as IMU breaks motors).
  exp1              ros2 run formica_experiments exp1_calibration  (run AFTER bringup-full).
  exp2              Full exp2_power (MISSION_DURATION_S=21600 unless you export shorter).
  exp2-quick        MISSION_DURATION_S=120 EXP2_ALLOW_SYNTHETIC=0 — short loop; still needs /power_monitor for real W.
  exp3-stack        bash exp1_logs/run_exp3_full_strict.sh  (SLAM+Nav2+lidar+base; then run exp3 in another terminal).
  exp3              ros2 run formica_experiments exp3_slam  (expects stack already up).
  exp4-diagnose     bash exp1_logs/run_exp4_stack.sh --diagnose
  exp4-stack        bash exp1_logs/run_exp4_stack.sh
  exp4-trials       bash exp1_logs/run_exp4_stack.sh --with-trials
  exp5              ros2 run formica_experiments exp5_fault  (Nav2 + map; align START/TARGET with map).
  exp6              ros2 run formica_experiments exp6_cnn  (needs /rgb/image_raw + optional TensorRT engine).
  exp7              Hardware pheromone trials (interactive / Enter as designed in node).
  exp7-demo         exp7 with mock_sensors + auto_run (software-only pipeline check).
  mapping           WAIT_SLAM_SEC=60 SAVE_B=1 run_mapping_then_chapter6.sh
  list              Print ros2 run lines only (no exec).

Environment (examples):
  WAIT_SLAM_SEC=90 SAVE_B=1  mapping
  MISSION_DURATION_S=7200    exp2
  EXP3_AUTO_START=0          exp3  (more operator control)
  EXP4_NUM_TRIALS=5          exp4-trials

EOF
}

_cmd_check() {
  python3 "${FORMICA_WS}/hardware_checker.py" --quick || true
}

_cmd_bringup() {
  local full="${1:-0}"
  _export_ports
  if [[ -z "${LIDAR_PORT:-}" || -z "${ARDUINO_PORT:-}" ]]; then
    echo "Could not detect LIDAR_PORT/ARDUINO_PORT. Fix USB or set ARDUINO_PORT / detect script." >&2
    exit 1
  fi
  if [[ "$full" == "1" ]]; then
    # CRITICAL: default imu_port in bringup_launch equals the Arduino CP2102. Two nodes cannot
    # open the same serial device — motors will not move. Only enable IMU on a *different* port.
    local imu_enable=false
    local imu_extra=()
    if [[ -n "${FORMICA_IMU_PORT:-}" && "${FORMICA_IMU_PORT}" != "${ARDUINO_PORT}" ]]; then
      imu_enable=true
      imu_extra=(imu_port:="${FORMICA_IMU_PORT}")
      echo "[bringup-full] IMU on separate serial: ${FORMICA_IMU_PORT}"
    else
      echo "[bringup-full] enable_imu=false (Arduino uses ${ARDUINO_PORT})." >&2
      echo "  For Exp 1 /imu/data on a USB IMU, export FORMICA_IMU_PORT=/dev/serial/by-id/... (not the Arduino)." >&2
    fi
    exec ros2 launch formica_experiments bringup_launch.py \
      lidar_port:="${LIDAR_PORT}" \
      arduino_port:="${ARDUINO_PORT}" \
      enable_imu:="${imu_enable}" \
      enable_camera:=true \
      "${imu_extra[@]}"
  fi
  exec ros2 launch formica_experiments bringup_launch.py \
    lidar_port:="${LIDAR_PORT}" \
    arduino_port:="${ARDUINO_PORT}"
}

_cmd_exp3_stack() {
  exec bash /home/jetson/exp1_logs/run_exp3_full_strict.sh
}

_cmd_mapping() {
  export WAIT_SLAM_SEC="${WAIT_SLAM_SEC:-60}"
  export SAVE_B="${SAVE_B:-1}"
  bash "${FORMICA_WS}/run_mapping_then_chapter6.sh"
}

_cmd_list() {
  cat <<'EOF'
ros2 run formica_experiments exp1_calibration
ros2 run formica_experiments ina219_power_monitor   # terminal A before exp2
ros2 run formica_experiments exp2_power
MISSION_DURATION_S=120 ros2 run formica_experiments exp2_power
ros2 run formica_experiments exp3_slam
ros2 run formica_experiments exp4_maze
bash /home/jetson/exp1_logs/run_exp4_stack.sh --with-trials
ros2 run formica_experiments exp5_fault
ros2 run formica_experiments exp6_cnn
ros2 run formica_experiments exp7_pheromone
ros2 run formica_experiments exp7_pheromone --ros-args -p mock_sensors:=true -p auto_run:=true
python3 /home/jetson/chapter6_formica_deliverables/build_chapter6_from_logs.py
EOF
}

main() {
  local cmd="${1:-help}"
  case "$cmd" in
    help|-h|--help) _help ;;
    check) _source_ros; _cmd_check ;;
    bringup) _source_ros; _cmd_bringup 0 ;;
    bringup-full) _source_ros; _cmd_bringup 1 ;;
    exp1)
      _source_ros
      echo "[exp1] Requires /scan /imu/data /odom /line_sensors /gas_sensor /rgb/image_raw @ rates in node."
      echo "[exp1] If IMU/camera missing: use ./scripts/chapter6_experiment_runner.sh bringup-full first."
      exec ros2 run formica_experiments exp1_calibration
      ;;
    exp2)
      _source_ros
      exec ros2 run formica_experiments exp2_power
      ;;
    exp2-quick)
      _source_ros
      export MISSION_DURATION_S="${MISSION_DURATION_S:-120}"
      exec ros2 run formica_experiments exp2_power
      ;;
    exp3-stack) _source_ros; _cmd_exp3_stack ;;
    exp3)
      _source_ros
      exec ros2 run formica_experiments exp3_slam
      ;;
    exp4-diagnose)
      _source_ros
      exec bash /home/jetson/exp1_logs/run_exp4_stack.sh --diagnose
      ;;
    exp4-stack)
      _source_ros
      exec bash /home/jetson/exp1_logs/run_exp4_stack.sh
      ;;
    exp4-trials)
      _source_ros
      exec bash /home/jetson/exp1_logs/run_exp4_stack.sh --with-trials
      ;;
    exp5)
      _source_ros
      exec ros2 run formica_experiments exp5_fault
      ;;
    exp6)
      _source_ros
      exec ros2 run formica_experiments exp6_cnn
      ;;
    exp7)
      _source_ros
      exec ros2 run formica_experiments exp7_pheromone
      ;;
    exp7-demo)
      _source_ros
      exec ros2 run formica_experiments exp7_pheromone --ros-args -p mock_sensors:=true -p auto_run:=true
      ;;
    mapping) _source_ros; _cmd_mapping ;;
    list) _cmd_list ;;
    *)
      echo "Unknown command: $cmd" >&2
      _help >&2
      exit 1
      ;;
  esac
}

main "$@"
