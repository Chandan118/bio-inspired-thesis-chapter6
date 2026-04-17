#!/usr/bin/env bash
# Watch robot Arduino traffic while keeping a normal shell free.
#
# The USB serial device is exclusive: if arduino_base (or similar) has the port
# open, do NOT run minicom/picocom on the same device — use this instead.
#
# Usage:
#   bash /home/jetson/exp1_logs/watch_robot_serial.sh topics
#       → streams decoded values (line sensors, distance, gas) via one ROS node
#         (avoids parallel `ros2 topic echo` + Fast DDS shared-memory errors).
#         Optional: WATCH_ROBOT_SERIAL_HEARTBEAT_SEC=30 for repeat lines while zero.
#
#   bash /home/jetson/exp1_logs/watch_robot_serial.sh log [FILE]
#       → follow FILE and print lines containing RAW SERIAL (default: newest *.log in exp1_logs).
#
#   bash /home/jetson/exp1_logs/watch_robot_serial.sh tmux [FILE]
#       → tmux: large pane = your shell (ROS sourced), small pane = serial tail.
#
set -euo pipefail

EXP1_LOGS="${EXP1_LOGS:-/home/jetson/exp1_logs}"
SESSION="${ROBOT_SERIAL_TMUX_SESSION:-robot_serial}"

_latest_log() {
  ls -t "$EXP1_LOGS"/*.log 2>/dev/null | head -1 || true
}

_source_ros() {
  # ROS setup.bash touches vars like AMENT_TRACE_SETUP_FILES without ${var:-};
  # this script uses nounset (-u), so relax it only for sourcing.
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  [[ -f /home/jetson/ros2_ws/install/setup.bash ]] && source /home/jetson/ros2_ws/install/setup.bash
  set -u
}

cmd_topics() {
  _source_ros
  # One Python node (not three `ros2 topic echo`). Still use UDP-only Fast DDS
  # so this process does not compete for SHM ports with arduino_base/rviz/etc.
  local fdds="${EXP1_LOGS}/fastdds_watch_serial_udp_only.xml"
  if [[ -f "$fdds" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$fdds"
    export FASTDDS_DEFAULT_PROFILES_FILE="$fdds"
  fi
  exec python3 /home/jetson/exp1_logs/watch_robot_serial_topics.py
}

cmd_log() {
  local f="${1:-}"
  if [[ -z "$f" ]]; then
    f="$(_latest_log)"
  fi
  if [[ -z "$f" || ! -f "$f" ]]; then
    echo "No log file. Pass one: $0 log /home/jetson/exp1_logs/your_run.log" >&2
    exit 1
  fi
  echo "[watch_robot_serial] Last RAW SERIAL lines in file (if any):"
  grep 'RAW SERIAL' "$f" 2>/dev/null | tail -n 15 || true
  echo "[watch_robot_serial] Live: new lines only (Ctrl+C to stop) ..."
  tail -n 0 -f "$f" | grep --line-buffered 'RAW SERIAL'
}

cmd_tmux() {
  if ! command -v tmux >/dev/null 2>&1; then
    echo "tmux not installed. Try: sudo apt-get install -y tmux" >&2
    exit 1
  fi
  local f="${1:-}"
  if [[ -z "$f" ]]; then
    f="$(_latest_log)"
  fi
  if [[ -z "$f" || ! -f "$f" ]]; then
    echo "No log file for tail pane. Usage: $0 tmux /path/to/run.log" >&2
    exit 1
  fi

  tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"
  tmux new-session -d -s "$SESSION" -n main
  # Bottom ~28%: serial tail; top: interactive shell
  tmux split-window -t "$SESSION:0" -v -p 28
  tmux send-keys -t "$SESSION:0.1" "bash /home/jetson/exp1_logs/watch_robot_serial.sh log $(printf '%q' "$f")" C-m
  tmux send-keys -t "$SESSION:0.0" "source /opt/ros/humble/setup.bash; source /home/jetson/ros2_ws/install/setup.bash 2>/dev/null; cd ~" C-m
  tmux select-pane -t "$SESSION:0.0"
  echo "[watch_robot_serial] Attaching tmux session '$SESSION' (detach: Ctrl+B then D)"
  exec tmux attach -t "$SESSION"
}

usage() {
  sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
}

main() {
  case "${1:-}" in
    topics) shift; cmd_topics "$@" ;;
    log) shift; cmd_log "$@" ;;
    tmux) shift; cmd_tmux "$@" ;;
    -h|--help|help) usage ;;
    *)
      usage
      exit 1
      ;;
  esac
}

main "$@"
