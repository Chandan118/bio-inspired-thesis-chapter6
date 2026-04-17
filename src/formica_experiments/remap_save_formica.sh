#!/usr/bin/env bash
# Bring up lidar + Arduino (auto-detected ports), run slam_toolbox, save occupancy map.
#
# Usage:
#   ./remap_save_formica.sh [map_base_path]
#   WAIT_SLAM_SEC=35 ./remap_save_formica.sh /home/jetson/formica_map
#   SAVE_B=1 ./remap_save_formica.sh   # also writes formica_map_b.pgm (+ yaml) for thesis subfigure (b)
#   MAPPING_MOTION=0 ./remap_save_formica.sh   # disable automatic /cmd_vel exploration (robot stays put)
#
# map_base_path: files become <path>.yaml and <path>.pgm (default /home/jetson/formica_map)
#
# By default this script runs mapping_motion_helper so the base moves during SLAM (Arduino listens on /cmd_vel).

set -eo pipefail

MAP_BASE="${1:-/home/jetson/formica_map}"
WAIT_SLAM_SEC="${WAIT_SLAM_SEC:-30}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set +u
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/install/setup.bash"
set -e

eval "$("${SCRIPT_DIR}/scripts/detect_formica_ports.py" --export-bash)"
if [[ -z "${ARDUINO_PORT:-}" || -z "${LIDAR_PORT:-}" ]]; then
  echo "Could not detect LIDAR_PORT / ARDUINO_PORT. Check USB." >&2
  exit 1
fi

echo "Using LIDAR_PORT=$LIDAR_PORT"
echo "Using ARDUINO_PORT=$ARDUINO_PORT"
echo "Saving map to ${MAP_BASE}.{yaml,pgm} after ${WAIT_SLAM_SEC}s of SLAM ..."

MOTION_PID=""

cleanup() {
  if [[ -n "${MOTION_PID}" ]]; then
    kill "${MOTION_PID}" 2>/dev/null || true
    wait "${MOTION_PID}" 2>/dev/null || true
  fi
  jobs -p 2>/dev/null | xargs -r kill 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 launch formica_experiments bringup_launch.py \
  lidar_port:="${LIDAR_PORT}" \
  arduino_port:="${ARDUINO_PORT}" &
sleep 7

ros2 launch slam_toolbox online_async_launch.py &
sleep 3

# Physical exploration: slam_toolbox alone does not publish /cmd_vel — base stays still without this.
if [[ "${MAPPING_MOTION:-1}" != "0" ]]; then
  MOT_SEC=$((WAIT_SLAM_SEC - 5))
  if [[ "${MOT_SEC}" -lt 12 ]]; then MOT_SEC=12; fi
  export MAPPING_MOTION_SEC="${MOT_SEC}"
  echo "Starting mapping_motion_helper for ${MOT_SEC}s on /cmd_vel ..."
  ros2 run formica_experiments mapping_motion_helper &
  MOTION_PID=$!
fi

sleep "${WAIT_SLAM_SEC}"

set +e
ros2 run nav2_map_server map_saver_cli -f "${MAP_BASE}" 2>&1
MS=$?
set -e
if [[ "${MS}" -ne 0 ]]; then
  echo "map_saver_cli failed (exit ${MS}). Is /map publishing? Check lidar scan errors above." >&2
  echo "Fallback for Chapter 6 subfigure (b) only (copies current saved map, not a new SLAM run):" >&2
  echo "  cp -f /home/jetson/formica_map.pgm /home/jetson/formica_map_b.pgm && cp -f /home/jetson/formica_map.yaml /home/jetson/formica_map_b.yaml" >&2
  echo "  sed -i 's|^image:.*|image: formica_map_b.pgm|' /home/jetson/formica_map_b.yaml" >&2
  echo "  cp -f /home/jetson/exp1_logs/exp3_trajectory_<timestamp>.csv /home/jetson/exp1_logs/exp3_trajectory_b.csv" >&2
  echo "  python3 /home/jetson/chapter6_formica_deliverables/build_chapter6_from_logs.py" >&2
fi

if [[ "${SAVE_B:-0}" == "1" && "${MS}" -eq 0 ]]; then
  cp -f "${MAP_BASE}.pgm" /home/jetson/formica_map_b.pgm
  cp -f "${MAP_BASE}.yaml" /home/jetson/formica_map_b.yaml
  sed -i 's|^image:.*|image: formica_map_b.pgm|' /home/jetson/formica_map_b.yaml || true
  echo "Also wrote formica_map_b.pgm / formica_map_b.yaml for Chapter 6 subfigure (b)."
fi

echo "Remap script finished. Regenerate thesis figures:"
echo "  python3 /home/jetson/chapter6_formica_deliverables/build_chapter6_from_logs.py"
