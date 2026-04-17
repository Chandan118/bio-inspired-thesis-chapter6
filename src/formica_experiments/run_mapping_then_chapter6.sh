#!/usr/bin/env bash
# One-shot: SLAM + moving base (/cmd_vel) + save map (+ optional map_b) + rebuild Chapter 6 deliverables.
#
# Usage:
#   ./run_mapping_then_chapter6.sh
#   WAIT_SLAM_SEC=120 SAVE_B=1 ./run_mapping_then_chapter6.sh
#   MAPPING_MOTION=0 WAIT_SLAM_SEC=45 ./run_mapping_then_chapter6.sh   # mapping without commanded motion

set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WAIT_SLAM_SEC="${WAIT_SLAM_SEC:-90}"
export WAIT_SLAM_SEC
export SAVE_B="${SAVE_B:-1}"

"${SCRIPT_DIR}/remap_save_formica.sh" /home/jetson/formica_map
python3 /home/jetson/chapter6_formica_deliverables/build_chapter6_from_logs.py
echo "Mapping + Chapter 6 rebuild complete."
