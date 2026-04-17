#!/usr/bin/env bash
# Fast sanity check (~3–5s USB) then regenerate Chapter 6 deliverables (incl. subfigures).
set -eo pipefail
echo "=== 1) USB quick check (Formica) ==="
python3 /home/jetson/formica_experiments/hardware_checker.py --quick || true

echo ""
echo "=== 2) ROS 2 topic snapshot (4s max) ==="
if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
  if [[ -f /home/jetson/formica_experiments/install/setup.bash ]]; then
    # shellcheck source=/dev/null
    source /home/jetson/formica_experiments/install/setup.bash
  fi
  timeout 4 ros2 topic list 2>&1 | head -25 || true
else
  echo "No /opt/ros/humble/setup.bash — skip ROS check."
fi

echo ""
echo "=== 3) Chapter 6 figures / tables ==="
python3 /home/jetson/chapter6_formica_deliverables/build_chapter6_from_logs.py

echo "Done."
