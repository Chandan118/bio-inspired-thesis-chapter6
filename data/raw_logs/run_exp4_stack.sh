#!/usr/bin/env bash
# Experiment 4: diagnose → stack → optional trials. Default is stack-only (no auto trials).
#   1) bash ... --diagnose     # map + S/T path, no robot
#   2) bash ...                # Nav2 up; check RViz / manual goal; tail -f log on long runs
#   3) bash ... --with-trials  # exp4_maze after the above look good
# Flags: --diagnose | --stack-only | --with-trials | --full
# Long IDE sessions: use tail -f on exp4_auto_*.log; script sets line-buffered ROS/Python output for trials.
#   EXP4_LEGACY_AUTO=1         # old behavior: bare script still runs all trials
#   EXP4_BACKGROUND=1 needs --with-trials
#   EXP4_NUM_TRIALS=3  EXP4_TRIAL_TIMEOUT_S=240  MAP=  PARAMS=  LIDAR_DEV=
set -eo pipefail

LOG=/home/jetson/exp1_logs
MAP=${MAP:-/home/jetson/formica_map.yaml}
PARAMS=${PARAMS:-/home/jetson/ros2_ws/install/formica_experiments/share/formica_experiments/config/nav2_params.yaml}
RUN_TAG="$(date +%Y%m%d_%H%M%S)"

RUN_TRIALS=0
DIAGNOSE=0
for _arg in "$@"; do
  case "$_arg" in
    --diagnose) DIAGNOSE=1 ;;
    --with-trials|--full) RUN_TRIALS=1 ;;
    --stack-only) RUN_TRIALS=0 ;;
  esac
done
[[ "${EXP4_LEGACY_AUTO:-0}" == "1" ]] && [[ $# -eq 0 ]] && RUN_TRIALS=1

[[ "$DIAGNOSE" == "1" ]] && {
  echo "=== [exp4stack] --diagnose (no hardware, no Nav2) ==="
  echo "MAP=$MAP"
  echo "PARAMS=$PARAMS"
  [[ -f "$MAP" ]] || { echo "[exp4stack] ERROR: MAP missing"; exit 1; }
  [[ -f "$PARAMS" ]] || { echo "[exp4stack] ERROR: PARAMS missing"; exit 1; }
  export MAP
  python3 << 'PYDIAG'
import ast, os, re, sys
from collections import deque
from pathlib import Path

MAP = Path(os.environ["MAP"])
text = MAP.read_text()
img = None
ox, oy, res = -5.9, -2.99, 0.05
for line in text.splitlines():
    if line.startswith("image:"):
        img = line.split(":", 1)[1].strip()
    elif line.startswith("origin:"):
        rest = line.split(":", 1)[1].strip().split("#")[0].strip()
        parts = ast.literal_eval(rest)
        ox, oy = float(parts[0]), float(parts[1])
    elif line.startswith("resolution:"):
        res = float(line.split(":", 1)[1].strip())
if not img:
    print("ERROR: no image: in yaml"); sys.exit(1)
pgm = MAP.parent / img
if not pgm.is_file():
    print("ERROR: PGM not found:", pgm); sys.exit(1)
data = pgm.read_bytes()
m = re.match(rb"P5\s+(\d+)\s+(\d+)\s+(\d+)\s", data)
if not m:
    print("ERROR: not binary P5 PGM"); sys.exit(1)
w, h = map(int, m.groups()[:2])
raw = data[m.end() : m.end() + w * h]
if len(raw) != w * h:
    print("ERROR: PGM size"); sys.exit(1)

S_W = (0.0, 0.0)
T_W = (2.525, 3.035)

def wc(wx, wy):
    return int((wx - ox) / res), int((wy - oy) / res)

def is_free(mx, my):
    if not (0 <= mx < w and 0 <= my < h):
        return False
    return raw[my * w + mx] == 254

def trinary(mx, my):
    if not (0 <= mx < w and 0 <= my < h):
        return "out"
    p = raw[my * w + mx]
    if p == 0:
        return "occupied"
    if p == 205:
        return "unknown"
    if p >= 250:
        return "free"
    return "other(%s)" % p

Sx, Sy = wc(*S_W)
Tx, Ty = wc(*T_W)
print(f"PGM {pgm.name}  grid {w}x{h}  origin=({ox},{oy})  res={res} m")
print(f"S {S_W} -> ({Sx},{Sy})  {trinary(Sx, Sy)}")
print(f"T {T_W} -> ({Tx},{Ty})  {trinary(Tx, Ty)}")
if trinary(Tx, Ty) != "free":
    print("WARNING: T not on free cell 254 — change TARGET_POS or map.")

q = deque([(Sx, Sy)])
seen = {(Sx, Sy)}
ok = False
while q:
    x, y = q.popleft()
    if (x, y) == (Tx, Ty):
        ok = True
        break
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        nx, ny = x + dx, y + dy
        if (nx, ny) in seen or not is_free(nx, ny):
            continue
        seen.add((nx, ny))
        q.append((nx, ny))
print("BFS free cells S->T:", "YES" if ok else "NO")
sys.exit(0 if ok else 2)
PYDIAG
  _ec=$?
  echo "=== diagnose exit $_ec (0=ok, 2=no path, 1=file) ==="
  exit "$_ec"
}

source /opt/ros/humble/setup.bash
source /home/jetson/ros2_ws/install/setup.bash

# Non-login shells (cron, systemd, some SSH) skip ~/.bashrc — without these, DDS uses
# domain 0 and exp4_maze cannot see Nav2 (falls back to synthetic / no motion).
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-$HOME/ros2_ws/config/cyclonedds.xml}"
# Jetson + Fast DDS: avoid shared-memory transport issues (same idea as tethered_navigation navigate.launch).
export RMW_FASTRTPS_USE_SHM="${RMW_FASTRTPS_USE_SHM:-0}"
_FASTRTPS_XML="$HOME/ros2_ws/install/tethered_navigation/share/tethered_navigation/config/fastdds_no_shm.xml"
if [[ -f "$_FASTRTPS_XML" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$_FASTRTPS_XML"
  export FASTDDS_DEFAULT_PROFILES_FILE="$_FASTRTPS_XML"
  export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
fi

export PYTHONUNBUFFERED=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0

[[ "$RUN_TRIALS" != "1" ]] && echo "[exp4stack] No trials this run. For exp4_maze: bash $0 --with-trials"

pkill -f "exp4_maze|arduino_base|rplidar_a1_launch|odom_tf_bridge.py|static_transform_publisher|localization_launch.py|navigation_launch.py|bringup_launch.py|nav2_container|ros2 bag record|formica_experiments jetson_camera" 2>/dev/null || true
# Nav2 Humble often runs inside component_container_*; stale ones break lifecycle / duplicate node names.
pkill -f "component_container_isolated|component_container_mt" 2>/dev/null || true
sleep 3

echo "[exp4stack] Restarting ros2 daemon (avoids stale graph after pkill; fixes empty lifecycle get)."
ros2 daemon stop 2>/dev/null || true
sleep 1
ros2 daemon start 2>/dev/null || true
sleep 2

BAG_PID_FILE=""
MP4_PID_FILE=""

launch_lidar() {
  local dev="$1"
  pkill -f "rplidar_a1_launch|rplidar_node" 2>/dev/null || true
  sleep 1
  local safe
  safe="$(echo "$dev" | tr '/' '_')"
  nohup ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:="$dev" \
    >"$LOG/exp4stack_lidar_${RUN_TAG}${safe}.log" 2>&1 &
  echo "[exp4stack] LiDAR launch serial_port=$dev (log $LOG/exp4stack_lidar_${RUN_TAG}${safe}.log)"
}

wait_for_scan() {
  local i
  # Cyclone/Fast DDS: /scan may appear on topic list before first echo; echo --once needs a matching publisher.
  for i in $(seq 1 45); do
    if timeout 3 ros2 topic list 2>/dev/null | grep -qx '/scan'; then
      if timeout 15 ros2 topic echo /scan --once 2>/dev/null | grep -q "ranges:"; then
        echo "[exp4stack] /scan is publishing."
        return 0
      fi
    fi
    sleep 1
  done
  return 1
}

# Two CP2102 UARTs are common (RPLidar + spare); ttyUSB0/1 order swaps on reboot.
# Prefer /dev/serial/by-id/... and try each CP2102 until /scan works. Override: LIDAR_DEV=/dev/...
try_lidar_until_scan() {
  local dev
  if [[ -n "${LIDAR_DEV:-}" ]]; then
    echo "[exp4stack] LiDAR port from LIDAR_DEV=$LIDAR_DEV"
    launch_lidar "$LIDAR_DEV"
    sleep 12
    wait_for_scan && return 0
    echo "[exp4stack] WARN: no /scan on LIDAR_DEV — check cable, power, baud (A1), or wrong port."
    return 1
  fi
  local -a cands=()
  while IFS= read -r dev; do
    [[ -n "$dev" ]] && cands+=("$dev")
  done < <(find /dev/serial/by-id -maxdepth 1 -name 'usb-Silicon_Labs_CP2102*if00-port0' 2>/dev/null | sort)
  if [[ ${#cands[@]} -eq 0 ]]; then
    cands=(/dev/ttyUSB0 /dev/ttyUSB1)
    echo "[exp4stack] No CP2102 by-id nodes — falling back to ${cands[*]}"
  else
    echo "[exp4stack] LiDAR candidates (by-id): ${cands[*]}"
  fi
  for dev in "${cands[@]}"; do
    [[ -e "$dev" ]] || continue
    echo "[exp4stack] Probing LiDAR on $dev ..."
    launch_lidar "$dev"
    sleep 12
    if wait_for_scan; then
      return 0
    fi
    echo "[exp4stack] No /scan on $dev — next candidate ..."
  done
  return 1
}

nohup ros2 run formica_experiments arduino_base --ros-args \
  -p port:=/dev/ttyCH341USB0 -p baud_rate:=115200 -p linear_deadband:=0.02 \
  >"$LOG/exp4stack_base_${RUN_TAG}.log" 2>&1 &
echo "[exp4stack] Base controller started."

try_lidar_until_scan || {
  echo "[exp4stack] ERROR: /scan still missing. Plug RPLidar USB, power motor, then:"
  echo "  ls -la /dev/serial/by-id/"
  echo "  export LIDAR_DEV=/dev/serial/by-id/usb-Silicon_Labs_CP2102_..._if00-port0"
  echo "  bash $0 ..."
  exit 1
}

nohup python3 /home/jetson/exp1_logs/odom_tf_bridge.py >"$LOG/exp4stack_odom_tf_${RUN_TAG}.log" 2>&1 &
nohup ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint laser \
  >"$LOG/exp4stack_tf_laser_${RUN_TAG}.log" 2>&1 &
nohup ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link \
  >"$LOG/exp4stack_tf_base_link_${RUN_TAG}.log" 2>&1 &
sleep 2

for _ in $(seq 1 30); do
  if timeout 2 ros2 topic echo /tf --once 2>/dev/null | grep -q "frame_id: odom"; then
    break
  fi
  sleep 0.5
done

echo "[exp4stack] Starting localization (map_server + amcl) ..."
nohup ros2 launch nav2_bringup localization_launch.py \
  map:="$MAP" use_sim_time:=False autostart:=True params_file:="$PARAMS" \
  >"$LOG/exp4stack_localization_${RUN_TAG}.log" 2>&1 &
# AMCL + map_server need time before /set_initial_pose works; too short → flaky pose.
sleep 18

echo "[exp4stack] Waiting for map_server + AMCL active (localization lifecycle) ..."
LOC_OK=0
for _ in $(seq 1 240); do
  MS="$(timeout 8 ros2 lifecycle get /map_server 2>&1)" || true
  A="$(timeout 8 ros2 lifecycle get /amcl 2>&1)" || true
  if [[ "$MS" == *"active [3]"* ]] && [[ "$A" == *"active [3]"* ]]; then
    LOC_OK=1
    break
  fi
  sleep 1
done
[[ "$LOC_OK" != 1 ]] && echo "[exp4stack] WARN: map_server or AMCL not active after 240s — see $LOG/exp4stack_localization_${RUN_TAG}.log"
sleep 2

# AMCL listens on /initialpose; wait until active above so a subscriber exists before ros2 topic pub waits.
INIT_POSE_YAML="{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]}}"
POSE_OK=0
for pose_try in $(seq 1 3); do
  echo "[exp4stack] Publishing initial pose on /initialpose (try $pose_try) ..."
  if timeout 60 ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "$INIT_POSE_YAML" \
    --qos-reliability reliable --keep-alive 1.5 \
    >"$LOG/exp4stack_initial_pose_${RUN_TAG}.log" 2>&1; then
    POSE_OK=1
    break
  fi
  sleep 4
done
[[ "$POSE_OK" != 1 ]] && echo "[exp4stack] WARN: initial pose publish failed — see $LOG/exp4stack_initial_pose_${RUN_TAG}.log"
sleep 6

echo "[exp4stack] Starting navigation stack ..."
nohup ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=False autostart:=True params_file:="$PARAMS" \
  >"$LOG/exp4stack_navigation_${RUN_TAG}.log" 2>&1 &
# Navigation lifecycle + costmaps on Orin Nano often need >18s before bt_navigator accepts activate.
# With-trials: give costmaps + bonds extra time before ros2 lifecycle get (avoids false "bt_navigator not active").
NAV_LAUNCH_SLEEP=28
[[ "${EXP4_RECORD_CAMERA:-0}" == "1" ]] && NAV_LAUNCH_SLEEP=32
[[ "$RUN_TRIALS" == "1" ]] && NAV_LAUNCH_SLEEP=$((NAV_LAUNCH_SLEEP + 14))
sleep "$NAV_LAUNCH_SLEEP"
# Large costmaps + BT load: querying lifecycle too soon yields empty / timed-out get_state (see nav log).
[[ "$RUN_TRIALS" == "1" ]] && {
  echo "[exp4stack] Extra settle before lifecycle probes (large Nav2 costmaps) ..."
  sleep 18
}

echo "[exp4stack] Waiting for /bt_navigator lifecycle node ..."
for _ in $(seq 1 55); do
  OUT="$(timeout 20 ros2 lifecycle get /bt_navigator 2>&1)" || true
  [[ "$OUT" != *"not found"* ]] && [[ -n "$OUT" ]] && break
  sleep 1
done

BT_RETRIES=5
[[ "${EXP4_RECORD_CAMERA:-0}" == "1" ]] && BT_RETRIES=10
for attempt in $(seq 1 "$BT_RETRIES"); do
  echo "amcl: $(timeout 20 ros2 lifecycle get /amcl 2>&1 || echo '?')"
  echo "planner_server: $(timeout 20 ros2 lifecycle get /planner_server 2>&1 || echo '?')"
  BT_STATE="$(timeout 20 ros2 lifecycle get /bt_navigator 2>&1)" || true
  echo "bt_navigator: $BT_STATE"
  # Do not pipe lifecycle get into grep with pipefail: grep -q can SIGPIPE ros2 and fail the test.
  if [[ "$BT_STATE" == *"active [3]"* ]]; then
    break
  fi
  echo "[exp4stack] Retrying bt_navigator activate (attempt $attempt)..."
  timeout 25 ros2 service call /bt_navigator/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3, label: activate}}" >/dev/null 2>&1 || true
  sleep 3
done

BT_FINAL="$(timeout 25 ros2 lifecycle get /bt_navigator 2>&1)" || true
if [[ "$BT_FINAL" != *"active [3]"* ]]; then
  echo "[exp4stack] ERROR: bt_navigator not active (got: $BT_FINAL) — see $LOG/exp4stack_navigation_${RUN_TAG}.log"
  exit 1
fi

# Camera + bag after Nav2 is active: avoids lifecycle/DDS races when the Jetson is busy at startup.
if [[ "${EXP4_RECORD_CAMERA:-0}" == "1" ]]; then
  MP4_PID_FILE="$LOG/exp4_rgb_${RUN_TAG}.pid"
  echo "[exp4stack] Starting camera → /rgb/image_raw (see $LOG/exp4stack_camera_${RUN_TAG}.log) ..."
  nohup ros2 run formica_experiments jetson_camera --ros-args -p topic_name:=/rgb/image_raw \
    >"$LOG/exp4stack_camera_${RUN_TAG}.log" 2>&1 &
  CAM_OK=0
  for _ in $(seq 1 25); do
    if timeout 4 ros2 topic info /rgb/image_raw 2>/dev/null | grep -qE "Publisher count: [1-9]"; then
      CAM_OK=1
      echo "[exp4stack] /rgb/image_raw is publishing."
      break
    fi
    sleep 1
  done
  MP4_OUT="$LOG/exp4_rgb_${RUN_TAG}.mp4"
  if [[ "$CAM_OK" != 1 ]] && [[ -e /dev/video0 ]]; then
    echo "[exp4stack] ROS camera not publishing — release device and try V4L2 MP4: $MP4_OUT"
    pkill -f "formica_experiments jetson_camera" 2>/dev/null || true
    sleep 2
    nohup python3 /home/jetson/exp1_logs/record_rgb_mp4.py "$MP4_OUT" \
      >"$LOG/exp4stack_rgb_mp4_${RUN_TAG}.log" 2>&1 &
    echo $! >"$MP4_PID_FILE"
    echo "[exp4stack] RGB MP4 recorder PID $(cat "$MP4_PID_FILE") — stop: kill \$(cat $MP4_PID_FILE)"
  else
    rm -f "$MP4_PID_FILE" 2>/dev/null || true
  fi
  BAG_DIR="$LOG/rosbag_exp4_${RUN_TAG}"
  BAG_PID_FILE="$LOG/rosbag_exp4_${RUN_TAG}.pid"
  if [[ "$CAM_OK" == 1 ]]; then
    echo "[exp4stack] Recording bag (with camera): $BAG_DIR"
    nohup ros2 bag record -o "$BAG_DIR" \
      /rgb/image_raw /scan /cmd_vel /odom /amcl_pose /plan \
      >"$LOG/exp4stack_bag_${RUN_TAG}.log" 2>&1 &
  else
    echo "[exp4stack] WARN: No ROS image topic — recording sensors in bag + optional MP4 above."
    nohup ros2 bag record -o "$BAG_DIR" \
      /scan /cmd_vel /odom /amcl_pose /plan \
      >"$LOG/exp4stack_bag_${RUN_TAG}.log" 2>&1 &
  fi
  echo $! >"$BAG_PID_FILE"
  echo "[exp4stack] Rosbag PID $(cat "$BAG_PID_FILE") — stop: kill \$(cat $BAG_PID_FILE)"
  {
    echo "run_tag=$RUN_TAG"
    echo "bag_dir=$BAG_DIR"
    echo "playback: ros2 bag play $BAG_DIR"
    echo "rviz: add Image display, topic /rgb/image_raw"
    echo "mp4_fallback=$MP4_OUT (if recorder started)"
  } >"$LOG/rosbag_exp4_${RUN_TAG}_README.txt"
  sleep 2
fi

echo "[exp4stack] NavigateToPose server check ..."
python3 - <<'PY'
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
rclpy.init()
n = Node("exp4_nav_probe")
c = ActionClient(n, NavigateToPose, "navigate_to_pose")
ok = c.wait_for_server(timeout_sec=15.0)
print("NAVIGATE_TO_POSE_READY", ok)
n.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if ok else 1)
PY

[[ "$RUN_TRIALS" != "1" ]] && {
  echo "[exp4stack] Stack ready. Place robot at S, verify AMCL, then either:"
  echo "  ros2 run formica_experiments exp4_maze"
  echo "  or re-run: bash $0 --with-trials"
  [[ -n "$BAG_PID_FILE" ]] && [[ -f "$BAG_PID_FILE" ]] && echo "[exp4stack] Stop bag: kill \$(cat $BAG_PID_FILE)"
  [[ -n "$MP4_PID_FILE" ]] && [[ -f "$MP4_PID_FILE" ]] && echo "[exp4stack] Stop MP4: kill \$(cat $MP4_PID_FILE)"
  exit 0
}

EXP_LOG="$LOG/exp4_auto_${RUN_TAG}.log"
echo "[exp4stack] Experiment log: $EXP_LOG"
echo "[exp4stack] Live view (recommended): tail -f $EXP_LOG"

if [[ "${EXP4_BACKGROUND:-0}" == "1" ]]; then
  nohup bash -c "export PYTHONUNBUFFERED=1 RCUTILS_LOGGING_BUFFERED_STREAM=0; source /opt/ros/humble/setup.bash && source /home/jetson/ros2_ws/install/setup.bash && stdbuf -oL -eL ros2 run formica_experiments exp4_maze" >"$EXP_LOG" 2>&1 &
  echo "exp4_pid=$!"
  echo "[exp4stack] Experiment 4 running in background. Tail: tail -f $EXP_LOG"
  echo "[exp4stack] After it finishes: python3 $LOG/generate_exp4_outputs.py"
  [[ -n "$BAG_PID_FILE" ]] && echo "[exp4stack] When trials are done, stop the bag: kill \$(cat $BAG_PID_FILE)"
  exit 0
fi

set +e
set -o pipefail
stdbuf -oL -eL ros2 run formica_experiments exp4_maze 2>&1 | stdbuf -oL -eL tee "$EXP_LOG"
EX="${PIPESTATUS[0]}"
set +o pipefail
set -e
# Ctrl+C leaves "^C" with no newline; next print would appear on the same line as EXP4_SOURCE,...
echo ""
if [[ "$EX" -eq 130 ]] || [[ "$EX" -eq 143 ]]; then
  echo "[exp4stack] Experiment interrupted (exit $EX). Latest CSV used for table/figure if present."
fi
python3 "$LOG/generate_exp4_outputs.py" || true
if [[ -n "$BAG_PID_FILE" ]] && [[ -f "$BAG_PID_FILE" ]]; then
  kill "$(cat "$BAG_PID_FILE")" 2>/dev/null || true
  echo "[exp4stack] Stopped rosbag (saved under $LOG/rosbag_exp4_${RUN_TAG}/)"
fi
if [[ -n "$MP4_PID_FILE" ]] && [[ -f "$MP4_PID_FILE" ]]; then
  kill "$(cat "$MP4_PID_FILE")" 2>/dev/null || true
  echo "[exp4stack] Stopped RGB MP4 recorder (see $LOG/exp4_rgb_${RUN_TAG}.mp4)"
fi
echo "[exp4stack] Done. CSV summary: $LOG/table6_3_maze_navigation.csv"
exit "$EX"
