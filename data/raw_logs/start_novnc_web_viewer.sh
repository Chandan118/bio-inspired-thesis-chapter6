#!/usr/bin/env bash
# Stream the Jetson's existing X11 desktop (:0) to a web page (noVNC + websockify).
# Run ON THE JETSON as the same user that is logged into the GUI (usually jetson).
#
# On your Mac: Chrome → http://<JETSON_LAN_IP>:6080/vnc.html  → Connect
# Example: http://192.168.1.35:6080/vnc.html
#
# From SSH: you must point at the *logged-in* desktop session, e.g.:
#   export DISPLAY=:0
#   export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority   # GDM
#   # or try:  export XAUTHORITY="$HOME/.Xauthority"
#   bash /home/jetson/exp1_logs/start_novnc_web_viewer.sh
#
# rviz2 / OpenGL in the browser: the 3D view often looks black in noVNC unless GL
# goes through Mesa software rasterization (x11vnc only sees normal X11 pixels).
# From SSH before rviz2:
#   export DISPLAY=:0
#   export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
#   LIBGL_ALWAYS_SOFTWARE=1 rviz2
#
# Security: VNC listens on 127.0.0.1 only; websockify exposes 6080 to the LAN.
#
# If you see "Address already in use" on port 5900, either run:
#   sudo fuser -k 5900/tcp
# or use another port:
#   VNC_PORT=5901 WEB_PORT=6081 bash /home/jetson/exp1_logs/start_novnc_web_viewer.sh
#
set -euo pipefail
export DISPLAY="${DISPLAY:-:0}"

WEB_PORT="${WEB_PORT:-6080}"
VNC_PORT="${VNC_PORT:-5900}"
PASSFILE="${HOME}/.vnc/x11vnc.pass"
NOVNC_WEB="${NOVNC_WEB:-/usr/share/novnc}"

if [[ ! -d "$NOVNC_WEB" ]]; then
  echo "noVNC web files not at $NOVNC_WEB. Run: sudo bash /home/jetson/exp1_logs/install_novnc_browser_viewer.sh"
  exit 1
fi

AUTH_ARGS=()
if [[ -f "$PASSFILE" ]]; then
  AUTH_ARGS=(-rfbauth "$PASSFILE")
  echo "Using VNC password from $PASSFILE"
else
  echo "WARN: No $PASSFILE — VNC has NO password (LAN only; set one with: x11vnc -storepasswd $PASSFILE)"
  AUTH_ARGS=(-nopw)
fi

pkill -x x11vnc 2>/dev/null || true
pkill -f "websockify.*${WEB_PORT}" 2>/dev/null || true
sleep 0.5

_free_tcp_port() {
  local port="$1"
  echo "Ensuring nothing is listening on TCP ${port} ..."
  if command -v fuser >/dev/null 2>&1; then
    fuser -k "${port}/tcp" 2>/dev/null || true
  fi
  if command -v lsof >/dev/null 2>&1; then
    local pids
    pids="$(lsof -t -iTCP:"$port" -sTCP:LISTEN 2>/dev/null || true)"
    if [[ -n "$pids" ]]; then
      # shellcheck disable=SC2086
      kill $pids 2>/dev/null || true
    fi
  fi
  sleep 0.5
}
_free_tcp_port "$VNC_PORT"
_free_tcp_port "$WEB_PORT"

# x11vnc may return non-zero under set -e; check process instead
set +e
x11vnc -display "$DISPLAY" -auth guess -rfbport "$VNC_PORT" -localhost -forever -shared \
  -noxdamage "${AUTH_ARGS[@]}" \
  -bg -o /tmp/x11vnc_jetson.log
set -e
sleep 0.5

if ! pgrep -x x11vnc >/dev/null 2>&1; then
  echo ""
  echo "ERROR: x11vnc did not stay running. Last log lines:"
  tail -25 /tmp/x11vnc_jetson.log 2>/dev/null || echo "(no log file)"
  echo ""
  if grep -q 'Address already in use' /tmp/x11vnc_jetson.log 2>/dev/null; then
    echo "Port ${VNC_PORT} is still in use. Try:"
    echo "  sudo fuser -k ${VNC_PORT}/tcp"
    echo "  sudo lsof -iTCP:${VNC_PORT} -sTCP:LISTEN"
    echo "Or use a different VNC port:"
    echo "  VNC_PORT=5901 WEB_PORT=6081 bash $0"
    echo ""
  fi
  echo "Fix: (1) Log into the Jetson graphical desktop (monitor or auto-login)."
  echo "     (2) From SSH, set X authority for that session, e.g.:"
  echo "         export DISPLAY=:0"
  for xa in "/run/user/$(id -u)/gdm/Xauthority" "/run/user/$(id -u)/lightdm/xauthority" "$HOME/.Xauthority"; do
    [[ -r "$xa" ]] && echo "         export XAUTHORITY=$xa   # (this file exists on your system)"
  done
  echo "     (3) Run this script again."
  exit 1
fi

echo "x11vnc on 127.0.0.1:${VNC_PORT} (log: /tmp/x11vnc_jetson.log)"
echo "Starting websockify on 0.0.0.0:${WEB_PORT} ..."
echo "Open in Mac Chrome: http://$(hostname -I | awk '{print $1}'):${WEB_PORT}/vnc.html"
echo "Press Ctrl+C to stop websockify (x11vnc keeps running; pkill x11vnc to stop VNC)."

exec websockify --web="$NOVNC_WEB" "$WEB_PORT" 127.0.0.1:"$VNC_PORT"
