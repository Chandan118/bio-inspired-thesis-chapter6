#!/usr/bin/env bash
# One-time: install HTML5 VNC stack (run ON THE JETSON, with sudo).
# After install, use start_novnc_web_viewer.sh while the Jetson graphical session is active.
set -e
sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y novnc websockify x11vnc
echo "OK. Next (as user jetson, with monitor/session or DISPLAY=:0):"
echo "  bash /home/jetson/exp1_logs/start_novnc_web_viewer.sh"
echo "Then on your Mac Chrome open:  http://<JETSON_IP>:6080/vnc.html"
