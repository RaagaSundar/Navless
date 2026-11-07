#!/usr/bin/env bash
set -euo pipefail
mkdir -p /root/dh_sim_logs
Xvfb "$DISPLAY" -screen 0 1440x900x24 +extension GLX +render -noreset \
  > /root/dh_sim_logs/xvfb.log 2>&1 &
for i in {1..100}; do
  xdpyinfo -display "$DISPLAY" >/dev/null 2>&1 && break
  sleep 0.1
done
xdpyinfo -display "$DISPLAY" >/dev/null
openbox > /root/dh_sim_logs/openbox.log 2>&1 &
x11vnc -display "$DISPLAY" -forever -shared -nopw -localhost -rfbport 5900 \
  > /root/dh_sim_logs/vnc.log 2>&1 &
exec websockify --web=/usr/share/novnc 6080 localhost:5900
