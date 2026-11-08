#!/usr/bin/env bash
# Stop only our two nodes. Leave camera, tracker, bridge and XRCE transport up.
docker exec skybeta-stack pkill -TERM -f '^python3 /root/team_ws/handheld_slam/map_server.py$' || true
docker exec skybeta-stack pkill -TERM -f '^python3 /root/team_ws/handheld_slam/odom_gate.py$' || true
