#!/usr/bin/env bash
# Stop only our two nodes. Leave camera, tracker, bridge and XRCE transport up.
# These patterns match mapping mode's child processes only. Flight mode's gate runs in its own tmux
# window as `python3 -u .../odom_gate.py`, which they deliberately don't match: when PX4 is fed through
# the gate (HANDHELD_SLAM_PX4_GATE=1), stopping this service must not cut PX4's vision.
docker exec skybeta-stack pkill -TERM -f '^python3 /root/team_ws/handheld_slam/map_server.py$' || true
docker exec skybeta-stack pkill -TERM -f '^python3 /root/team_ws/handheld_slam/odom_gate.py$' || true
