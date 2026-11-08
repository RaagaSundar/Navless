#!/usr/bin/env bash
# Run on the Jetson host; all ROS work stays inside the existing container.
# Preserve the XRCE agent and the organisers' checkout. No arming/mode commands.
set -euo pipefail
name=skybeta-stack
while ! docker exec "$name" tmux has-session -t skybeta 2>/dev/null; do sleep 2; done
while ! docker exec "$name" tmux list-windows -t skybeta -F '#{window_name}' | grep -qx bridge; do sleep 2; done
exec docker exec "$name" bash -lc '
set -e
source skybeta_env.sh
# The walking mapper has no reason to run a square mission or flight heartbeat.
tmux respawn-pane -k -t skybeta:competitor "sleep infinity"
if ! ros2 param get /visual_slam_node enable_localization_n_mapping | grep -q True; then
  tmux respawn-pane -k -t skybeta:slam "bash -lc '\''source skybeta_env.sh; exec ros2 launch oakd_vslam_bringup oakd_vslam.launch.py camera_x:=\"\${CAMERA_X:-0.1053}\" camera_y:=\"\${CAMERA_Y:-0}\" camera_z:=\"\${CAMERA_Z:-0.0304}\" camera_roll:=\"\${CAMERA_ROLL:-0}\" camera_pitch:=\"\${CAMERA_PITCH:-0}\" camera_yaw:=\"\${CAMERA_YAW:-0}\" enable_localization_n_mapping:=True enable_slam_visualization:=True enable_landmarks_view:=True enable_observations_view:=False path_max_size:=10000'\''"
fi
if ! ros2 param get /px4_odom_bridge input_topic | grep -q /mapping/validated_odometry; then
  tmux respawn-pane -k -t skybeta:bridge "bash -lc '\''source skybeta_env.sh; exec ros2 launch oakd_vslam_bringup px4_bridge.launch.py input_topic:=/mapping/validated_odometry'\''"
fi
python3 /root/team_ws/handheld_slam/odom_gate.py &
gate_pid=$!
python3 /root/team_ws/handheld_slam/map_server.py &
viewer_pid=$!
trap "kill $gate_pid $viewer_pid 2>/dev/null || true" EXIT TERM INT
wait -n "$gate_pid" "$viewer_pid"
'
