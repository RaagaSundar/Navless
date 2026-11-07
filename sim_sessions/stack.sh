#!/bin/bash
set -eo pipefail
export ROS_DOMAIN_ID="$1" ROS_LOCALHOST_ONLY=1 SKYBETA_UNDERLAY_ONLY=1 SPLIT_INERTIAL=0
source /usr/local/bin/skybeta_env.sh
[[ "$ROS_DOMAIN_ID" == "$1" && "$ROS_LOCALHOST_ONLY" == 1 ]]
# Trusted helpers have no participant source or writable workspace on PYTHONPATH.
cd /run/stack
export PYTHONDONTWRITEBYTECODE=1 ROS_LOG_DIR=/run/stack/ros
python3 /run/stack/sensor_link.py jetson --port "$2" &
python3 /opt/session/datagrams.py jetson "$3" "$4" &
MicroXRCEAgent udp4 -p "$4" &
ros2 launch /opt/source/sim_slam.launch.py &
python3 /opt/source/viewer/odom_gate.py --ros-args -p use_sim_time:=true &
ros2 run oakd_vslam_bringup px4_odom_bridge --ros-args -p use_sim_time:=true -p input_topic:=/mapping/validated_odometry &
wait -n
exit 1
