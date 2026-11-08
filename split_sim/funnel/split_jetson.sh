#!/usr/bin/env bash
set -eo pipefail
source /usr/local/bin/skybeta_env.sh
cd /run-data
python3 /split/sensor_link.py jetson --port 17457 >link.log 2>&1 &
ros2 launch /split/sim_slam.launch.py >slam.log 2>&1 &
python3 /split/viewer/odom_gate.py --ros-args -p use_sim_time:=true >gate.log 2>&1 &
ros2 run oakd_vslam_bringup px4_odom_bridge --ros-args -p use_sim_time:=true -p input_topic:=/mapping/validated_odometry >ev.log 2>&1 &
MicroXRCEAgent udp4 -p 8889 >xrce.log 2>&1 &
wait -n
