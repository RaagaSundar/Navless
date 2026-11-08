#!/usr/bin/env bash
# Runs only inside the separate simulation container, without USB/serial mounts.
set -eo pipefail
[[ "${ROS_DOMAIN_ID:-}" == 219 ]] || exit 2
export SKYBETA_UNDERLAY_ONLY=1
source /usr/local/bin/skybeta_env.sh
mkdir -p /root/split_state/logs
cd /root/split_state
pids=()
cleanup() { kill "${pids[@]}" 2>/dev/null || true; wait || true; }
trap cleanup EXIT TERM INT
python3 /split/sensor_link.py jetson --port 17447 > logs/sensor_link.log 2>&1 & pids+=("$!")
export SPLIT_INERTIAL=0
if [[ -f /root/split_state/estimator-profile ]]; then
  read -r profile < /root/split_state/estimator-profile
  case "$profile" in
    stereo) ;;
    stereo-imu)
      export SPLIT_INERTIAL=1
      python3 -u /split/imu_link.py receiver > logs/imu-receiver.log 2>&1 & pids+=("$!")
      ;;
    *) echo "invalid split estimator profile" >&2; exit 2 ;;
  esac
fi
ros2 launch /split/sim_slam.launch.py > logs/slam.log 2>&1 & pids+=("$!")
python3 /split/viewer/odom_gate.py --ros-args -p use_sim_time:=true > logs/gate.log 2>&1 & pids+=("$!")
python3 /split/viewer/map_server.py --ros-args -p use_sim_time:=true > logs/viewer.log 2>&1 & pids+=("$!")
ros2 run oakd_vslam_bringup px4_odom_bridge --ros-args \
  -p use_sim_time:=true -p input_topic:=/mapping/validated_odometry \
  > logs/px4_bridge.log 2>&1 & pids+=("$!")
if [[ -f /split/perception/stereo_cloud.py ]]; then
  python3 /split/perception/stereo_cloud.py --ros-args -p use_sim_time:=true \
    -p rate_hz:=2.0 -p http_port:=18081 > logs/stereo.log 2>&1 & pids+=("$!")
fi
if [[ -f /split/perception/course.obj ]]; then
  python3 /split/perception/cloud_localize.py --ros-args -p use_sim_time:=true \
    -p obj:=/split/perception/course.obj -p http_port:=18082 \
    -p init_x:=-2.5146 -p init_y:=0.9144 > logs/localize.log 2>&1 & pids+=("$!")
fi
# UDP serves *SITL only*. No UART device exists in this container.
MicroXRCEAgent udp4 -p 8889 > logs/xrce.log 2>&1 & pids+=("$!")
wait -n "${pids[@]}"
