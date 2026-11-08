#!/usr/bin/env bash
# Isolated stationary EGL Gazebo + cuVSLAM benchmark; no autopilot or commands.
set -eo pipefail
[[ "${ROS_DOMAIN_ID:-}" == 219 ]] || exit 2
export SKYBETA_UNDERLAY_ONLY=1
source /usr/local/bin/skybeta_env.sh
if [[ "${SENSOR_ADAPTER:-native}" == ros_gz ]]; then
  source /opt/jetson_gazebo_ws/install/setup.bash
fi
export GZ_PARTITION=drone19_gazebo_bench GZ_IP=127.0.0.1
export GZ_SIM_RESOURCE_PATH=/split/gazebo_assets/models
mkdir -p /root/gazebo_state/logs /root/gazebo_state/viewer
cd /root/gazebo_state
cp /split/viewer/{map_server.py,viewer.html} viewer/
sed -i 's/Mac images/Jetson-local simulated images/g; s/Mac Gazebo/Jetson-local Gazebo/g; s/Gazebo on the Mac/Gazebo locally on drone 19/g' viewer/viewer.html
pids=()
cleanup() { kill "${pids[@]}" 2>/dev/null || true; wait || true; }
trap cleanup EXIT TERM INT
gz sim -v 4 -s -r --headless-rendering /split/dronehacks.sdf > logs/gazebo.log 2>&1 & pids+=("$!")
ready=0
for i in {1..90}; do
  if gz service -i --service /world/dronehacks/create 2>/dev/null | grep -q 'Service providers'; then ready=1; break; fi
  sleep 0.5
done
[[ "$ready" == 1 ]] || { echo 'Gazebo service did not appear'; exit 3; }
gz service -s /world/dronehacks/create --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean --timeout 10000 \
  --req 'sdf_filename: "/split/gazebo_assets/dh_quad_stereo.sdf", name: "dh_quad_0", allow_renaming: false, pose: {position: {x: -2.5146, y: 0.9144, z: 0.075}, orientation: {z: 0.70710678, w: 0.70710678}}' > logs/spawn.log 2>&1
if [[ "${SENSOR_ADAPTER:-native}" == native ]]; then
  python3 /split/jetson_gazebo/native_sensor_bridge.py > logs/bridge.log 2>&1 & pids+=("$!")
else
  ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/split/jetson_gazebo/bridge.yaml > logs/bridge.log 2>&1 & pids+=("$!")
fi
ros2 launch /split/sim_slam.launch.py > logs/slam.log 2>&1 & pids+=("$!")
python3 /split/viewer/odom_gate.py --ros-args -p use_sim_time:=true > logs/gate.log 2>&1 & pids+=("$!")
python3 viewer/map_server.py --ros-args -p use_sim_time:=true > logs/viewer.log 2>&1 & pids+=("$!")
python3 /split/perception/stereo_cloud.py --ros-args -p use_sim_time:=true -p rate_hz:=2.0 -p http_port:=18081 > logs/stereo.log 2>&1 & pids+=("$!")
# Same optional point-cloud/CAD localization workload as the split baseline.
if [[ -f /split/perception/course.obj ]]; then
  python3 /split/perception/cloud_localize.py --ros-args -p use_sim_time:=true \
    -p obj:=/split/perception/course.obj -p http_port:=18082 \
    -p init_x:=-2.5146 -p init_y:=0.9144 > logs/localize.log 2>&1 & pids+=("$!")
fi
wait -n "${pids[@]}"
