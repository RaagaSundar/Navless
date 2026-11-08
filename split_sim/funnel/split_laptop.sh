#!/usr/bin/env bash
# Inside the dedicated Mac SITL container. Physics starts with a landed quad.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
export GZ_IP=127.0.0.1 GZ_PARTITION=drone19_split
cd /run-data
gz sim -s -r --headless-rendering /funnel/funnel.sdf >gazebo.log 2>&1 &
for i in {1..60}; do
  if gz service -i --service /world/funnel/create | grep -q 'Service providers'; then break; fi
  sleep 0.5
done
spawn='sdf_filename: "/split/funnel/dh_quad_stereo.sdf", name: "dh_quad_0", pose: {position:{z:0.075},orientation:{z:-0.052335956,w:0.998629535}}'
if [[ -f /funnel/spawn-request.txt ]]; then spawn=$(cat /funnel/spawn-request.txt); fi
gz service -s /world/funnel/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 10000 --req "$spawn" | tee spawn.log | grep -q 'true'
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/workspace/Navless/master_sim/bridge.yaml >bridge.log 2>&1 &
python3 /split/sensor_link.py laptop >sensor_link.log 2>&1 &
bash /split/start_px4.sh >px4-start.log 2>&1 &
wait -n
