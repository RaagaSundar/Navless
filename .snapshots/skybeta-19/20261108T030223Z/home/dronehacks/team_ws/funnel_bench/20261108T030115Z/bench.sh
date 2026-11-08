#!/usr/bin/env bash
# Run inside the isolated Jetson Gazebo image with /split and /funnel read-only.
set -eo pipefail
cd /run-data
mkdir -p /root/gazebo_state
source /usr/local/bin/skybeta_env.sh
export GZ_PARTITION=funnel_bench GZ_IP=127.0.0.1
pids=()
trap 'kill "${pids[@]}" 2>/dev/null || true; wait || true' EXIT
# Copy only the installed stereo sensors. This fixed rig cannot move or arm.
python3 - <<'PY'
from xml.etree import ElementTree as E
tree = E.parse('/funnel/funnel.sdf')
model = E.SubElement(tree.find('world'), 'model', name='camera_fixture')
E.SubElement(model, 'static').text = 'true'
E.SubElement(model, 'pose').text = '0 0 0.9 0 0 -0.104719755'
link = E.SubElement(model, 'link', name='base_link')
quad = E.parse('/split/gazebo_assets/dh_quad_stereo.sdf')
sensors = quad.findall(".//link[@name='base_link']/sensor[@type='camera']")
assert len(sensors) == 2
for camera in sensors:
    link.append(camera)
tree.write('bench.sdf')
PY
gz sim -s -r --headless-rendering bench.sdf >gazebo.log 2>&1 & pids+=("$!")
python3 /split/jetson_gazebo/native_sensor_bridge.py >bridge.log 2>&1 & pids+=("$!")
ros2 launch /split/sim_slam.launch.py >slam.log 2>&1 & pids+=("$!")
python3 -u /funnel/observe.py --ros-args -p use_sim_time:=true >observations.jsonl
