#!/usr/bin/env bash
set -eo pipefail
export GZ_PARTITION=drone19_gazebo_probe GZ_IP=127.0.0.1
export GZ_SIM_RESOURCE_PATH=/split/gazebo_assets/models
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
mkdir -p /root/gazebo_state/probe
cleanup() {
  kill "$gzpid" 2>/dev/null || true
  wait "$gzpid" 2>/dev/null || true
  cp /root/.gz/rendering/ogre2.log /root/gazebo_state/probe/ogre2.log 2>/dev/null || true
}
timeout --signal=TERM --kill-after=5s 35s gz sim -v 4 -s -r --headless-rendering /split/dronehacks.sdf > /root/gazebo_state/probe/gazebo.log 2>&1 & gzpid=$!
trap cleanup EXIT TERM INT
ready=0
for i in {1..20}; do
  if gz service -i --service /world/dronehacks/create 2>/dev/null | grep -q 'Service providers'; then ready=1; break; fi
  sleep .5
done
[[ "$ready" == 1 ]] || exit 3
gz service -s /world/dronehacks/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 10000 \
  --req 'sdf_filename: "/split/gazebo_assets/dh_quad_stereo.sdf", name: "dh_quad_0", allow_renaming: false, pose: {position: {x: -2.5146, y: 0.9144, z: 0.075}, orientation: {z: 0.70710678, w: 0.70710678}}' > /root/gazebo_state/probe/spawn.log 2>&1
python3 /split/jetson_gazebo/probe_images.py
