#!/usr/bin/env bash
# Stationary camera/SLAM bench. Does not run the automatic master_sim pilot.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
export GZ_PARTITION=drone19_split GZ_IP=127.0.0.1
export GZ_SIM_RESOURCE_PATH="/root/assets/models:/workspace/Navless/gazebo_sim/models"
world=/workspace/Navless/gazebo_sim/worlds/dronehacks.sdf
case "${SPLIT_APPEARANCE:-original}" in
  original) ;;
  portable-neutral-v1|portable-open-net-v2)
    world=/split-appearance/worlds/dronehacks-synthetic.sdf
    export GZ_SIM_RESOURCE_PATH="/split-appearance/models:$GZ_SIM_RESOURCE_PATH"
    ;;
  m3-bounded)
    world=/split-m3-fixture/assets/worlds/dronehacks-synthetic.sdf
    export GZ_SIM_RESOURCE_PATH="/split-m3-fixture/assets/models:$GZ_SIM_RESOURCE_PATH"
    ;;
  *) echo 'unknown portable appearance profile' >&2; exit 2 ;;
esac
mkdir -p /root/logs
python3 /split/make_assets.py /root/assets
if [[ "${SPLIT_APPEARANCE:-original}" == original ]]; then
  python3 - "$world" > /root/render-profile.json <<'PYMETA'
import hashlib,json,sys
from pathlib import Path
p=Path(sys.argv[1]);print(json.dumps({'profile':'original','world':str(p),'world_sha256':hashlib.sha256(p.read_bytes()).hexdigest()}))
PYMETA
elif [[ "${SPLIT_APPEARANCE:-original}" == m3-bounded ]]; then
  python3 /split/m3_bench/scene.py verify /split-m3-fixture \
    --expected-sha256 "${SPLIT_M3_BUNDLE_SHA256:?reviewed M3 bundle required}" > /root/render-profile.json
else
  python3 /workspace/Navless/appearance_profile/verify.py --overlay /split-appearance \
    --manifest-sha256 "${APPEARANCE_MANIFEST_SHA256:?reviewed appearance manifest required}" \
    --world "$world" --resource-path "$GZ_SIM_RESOURCE_PATH" \
    --camera /root/assets/dh_quad_stereo.sdf > /root/render-profile.json
fi
pids=()
cleanup() { for pid in "${pids[@]}"; do kill -- "-$pid" 2>/dev/null || true; done; wait || true; }
trap cleanup EXIT TERM INT
setsid gz sim -s -r "$world" > /root/logs/gazebo.log 2>&1 & pids+=("$!")
for i in {1..60}; do
  gz service -i --service /world/dronehacks/create 2>/dev/null | grep -q 'Service providers' && break
  sleep 0.5
done
gz service -s /world/dronehacks/create --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean --timeout 10000 \
  --req 'sdf_filename: "/root/assets/dh_quad_stereo.sdf", name: "dh_quad_0", allow_renaming: false, pose: {position: {x: -2.5146, y: 0.9144, z: 0.075}, orientation: {z: 0.70710678, w: 0.70710678}}'
setsid ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=/workspace/Navless/master_sim/bridge.yaml > /root/logs/bridge.log 2>&1 & pids+=("$!")
setsid python3 /split/sensor_link.py laptop > /root/logs/sensor_link.log 2>&1 & pids+=("$!")
if [[ "${SPLIT_INERTIAL:-0}" == 1 ]]; then
  setsid python3 -u /split/imu_link.py sender > /root/logs/imu-sender.log 2>&1 & pids+=("$!")
fi
setsid gz sim -g > /root/logs/gui.log 2>&1 & pids+=("$!")
wait -n "${pids[@]}"
