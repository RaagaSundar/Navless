#!/usr/bin/env bash
# Oracle use is limited to range_sensor.py's noisy laser sensor model.
# No truth odometry producer or flight controller is launched.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /root/px4_agent_ws/install/setup.bash
export ROS_DOMAIN_ID=220 ROS_LOCALHOST_ONLY=1 GZ_PARTITION=local_course GZ_IP=127.0.0.1
export GZ_SIM_RESOURCE_PATH=/course-state/assets/models:/workspace/Navless/gazebo_sim/models
export RUN=/course-state/runs/$(date -u +%Y%m%dT%H%M%SZ)
mkdir -p "$RUN"
ln -sfn "$RUN" /course-state/current
python3 /local_course/make_assets.py /course-state/assets
python3 /local_course/sim_fixture.py
pids=()
cleanup() { for pid in "${pids[@]}"; do kill -- "-$pid" 2>/dev/null || true; done; wait || true; }
trap cleanup EXIT TERM INT
# Runtime-only world copy: cap wall execution speed; simulation step and sensor rates remain identical.
python3 - "$RUN" <<'PYCODE'
import json,math,os,sys,xml.etree.ElementTree as ET
from pathlib import Path
run=Path(sys.argv[1]); value=float(os.environ.get('COURSE_RTF','0.5'))
if not math.isfinite(value) or not 0 < value <= 1: raise ValueError('COURSE_RTF must be >0 and <=1')
src=Path('/workspace/Navless/gazebo_sim/worlds/dronehacks.sdf').read_text()
root=ET.fromstring(src); physics=root.find('./world/physics')
old=physics.find('real_time_factor').text
src=src.replace(f'<real_time_factor>{old}</real_time_factor>',f'<real_time_factor>{value}</real_time_factor>',1)
(run/'runtime-world.sdf').write_text(src)
check=ET.parse(run/'runtime-world.sdf').find('./world/physics')
assert float(check.find('real_time_factor').text)==value
(run/'physics-settings.json').write_text(json.dumps({'requested_real_time_factor':value,'runtime_sdf_real_time_factor':float(check.find('real_time_factor').text),'max_step_size':float(check.find('max_step_size').text),'real_time_update_rate':float(check.find('real_time_update_rate').text),'source':'runtime copy; original world and sensor rates unchanged'},indent=2))
PYCODE
setsid gz sim -s -r "$RUN/runtime-world.sdf" > "$RUN/gazebo.log" 2>&1 & pids+=("$!")
for i in {1..120}; do
 gz service -i --service /world/dronehacks/create 2>/dev/null | grep -q 'Service providers' && break
 sleep 0.5
done
gz service -s /world/dronehacks/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 10000 --req "$(cat "$RUN/spawn-request.txt")" > "$RUN/spawn.log"
setsid ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/workspace/Navless/master_sim/bridge.yaml > "$RUN/ros_bridge.log" 2>&1 & pids+=("$!")
setsid MicroXRCEAgent udp4 -p 8888 > "$RUN/xrce.log" 2>&1 & pids+=("$!")
setsid python3 /local_course/range_sensor.py > "$RUN/range.log" 2>&1 & pids+=("$!")
export PX4_SYS_AUTOSTART=4001 PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=dronehacks PX4_GZ_MODEL_NAME=dh_quad_0 PX4_SIMULATOR=gz
# Agent and ROS consumers share /clock simulation time; preserve PX4 boot timestamps.
export PX4_PARAM_UXRCE_DDS_SYNCT=0
export PX4_UXRCE_DDS_PORT=8888 PX4_PARAM_UXRCE_DDS_AG_IP=2130706433 PX4_PARAM_UXRCE_DDS_PTCFG=1 PX4_PARAM_UXRCE_DDS_DOM_ID=220
# Existing simulated sensor profile; does not alter any real aircraft settings.
export PX4_PARAM_EKF2_GPS_CTRL=0 PX4_PARAM_EKF2_EV_CTRL=9 PX4_PARAM_EKF2_EV_DELAY=60 PX4_PARAM_EKF2_EV_NOISE_MD=0
export PX4_PARAM_EKF2_HGT_REF=2 PX4_PARAM_EKF2_BARO_CTRL=1 PX4_PARAM_EKF2_MAG_TYPE=0
export PX4_PARAM_EKF2_RNG_CTRL=1 PX4_PARAM_EKF2_RNG_A_HMAX=3.0 PX4_PARAM_EKF2_RNG_DELAY=5 PX4_PARAM_EKF2_RNG_NOISE=0.01
export PX4_PARAM_EKF2_RNG_POS_X=0 PX4_PARAM_EKF2_RNG_POS_Y=-0.027432 PX4_PARAM_EKF2_RNG_POS_Z=0.03445
# Match the drone-profile allocation geometry, in PX4 body FRD metres.
export PX4_PARAM_CA_ROTOR0_PX=0.08 PX4_PARAM_CA_ROTOR0_PY=0.16
export PX4_PARAM_CA_ROTOR1_PX=-0.15 PX4_PARAM_CA_ROTOR1_PY=-0.14
export PX4_PARAM_CA_ROTOR2_PX=0.08 PX4_PARAM_CA_ROTOR2_PY=-0.16
export PX4_PARAM_CA_ROTOR3_PX=-0.15 PX4_PARAM_CA_ROTOR3_PY=0.14
cd "$RUN"
setsid /root/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i 0 -d /root/PX4-Autopilot/build/px4_sitl_default/etc > px4.log 2>&1 & pids+=("$!")
setsid bash /local_course/station.sh > "$RUN/station-start.log" 2>&1 & pids+=("$!")
# Software camera rendering uses Xvfb; GUI is opt-in to spare CPU.
if [[ "${COURSE_GUI:-0}" == 1 ]]; then
 setsid gz sim -g > gui.log 2>&1 & pids+=("$!")
fi
wait -n "${pids[@]}"
