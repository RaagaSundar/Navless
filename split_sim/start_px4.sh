#!/usr/bin/env bash
# SITL ONLY. Attach to this setup's Gazebo. Never arm or start an automatic pilot.
set -euo pipefail
[[ "${ROS_DOMAIN_ID:-}" == 219 ]] || { echo 'ROS domain must be 219'; exit 2; }
[[ -n "${JETSON_IP:-}" ]] || { echo 'Set JETSON_IP to drone 19 WiFi IPv4'; exit 2; }
PX4_DIR=/root/PX4-Autopilot
BUILD="$PX4_DIR/build/px4_sitl_default"
[[ -x "$BUILD/bin/px4" ]] || { echo 'Build PX4 first'; exit 2; }
export GZ_PARTITION=drone19_split GZ_IP=127.0.0.1
export PX4_SYS_AUTOSTART=4001 PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=dronehacks
export PX4_GZ_MODEL_NAME=dh_quad_0 PX4_SIMULATOR=gz PX4_UXRCE_DDS_PORT=8889
export PX4_PARAM_UXRCE_DDS_AG_IP
PX4_PARAM_UXRCE_DDS_AG_IP="$(python3 -c 'import ipaddress,os; n=int(ipaddress.IPv4Address(os.environ["JETSON_IP"])); print(n if n < 2**31 else n-2**32)')"
export PX4_PARAM_UXRCE_DDS_PTCFG=1
# Explicit isolated SITL DDS domain; never inherit a saved/default domain.
export PX4_PARAM_UXRCE_DDS_DOM_ID=219
# All endpoints use Gazebo /clock. PX4's documented simulation-time setup
# disables conversion to the agent's wall clock; hardware keeps normal sync.
# https://docs.px4.io/v1.16/en/ros2/user_guide (ROS 2 / Gazebo time sync)
export PX4_PARAM_UXRCE_DDS_SYNCT=0
# Match the documented drone-profile estimator path, using *real image SLAM*
# instead of the harness's oracle visual odometry. These affect SITL only.
export PX4_PARAM_EKF2_GPS_CTRL=0 PX4_PARAM_EKF2_EV_CTRL=9
export PX4_PARAM_EKF2_EV_DELAY=60 PX4_PARAM_EKF2_EV_NOISE_MD=0
export PX4_PARAM_EKF2_HGT_REF=2 PX4_PARAM_EKF2_BARO_CTRL=1 PX4_PARAM_EKF2_MAG_TYPE=0
case "${SPLIT_HEIGHT_PROFILE:-range}" in
  range) ;;
  ev-terrain)
    # Fixed launch-frame visual height with independent barometer and actual
    # conditional range/terrain fusion. A sill changes the measured AGL.
    export PX4_PARAM_EKF2_EV_CTRL=11 PX4_PARAM_EKF2_HGT_REF=3
    ;;
  *) echo "unreviewed split height profile" >&2; exit 2 ;;
esac
export PX4_PARAM_EKF2_RNG_CTRL=1 PX4_PARAM_EKF2_RNG_A_HMAX=3.0
export PX4_PARAM_EKF2_RNG_DELAY=5 PX4_PARAM_EKF2_RNG_NOISE=0.01
export PX4_PARAM_EKF2_RNG_POS_X=0 PX4_PARAM_EKF2_RNG_POS_Y=-0.027432 PX4_PARAM_EKF2_RNG_POS_Z=0.03445
export PX4_PARAM_CA_ROTOR0_PX=0.08 PX4_PARAM_CA_ROTOR0_PY=0.16
export PX4_PARAM_CA_ROTOR1_PX=-0.15 PX4_PARAM_CA_ROTOR1_PY=-0.14
export PX4_PARAM_CA_ROTOR2_PX=0.08 PX4_PARAM_CA_ROTOR2_PY=-0.16
export PX4_PARAM_CA_ROTOR3_PX=-0.15 PX4_PARAM_CA_ROTOR3_PY=0.14
RUN="/root/px4-runs/$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$RUN"
ln -sfn "$RUN" /root/px4-current
cd "$RUN"
python3 /split/range_sensor.py > range.log 2>&1 & range_pid=$!
trap 'kill "$range_pid" 2>/dev/null || true' EXIT
"$BUILD/bin/px4" -i 0 -d "$BUILD/etc" > px4.log 2>&1 & px4_pid=$!
trap 'kill "$range_pid" "$px4_pid" 2>/dev/null || true; wait || true' EXIT TERM INT
wait -n "$range_pid" "$px4_pid"
