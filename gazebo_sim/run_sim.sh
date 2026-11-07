#!/usr/bin/env bash
# Start Gazebo (DroneHacks course) + PX4 SITL with the team quad stand-in on the launch mark, facing north.
#
#   ./run_sim.sh            headless server + PX4 (fastest)
#   ./run_sim.sh --gui      also open the Gazebo window to watch
#
# Environment:
#   DH_INSTANCE=N       run an isolated copy (default 0): GZ_PARTITION=dhN, PX4 -i N, offboard MAVLink:
#                       PX4 listens on 14580+N and sends to 14540+N (GCS link 18570+N -> 14550), logs ~/dh_sim_logsN
#                       (instance 0 logs to ~/dh_sim_logs). The model is named dh_quad_0 in every instance.
#   DH_PX4_PROFILE=     sim    (default) controller tuned for the 50 Hz pos+vel+acc follower in fly_course.py;
#                              ideal estimator (true pose as vision: position, height, velocity and yaw).
#                       stock  the real airframe's controller params (PX4 stock: MPC gains, 3 s takeoff ramp,
#                              12 deg takeoff tilt, 0.7 m/s land, 2 s disarm), same ideal estimator as sim.
#                       drone  stock controller AND the real estimator: sensor_bridge.py delivers vision at 30 Hz with
#                              60-106 ms latency in a start-anchored FRD frame (horizontal position + yaw fused,
#                              EKF2_EV_DELAY 60), plus a downward laser rangefinder as the height reference
#                              (EKF2_HGT_REF 2, conditional range aid, barometer behind it).
#   DH_TUBE_BLACKOUT=stop|freeze  drone profile only: no vision (or the last pose repeated) inside the dark tube;
#                       DH_TUBE_RECOVER_DELAY / DH_TUBE_RESET_JUMP as in dh/tube.py. Export the same values for
#                       fly_navless.py, which blinds the node's odometry identically.
#   PX4_PARAM_<NAME>=   any extra PX4 parameter override (applied every boot)
#
# Leaves gz + PX4 (+ sensor bridge) running in the background. Stop this instance with ./stop_sim.sh (same DH_INSTANCE).
# PX4 local frame: with sim/stock the EKF origin is the world origin, so PX4 local NED = (y, x, -z) of the
# Gazebo world. With drone it is the start pose (like a VIO frame); fly_course.py aligns the frames at startup.
set -euo pipefail
SIM_DIR="$(cd "$(dirname "$0")" && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
N="${DH_INSTANCE:-0}"
PROFILE="${DH_PX4_PROFILE:-sim}"
if [[ "$N" == 0 ]]; then LOG_DIR="${LOG_DIR:-$HOME/dh_sim_logs}"; else LOG_DIR="${LOG_DIR:-$HOME/dh_sim_logs$N}"; fi
GUI=0
[[ "${1:-}" == "--gui" ]] && GUI=1
case "$PROFILE" in sim|stock|drone) ;; *) echo "[run_sim:$N] unknown DH_PX4_PROFILE=$PROFILE (sim, stock or drone)"; exit 1 ;; esac
mkdir -p "$LOG_DIR"

DH_INSTANCE="$N" LOG_DIR="$LOG_DIR" "$SIM_DIR/stop_sim.sh" >/dev/null 2>&1 || true

export GZ_SIM_RESOURCE_PATH="$SIM_DIR/models:$PX4_DIR/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_IP=127.0.0.1          # keep gz-transport on loopback
export GZ_PARTITION="dh$N"      # isolates this instance's topics and services

# Launch mark = (21, 156) in from the SW corner = (-2.515, 0.914) m from the floor centre. Yaw 90 deg = nose north.
POSE_X="${DH_START_X:--2.5146}"
POSE_Y="${DH_START_Y:-0.9144}"

echo "[run_sim:$N] starting gz server (partition $GZ_PARTITION)"
setsid gz sim -s -r "$SIM_DIR/worlds/dronehacks.sdf" > "$LOG_DIR/gz_server.log" 2>&1 &
echo $! > "$LOG_DIR/gz_server.pid"

for i in $(seq 1 60); do
  if gz service -i --service /world/dronehacks/scene/info 2>/dev/null | grep -q "Service providers"; then break; fi
  sleep 0.5
done

MODEL_SDF="$SIM_DIR/models/dh_quad/model.sdf"
if [[ "$PROFILE" == "drone" ]]; then
  # Hide the model's built-in vision feed from PX4: sensor_bridge.py delivers it the way the drone does.
  MODEL_SDF="$LOG_DIR/dh_quad_drone.sdf"
  sed 's#<dimensions>3</dimensions>#<dimensions>3</dimensions><odom_covariance_topic>/dh/odometry_cov_unused</odom_covariance_topic>#' \
    "$SIM_DIR/models/dh_quad/model.sdf" > "$MODEL_SDF"
fi
echo "[run_sim:$N] spawning dh_quad_0"
gz service -s /world/dronehacks/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 10000 \
  --req "sdf_filename: \"$MODEL_SDF\", name: \"dh_quad_0\", allow_renaming: false, pose: {position: {x: $POSE_X, y: $POSE_Y, z: 0.075}, orientation: {x: 0, y: 0, z: 0.70710678, w: 0.70710678}}" \
  > /dev/null

if [[ "$PROFILE" == "drone" ]]; then
  echo "[run_sim:$N] starting sensor bridge (vision + rangefinder)"
  setsid python3 "$SIM_DIR/sensor_bridge.py" --instance "$N" > "$LOG_DIR/sensor_bridge.log" 2>&1 &
  echo $! > "$LOG_DIR/sensor_bridge.pid"
fi

if [[ $GUI == 1 ]]; then
  echo "[run_sim:$N] opening gz gui"
  setsid gz sim -g > "$LOG_DIR/gz_gui.log" 2>&1 &
  echo $! > "$LOG_DIR/gz_gui.pid"
fi

echo "[run_sim:$N] starting PX4 SITL (profile: $PROFILE)"
# --- PX4 parameters (env overrides, applied every boot) ---
export PX4_SYS_AUTOSTART=4001 PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=dronehacks PX4_GZ_MODEL_NAME=dh_quad_0 PX4_SIMULATOR=gz
# Rotor geometry: the real airframe's measured control-allocation values (FRD, metres from the centre of mass)
export PX4_PARAM_CA_ROTOR0_PX=0.08  PX4_PARAM_CA_ROTOR0_PY=0.16
export PX4_PARAM_CA_ROTOR1_PX=-0.15 PX4_PARAM_CA_ROTOR1_PY=-0.14
export PX4_PARAM_CA_ROTOR2_PX=0.08  PX4_PARAM_CA_ROTOR2_PY=-0.16
export PX4_PARAM_CA_ROTOR3_PX=-0.15 PX4_PARAM_CA_ROTOR3_PY=0.14
export PX4_PARAM_MPC_THR_HOVER=0.5
# Ideal GPS-denied EKF (sim/stock): position, height, velocity and yaw from the true pose; no GPS, baro or mag.
export PX4_PARAM_EKF2_GPS_CTRL=0 PX4_PARAM_EKF2_EV_CTRL=15 PX4_PARAM_EKF2_HGT_REF=3 PX4_PARAM_EKF2_BARO_CTRL=0 PX4_PARAM_EKF2_MAG_TYPE=5
export PX4_PARAM_EKF2_EVP_NOISE=0.02 PX4_PARAM_EKF2_EVV_NOISE=0.05 PX4_PARAM_EKF2_EVA_NOISE=0.03
# No RC or GCS in the loop: the mission script is the only commander.
export PX4_PARAM_COM_RC_IN_MODE=4 PX4_PARAM_NAV_RCL_ACT=1 PX4_PARAM_COM_RCL_EXCEPT=4 PX4_PARAM_NAV_DLL_ACT=0
export PX4_PARAM_COM_OF_LOSS_T=${PX4_PARAM_COM_OF_LOSS_T:-0.5} PX4_PARAM_COM_OBL_RC_ACT=4

if [[ "$PROFILE" == "sim" ]]; then
  # Tighter trajectory tracking than PX4's defaults (0.95 / 1.8 / 0.2 / 1.0 / 4.0). Override from the caller's env.
  export PX4_PARAM_MPC_XY_P=${PX4_PARAM_MPC_XY_P:-1.2} PX4_PARAM_MPC_XY_VEL_P_ACC=${PX4_PARAM_MPC_XY_VEL_P_ACC:-4.5}
  export PX4_PARAM_MPC_XY_VEL_D_ACC=${PX4_PARAM_MPC_XY_VEL_D_ACC:-0.3}
  export PX4_PARAM_MPC_Z_P=${PX4_PARAM_MPC_Z_P:-1.5} PX4_PARAM_MPC_Z_VEL_P_ACC=${PX4_PARAM_MPC_Z_VEL_P_ACC:-6.0}
  # Takeoff: PX4's default 3 s climb-rate ramp would make the drone lag the trajectory at liftoff
  export PX4_PARAM_MPC_TKO_RAMP_T=${PX4_PARAM_MPC_TKO_RAMP_T:-0.2}
  # Max tilt (PX4 default 45 deg). The stand-in has thrust-to-weight ~3, so faster profiles can allow more.
  export PX4_PARAM_MPC_TILTMAX_AIR=${PX4_PARAM_MPC_TILTMAX_AIR:-45}
  # PX4 caps tilt at MPC_TILTMAX_LND (12 deg) during takeoff, then raises it at only 0.2 rad/s (~3 s to reach 45 deg).
  # The route climbs vertically before moving off, so allow the full tilt straight away.
  export PX4_PARAM_MPC_TILTMAX_LND=${PX4_PARAM_MPC_TILTMAX_LND:-$PX4_PARAM_MPC_TILTMAX_AIR}
  export PX4_PARAM_MPC_LAND_SPEED=${PX4_PARAM_MPC_LAND_SPEED:-1.0} PX4_PARAM_COM_DISARM_LAND=${PX4_PARAM_COM_DISARM_LAND:-0.5}
else
  # The real airframe's controller params: PX4 stock values.
  export PX4_PARAM_MPC_XY_P=0.95 PX4_PARAM_MPC_XY_VEL_P_ACC=1.8 PX4_PARAM_MPC_XY_VEL_D_ACC=0.2 PX4_PARAM_MPC_XY_VEL_I_ACC=0.4
  export PX4_PARAM_MPC_Z_P=1.0 PX4_PARAM_MPC_Z_VEL_P_ACC=4.0 PX4_PARAM_MPC_TKO_RAMP_T=3.0 PX4_PARAM_MPC_TKO_SPEED=1.5
  export PX4_PARAM_MPC_TILTMAX_AIR=45 PX4_PARAM_MPC_TILTMAX_LND=12 PX4_PARAM_MPC_LAND_SPEED=0.7 PX4_PARAM_COM_DISARM_LAND=2.0
  export PX4_PARAM_MPC_XY_VEL_MAX=12 PX4_PARAM_MPC_Z_VEL_MAX_UP=3 PX4_PARAM_MPC_Z_VEL_MAX_DN=1.5 PX4_PARAM_COM_OF_LOSS_T=1.0
fi
if [[ "$PROFILE" == "drone" ]]; then
  # The real estimator: vision = horizontal position + yaw with its measured delay; laser range as the height
  # reference while low and slow, barometer behind it; magnetometer left on automatic.
  export PX4_PARAM_EKF2_EV_CTRL=9 PX4_PARAM_EKF2_EV_DELAY=${PX4_PARAM_EKF2_EV_DELAY:-60} PX4_PARAM_EKF2_EV_NOISE_MD=0
  export PX4_PARAM_EKF2_EVP_NOISE=0.1 PX4_PARAM_EKF2_EVV_NOISE=0.1 PX4_PARAM_EKF2_EVA_NOISE=0.05
  export PX4_PARAM_EKF2_HGT_REF=2 PX4_PARAM_EKF2_BARO_CTRL=1 PX4_PARAM_EKF2_MAG_TYPE=0
  export PX4_PARAM_EKF2_RNG_CTRL=1 PX4_PARAM_EKF2_RNG_A_HMAX=3.0 PX4_PARAM_EKF2_RNG_DELAY=5 PX4_PARAM_EKF2_RNG_NOISE=0.01
  export PX4_PARAM_EKF2_RNG_POS_X=0 PX4_PARAM_EKF2_RNG_POS_Y=-0.027432 PX4_PARAM_EKF2_RNG_POS_Z=0.03445 PX4_PARAM_EKF2_RNG_QLTY_T=1.0
fi

WORK="$PX4_BUILD/rootfs/$N"
mkdir -p "$WORK"
cd "$WORK"
# PX4 SITL saves every parameter change (including PX4_PARAM_* and MAVLink PARAM_SET) to parameters.bson, so a
# previous run's profile would leak into this one. Start every boot from the airframe defaults + this profile.
rm -f parameters.bson parameters_backup.bson
setsid "$PX4_BUILD/bin/px4" -i "$N" -d "$PX4_BUILD/etc" > "$LOG_DIR/px4.log" 2>&1 &
echo $! > "$LOG_DIR/px4.pid"

for i in $(seq 1 60); do
  if grep -q "Ready for takeoff" "$LOG_DIR/px4.log" 2>/dev/null; then echo "[run_sim:$N] PX4 ready for takeoff"; exit 0; fi
  sleep 0.5
done
echo "[run_sim:$N] PX4 not ready after 30 s; see $LOG_DIR/px4.log (it may still finish starting)"
