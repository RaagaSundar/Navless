#!/usr/bin/env bash
# Master sim: Gazebo + PX4 SITL (gazebo_sim/) + stereo cameras + depthcloud + a flight brain, on real ROS 2.
# Run from WSL Ubuntu-24.04 (ROS 2 Jazzy, Gazebo Harmonic, PX4 v1.16 in ~/PX4-Autopilot):
#
#   master_sim/run_master.sh                         # navless_nav with the drone's launch defaults
#   master_sim/run_master.sh --brain planner         # Favour's live replanner (px4_bridge_node --skybeta-mode)
#   master_sim/run_master.sh --brain none            # bench: perception only, drone sits on the launch mark
#   master_sim/run_master.sh --route route_gate_b.json --gui
#   master_sim/run_master.sh -- -p fly_home:=true    # extra ROS params for the brain (fly_home: pilot re-arms)
#   DH_TUBE_BLACKOUT=stop master_sim/run_master.sh   # dark tube: PX4's vision and the brain both go blind
#   master_sim/run_master.sh -- -p pad_vision:=true  # land on the camera's pad fix (starts pad_observer too)
#
# Runs until the vehicle is back on the ground (or the timeout), prints the score, then stops everything.
# Logs: ~/dh_sim_logs/master/*.log. Run summary: gazebo_sim/runs/<time>-master-<brain>/summary.json.
# While it runs: http://localhost:8081 (stereo depth + cloud), http://localhost:8082 (localization).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(dirname "$HERE")"
BRAIN=navless ROUTE="" GUI="" EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --brain) BRAIN="$2"; shift 2 ;;
    --route) ROUTE="$2"; shift 2 ;;
    --gui) GUI=--gui; shift ;;
    --) shift; EXTRA=("$@"); break ;;
    *) echo "unknown argument $1"; exit 1 ;;
  esac
done
case "$BRAIN" in navless|planner|none) ;; *) echo "--brain navless|planner|none"; exit 1 ;; esac

N="${DH_INSTANCE:-0}"
LOG="$HOME/dh_sim_logs/master"
WS="$HOME/master_sim_ws"            # sim-only interface packages (master_sim/ros_ws)
mkdir -p "$LOG"
set +u
source /opt/ros/jazzy/setup.bash
[[ -f "$HOME/ros2_ws/install/setup.bash" ]] && source "$HOME/ros2_ws/install/setup.bash"   # px4_msgs
set -u
# The system numpy first: a numpy 2.x in ~/.local breaks the system OpenCV and cv_bridge.
export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo "[master] building the sim's VisualSlamStatus stand-in into $WS (once)"
  (mkdir -p "$WS" && cd "$WS" && colcon build --base-paths "$HERE/ros_ws/src" > "$LOG/ws_build.log" 2>&1) \
    || { echo "build failed, see $LOG/ws_build.log"; exit 1; }
fi
set +u; source "$WS/install/setup.bash"; set -u
# WSL: render the cameras on the GPU (Mesa d3d12) rather than llvmpipe on the CPU.
[[ -e /dev/dxg ]] && export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
OBJ="$REPO/gazebo_sim/models/dh_course/meshes/course_visual.obj"     # official course v3 mesh, net removed

python3 "$HERE/make_assets.py" "$LOG/assets" >/dev/null
"$HERE/stop_master.sh" >/dev/null 2>&1 || true
# DH_TUBE_* (if exported) reach sensor_bridge.py through run_sim.sh and sim_core.py, so PX4's vision
# and the brain's odometry go blind in the tube together.
DH_MODEL_SDF="$LOG/assets/dh_quad_stereo.sdf" DH_EXTRA_MODELS="$LOG/assets/models" DH_PX4_PROFILE=drone \
  LOG_DIR="$HOME/dh_sim_logs" "$REPO/gazebo_sim/run_sim.sh" $GUI
export GZ_IP=127.0.0.1 GZ_PARTITION="dh$N"

start() {   # start NAME CMD...: background, own process group, log + pid in $LOG
  local name="$1"; shift
  setsid "$@" > "$LOG/$name.log" 2>&1 &
  echo $! > "$LOG/$name.pid"
}
trap '"$HERE/stop_master.sh"' EXIT

SIMTIME=(--ros-args -p use_sim_time:=true)
start bridge ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$HERE/bridge.yaml"
start stereo_cloud python3 "$REPO/depthcloud/stereo_cloud.py" "${SIMTIME[@]}"
start cloud_localize python3 "$REPO/depthcloud/cloud_localize.py" "${SIMTIME[@]}" -p obj:="$OBJ" \
  -p init_x:=-2.5146 -p init_y:=0.9144
REARM=0
if [[ "$BRAIN" == none ]]; then
  :
elif [[ "$BRAIN" == navless ]]; then
  # The drone's own launch defaults (navless_course.launch.py -> navless.launch.py), from the working
  # tree: edit, re-run.
  read -r -a MISSION <<< "$(python3 "$HERE/launch_params.py" mission ${ROUTE:+route_file:="$ROUTE"})"
  start brain env PYTHONPATH="$REPO/navless_nav:$PYTHONPATH" python3 -c \
    'from navless_nav.course_mission import main; main()' "${SIMTIME[@]}" "${MISSION[@]}" "${EXTRA[@]}"
  [[ " ${EXTRA[*]-} " == *"fly_home:=true"* ]] && REARM=1     # the pilot re-arms for the home leg
  if [[ " ${EXTRA[*]-} " == *"pad_vision:=true"* ]]; then     # camera pad landing: its observer too
    start pad_observer env PYTHONPATH="$REPO/navless_nav:$PYTHONPATH" python3 -c \
      'from navless_nav.pad_observer import main; main()' "${SIMTIME[@]}"       ${DH_PAD_DEBUG_DIR:+-p debug_dir:="$DH_PAD_DEBUG_DIR"}       # frames' floor points, for tuning
  fi
else
  ros2 interface show px4_msgs/msg/TrajectorySetpoint >/dev/null 2>&1 \
    || { echo "px4_msgs not built: colcon build it in ~/ros2_ws first"; exit 1; }
  start brain bash -c "cd '$REPO' && exec python3 -m planner.px4_bridge_node --skybeta-mode --approve-flight \
    --sim-time --prior '$OBJ' ${EXTRA[*]+$(printf '%q ' "${EXTRA[@]}")}"
fi
echo "[master] brain: $BRAIN. Logs in $LOG. Depth view http://localhost:8081, localization http://localhost:8082"
WAIT=60; [[ "$BRAIN" == none ]] && WAIT="${DH_BENCH_S:-90}"
read -r -a ENVELOPE <<< "$(python3 "$HERE/launch_params.py" envelope)"   # flight_interface limits
python3 "$HERE/sim_core.py" --instance "$N" --brain "$BRAIN" --rearm "$REARM" \
  --wait-takeover-s "$WAIT" "${ENVELOPE[@]}" ${DH_SIM_ARGS:-} 2>&1 \
  | tee "$LOG/sim_core.log"
