#!/usr/bin/env bash
# First-flight tests on skybeta-19, the organisers' way. Run on the Jetson HOST (ssh dronehacks@<drone>).
#
#   bash fly_test.sh install            build the tests into ~/team_ws/flight_tests_ws (skipped by the boot build)
#   bash fly_test.sh hover|yaw|forward  DISARMED ONLY: put that test in the stack's competitor window
#   bash fly_test.sh status             what the competitor window is running, last log lines
#   bash fly_test.sh stop               DISARMED ONLY: competitor window -> idle ("sleep infinity")
#   bash fly_test.sh restore            DISARMED ONLY: competitor window -> the team's normal code (navless_nav)
#   bash fly_test.sh logs               list the flight logs (~/team_ws/flight_logs, one CSV per test run)
#   bash fly_test.sh offset             print the live PX4-vs-cuVSLAM frame offset from the newest log
#
# After `hover|yaw|forward` the window runs flight_interface + the test node, which streams "stay here"
# and waits. Then, props on, drone on the floor and still, cuVSLAM tracking:
#   pilot ARMS (mode switch on Altitude, throttle low) -> pilot switches to POSITION -> the test flies.
# The node never arms and never publishes to /fmu/in/*. The pilot takes over any time (throttle up / mode
# switch / kill). One test per arming: after it lands and disarms (or the pilot takes over), the node is
# done. Run `bash fly_test.sh <test>` again for the next one. Rehearse with PROPS OFF first.
set -uo pipefail
STACK=skybeta-stack
HOST_WS="$HOME/team_ws/flight_tests_ws"
WS=/root/team_ws/flight_tests_ws
SRC_DIR="$(cd "$(dirname "$0")" && pwd)"
ENV='export SKYBETA_UNDERLAY_ONLY=1; source skybeta_env.sh >/dev/null 2>&1'

[ "$(hostname)" = skybeta-19 ] || { echo "not skybeta-19 ($(hostname)): stop"; exit 1; }

armed() {   # 0 = armed or unknown (refuse), 1 = disarmed
  local s
  s=$(docker exec "$STACK" bash -lc "$ENV; timeout 6 ros2 topic echo --once /fmu/out/vehicle_status_v1 2>/dev/null | grep -m1 '^arming_state:'" | awk '{print $2}')
  [ "$s" = "1" ] && return 1
  echo "PX4 arming_state is '${s:-no data}' (1 = disarmed). Refusing: only change this with the drone disarmed and PX4 linked."
  return 0
}

respawn() {   # command, description
  docker exec "$STACK" tmux respawn-pane -k -t skybeta:competitor "$1" && echo "competitor window: $2"
}

case "${1:-}" in
  install)
    mkdir -p "$HOST_WS/src"
    touch "$HOST_WS/COLCON_IGNORE"                    # the boot build (colcon in ~/team_ws) skips this folder
    rm -rf "$HOST_WS/src/navless_flight_tests"
    rsync -a --exclude test --exclude '__pycache__' "$SRC_DIR/" "$HOST_WS/src/navless_flight_tests/" 2>/dev/null \
      || { cp -r "$SRC_DIR" "$HOST_WS/src/navless_flight_tests"; rm -rf "$HOST_WS/src/navless_flight_tests/test"; }
    docker exec "$STACK" bash -lc "$ENV; cd $WS && colcon build --base-paths src --packages-select navless_flight_tests 2>&1 | tail -3"
    docker exec "$STACK" bash -lc "$ENV; source $WS/install/setup.bash; ros2 pkg executables navless_flight_tests"
    ;;
  hover|yaw|forward)
    [ -f "$HOST_WS/install/setup.bash" ] || { echo "not installed: bash $0 install"; exit 1; }
    armed && exit 1
    respawn "bash -lc '$ENV; source $WS/install/setup.bash; echo \"[flight test] $1\"; exec ros2 launch navless_flight_tests $1.launch.py'" \
      "test '$1' (flight_interface + navless_basic_flight). Waiting for the pilot to arm, then Position."
    sleep 8
    docker exec "$STACK" tmux capture-pane -p -t skybeta:competitor | grep -v '^$' | tail -6
    ;;
  status)
    docker exec "$STACK" tmux list-panes -t skybeta:competitor -F '#{pane_current_command}'
    docker exec "$STACK" tmux capture-pane -p -S -40 -t skybeta:competitor | grep -v '^$' | tail -12
    ;;
  stop)
    armed && exit 1
    respawn "sleep infinity" "idle (sleep infinity)"
    ;;
  logs)
    ls -lt "$HOME/team_ws/flight_logs" 2>/dev/null | head -15 || echo "no logs yet"
    echo "copy one to a laptop:  scp dronehacks@<drone>:team_ws/flight_logs/<file>.csv ."
    ;;
  offset)
    f=$(ls -t "$HOME/team_ws/flight_logs"/*.csv 2>/dev/null | head -1)
    [ -n "$f" ] || { echo "no logs yet: run a test first (it logs while waiting, before arming)"; exit 1; }
    echo "$f"
    python3 - "$f" <<'PY'
import csv, sys
rows = [r for r in csv.DictReader(open(sys.argv[1])) if r['off_x'] not in ('', None)]
if not rows:
    sys.exit('no offset rows yet (needs both PX4 position and cuVSLAM odometry)')
last = rows[-50:]
f = lambda k: [float(r[k]) for r in last if r[k] not in ('', None)]
for k in ('off_x', 'off_y', 'off_z', 'off_yaw_deg'):
    v = f(k)
    print(f"{k:12s} now {v[-1]:+.3f}   last {len(v)} rows: min {min(v):+.3f} max {max(v):+.3f}")
r = last[-1]
print("EKF2 fusing: vision pos", r.get('ekf_ev_pos'), " vision yaw", r.get('ekf_ev_yaw'),
      " magnetometer heading", r.get('ekf_mag_hdg'), " | cuVSLAM vo_state", r.get('vo_state'))
PY
    ;;
  restore)
    armed && exit 1
    respawn "bash -lc 'source skybeta_env.sh; exec run_competitor.sh'" "the team's code (COMPETITOR_PKG from drone.env)"
    ;;
  *)
    sed -n "2,18p" "$0"; exit 1 ;;
esac
