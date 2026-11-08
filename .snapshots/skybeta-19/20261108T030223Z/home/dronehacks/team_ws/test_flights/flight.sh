#!/usr/bin/env bash
# Test flights on skybeta-19: Skybeta interface only. Pilot arms, grants control, takes back control and lands.
# Run on the Jetson HOST. Files: ~/team_ws/test_flights (= /root/team_ws/test_flights in the stack container).
# Full runbook: README.md next to this file. NOT flight-tested on a real aircraft.
#
#   bash ~/team_ws/test_flights/flight.sh stop-navless            # 1. drone DISARMED: stop navless_nav (the other controller)
#   bash ~/team_ws/test_flights/flight.sh health                  # 2. terminal A: SLAM health adapter, leave it running
#   bash ~/team_ws/test_flights/flight.sh check                   # 3. terminal B: 30 s passive preflight, must say exit 0
#   bash ~/team_ws/test_flights/flight.sh start hover|yaw|forward # 4. terminal B: Pilot approves; node waits for RC handover.
#   bash ~/team_ws/test_flights/flight.sh kill                    # any time: stops the script. NOT a landing: PILOT LANDS on RC.
#   bash ~/team_ws/test_flights/flight.sh restore-navless         # afterwards, on the ground: navless_nav back
#
# hover   = examples/hover_land.py             climb 1 m, hold 10 s, pilot lands
# yaw     = examples/hover_yaw_180_land.py     climb 1 m, yaw 180 deg clockwise at 10 deg/s, pilot lands
# forward = examples/forward_2m_return_land.py climb 1 m, 2 m forward at 0.4 m/s, hold, return, pilot lands
set -uo pipefail
STACK="${SKYBETA_CONTAINER:-skybeta-stack}"
DIR=/root/team_ws/test_flights
PAT='examples/(hover_land|hover_yaw_180_land|forward_2m_return_land)\.py'
ENV="export SKYBETA_UNDERLAY_ONLY=1; source skybeta_env.sh >/dev/null 2>&1; cd $DIR || exit 1; \
test \"\${ROS_DOMAIN_ID:-}\" = 19 || { echo \"ROS_DOMAIN_ID is '\${ROS_DOMAIN_ID:-unset}', not 19: stop\"; exit 1; }; \
export DRONE_PARAMS_FILE=$DIR/config/jetson_test_flights.yaml"

# A missing bind mount is an installation fault; never copy an old launcher/payload as fallback.
docker exec "$STACK" test -f "$DIR/examples/skybeta_test.py" || { echo "Skybeta test payload missing"; exit 1; }

others() { docker exec "$STACK" pgrep -af 'course_mission|navless_course|px4_bridge_node|mission/mission.py|square_test.py|navless_basic_flight|navless_flight_tests/.*/basic_flight' | grep -v pgrep; }

case "${1:-}" in
  interface)
    if others; then echo "Another controller is present; stop here."; exit 1; fi
    if docker exec "$STACK" pgrep -f "[s]kybeta_flight_interface/.*/flight_interface|[c]ompetitor.launch.py" >/dev/null; then echo "Interface/launch already running"; exit 1; fi
    exec docker exec -it "$STACK" bash -lc "$ENV; exec ros2 launch competitor_example competitor.launch.py competitor:=false auto_offboard_on_arm:=true allow_mode_request:=false min_altitude_m:=0.3 max_altitude_m:=2.5 max_speed_mps:=1.0" ;;
  stop-navless)
    if docker exec "$STACK" pgrep -f '[n]avless_flight_tests/.*/basic_flight' >/dev/null; then
      echo "Another test process owns the competitor pane. Coordinate with its pilot; leaving it running."; exit 1
    fi
    docker exec "$STACK" tmux send-keys -t skybeta:competitor C-c
    sleep 3
    docker exec "$STACK" tmux respawn-pane -k -t skybeta:competitor "sleep infinity"
    if others; then echo "STILL RUNNING (above): stop it before flying"; exit 1; fi
    echo "navless_nav stopped; competitor window idle." ;;
  health)
    docker exec "$STACK" pgrep -f "[j]etson_slam_health.py" >/dev/null && { echo "Health adapter already running: do not start another."; exit 1; }
    exec docker exec -it "$STACK" bash -lc "$ENV; exec python3 jetson_slam_health.py --ros-args --params-file \$DRONE_PARAMS_FILE" ;;
  check)
    exec docker exec -it "$STACK" bash -lc "$ENV; id=\$(date -u +%Y%m%dT%H%M%SZ)-jetson-slam-preflight; mkdir -p evidence/\$id; \
python3 check_odometry.py --seconds 30 --ros-args --params-file \$DRONE_PARAMS_FILE > evidence/\$id/preflight.jsonl 2>&1; rc=\$?; \
cat evidence/\$id/preflight.jsonl; echo \"Checker exit status: \$rc (0 = ready; anything else = stop and read the reason)\"; exit \$rc" ;;
  start)
    case "${2:-}" in
      hover) s=hover_land.py ;; yaw) s=hover_yaw_180_land.py ;; forward) s=forward_2m_return_land.py ;;
      *) echo "usage: $0 start hover|yaw|forward"; exit 2 ;;
    esac
    if others; then echo "NOT STARTED: another controller is running (above). Coordinate with the controller owner first."; exit 1; fi
    if docker exec "$STACK" pgrep -f "$PAT" >/dev/null; then echo "NOT STARTED: a test flight is already running"; exit 1; fi
    docker exec "$STACK" pgrep -f jetson_slam_health.py >/dev/null || { echo "NOT STARTED: run '$0 health' in another terminal first"; exit 1; }
    echo "examples/$s: pilot arms in Altitude after READY, then selects Position. Take back control and land on RC."
    read -r -p "Pilot approved this exact plan? Type FLY: " ok; [[ "$ok" == FLY ]] || { echo "not started"; exit 1; }
    exec docker exec -it "$STACK" bash -lc "$ENV; exec python3 examples/$s --ros-args --params-file \$DRONE_PARAMS_FILE" ;;
  kill)
    docker exec "$STACK" pkill -INT -f "$PAT" && echo "SIGINT sent." || echo "no test flight running"
    echo "This stops the test setpoints. The interface can hold its last pose. PILOT: take RC control and land."
    for _ in 1 2 3 4 5 6; do docker exec "$STACK" pgrep -f "$PAT" >/dev/null || { echo "script exited."; exit 0; }; sleep 0.5; done
    docker exec "$STACK" pkill -KILL -f "$PAT"; echo "script killed (did not exit on SIGINT)." ;;
  restore-navless)
    docker exec "$STACK" pgrep -f "[c]ompetitor.launch.py" >/dev/null && { echo "Stop the standalone interface on the ground before restoring navless."; exit 1; }
    docker exec "$STACK" pgrep -f "$PAT" >/dev/null && { echo "a test flight is still running: kill it first"; exit 1; }
    docker exec "$STACK" tmux respawn-pane -k -t skybeta:competitor "bash -lc 'source skybeta_env.sh; exec run_competitor.sh'"
    echo "navless_nav restarted in the competitor window (it never arms by itself)." ;;
  *) sed -n '2,17p' "$0"; exit 2 ;;
esac
