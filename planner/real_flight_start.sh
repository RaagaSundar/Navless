#!/usr/bin/env bash
# Start a flight script for a real flight. Run on the Jetson host (not inside the container), drone on
# the launch mark, nose north, DISARMED, stack up:
#
#   ssh dronehacks@skybeta-<n>.local
#   bash ~/team_ws/src/Navless/planner/real_flight_start.sh                          # px4_bridge_node
#   bash ~/team_ws/src/Navless/planner/real_flight_start.sh --stop-on-pad            # ... with bridge args
#   bash ~/team_ws/src/Navless/planner/real_flight_start.sh mission/square_test.py   # any repo script + args
#
# It runs the script in the stack's `competitor` window in place of navless_nav, so there is exactly one
# commander on /skybeta/setpoint/*, and an SSH drop doesn't kill it. Its PID goes to /tmp/flight.pid and
# its output to /tmp/flight.log (both inside the container). The pilot arms: Altitude, arm, then Position;
# any mode change takes control back.
#
# A script run this way must: command through /skybeta/setpoint/*, never wait for keyboard input, treat
# SIGINT/SIGTERM as "land, then exit" (rclpy.init(signal_handler_options=SignalHandlerOptions.NO), see
# mission/square_test.py), and exit once disarmed.
#
# Stop it (lands first if it is flying):  bash ~/team_ws/src/Navless/planner/real_flight_kill.sh
# Put navless_nav back afterwards (on the ground only; restarts SLAM/odometry): docker rm -f skybeta-stack
set -euo pipefail
STACK="${SKYBETA_CONTAINER:-skybeta-stack}"
REPO="/root/team_ws/src/Navless"
LOG="/tmp/flight.log"
PIDF="/tmp/flight.pid"
MESH="dronehacks-course-asbuilt.obj"
if [[ "${1:-}" == *.py ]]; then
  SCRIPT="$1"; shift
  RUN="python3 -u $SCRIPT $*"
else
  SCRIPT="$MESH"   # checked below: the bridge needs the course mesh
  # EGO B-spline planner backed by the as-built course scan.
  RUN="python3 -u -m planner.px4_bridge_node --skybeta-mode --approve-flight --prior $MESH $*"
fi
running() { docker exec "$STACK" bash -c "p=\$(cat $PIDF 2>/dev/null) && kill -0 \"\$p\" 2>/dev/null"; }
die() { echo "NOT STARTED: $1"; exit 1; }

docker ps --format '{{.Names}}' | grep -qx "$STACK" || die "container $STACK is not running"
docker exec "$STACK" tmux list-windows -t skybeta -F '#{window_name}' 2>/dev/null | grep -qx competitor \
  || die "no tmux window skybeta:competitor in $STACK (stack still starting?)"
! running || die "a flight script is already running (stop it: planner/real_flight_kill.sh)"
docker exec "$STACK" test -f "$REPO/$SCRIPT" || die "$REPO/$SCRIPT not found: pull main into the team repo"

# Swapping commanders while armed would cut navless_nav off mid-flight. (`ros2 topic echo` truncates
# long strings, so read flight_interface's JSON state with rclpy.)
armed=$(docker exec -i "$STACK" bash -lc "source skybeta_env.sh >/dev/null 2>&1; \
  [ -f /root/team_ws/install/setup.bash ] && source /root/team_ws/install/setup.bash; python3 -" 2>/dev/null <<'EOF' | tail -1
import json, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
rclpy.init()
node = rclpy.create_node("real_flight_start_state_check")
got = []
node.create_subscription(String, "/skybeta/state", lambda m: got.append(m.data), qos_profile_sensor_data)
t0 = time.time()
while not got and time.time() - t0 < 5.0:
    rclpy.spin_once(node, timeout_sec=0.2)
print(json.loads(got[-1]).get("armed") if got else "unknown")
EOF
)
case "$armed" in
  False) ;;
  True) die "the drone is ARMED: land and disarm first" ;;
  *) [[ "${FORCE:-}" == 1 ]] || die "can't read /skybeta/state to confirm the drone is disarmed (FORCE=1 to start anyway)"
     echo "WARN: /skybeta/state unreadable; starting anyway (FORCE=1)" ;;
esac

# The subshell writes its own PID, then execs the script in its place: the PID is the script's, and the
# script stays in the foreground (Ctrl-C in the window reaches it too). \\\$BASHPID survives the host and
# tmux's sh -c, and reaches bash as $BASHPID.
CMD="source skybeta_env.sh; [ -f /root/team_ws/install/setup.bash ] && source /root/team_ws/install/setup.bash; \
cd $REPO && (echo \\\$BASHPID > $PIDF; exec $RUN) > $LOG 2>&1; rm -f $PIDF; \
echo [real_flight] script exited, competitor window idle, log in $LOG; exec sleep infinity"
docker exec "$STACK" bash -c ": > $LOG; rm -f $PIDF"
docker exec "$STACK" tmux respawn-pane -k -t skybeta:competitor "bash -lc \"$CMD\""
echo "started in $STACK tmux window skybeta:competitor: $RUN"

sleep 5
if running; then
  docker exec "$STACK" tail -n 5 "$LOG"
  cat <<EOF

RUNNING. Watch it:  docker exec $STACK tail -f $LOG
  Pilot: Altitude, arm, then Position. Any mode change takes control back.
  Stop (lands where it is if flying):  bash $(cd "$(dirname "$0")" && pwd)/real_flight_kill.sh
EOF
  exit 0
fi
echo "NOT RUNNING: the script exited within 5 s. Log:"; docker exec "$STACK" tail -n 30 "$LOG"; exit 1
