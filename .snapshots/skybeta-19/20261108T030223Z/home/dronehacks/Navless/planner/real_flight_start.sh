#!/usr/bin/env bash
# Start px4_bridge_node for a real flight. Run on the Jetson host (not inside the container), drone on
# the launch mark, nose north, DISARMED, stack up:
#
#   ssh dronehacks@skybeta-<n>.local
#   bash ~/team_ws/src/Navless/planner/real_flight_start.sh [extra bridge args, e.g. --stop-on-pad]
#
# It replaces the stack's `competitor` window (navless_nav) with the bridge in --skybeta-mode, so there
# is exactly one commander on /skybeta/setpoint/*, and an SSH drop doesn't kill it. The bridge never
# arms: the pilot arms in Altitude, then switches to Position, and flight_interface hands it Offboard.
# Any mode change by the pilot takes control back. Log: /tmp/px4_bridge_flight.log in the container.
#
# Stop it (lands first if it is flying):  bash ~/team_ws/src/Navless/planner/real_flight_kill.sh
# Put navless_nav back afterwards (on the ground only; restarts SLAM/odometry): docker rm -f skybeta-stack
set -euo pipefail
STACK="${SKYBETA_CONTAINER:-skybeta-stack}"
REPO="/root/team_ws/src/Navless"
LOG="/tmp/px4_bridge_flight.log"
PAT='^python3 -u -m planner\.px4_bridge_node'
MESH="gazebo_sim/models/dh_course/meshes/course_visual.obj"
# --planner ego: the planner flown in the FlyX lab. The newer NMPC default stalls in tools/sim_px4_mission.py.
ARGS="--skybeta-mode --approve-flight --planner ego --prior $MESH $*"

die() { echo "NOT STARTED: $1"; exit 1; }

docker ps --format '{{.Names}}' | grep -qx "$STACK" || die "container $STACK is not running"
docker exec "$STACK" tmux list-windows -t skybeta -F '#{window_name}' 2>/dev/null | grep -qx competitor \
  || die "no tmux window skybeta:competitor in $STACK (stack still starting?)"
! docker exec "$STACK" pgrep -f "$PAT" >/dev/null || die "the bridge is already running (stop it: planner/real_flight_kill.sh)"
docker exec "$STACK" test -f "$REPO/$MESH" || die "course mesh $REPO/$MESH missing: pull main into the team repo"

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

CMD="source skybeta_env.sh; [ -f /root/team_ws/install/setup.bash ] && source /root/team_ws/install/setup.bash; \
cd $REPO && python3 -u -m planner.px4_bridge_node $ARGS > $LOG 2>&1; \
echo [real_flight] bridge exited, competitor window idle, log in $LOG; exec sleep infinity"
docker exec "$STACK" bash -c ": > $LOG"
docker exec "$STACK" tmux respawn-pane -k -t skybeta:competitor "bash -lc \"$CMD\""
echo "started in $STACK tmux window skybeta:competitor: python3 -m planner.px4_bridge_node $ARGS"

for _ in $(seq 1 20); do
  sleep 1
  if docker exec "$STACK" grep -q "Bridge started" "$LOG" 2>/dev/null; then
    docker exec "$STACK" tail -n 3 "$LOG"
    cat <<EOF

READY. The bridge streams the drone's own position until Offboard; it never arms.
  1. Wait for '[WAIT_FOR_OFFBOARD] ... prestreaming_current_pose_for_offboard_handover' in the log:
       docker exec $STACK tail -f $LOG
  2. Pilot: Altitude, arm, then Position. Any mode change takes control back.
  3. Stop (lands where it is if flying):  bash $(cd "$(dirname "$0")" && pwd)/real_flight_kill.sh
EOF
    exit 0
  fi
  docker exec "$STACK" pgrep -f "$PAT" >/dev/null || break
done
echo "NOT RUNNING: the bridge did not start. Log:"; docker exec "$STACK" tail -n 30 "$LOG"; exit 1
