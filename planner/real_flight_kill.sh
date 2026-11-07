#!/usr/bin/env bash
# Stop px4_bridge_node started by planner/real_flight_start.sh. Run on the Jetson host.
#
#   bash ~/team_ws/src/Navless/planner/real_flight_kill.sh         # controlled stop
#   bash ~/team_ws/src/Navless/planner/real_flight_kill.sh --now   # stop publishing immediately
#
# Controlled stop (SIGINT): if the bridge is flying (armed + Offboard), it descends where it is until PX4
# disarms on landing, then exits; on the ground, or once the pilot has taken over, it exits at once.
# --now: a second SIGINT, then SIGKILL. In the air, flight_interface holds the last setpoint and the
# pilot must take over (any mode change).
set -uo pipefail
STACK="${SKYBETA_CONTAINER:-skybeta-stack}"
LOG="/tmp/px4_bridge_flight.log"
PAT='^python3 -u -m planner\.px4_bridge_node'
WAIT_S="${KILL_WAIT_S:-60}"
running() { docker exec "$STACK" pgrep -f "$PAT" >/dev/null 2>&1; }

running || { echo "bridge not running"; exit 0; }
if [[ "${1:-}" == --now ]]; then
  docker exec "$STACK" pkill -INT -f "$PAT"; sleep 0.5; docker exec "$STACK" pkill -INT -f "$PAT"
  for _ in 1 2 3 4 5 6; do running || break; sleep 0.5; done
  running && docker exec "$STACK" pkill -KILL -f "$PAT"
  docker exec "$STACK" tail -n 5 "$LOG" 2>/dev/null
  echo "stopped (--now). If it was flying, the pilot has it."
  exit 0
fi

docker exec "$STACK" pkill -INT -f "$PAT"
echo "stop requested: landing where it is if flying (waiting up to ${WAIT_S} s)"
for i in $(seq 1 "$WAIT_S"); do
  if ! running; then
    docker exec "$STACK" tail -n 6 "$LOG" 2>/dev/null
    echo "stopped."
    exit 0
  fi
  (( i % 5 == 0 )) && docker exec "$STACK" tail -n 1 "$LOG" 2>/dev/null
  sleep 1
done
echo "STILL RUNNING after ${WAIT_S} s (still landing, or PX4 hasn't disarmed)."
echo "Pilot: take over with a mode switch and land. Then: $0 --now"
exit 1
