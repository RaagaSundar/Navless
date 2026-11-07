#!/usr/bin/env bash
# Stop the flight script started by planner/real_flight_start.sh (any script). Run on the Jetson host.
#
#   bash ~/team_ws/src/Navless/planner/real_flight_kill.sh         # controlled stop
#   bash ~/team_ws/src/Navless/planner/real_flight_kill.sh --now   # stop publishing immediately
#
# Controlled stop: SIGINT to the PID in /tmp/flight.pid. Scripts that follow the start script's rules
# (px4_bridge_node, mission/square_test.py) then descend where they are until PX4 disarms, and exit; on
# the ground, or once the pilot has taken over, they exit at once. --now: a second SIGINT, then SIGKILL.
# In the air, flight_interface holds the last setpoint and the pilot must take over (any mode change).
set -uo pipefail
STACK="${SKYBETA_CONTAINER:-skybeta-stack}"
LOG="/tmp/flight.log"
PIDF="/tmp/flight.pid"
WAIT_S="${KILL_WAIT_S:-60}"
running() { docker exec "$STACK" bash -c "p=\$(cat $PIDF 2>/dev/null) && kill -0 \"\$p\" 2>/dev/null"; }
sig() { docker exec "$STACK" bash -c "p=\$(cat $PIDF 2>/dev/null) && kill -$1 \"\$p\" 2>/dev/null"; }

running || { echo "no flight script running"; exit 0; }
if [[ "${1:-}" == --now ]]; then
  sig INT; sleep 0.5; sig INT
  for _ in 1 2 3 4 5 6; do running || break; sleep 0.5; done
  running && sig KILL
  docker exec "$STACK" tail -n 5 "$LOG" 2>/dev/null
  echo "stopped (--now). If it was flying, the pilot has it."
  exit 0
fi

sig INT
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
