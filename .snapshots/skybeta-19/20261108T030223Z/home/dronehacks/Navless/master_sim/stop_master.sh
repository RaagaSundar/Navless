#!/usr/bin/env bash
# Stop everything run_master.sh started, then the Gazebo + PX4 sim.
HERE="$(cd "$(dirname "$0")" && pwd)"
LOG="$HOME/dh_sim_logs/master"
for f in "$LOG"/*.pid; do
  [[ -f "$f" ]] || continue
  kill -- -"$(cat "$f")" 2>/dev/null || kill "$(cat "$f")" 2>/dev/null || true
  rm -f "$f"
done
pkill -f "$HERE/sim_core.py" 2>/dev/null || true      # the foreground one, if the run was cut short
LOG_DIR="$HOME/dh_sim_logs" "$(dirname "$HERE")/gazebo_sim/stop_sim.sh"
