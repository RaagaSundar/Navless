#!/usr/bin/env bash
# Stop the PX4 SITL + Gazebo instance started by run_sim.sh (DH_INSTANCE=N, default 0). Other instances keep running.
N="${DH_INSTANCE:-0}"
if [[ "$N" == 0 ]]; then LOG_DIR="${LOG_DIR:-$HOME/dh_sim_logs}"; else LOG_DIR="${LOG_DIR:-$HOME/dh_sim_logs$N}"; fi
for f in px4 sensor_bridge gz_gui gz_server; do
  pidf="$LOG_DIR/$f.pid"
  [[ -f "$pidf" ]] || continue
  pid="$(cat "$pidf")"
  # each component was started with setsid, so its pid is also its process-group id
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
done
sleep 1
for f in px4 sensor_bridge gz_gui gz_server; do
  pidf="$LOG_DIR/$f.pid"
  [[ -f "$pidf" ]] || continue
  pid="$(cat "$pidf")"
  kill -KILL -- "-$pid" 2>/dev/null || true
  rm -f "$pidf"
done
exit 0
