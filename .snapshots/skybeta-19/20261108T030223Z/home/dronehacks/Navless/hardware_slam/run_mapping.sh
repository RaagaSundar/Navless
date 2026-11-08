#!/usr/bin/env bash
# Host side of handheld-slam.service. All ROS work stays inside the stack container.
# Preserve the XRCE agent and the organisers' checkout. No arming/mode commands.
#
# Settings (environment, else /etc/skybeta/drone.env, else the default):
#
# HANDHELD_SLAM_MODE
#   flight   (default) Leave the stack as the organisers run it: the competitor window runs
#            run_competitor.sh (COMPETITOR_PKG, i.e. the team's flight code) and SLAM runs stock.
#            odom_gate.py runs in its own tmux window `gate`, publishing /mapping/validated_odometry
#            (used by depthcloud/cloud_localize.py and the planner); this service restarts it if it dies.
#            If an earlier mapping session changed the windows, they're put back (restarting SLAM resets
#            odometry: only switch modes on the ground).
#   mapping  Hand-carried map making: the competitor window becomes `sleep infinity`, SLAM runs
#            localization + mapping with visualisation, the PX4 bridge is fed from the gate, and
#            map_server.py (the viewer, ~50% CPU) runs. NEVER on a drone that's going to fly:
#            the team's flight code cannot start.
#
# HANDHELD_SLAM_PX4_GATE   (flight mode only)
#   0  (default) PX4's vision feed is the stack's stock bridge: every cuVSLAM sample, even while
#      tracking is lost (a frozen pose keeps arriving, PX4 thinks it's stationary).
#   1  PX4's vision feed goes through the gate: nothing while cuVSLAM is lost or samples are stale, so
#      PX4 knows it's blind and dead-reckons. The bridge is only rerouted once the gate is passing
#      samples. Costs: team code in the VIO -> PX4 path, the gate is CPU-sensitive (keep stereo at
#      <= 3 Hz, no map viewer), and with the magnetometer on, PX4 swings its heading ~90 deg after a
#      0.5 s vision gap (EKF2_MAG_TYPE=5 avoids that). A team decision.
#
# HANDHELD_SLAM_DRYRUN=1  print what would change, change nothing, exit.
set -euo pipefail
name=skybeta-stack
env_file=/etc/skybeta/drone.env
setting() {   # NAME DEFAULT: environment, else drone.env, else DEFAULT
  local v="${!1:-}"
  [ -n "$v" ] || v="$(sed -n "s/^$1=//p" "$env_file" 2>/dev/null | tail -n1 | tr -d "\"' ")"
  echo "${v:-$2}"
}
mode="$(setting HANDHELD_SLAM_MODE flight)"
px4_gate="$(setting HANDHELD_SLAM_PX4_GATE 0)"
case "$mode" in
  flight|mapping) ;;
  *) echo "[handheld-slam] unknown HANDHELD_SLAM_MODE='$mode' (use flight or mapping)" >&2; exit 1 ;;
esac
case "$px4_gate" in
  0|1) ;;
  *) echo "[handheld-slam] unknown HANDHELD_SLAM_PX4_GATE='$px4_gate' (use 0 or 1)" >&2; exit 1 ;;
esac
echo "[handheld-slam] mode: $mode, PX4 through the gate: $px4_gate"
# The stack creates its windows in order: agent, slam, bridge, competitor. Wait for the last one.
while ! docker exec "$name" tmux has-session -t skybeta 2>/dev/null; do sleep 2; done
while ! docker exec "$name" tmux list-windows -t skybeta -F '#{window_name}' | grep -qx competitor; do sleep 2; done
exec docker exec -e HANDHELD_SLAM_MODE="$mode" -e HANDHELD_SLAM_PX4_GATE="$px4_gate" \
  -e HANDHELD_SLAM_DRYRUN="${HANDHELD_SLAM_DRYRUN:-0}" "$name" \
  bash -lc 'exec bash /root/team_ws/handheld_slam/container_mode.sh'
