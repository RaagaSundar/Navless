#!/usr/bin/env bash
# Inside skybeta-stack, started by run_mapping.sh with HANDHELD_SLAM_MODE, HANDHELD_SLAM_PX4_GATE and
# HANDHELD_SLAM_DRYRUN. See run_mapping.sh for what they mean. Idempotent: a window that's already
# right is left alone, so systemd restarting the service (Restart=always) never disturbs a running stack.
set -eo pipefail
source skybeta_env.sh >/dev/null 2>&1 || true
mode="${HANDHELD_SLAM_MODE:-flight}"
px4_gate="${HANDHELD_SLAM_PX4_GATE:-0}"
dry="${HANDHELD_SLAM_DRYRUN:-0}"
DIR="$(cd "$(dirname "$0")" && pwd)"
S=skybeta
GATED=/mapping/validated_odometry
log() { echo "[handheld-slam:$mode] $*"; }
start_cmd() { tmux display-message -p -t "$S:$1" '#{pane_start_command}'; }
has_window() { tmux list-windows -t "$S" -F '#{window_name}' | grep -qx "$1"; }
pane_dead() { [ "$(tmux display-message -p -t "$S:$1" '#{pane_dead}')" = 1 ]; }
respawn() {   # window, command, reason
  if [ "$dry" = 1 ]; then
    log "DRY RUN: would restart window '$1' ($3) with: $2"
  else
    log "restarting window '$1': $3"
    tmux respawn-pane -k -t "$S:$1" "$2"
  fi
}
slam_mapping_on() { ros2 param get /visual_slam_node enable_localization_n_mapping 2>/dev/null | grep -q True; }
bridge_gated() { ros2 param get /px4_odom_bridge input_topic 2>/dev/null | grep -q "$GATED"; }
gate_passing() {   # the gate is up and has accepted samples within the last few seconds
  local a1 a2
  a1="$(timeout 4 ros2 topic echo --once --field data /mapping/gate_status 2>/dev/null | grep -o '"accepted": *[0-9]*' | grep -o '[0-9]*$')" || return 1
  sleep 2
  a2="$(timeout 4 ros2 topic echo --once --field data /mapping/gate_status 2>/dev/null | grep -o '"accepted": *[0-9]*' | grep -o '[0-9]*$')" || return 1
  [ -n "$a1" ] && [ -n "$a2" ] && [ "$a2" -gt "$a1" ]
}

# The gate in its own window (not a child of this service), so stopping or restarting the service never
# cuts PX4's vision when PX4 is fed through it. `python3 -u` keeps its command line different from
# mapping mode's child gate, which stop_mapping.sh kills.
GATE_CMD="bash -lc 'export SKYBETA_UNDERLAY_ONLY=1; source skybeta_env.sh; exec python3 -u $DIR/odom_gate.py'"
ensure_gate() {
  if ! has_window gate; then
    if [ "$dry" = 1 ]; then log "DRY RUN: would open window 'gate': $GATE_CMD"
    else log "opening window 'gate' (odom_gate.py -> $GATED)"; tmux new-window -d -t "$S" -n gate "$GATE_CMD"; fi
  elif pane_dead gate; then
    respawn gate "$GATE_CMD" "the gate had exited"
  fi
}

if [ "$mode" = flight ]; then
  # Put back anything a mapping session changed, using the stack's own window commands
  # (container_scripts/run_full_stack.sh).
  if start_cmd competitor | grep -q 'sleep infinity'; then
    respawn competitor "bash -lc 'source skybeta_env.sh; exec run_competitor.sh'" \
      "it was 'sleep infinity' from a mapping session; the team's flight code (COMPETITOR_PKG=${COMPETITOR_PKG:-?}) never started"
  else
    log "competitor window untouched"
  fi
  if slam_mapping_on; then
    respawn slam "bash -lc 'source skybeta_env.sh; exec run_slam.sh ${CAMERA_X:-0.1053} ${CAMERA_Y:-0} ${CAMERA_Z:-0.0304} ${CAMERA_ROLL:-0} ${CAMERA_PITCH:-0} ${CAMERA_YAW:-0}'" \
      "SLAM is in localization+mapping mode from a mapping session (loop closures can jump the pose)"
  else
    log "slam window untouched"
  fi
  ensure_gate
  if [ "$px4_gate" = 1 ]; then
    if bridge_gated; then
      log "bridge already feeds PX4 from the gate"
    elif [ "$dry" = 1 ]; then
      log "DRY RUN: would wait for the gate to pass samples, then restart window 'bridge' with input_topic:=$GATED"
    else
      ok=0
      for _ in 1 2 3 4 5 6; do gate_passing && { ok=1; break; }; sleep 2; done
      if [ "$ok" = 1 ]; then
        respawn bridge "bash -lc 'export SKYBETA_UNDERLAY_ONLY=1; source skybeta_env.sh; exec ros2 launch oakd_vslam_bringup px4_bridge.launch.py input_topic:=$GATED'" \
          "HANDHELD_SLAM_PX4_GATE=1: feed PX4 through the gate (it's passing samples)"
      else
        log "WARNING: the gate isn't passing samples (cuVSLAM not tracking? CPU starved?), so PX4 stays on the stock feed. The service retries on its next restart."
      fi
    fi
  elif bridge_gated; then
    respawn bridge "bash -lc 'source skybeta_env.sh; exec run_bridge.sh'" \
      "HANDHELD_SLAM_PX4_GATE=0: PX4 back on the stack's stock vision feed"
  else
    log "bridge window untouched (stock PX4 feed)"
  fi
  [ "$dry" = 1 ] && { log "DRY RUN: would now watch the 'gate' window and restart it if it exits"; exit 0; }
  log "watching the 'gate' window"
  while true; do
    sleep 5
    if ! has_window gate || pane_dead gate; then
      log "the gate window had stopped$([ "$px4_gate" = 1 ] && echo ' (PX4 had no vision feed!)')"
      ensure_gate
    fi
  done
fi

# mapping mode: the original hand-carried mapper (its own gate runs as a child below)
log "mapping mode: the competitor window will NOT run the team's flight code"
if has_window gate; then
  if [ "$dry" = 1 ]; then log "DRY RUN: would close the flight-mode 'gate' window (mapping runs its own gate)"
  else log "closing the flight-mode 'gate' window (mapping runs its own gate)"; tmux kill-window -t "$S:gate"; fi
fi
respawn competitor "sleep infinity" "the walking mapper has no reason to run a mission or flight heartbeat"
if ! slam_mapping_on; then
  respawn slam "bash -lc 'source skybeta_env.sh; exec ros2 launch oakd_vslam_bringup oakd_vslam.launch.py camera_x:=\"${CAMERA_X:-0.1053}\" camera_y:=\"${CAMERA_Y:-0}\" camera_z:=\"${CAMERA_Z:-0.0304}\" camera_roll:=\"${CAMERA_ROLL:-0}\" camera_pitch:=\"${CAMERA_PITCH:-0}\" camera_yaw:=\"${CAMERA_YAW:-0}\" enable_localization_n_mapping:=True enable_slam_visualization:=True enable_landmarks_view:=True enable_observations_view:=False path_max_size:=10000'" \
    "turn on localization + mapping"
fi
if ! bridge_gated; then
  respawn bridge "bash -lc 'source skybeta_env.sh; exec ros2 launch oakd_vslam_bringup px4_bridge.launch.py input_topic:=$GATED'" \
    "feed PX4 from the validated-odometry gate"
fi
[ "$dry" = 1 ] && { log "DRY RUN: would now run odom_gate.py + map_server.py"; exit 0; }
python3 "$DIR/odom_gate.py" &
gate_pid=$!
python3 "$DIR/map_server.py" &
viewer_pid=$!
trap 'kill $gate_pid $viewer_pid 2>/dev/null || true' EXIT TERM INT
wait -n "$gate_pid" "$viewer_pid"
