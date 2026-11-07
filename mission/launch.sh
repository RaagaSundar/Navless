#!/usr/bin/env bash
# One command for the whole stack: ./launch.sh [route.json] [mission.py flags...]
# Starts every teammate's node in the background, waits for their topics, then runs the
# mission in the foreground (it asks for ARM). Ctrl-C or mission exit stops everything.
# Reset world first if keyboard RC was used.
set -euo pipefail
cd "$(dirname "$0")/.."
ROUTE="${1:-route_gate_b.json}"; shift || true
SIM="--ros-args -p use_sim_time:=true"

# "command|topic it must publish before we fly". Fill in as owners confirm entry points.
# Never add planner/px4_bridge_node.py here: it also streams setpoints.
HELPERS=()
# Shining: stereo depth/cloud. mission.py doesn't consume it, and it costs ~80% of a core at 5 Hz,
# so it runs at 3 Hz and DEPTH=0 skips it (do that on the real drone). It reads /oak/* topics;
# on the sim, append remaps for the live stereo topic names, e.g. -r /oak/left/image_raw:=<live>.
[ "${DEPTH:-1}" = 1 ] && HELPERS+=("python3 depthcloud/stereo_cloud.py $SIM -p rate_hz:=3.0|/team/stereo/points")
# Raaga: tube sensor (TBD) -> HELPERS+=("python3 <tube node> $SIM|<topic>")

pids=()
cleanup() { for p in "${pids[@]}"; do kill "$p" 2>/dev/null || true; done; wait 2>/dev/null || true; }
trap cleanup EXIT

for h in "${HELPERS[@]}"; do
  cmd="${h%%|*}"; topic="${h##*|}"
  echo "starting: $cmd"; $cmd & pids+=($!)
  for _ in $(seq 30); do
    timeout 3 ros2 topic echo --once "$topic" >/dev/null 2>&1 && break
    kill -0 "${pids[-1]}" 2>/dev/null || { echo "helper died: $cmd"; exit 1; }
  done || true
  timeout 3 ros2 topic echo --once "$topic" >/dev/null 2>&1 || { echo "no data on $topic; not flying"; exit 1; }
done

python3 mission/mission.py --check "$ROUTE" "$@"
python3 mission/mission.py "$ROUTE" "$@" $SIM
