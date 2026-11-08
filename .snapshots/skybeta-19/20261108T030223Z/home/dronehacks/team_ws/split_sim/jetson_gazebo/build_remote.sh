#!/usr/bin/env bash
# OPTIONAL artifact assembly only, on Jetson. Native adapter needs no build.
# Logs/status survive SSH loss. This script never compiles the C++ bridge.
set -uo pipefail
state="$HOME/team_ws/gazebo_bench_evidence"
mkdir -p "$state"
exec 9> "$state/build.lock"
flock -n 9 || { echo 'This experimental build is already running.' >&2; exit 3; }
# Check for an earlier foreground build separately before invoking this script.
log="$state/build-$(date -u +%Y%m%dT%H%M%SZ).log"
printf '%s\n' "$log" > "$state/latest-build-log"
printf '%s\n' "$$" > "$state/build.pid"
cd "$HOME/team_ws/split_sim" || exit 2
[[ -d jetson_gazebo/bridge_install ]] || { echo 'No completed offloaded artifact.' >&2; exit 2; }
docker image inspect drone19-gazebo-deps:humble-harmonic >/dev/null || exit 2
rm -f "$state/build.exit"
docker build --progress=plain -f jetson_gazebo/Runtime.Dockerfile \
  -t drone19-gazebo:humble-harmonic jetson_gazebo > "$log" 2>&1
result=$?
printf '%s\n' "$result" > "$state/build.exit"
rm -f "$state/build.pid"
exit "$result"
