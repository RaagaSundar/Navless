#!/usr/bin/env bash
# Stop this simulation's containers and tunnel, preserving logs/maps/builds.
set -euo pipefail
cd "$(dirname "$0")"
source ./split_network.sh
ssh -S "$SOCKET" -o BatchMode=yes "$TARGET" '
  docker stop -t 10 drone19-split-sim
  if docker inspect drone19-gazebo-bench >/dev/null 2>&1; then
    docker stop -t 10 drone19-gazebo-bench
  fi' || true
docker compose down
ssh -S "$SOCKET" -O exit "$TARGET" || true
