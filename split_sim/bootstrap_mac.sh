#!/usr/bin/env bash
# Explicit build-only step. Never starts a simulator, ROS node, PX4 or flight.
set -euo pipefail
cd "$(dirname "$0")"
[[ "${1:-}" == --build && $# == 1 ]] || { echo 'Usage: bootstrap_mac.sh --build (bounded build only)'; exit 2; }
mkdir -p state ../local_course/state/nvidia-runs
if [[ "$(uname -s)" != Darwin || "$(uname -m)" != arm64 ]]; then echo 'Apple Silicon Mac required'; exit 2; fi
if docker ps --format '{{.Names}}' | grep -Eq '^(drone19-split-sim-laptop-1|local-course-sim)$'; then echo 'Release active simulation window before building'; exit 2; fi
docker build -t dronehacks-local:jazzy-harmonic ../local_sim
docker run --rm --name split-sim-px4-build --cpus 2 --memory 6g \
  --entrypoint bash -v "$PWD/state:/root" -v "$(cd .. && pwd):/workspace/Navless:ro" \
  dronehacks-local:jazzy-harmonic /workspace/Navless/local_sim/bootstrap-px4.sh
