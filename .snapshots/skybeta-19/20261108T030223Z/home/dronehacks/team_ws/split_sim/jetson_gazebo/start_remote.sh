#!/usr/bin/env bash
# Run ON Jetson after explicit coordination with the split simulation owner.
# No physical stack action, UART/USB mount, host install or autopilot process.
set -euo pipefail
root="${HOME}/team_ws"
[[ "$(docker inspect -f '{{.State.Running}}' drone19-split-sim 2>/dev/null || true)" != true ]] || {
  echo 'Stop only drone19-split-sim after coordination before using domain 219.' >&2; exit 2;
}
mkdir -p "$root/gazebo_bench_state"
docker run -d --name drone19-gazebo-bench --runtime nvidia --network host --memory=3g --memory-swap=3g --cpus=4 --pids-limit=512 \
  --device /dev/dri/renderD128 --device /dev/dri/renderD129 \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e SENSOR_ADAPTER="${SENSOR_ADAPTER:-native}" -e ROS_DOMAIN_ID=219 -e ROS_LOCALHOST_ONLY=1 -e SKYBETA_UNDERLAY_ONLY=1 \
  -v "$root/split_sim:/split:ro" -v "$root/gazebo_bench_state:/root/gazebo_state" \
  --entrypoint /bin/bash "${GAZEBO_IMAGE:-drone19-gazebo-deps:humble-harmonic}" /split/jetson_gazebo/run_inside.sh
