#!/usr/bin/env bash
# Build-only setup: never starts Gazebo, PX4, ROS publishers or a flight.
set -euo pipefail
cd "$(dirname "$0")"
base=${BASE_SIM_DIR:?Set BASE_SIM_DIR to the local_sim directory from PR 32}
base=$(cd "$base" && pwd)
for f in Dockerfile bootstrap-px4.sh bootstrap-agent.sh; do
  [[ -f "$base/$f" ]] || { echo "Missing base prerequisite: $f" >&2; exit 2; }
done
[[ $(uname -s) == Darwin && $(uname -m) == arm64 ]] || { echo 'Apple Silicon macOS required' >&2; exit 2; }
mkdir -p state/home
# Setup needs network access; the runtime Compose network is internal.
docker build --platform linux/arm64 -t dronehacks-local:jazzy-harmonic "$base"
docker build --platform linux/arm64 -f vision.Dockerfile -t local-course:vision .
docker run --rm --platform linux/arm64 --cpus 2 --memory 6g \
  --entrypoint bash -v "$PWD/state/home:/root" -v "$base:/base:ro" \
  local-course:vision -lc 'bash /base/bootstrap-px4.sh --with-messages && bash /base/bootstrap-agent.sh' \
  > state/setup.log 2>&1
# Record installed packages/image/source identities rather than claiming reproducibility across apt updates.
docker image inspect local-course:vision > state/image-inspect.json
docker run --rm --platform linux/arm64 --entrypoint bash \
  -v "$PWD/state/home:/root:ro" local-course:vision -lc \
  'dpkg-query -W; git -C /root/PX4-Autopilot rev-parse HEAD; git -C /root/ros2_ws/src/px4_msgs rev-parse HEAD; git -C /root/px4_agent_ws/src/Micro-XRCE-DDS-Agent rev-parse HEAD' \
  > state/build-identity.txt
printf '%s\n' 'Build complete. Review state/setup.log and state/build-identity.txt before starting.'
