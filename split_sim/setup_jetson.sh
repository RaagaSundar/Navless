#!/usr/bin/env bash
# Install only an inactive separate SIM container. No physical device mounts.
set -euo pipefail
cd "$(dirname "$0")"
case "${1:---dry-run}" in
  --dry-run) python3 doctor.py --source-only; echo 'Plan: authenticate SSH; verify vendor image; copy complete packaged SIM payload; create inactive isolated domain219 container. No operation executed.'; exit ;;
  --install) ;;
  *) echo 'Usage: setup_jetson.sh [--dry-run|--install]'; exit 2 ;;
esac
python3 doctor.py --source-only
source ./split_network.sh
split_ensure_ssh
image="${SKYBETA_SIM_IMAGE:-skybeta:0.3.3}"
[[ "$image" =~ ^[a-zA-Z0-9._:/@-]+$ ]] || exit 2
"${SSH[@]}" "$TARGET" bash -s -- "$image" <<'CHECK'
set -euo pipefail
image=$1
docker image inspect "$image" >/dev/null || { echo 'Organiser-provided compatible skybeta image missing; this repository cannot install JetPack/CUDA/cuVSLAM'; exit 2; }
if docker inspect drone19-split-sim >/dev/null 2>&1; then echo 'Existing split installation: refusing overwrite; use reviewed update procedure'; exit 2; fi
if [[ "$(docker inspect -f '{{.State.Running}}' drone19-gazebo-bench 2>/dev/null || true)" == true ]]; then echo 'Jetson-local bench owns domain219'; exit 2; fi
mkdir -p ~/team_ws/split_sim ~/team_ws/split_sim_state/course_stage
CHECK
payload=$(mktemp -d "${TMPDIR:-/tmp}/split-payload.XXXXXX")
trap 'rm -rf "$payload"' EXIT
mkdir -p "$payload/split_sim"
for f in sensor_link.py imu_link.py sim_slam.launch.py run_jetson.sh; do cp "$f" "$payload/split_sim/"; done
cp -R viewer "$payload/split_sim/"
mkdir -p "$payload/split_sim/perception"
cp ../depthcloud/{stereo_cloud.py,cloud_localize.py} "$payload/split_sim/perception/"
cp ../gazebo_sim/models/dh_course/meshes/course_visual.obj "$payload/split_sim/perception/course.obj"
COPYFILE_DISABLE=1 tar -czf "$payload/payload.tgz" -C "$payload" split_sim
scp -o ControlPath="$SOCKET" -o BatchMode=yes "$payload/payload.tgz" "$TARGET":team_ws/split_sim_state/install-payload.tgz
"${SSH[@]}" "$TARGET" bash -s -- "$image" <<'INSTALL'
set -euo pipefail
image=$1
tar -xzf ~/team_ws/split_sim_state/install-payload.tgz -C ~/team_ws
# Fresh container is CREATED, not started. No application starts at installation.
docker create --name drone19-split-sim --label navless.owner=split-simulation \
  --runtime nvidia --network host --shm-size 512m --cap-drop ALL --cap-add DAC_OVERRIDE \
  --security-opt no-new-privileges -e ROS_DOMAIN_ID=219 -e ROS_LOCALHOST_ONLY=1 \
  -e SKYBETA_UNDERLAY_ONLY=1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
  -v "$HOME/team_ws/split_sim":/split:ro -v "$HOME/team_ws/split_sim_state":/root/split_state \
  --entrypoint bash "$image" /split/run_jetson.sh
docker inspect -f '{{.Image}}' drone19-split-sim > ~/team_ws/split_sim_state/image-identity.txt
printf 'stereo\n' > ~/team_ws/split_sim_state/estimator-profile
echo 'Created separate simulation container, not started. Physical stack unchanged.'
INSTALL
