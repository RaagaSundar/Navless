#!/usr/bin/env bash
# Select simulation-only acquisition profile while both ends are stopped.
set -euo pipefail
cd "$(dirname "$0")"
[[ $# == 1 && ( "$1" == stereo || "$1" == stereo-imu ) ]] || { echo 'Usage: configure_estimator.sh stereo|stereo-imu'; exit 2; }
profile=$1
source ./split_network.sh
split_ensure_ssh
"${SSH[@]}" "$TARGET" bash -s -- "$profile" <<'REMOTE'
set -euo pipefail
[[ "$(docker inspect -f '{{.State.Running}}' drone19-split-sim)" == false ]] || { echo 'Stop/release simulation before profile selection'; exit 2; }
printf '%s\n' "$1" > ~/team_ws/split_sim_state/estimator-profile
REMOTE
echo "Selected SIM profile $profile; renderer must use matching SPLIT_INERTIAL=0 or1 before next fresh world."
