#!/bin/bash
set -eo pipefail
export ROS_DOMAIN_ID="$1" ROS_LOCALHOST_ONLY=1 SKYBETA_UNDERLAY_ONLY=1
export HOME=/work ROS_LOG_DIR=/work/ros-logs PYTHONDONTWRITEBYTECODE=1
source /usr/local/bin/skybeta_env.sh
[[ "$ROS_DOMAIN_ID" == "$1" && "$ROS_LOCALHOST_ONLY" == 1 ]]
# The sole session command decodes source as an unprivileged user inside a fresh private directory.
cd /work
mkdir -p runs
rundir="$(mktemp -d /work/runs/run.XXXXXXXX)"
cd "$rundir"
python3 /opt/session/bundle.py
echo "Session workspace: $rundir" >&2
exec python3 -u main.py
