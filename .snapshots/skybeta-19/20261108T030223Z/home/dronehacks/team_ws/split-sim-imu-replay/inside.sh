#!/usr/bin/env bash
set -eo pipefail
[[ ${ROS_DOMAIN_ID:-} = 218 ]] || exit 1
source /opt/ros/humble/setup.bash
ros2 launch /replay/vio.launch.py > /replay/launch.log 2>&1 & tracker=$!
trap 'kill -INT "$tracker" 2>/dev/null || true; wait "$tracker" || true' EXIT
python3 /replay/play.py /replay/flight.sqlite /replay/poses.json
