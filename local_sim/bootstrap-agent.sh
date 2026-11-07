#!/usr/bin/env bash
# Build the version-matched local SITL transport; do not start it here.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
WS=/root/px4_agent_ws
mkdir -p "$WS/src"
if [[ ! -d "$WS/src/Micro-XRCE-DDS-Agent/.git" ]]; then
  git clone --branch v2.4.3 --depth 1 \
    https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$WS/src/Micro-XRCE-DDS-Agent"
fi
cd "$WS"
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
git -C "$WS/src/Micro-XRCE-DDS-Agent" rev-parse HEAD
