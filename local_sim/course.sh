#!/usr/bin/env bash
# Scene inspection only: no autopilot, mission, arming or external odometry feed.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH="/workspace/Navless/gazebo_sim/models:${GZ_SIM_RESOURCE_PATH:-}"
exec gz sim -v 3 /workspace/Navless/gazebo_sim/worlds/dronehacks.sdf
