#!/usr/bin/env bash
# Execute inside the isolated local-course-sim container. Never arms.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=220 ROS_LOCALHOST_ONLY=1
exec ros2 launch /local_course/vision.launch.py
