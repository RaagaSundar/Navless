# Source in local-course-sim.
source /opt/ros/jazzy/setup.bash
if [[ -f /course-state/fast_msgs/local_setup.bash ]]; then
  source /course-state/fast_msgs/local_setup.bash
else
  source /root/ros2_ws/install/setup.bash
fi
