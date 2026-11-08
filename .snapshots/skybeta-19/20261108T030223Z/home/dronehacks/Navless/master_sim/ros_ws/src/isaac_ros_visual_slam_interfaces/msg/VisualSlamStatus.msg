# Stand-in for Isaac ROS's isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus, for the master sim
# only (the drone has the real one). Team code reads vo_state alone; the other fields follow the
# Isaac ROS definition so nothing else needs to change if more of it gets used.
uint8 VO_STATE_UNKNOWN = 0
uint8 VO_STATE_SUCCESS = 1
uint8 VO_STATE_FAILED = 2

uint8 vo_state
uint8 integrator_state
float64 node_callback_execution_time
float64 track_execution_time
float64 track_execution_time_mean
float64 track_execution_time_max
