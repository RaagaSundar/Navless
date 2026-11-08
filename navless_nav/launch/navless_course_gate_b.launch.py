"""skybeta-19's backup: the Gate B route on the as-built course, in place of the window.

Fly this only if the organisers say the window does not earn the chicane's points. This route flies
the whole chicane and leaves it through Gate B, so it earns them either way; it is a little slower.
It is navless_course.launch.py with the route and speed settings below; ToF centring, the net margin
and everything else stay the same. To switch, change the drone's /etc/skybeta/drone.env to
COMPETITOR_LAUNCH=navless_course_gate_b.launch.py (navless_course_slow.launch.py switches back), or
start it by hand:

  ros2 launch navless_nav navless_course_gate_b.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

# The route: tools/export_route.py --route gate_b --course course/course_asbuilt.json --sdf <as-built
# model.sdf> --pad-from-south-in 26 --gate-b-turns-in 59 151.5 118 86.5, on the organisers' 2026-11-07
# maze. The same head as the window route up to the tube (north-east up corridor 1 beside MD, round MD's
# tip through Gate A), then west under C1's south end, up the chicane, west through Gate B, south east of
# A1 and onto the pad from the south; 15.9 m. Planned with the props, nose north: >= 17.9 cm to every wall
# (Barrier 1 behind the launch mark; A1 19.6, C1 24.5, C2b 25.2, MD 25.3); the rangefinder beam stays
# >= 20.9 cm off every boulder. Speeds: navless.launch.py's 0.5 / 0.6 / 0.25 (navless_course.launch.py's
# faster ones were tuned for the window route), with the Gate B turn cap of 0.08 m.
GATE_B = {
    'route_file': 'gazebo_sim/course/route_centreline_gate_b_asbuilt.json',
    'cruise_speed_mps': '0.5',
    'max_lead_m': '0.6',
    'max_lat_accel_mps2': '0.25',
    'max_turn_sagitta_m': '0.08',
}


def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('navless_nav'), 'launch', 'navless_course.launch.py')),
        launch_arguments=GATE_B.items(),
    )])
