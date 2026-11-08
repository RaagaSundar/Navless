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
# maze. The same head as the window route up to the tube (north up corridor 1, east through Gate A),
# then west under C1's south end, up the chicane, west through Gate B, south east of A1 and onto the
# pad from the south; 16.5 m. Planned >= 17.9 cm to every wall (Barrier 1 behind the launch mark; A1
# 19.6, MN 23.1, MD's tip 23.1, C1 24.5); the rangefinder beam misses every boulder. Speeds:
# navless.launch.py's 0.5 / 0.6 / 0.25 (navless_course.launch.py's faster ones were tuned for the
# window route), with the Gate B turn cap of 0.08 m. As-built Gazebo gate on the new maze
# (gate_asbuilt.sh, drone profile, max_turn_sagitta_m 0.08, 3 laps, before the heading hold): all
# landed 0.7-2.2 cm from the pad centre, no contacts, 60.6-61.3 s from arming to touchdown, through
# Gate B at ~44.5 s. Closest: boulder 2 19.0 cm, MN 21.3, A1 21.9, MD 22.0.
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
