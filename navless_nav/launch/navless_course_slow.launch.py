"""The competition stack at the conservative speeds, for the first real flights.

This is navless_course.launch.py run at navless.launch.py's original speeds: 0.5 m/s on straights, a
0.6 m carrot lead and bends at 0.25 m/s^2. The course launch's own window-route tuning is 0.7 / 0.8 /
0.5. The route, the ToF chain and the relay all stay as navless_course.launch.py sets them. To fly
it, set the drone's drone.env to

  COMPETITOR_PKG=navless_nav
  COMPETITOR_LAUNCH=navless_course_slow.launch.py

blind_carry (dark tube) is on, from navless_course.launch.py; these are the speeds it was proven at.
Switch back to navless_course.launch.py once these flights look like the sim. Nothing on the drone
needs editing, and there is nothing to remember to revert. In Gazebo these speeds flew the window route
from arming to touchdown in about 43 s (the tuned launch: 32.6 s).

The three speeds are defaults, not forced values: for example, cruise_speed_mps:=0.3 and
max_lead_m:=0.4 are forwarded unchanged to the course launch. Its other arguments (route_file,
wall_correction, localizer, ...) remain available. A changed speed or lead needs separate
qualification; this wrapper does not establish hardware readiness or authorize arming.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

# navless.launch.py's defaults: test_course_launch.py checks they still match.
SLOW = {'cruise_speed_mps': '0.5', 'max_lead_m': '0.6', 'max_lat_accel_mps2': '0.25'}


def generate_launch_description():
    declared = [DeclareLaunchArgument(name, default_value=value) for name, value in SLOW.items()]
    return LaunchDescription(declared + [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('navless_nav'), 'launch', 'navless_course.launch.py')),
        launch_arguments={name: LaunchConfiguration(name) for name in SLOW}.items(),
    )])
