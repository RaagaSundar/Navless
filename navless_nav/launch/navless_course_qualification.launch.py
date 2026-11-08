"""Staged localizer qualification on top of navless_course_slow.launch.py.

`phase` decides what the localizer and the live map may do to the flown route, and nothing
else may override that (a conflicting wall_correction:=, replan_on_obstacles:=, localizer:= or
stereo_cloud:= is refused at launch):

  shadow      (default) localizer + stereo run and report; the route ignores them
  correction  the localizer's wall correction moves the route; no live replanning
  mapping     correction and live obstacle replanning

Pass a later phase only once the earlier one's recorded evidence supports it. The speed, lead,
bend, sagitta and heading defaults below are UNQUALIFIED starting settings for the first such
flights, not evidence that they are safe; the takeoff slide seed is explicitly 0 (the course
launch's -0.15 m is not inherited unknowingly). Override any of them as usual.

odometry_topic and validated_odom_topic default to hardware_slam/odom_gate.py's
/mapping/validated_odometry, so the mission, flight_interface's origin and the localizer all
use the gated stream. This launch does not start the gate (run it as the hardware_slam
service does) and does not change what PX4 fuses: that is the stack's bridge, which operators
must verify separately. Without the gate running nothing streams, and flight_interface never
takes over. It does not arm, change modes or publish to /fmu/in.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

# phase -> what it lets the localizer and the live map do (course launch arguments)
PHASES = {
    'shadow': {'wall_correction': 'false', 'replan_on_obstacles': 'false'},
    'correction': {'wall_correction': 'true', 'replan_on_obstacles': 'false'},
    'mapping': {'wall_correction': 'true', 'replan_on_obstacles': 'true'},
}
OBSERVERS = {'localizer': 'true', 'stereo_cloud': 'true'}      # every phase records them
DEFAULTS = {
    'cruise_speed_mps': '0.3', 'max_lead_m': '0.4', 'max_lat_accel_mps2': '0.12',
    'max_turn_sagitta_m': '0.015', 'heading_mode': 'hold',
    'drift_seed_body_x_m': '0.0', 'drift_seed_body_y_m': '0.0',
    'odometry_topic': '/mapping/validated_odometry',
    'validated_odom_topic': '/mapping/validated_odometry',
}


def phase_arguments(configurations):
    """The slow launch's arguments for these launch configurations; ValueError if not allowed."""
    phase = configurations.get('phase', 'shadow')
    if phase not in PHASES:
        raise ValueError(f"phase:={phase!r} is not one of {', '.join(PHASES)}")
    owned = dict(PHASES[phase], **OBSERVERS)
    for name, value in owned.items():
        given = configurations.get(name)
        if given is not None and given.strip().lower() != value:
            raise ValueError(f'{name}:={given} conflicts with phase:={phase} (which sets {value}); '
                             'choose the phase instead')
    arguments = {name: configurations.get(name, default) for name, default in DEFAULTS.items()}
    arguments.update(owned)
    return arguments


def _include(context):
    arguments = phase_arguments(context.launch_configurations)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('navless_nav'), 'launch', 'navless_course_slow.launch.py')),
        launch_arguments=arguments.items())]


def generate_launch_description():
    declared = [DeclareLaunchArgument('phase', default_value='shadow',
                                      description='shadow | correction | mapping')]
    declared += [DeclareLaunchArgument(name, default_value=value) for name, value in DEFAULTS.items()]
    return LaunchDescription(declared + [OpaqueFunction(function=_include)])
