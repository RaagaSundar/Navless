"""Shared launch description: the organisers' flight_interface (unchanged) + our test node.

Copied from skybeta's competitor_example/launch/competitor.launch.py: flight_interface keeps its own
node and parameters (it owns Offboard, the heartbeat and the envelope); only the competitor node is
ours. The envelope is tightened for first flights: 2.0 m ceiling, 1.0 m/s twist cap.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    ('max_altitude_m', '2.0', 'ceiling above the odometry origin, metres (first flights: 2.0)'),
    ('min_altitude_m', '0.3', 'floor above the odometry origin, metres'),
    ('max_speed_mps', '1.0', 'cap on a twist setpoint, m/s (first flights: 1.0)'),
    ('setpoint_rate_hz', '10.0', 'OffboardControlMode heartbeat rate'),
    ('allow_mode_request', 'false', 'SIM ONLY. On the drone Offboard follows the pilot arming, then Position'),
    ('auto_offboard_on_arm', 'true', 'enter Offboard once when the pilot arms, then switches to Position'),
    ('odometry_topic', '/visual_slam/tracking/odometry', 'pose the altitude band is measured against'),
    ('climb_m', '1.0', 'test: climb above the start, metres'),
    ('hover_s', '10.0', 'test: long hold, seconds'),
    ('forward_m', '2.0', 'forward test: distance along the starting heading, metres'),
    ('speed_mps', '0.4', 'forward test: speed, m/s'),
    ('yaw_deg', '180.0', 'yaw test: clockwise turn, degrees'),
    ('yaw_rate_dps', '30.0', 'yaw test: turn rate, deg/s'),
]


def description(test):
    declared = [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in ARGUMENTS]
    flight_interface = Node(
        package='skybeta_flight_interface', executable='flight_interface', name='flight_interface',
        output='screen', emulate_tty=True,
        parameters=[{k: LaunchConfiguration(k) for k in (
            'max_altitude_m', 'min_altitude_m', 'max_speed_mps', 'setpoint_rate_hz',
            'allow_mode_request', 'auto_offboard_on_arm', 'odometry_topic')}])
    node = Node(
        package='navless_flight_tests', executable='basic_flight', name='navless_basic_flight',
        output='screen', emulate_tty=True,
        parameters=[{'test': test, **{k: LaunchConfiguration(k) for k in (
            'climb_m', 'hover_s', 'forward_m', 'speed_mps', 'yaw_deg', 'yaw_rate_dps')}}])
    return LaunchDescription(declared + [flight_interface, node])
