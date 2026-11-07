"""skybeta flight_interface + the Navless course node.

What the stack's `competitor` window runs once /etc/skybeta/drone.env has
COMPETITOR_PKG=navless_nav and COMPETITOR_LAUNCH=navless.launch.py.

flight_interface is started exactly as competitor_example's launch file does,
with auto_offboard_on_arm:=true (the judged takeover). The envelope is only
ever tightened from skybeta's defaults (3.0 m ceiling, 1.5 m/s): the course
tops out at 2.3 m on the over-wall fly-home leg.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    ('max_altitude_m', '2.5', 'ceiling above the odometry origin, metres'),
    ('min_altitude_m', '0.3', 'floor above the odometry origin, metres'),
    ('max_speed_mps', '1.0', 'cap on the norm of a twist setpoint, m/s'),
    ('setpoint_rate_hz', '10.0', 'OffboardControlMode heartbeat rate'),
    ('auto_offboard_on_arm', 'true',
     'enter Offboard once when the pilot arms, then switches to Position'),
    ('odometry_topic', '/visual_slam/tracking/odometry',
     'source of the ENU pose the altitude band is measured against'),
    ('takeoff_altitude_m', '0.85', 'climb-out height above the origin, metres'),
    ('arrival_radius_m', '0.25', 'how close counts as arrived, metres'),
    ('takeoff_timeout_s', '15.0', 'give up climbing and hold after this'),
    ('competitor', 'true', 'also start the course node'),
]


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=help_)
        for name, default, help_ in ARGUMENTS
    ]

    flight_interface = Node(
        package='skybeta_flight_interface',
        executable='flight_interface',
        name='flight_interface',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'max_altitude_m': LaunchConfiguration('max_altitude_m'),
            'min_altitude_m': LaunchConfiguration('min_altitude_m'),
            'max_speed_mps': LaunchConfiguration('max_speed_mps'),
            'setpoint_rate_hz': LaunchConfiguration('setpoint_rate_hz'),
            'allow_mode_request': False,
            'auto_offboard_on_arm': LaunchConfiguration('auto_offboard_on_arm'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
        }],
    )

    mission = Node(
        package='navless_nav',
        executable='course_mission',
        name='navless_nav',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('competitor')),
        parameters=[{
            'takeoff_altitude_m': LaunchConfiguration('takeoff_altitude_m'),
            'arrival_radius_m': LaunchConfiguration('arrival_radius_m'),
            'takeoff_timeout_s': LaunchConfiguration('takeoff_timeout_s'),
        }],
    )

    return LaunchDescription(declared + [flight_interface, mission])
