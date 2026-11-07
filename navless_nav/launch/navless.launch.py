"""flight_interface + the Navless course node.

What the stack's `competitor` window runs once the drone's drone.env has
COMPETITOR_PKG=navless_nav and COMPETITOR_LAUNCH=navless.launch.py.

flight_interface is started with auto_offboard_on_arm:=true (the judged
takeover). The envelope is only ever tightened from the interface's defaults
(3.0 m ceiling, 1.5 m/s): the course tops out at 2.3 m on the over-wall
fly-home leg.
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
    ('takeoff_altitude_m', '0.85',
     'hover height above the origin when no route loads; with a route, the '
     'climb goes to its first point'),
    ('takeoff_lead_m', '0.25',
     'climb target this far up the route\'s first leg (away from Barrier 1 '
     'behind the launch mark), m; 0 = straight up'),
    ('arrival_radius_m', '0.25', 'how close counts as arrived, metres'),
    ('takeoff_timeout_s', '15.0', 'give up climbing and hold after this'),
    ('route_file', 'route_gate_b.json',
     'planner route: absolute, or relative to the Navless repo root'),
    ('start_bearing_deg', '0.0',
     'nose bearing on the launch mark at power-up, deg clockwise from course '
     'north (0 = facing up the first corridor)'),
    ('start_height_m', '0.07',
     'height of the odom origin (Pixhawk IMU) above the floor at power-up, m'),
    ('cruise_speed_mps', '0.5', 'carrot speed along the route, m/s'),
    ('max_lead_m', '0.6', 'how far the carrot may run ahead of the drone, m'),
    ('max_lat_accel_mps2', '0.25',
     'slow through bends: speed <= sqrt(this * radius); 0 = off'),
    ('net_margin_m', '0.4', 'keep setpoints this far inside the net, m'),
    ('land_at_end', 'true', 'land on the route end (the pad) and disarm'),
    ('land_speed_mps', '0.3', 'descent rate onto the pad, m/s'),
    ('fly_home', 'false',
     'after the pad landing, the next arming flies back over the wall to '
     'the launch mark and lands there'),
    ('home_altitude_m', '2.2', 'fly-home hop height above the floor, m'),
    ('home_climb_tol_m', '0.05',
     'fly-home: start the leg over the wall only this close to hop height, m'),
    ('home_arrival_radius_m', '0.10',
     'fly-home: land only this close to the launch mark (Barrier 1 is behind it), m'),
    ('prearm_height_tol_m', '0.15',
     'pre-arm check: no setpoint stream (so no takeover) until PX4 height '
     'and odom z agree to this, m; 0 = off'),
    ('prearm_settle_s', '2.0',
     'pre-arm check: how long they must agree without a break, s'),
    ('blind_carry', 'false',
     'on a VIO loss inside the dark tube, keep the route moving on PX4\'s own '
     'estimate instead of freezing (elsewhere a loss still freezes it)'),
    ('blind_carry_max_s', '4.0',
     'longest blind carry per VIO loss, s (PX4 dead reckoning ends at 5 s)'),
    ('blind_zone_margin_m', '0.4',
     'blind carry also allowed this far outside each tube mouth, m'),
    ('competitor', 'true', 'also start the course node'),
]

# The stack's flight-interface package. Its name is fixed by the image.
FLIGHT_INTERFACE_PKG = 'skybeta_flight_interface'

MISSION_PARAMS = ('takeoff_altitude_m', 'takeoff_lead_m', 'arrival_radius_m',
                  'takeoff_timeout_s',
                  'route_file', 'start_bearing_deg', 'start_height_m',
                  'cruise_speed_mps', 'max_lead_m', 'max_lat_accel_mps2',
                  'net_margin_m',
                  'land_at_end', 'land_speed_mps', 'fly_home',
                  'home_altitude_m', 'home_climb_tol_m', 'home_arrival_radius_m',
                  'prearm_height_tol_m', 'prearm_settle_s',
                  'blind_carry', 'blind_carry_max_s', 'blind_zone_margin_m')


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=help_)
        for name, default, help_ in ARGUMENTS
    ]

    flight_interface = Node(
        package=FLIGHT_INTERFACE_PKG,
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
        parameters=[{name: LaunchConfiguration(name)
                     for name in MISSION_PARAMS}],
    )

    return LaunchDescription(declared + [flight_interface, mission])
