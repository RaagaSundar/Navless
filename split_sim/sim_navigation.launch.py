"""Simulation-only flight interface + staged, pinned navigation code.

No arming or automatic mode request. Launch only after the disarmed bench
passes; a separately reviewed pilot action is required for a moving test.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='skybeta_flight_interface', executable='flight_interface',
             name='flight_interface', output='screen', parameters=[{
                 'use_sim_time': True,
                 'auto_offboard_on_arm': False, 'allow_mode_request': False,
                 'odometry_topic': '/mapping/validated_odometry',
                 'max_altitude_m': 2.5, 'max_speed_mps': 1.0,
             }]),
        Node(package='navless_nav', executable='course_mission',
             name='navless_nav', output='screen', parameters=[{
                 'use_sim_time': True,
                 'odometry_topic': '/mapping/validated_odometry',
                 'route_file': '/split/navless/gazebo_sim/course/route_centreline_window.json',
                 'start_bearing_deg': 0.0, 'cruise_speed_mps': 0.5,
                 'max_lead_m': 0.6, 'max_lat_accel_mps2': 0.25,
             }]),
    ])
