"""The whole competition stack for skybeta-19, from one launch file.

This is what the stack's `competitor` window starts once the drone's drone.env has
COMPETITOR_PKG=navless_nav and COMPETITOR_LAUNCH=navless_course.launch.py.

  flight_interface + course_mission   navless.launch.py, unchanged: the ONE node that commands
                                      the drone (/skybeta/setpoint/*). Nothing else here does.
  tube/tof_driver.py                  side ToF sensors -> /tube/left/range, /tube/right/range
  tube/tube_node.py                   -> /tube/offset (tube centring), published only when both
                                      readings fit the tube's width
  mirror_relay (relay:=true)          read-only telemetry for the laptop dashboard, port 8775

The skybeta-19 defaults use the as-built route and side ToF tube centering, as on its flight branch.
The sensors were checked independently on 19, with bus 7 on the left. Corrections are bounded to
0.15 m and expire with stale ranges. Other hardware must qualify its sensors before using these defaults.

Never start planner/px4_bridge_node.py or mission/mission.py alongside this: both are
commanders too, and the drone gets exactly one.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

# tube/tof_driver.py's mount: sensor faces 0.075 m either side of the centreline, so
# tube_node adds it back to get centreline-to-wall distances.
TOF_MOUNT_M = '0.075'
# Tube interior at flight height, wall to wall (course_v3 oval; Gazebo's dh/tof reads
# 0.652 m per side at the centre + 2 x 0.075 m mounts = 1.45 m).
TUBE_WIDTH_M = '1.45'

ARGUMENTS = [
    ('use_sim_time', 'false', 'use ROS simulation clock for launch-managed components'),
    ('route_file', 'gazebo_sim/course/route_centreline_window_asbuilt.json',
     'route to fly (repo-relative): the window route on the confirmed as-built course'),
    # The as-built M1/M3 gaps require this margin: 0.4 m pulls their route points into wall ends.
    ('net_margin_m', '0.25', 'keep setpoints this far inside the net, m'),
    ('start_bearing_deg', '0.0', 'nose bearing on the launch mark at power-up (0 = north)'),
    # Speed, tuned for the centreline window route above (Gazebo + PX4 SITL, drone profile, real pilot
    # sequence). Arm to touchdown took 32.6 s on 3 of 3 laps, against 42.9 s at navless.launch.py's
    # 0.5 / 0.6 / 0.25. Every lap kept >= 11.9 cm to every wall (window wall >= 12.9 cm) and landed
    # 12-13 cm from the pad centre. Going faster costs window-wall margin first: 0.9 m/s left 11.9 cm.
    # Placement: the route aims its window crossing 4 cm off-centre to cancel the carrot's cut (tools/
    # export_route.py). With that, a 1 deg error either way keeps >= 10.9 cm at these speeds and at the
    # slow launch's (the maze is the limit; the window keeps >= 11.5 cm). Without it, 1 deg east left
    # 7.4 cm at the window. The aim matches this cut, so re-check the window crossing if these speeds or
    # max_turn_sagitta_m change. For another route, set the three back to navless.launch.py's 0.5 / 0.6 /
    # 0.25; for the Gate B route also set max_turn_sagitta_m below to 0.08.
    ('cruise_speed_mps', '0.7', 'route speed on straights, m/s'),
    ('max_lead_m', '0.8',
     'how far the carrot may run ahead of the drone, m: PX4 flies at about 0.95 x this, so keep it '
     'above cruise_speed_mps / 0.95'),
    ('max_lat_accel_mps2', '0.5', 'slow through bends: speed <= sqrt(this * radius); 0 = off'),
    ('max_turn_sagitta_m', '0.0',
     'shrink the carrot lead in tight turns so the drone cuts inside by at most this, m; 0 = off '
     '(0.08 for the Gate B route)'),
    ('odometry_topic', '/visual_slam/tracking/odometry',
     'odometry for flight_interface and course_mission: the topic that feeds the PX4 vision '
     "fusion (/mapping/validated_odometry if PX4 is fed through Koji's gate)"),
    # Dark tube: on a VIO loss INSIDE the tube (or within blind_zone_margin_m of a mouth), keep pacing
    # PX4's own estimate along the route for up to blind_carry_max_s, instead of freezing blind in the
    # dark. A loss anywhere else still freezes the route. Turned on by the team (Shining, 2026-11-07).
    # Evidence: the default (freeze) hit the tube 3/3 in the master sim; blind_carry landed 4/4 there and
    # made 6/6 clean passes at the slow launch's 0.5 m/s. At 0.7 m/s, late recovery with magnetometer
    # fusion left only 3.9 cm clearance; with EKF2_MAG_TYPE=5, 4/4 passed with >=40 cm. Frozen
    # odometry must first be stopped by the PX4 odometry gate; blind carry never carries a frozen feed.
    ('blind_carry', 'true', 'dark tube: keep moving on PX4 estimate through a VIO loss (tube only)'),
    ('tof', 'true', 'run the side ToF driver'),
    ('tube', 'true', 'run tube_node (needs tof)'),
    # The prop tips reach 0.1625 m past each sensor face (0.475 m wide with props, sensors 0.075 m
    # out): anything closer is the drone's own prop, or a wall the props already touch.
    ('tof_min_range_m', '0.18', 'tube_node: reject closer readings (raise until a hand-spun prop never shows)'),
    ('relay', 'false', 'run the read-only dashboard relay (TCP 8775)'),
    ('relay_img_hz', '0.0', 'relay camera thumbnails per second; keep 0 in flight'),
    ('stereo_tube_centering', 'false', 'experimental stereo centering; simulation validation only'),
    ('tof_tube_centering', 'true',
     'steer toward the side ToF tube centre (needs tof and tube; check the sensors first)'),
    ('stereo_tube_cloud_topic', '/team/stereo/points', 'qualified calibrated stereo cloud'),
]


def generate_launch_description():
    from navless_nav.route import REPO_ROOT          # the repo this package was built from

    declared = [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in ARGUMENTS]
    navless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('navless_nav'), 'launch', 'navless.launch.py')),
        launch_arguments={n: LaunchConfiguration(n)
                          for n in ('route_file', 'start_bearing_deg', 'cruise_speed_mps',
                                    'max_lead_m', 'max_lat_accel_mps2', 'max_turn_sagitta_m',
                                    'odometry_topic', 'blind_carry', 'net_margin_m', 'stereo_tube_centering',
                                    'stereo_tube_cloud_topic', 'tof_tube_centering',
                                    'use_sim_time')}.items(),
    )
    # VL53L1X: from the team image, or from ~/team_ws/pylibs (pip install --target) if it is there.
    pylibs = '/root/team_ws/pylibs'
    pythonpath = ':'.join(p for p in (pylibs if os.path.isdir(pylibs) else '',
                                      os.environ.get('PYTHONPATH', '')) if p)
    tof = ExecuteProcess(
        cmd=['python3', os.path.join(REPO_ROOT, 'tube', 'tof_driver.py')],
        additional_env={'PYTHONPATH': pythonpath},
        output='screen', name='tof_driver', condition=IfCondition(LaunchConfiguration('tof')))
    tube = ExecuteProcess(
        cmd=['python3', os.path.join(REPO_ROOT, 'tube', 'tube_node.py'), '--ros-args',
             '-p', f'left_mount:={TOF_MOUNT_M}', '-p', f'right_mount:={TOF_MOUNT_M}',
             '-p', ['min_range:=', LaunchConfiguration('tof_min_range_m')],
             '-p', 'max_range:=1.3',                  # tof_driver runs VL53L1X short mode (~1.3 m)
             '-p', f'expected_width:={TUBE_WIDTH_M}', '-p', 'width_tol:=0.15',
             # its stamps are checked against course_mission's clock
             '-p', ['use_sim_time:=', LaunchConfiguration('use_sim_time')]],
        output='screen', name='tube_node', condition=IfCondition(LaunchConfiguration('tube')))
    relay = Node(
        package='navless_nav', executable='mirror_relay', name='navless_mirror_relay',
        output='screen', parameters=[{'img_hz': LaunchConfiguration('relay_img_hz')}],
        condition=IfCondition(LaunchConfiguration('relay')))
    return LaunchDescription(declared + [navless, tof, tube, relay])
