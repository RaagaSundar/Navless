"""The whole competition stack for skybeta-19, from one launch file.

This is what the stack's `competitor` window starts once the drone's drone.env has
COMPETITOR_PKG=navless_nav and COMPETITOR_LAUNCH=navless_course.launch.py.

  flight_interface + course_mission   navless.launch.py, unchanged: the ONE node that commands
                                      the drone (/skybeta/setpoint/*). Nothing else here does.
  tube/tof_driver.py                  side ToF sensors -> /tube/left/range, /tube/right/range
  tube/tube_node.py                   -> /tube/offset (tube centring), published only when both
                                      readings fit the tube's width
  mirror_relay (relay:=true)          read-only telemetry for the laptop dashboard, port 8775

skybeta-19's flight branch: the route is planned on the as-built course map (Shining's phone
scan of the real course, gazebo_sim/course/course_asbuilt.json), and course_mission steers on
the ToF chain in the tube (tof_tube_centering: at most 0.15 m from the route, dropped as soon
as the readings stop). The sensors were checked on 19: each side reacts alone, and the bus-7
sensor is on the drone's left.

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
    # The window route on the as-built course (tools/export_route.py --course course/course_asbuilt.json
    # --tube-exit-drop-in 42 --pad-from-south-in 26 --maze-top-north-m 0.04 --window-via-in 136 44):
    # planned >= 8.6 cm to every wall (M1's north end; M3 11.1, net 9.8, A1 19.6, window 21.0); flown
    # on the stub drone at both speed settings >= 8.4 cm. The via point keeps the downward rangefinder
    # (PX4's height reference) 28 cm off boulder 3, which would read 0.4 m short. The plan's route
    # (route_centreline_window.json) passes 3.6 cm from a real wall. Shining confirmed the as-built walls.
    ('route_file', 'gazebo_sim/course/route_centreline_window_asbuilt.json',
     'route to fly (repo-relative): the window route on the as-built course map'),
    # On the as-built course the gaps between M1's and M3's north ends and the north net are only
    # ~0.6 m, and the route crosses them 0.256 m from the net. navless.launch.py's 0.4 m would pull
    # those setpoints into the wall ends (stub drone: M1 -4.3 cm, M3 -1.2 cm).
    ('net_margin_m', '0.25', 'keep setpoints this far inside the net, m'),
    ('tof_tube_centering', 'true',
     'in the tube, steer toward the centre the side ToF sensors measure (at most 0.15 m from the route)'),
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
    ('tof', 'true', 'run the side ToF driver'),
    ('tube', 'true', 'run tube_node (needs tof)'),
    ('tof_min_range_m', '0.15', 'tube_node: reject closer readings (raise until a hand-spun prop never shows)'),
    ('relay', 'false', 'run the read-only dashboard relay (TCP 8775)'),
    ('relay_img_hz', '0.0', 'relay camera thumbnails per second; keep 0 in flight'),
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
                                    'odometry_topic', 'net_margin_m', 'tof_tube_centering')}.items(),
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
             '-p', f'expected_width:={TUBE_WIDTH_M}', '-p', 'width_tol:=0.15'],
        output='screen', name='tube_node', condition=IfCondition(LaunchConfiguration('tube')))
    relay = Node(
        package='navless_nav', executable='mirror_relay', name='navless_mirror_relay',
        output='screen', parameters=[{'img_hz': LaunchConfiguration('relay_img_hz')}],
        condition=IfCondition(LaunchConfiguration('relay')))
    return LaunchDescription(declared + [navless, tof, tube, relay])
