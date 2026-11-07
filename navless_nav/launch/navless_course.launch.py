"""The whole competition stack for skybeta-11, from one launch file.

This is what the stack's `competitor` window starts once the drone's drone.env has
COMPETITOR_PKG=navless_nav and COMPETITOR_LAUNCH=navless_course.launch.py.

  flight_interface + course_mission   navless.launch.py, unchanged: the ONE node that commands
                                      the drone (/skybeta/setpoint/*). Nothing else here does.
  tube/tof_driver.py                  side ToF sensors -> /tube/left/range, /tube/right/range
  tube/tube_node.py                   -> /tube/offset (tube centring), published only when both
                                      readings fit the tube's width
  mirror_relay (relay:=true)          read-only telemetry for the laptop dashboard, port 8775

The ToF chain is recorded and shown on the dashboard, but course_mission does not steer on
it yet: that waits until the sensors are proven on the real drone and the steering has
flown in Gazebo with simulated ToF.

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
    ('route_file', 'gazebo_sim/course/route_centreline_window.json',
     'route to fly (repo-relative): the centreline route keeps >= 16.6 cm to every wall'),
    ('start_bearing_deg', '0.0', 'nose bearing on the launch mark at power-up (0 = north)'),
    ('cruise_speed_mps', '0.5', 'route speed on straights, m/s'),
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
                                    'odometry_topic')}.items(),
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
