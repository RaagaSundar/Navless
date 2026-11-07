"""ROS 2 Launch file for the DroneHacks 2026 EGO-Planner & `/skybeta` Setpoint Bridge."""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _launch_node(context, *args, **kwargs):
    pkg_dir = Path(__file__).resolve().parent.parent
    repo_dir = pkg_dir.parent
    cfg = LaunchConfiguration("config").perform(context)
    prior = LaunchConfiguration("prior").perform(context)
    skybeta_mode = LaunchConfiguration("skybeta_mode").perform(context).lower() in ("true", "1", "yes")
    approve_flight = LaunchConfiguration("approve_flight").perform(context).lower() in ("true", "1", "yes")
    sim_time = LaunchConfiguration("sim_time").perform(context).lower() in ("true", "1", "yes")
    fast_mode = LaunchConfiguration("fast_mode").perform(context).lower() in ("true", "1", "yes")
    stop_on_pad = LaunchConfiguration("stop_on_pad").perform(context).lower() in ("true", "1", "yes")
    start_bearing_deg = LaunchConfiguration("start_bearing_deg").perform(context)
    start_height_m = LaunchConfiguration("start_height_m").perform(context)

    cmd = [
        "python3",
        "-m",
        "planner.px4_bridge_node",
        "--config",
        cfg,
        "--prior",
        prior,
        "--start-bearing-deg",
        start_bearing_deg,
        "--start-height-m",
        start_height_m,
    ]
    if skybeta_mode:
        cmd.append("--skybeta-mode")
    if approve_flight:
        cmd.append("--approve-flight")
    if sim_time:
        cmd.append("--sim-time")
    if fast_mode:
        cmd.append("--fast-mode")
    if stop_on_pad:
        cmd.append("--stop-on-pad")

    return [
        ExecuteProcess(
            cmd=cmd,
            cwd=str(repo_dir),
            output="screen",
        )
    ]


def generate_launch_description() -> LaunchDescription:
    pkg_dir = Path(__file__).resolve().parent.parent
    default_cfg = str(pkg_dir / "config" / "course_config.json")
    default_prior = str(pkg_dir.parent / "dronehacks-course-asbuilt.obj")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_cfg),
            DeclareLaunchArgument("prior", default_value=default_prior),
            DeclareLaunchArgument("skybeta_mode", default_value="true"),
            DeclareLaunchArgument("approve_flight", default_value="false"),
            DeclareLaunchArgument("sim_time", default_value="false"),
            DeclareLaunchArgument("fast_mode", default_value="false"),
            DeclareLaunchArgument("stop_on_pad", default_value="false"),
            DeclareLaunchArgument("start_bearing_deg", default_value="0.0"),
            DeclareLaunchArgument("start_height_m", default_value="0.07"),
            OpaqueFunction(function=_launch_node),
        ]
    )
