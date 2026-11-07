"""ROS 2 Launch file for the DroneHacks 2026 EGO-Planner & PX4 Trajectory Setpoint Bridge."""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    pkg_dir = Path(__file__).resolve().parent.parent
    default_cfg = str(pkg_dir / "config" / "course_config.json")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_cfg),
            DeclareLaunchArgument("fast_mode", default_value="false"),
            DeclareLaunchArgument("sim_time", default_value="true"),
            DeclareLaunchArgument("approve_flight", default_value="false"),
            ExecuteProcess(
                cmd=[
                    "python3",
                    "-m",
                    "planner.px4_bridge_node",
                    "--config",
                    LaunchConfiguration("config"),
                    "--sim-time",
                ],
                cwd=str(pkg_dir.parent),
                output="screen",
            ),
        ]
    )
