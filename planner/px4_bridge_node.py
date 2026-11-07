"""ROS 2 & PX4 uXRCE-DDS Trajectory Tracking Bridge (30 Hz) + 1 Hz EGO-Planner Node.

Strictly adheres to `AGENTS.md` and `AI_CONTEXT.md` rules:
- Uses ordinary ROS 2 (`rclpy`) and `px4_msgs` topics:
  Inputs:
    - `/fmu/out/vehicle_local_position_v1` (`px4_msgs/msg/VehicleLocalPosition`)
    - `/fmu/out/vehicle_status_v1` (`px4_msgs/msg/VehicleStatus`)
    - `/estimator/healthy` (`std_msgs/msg/Bool`)
    - `/visual_slam/tracking/odometry` (`nav_msgs/msg/Odometry`)
    - `/camera/depth/image_raw` (`sensor_msgs/msg/Image`)
    - `/camera/depth/camera_info` (`sensor_msgs/msg/CameraInfo`)
  Outputs:
    - `/fmu/in/offboard_control_mode` (`px4_msgs/msg/OffboardControlMode`)
    - `/fmu/in/trajectory_setpoint` (`px4_msgs/msg/TrajectorySetpoint`)
    - `/fmu/in/vehicle_command` (`px4_msgs/msg/VehicleCommand`)
- Preserves sensor/estimator health gates:
  Checks fresh telemetry timestamps, `xy_valid`, `z_valid`, and `/estimator/healthy`.
  Stops publishing setpoints immediately if authority is lost (pilot RC takeover)
  or telemetry becomes stale.
- Never arms without explicit team approval (`--approve-flight`).
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

from .config import PlannerConfig
from .frames import (
    clamp_altitude_enu,
    enu_to_ned_pos,
    enu_to_ned_vel,
    enu_yaw_to_ned_yaw,
    ned_to_enu_pos,
)
from .fsm_router import CourseStage, GlobalCourseRouter
from .grid_map import OccupancyGrid3D


# Standard PX4 VehicleStatus constants
ARMING_STATE_ARMED = 2
NAVIGATION_STATE_OFFBOARD = 14


@dataclass
class TelemetryHealthState:
    """Snapshot of PX4 & VIO estimator readiness and safety status."""

    last_local_pos_time_s: Optional[float] = None
    last_status_time_s: Optional[float] = None
    xy_valid: bool = False
    z_valid: bool = False
    estimator_healthy: bool = False
    arming_state: int = 0
    nav_state: int = 0
    was_in_offboard: bool = False
    pos_ned: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    vel_ned: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    heading_ned_rad: float = 0.0

    def check_authority_and_health(
        self,
        now_s: float,
        *,
        max_age_s: float = 0.50,
        require_xy_valid: bool = True,
        require_z_valid: bool = True,
        require_estimator_healthy: bool = True,
    ) -> Tuple[bool, str]:
        """Validate fresh telemetry, position validity flags, VIO health, and offboard authority."""
        if self.last_local_pos_time_s is None:
            return False, "missing_vehicle_local_position"
        if (now_s - self.last_local_pos_time_s) > max_age_s:
            return False, "stale_vehicle_local_position"
        if self.last_status_time_s is not None and (now_s - self.last_status_time_s) > max_age_s:
            return False, "stale_vehicle_status"
        if require_xy_valid and not self.xy_valid:
            return False, "xy_valid_false"
        if require_z_valid and not self.z_valid:
            return False, "z_valid_false"
        if require_estimator_healthy and not self.estimator_healthy:
            return False, "estimator_unhealthy"
        # Yield immediately if pilot takes over out of OFFBOARD while armed
        if self.was_in_offboard and self.arming_state == ARMING_STATE_ARMED and self.nav_state != NAVIGATION_STATE_OFFBOARD:
            return False, "pilot_takeover_or_offboard_lost"
        return True, "healthy"


@dataclass(frozen=True)
class PX4SetpointSample:
    """Evaluated 30-50 Hz PX4 setpoint ready for `px4_msgs/msg/TrajectorySetpoint`."""

    position_ned: Tuple[float, float, float]
    velocity_ned: Tuple[float, float, float]
    yaw_ned: float
    timestamp_us: int
    stage: CourseStage
    branch: str


class PX4TrajectoryTrackerCore:
    """Testable core tracker evaluating the active EGO B-spline at 30-50 Hz for PX4."""

    def __init__(
        self,
        router: GlobalCourseRouter,
        *,
        frame_convention: str = "enu_to_ned",
        launch_origin_enu: Optional[Tuple[float, float, float]] = None,
        lookahead_s: float = 0.25,
    ) -> None:
        self.router = router
        self.config = router.config
        self.frame_convention = frame_convention
        self.lookahead_s = float(lookahead_s)
        if launch_origin_enu is None:
            sp = self.config.checkpoints.start_pad
            self.launch_origin_enu: Tuple[float, float, float] = (float(sp[0]), float(sp[1]), 0.0)
        else:
            self.launch_origin_enu = launch_origin_enu
        self.telemetry = TelemetryHealthState()

    def update_local_position(
        self,
        now_s: float,
        x_ned: float,
        y_ned: float,
        z_ned: float,
        vx_ned: float,
        vy_ned: float,
        vz_ned: float,
        heading_rad: float,
        xy_valid: bool,
        z_valid: bool,
    ) -> None:
        self.telemetry.last_local_pos_time_s = now_s
        self.telemetry.pos_ned = (float(x_ned), float(y_ned), float(z_ned))
        self.telemetry.vel_ned = (float(vx_ned), float(vy_ned), float(vz_ned))
        self.telemetry.heading_ned_rad = float(heading_rad)
        self.telemetry.xy_valid = bool(xy_valid)
        self.telemetry.z_valid = bool(z_valid)

    def update_vehicle_status(self, now_s: float, arming_state: int, nav_state: int) -> None:
        self.telemetry.last_status_time_s = now_s
        self.telemetry.arming_state = int(arming_state)
        self.telemetry.nav_state = int(nav_state)
        if self.telemetry.nav_state == NAVIGATION_STATE_OFFBOARD:
            self.telemetry.was_in_offboard = True

    def update_estimator_health(self, healthy: bool) -> None:
        self.telemetry.estimator_healthy = bool(healthy)

    def sample_setpoint(
        self,
        now_s: float,
        *,
        external_window_confidence: Optional[float] = None,
    ) -> Tuple[Optional[PX4SetpointSample], str]:
        """Run safety gate, update 1 Hz router if due, and sample 30 Hz PX4 TrajectorySetpoint."""
        safety = self.config.safety
        ok, reason = self.telemetry.check_authority_and_health(
            now_s,
            max_age_s=safety.max_telemetry_age_s,
            require_xy_valid=safety.require_xy_valid,
            require_z_valid=safety.require_z_valid,
            require_estimator_healthy=safety.require_estimator_healthy,
        )
        if not ok:
            return None, reason

        drone_pos_enu = ned_to_enu_pos(
            self.telemetry.pos_ned,
            convention=self.frame_convention,
            origin_enu=self.launch_origin_enu,
        )
        decision = self.router.step(
            now_s,
            drone_pos_enu,
            external_window_confidence=external_window_confidence,
        )

        spline = decision.spline
        if spline is None:
            return None, f"no_feasible_spline ({decision.reason})"

        profile = self.router.profile_for_stage(decision.stage)
        t_rel = max(0.0, now_s - self.router.spline_start_time_s)
        t_eval = min(spline.duration, t_rel + self.lookahead_s)
        pos_enu = clamp_altitude_enu(
            spline.evaluate_position(t_eval),
            profile.z_min_enu,
            profile.z_max_enu,
        )
        vel_enu = spline.evaluate_velocity(t_eval)
        yaw_enu = spline.evaluate_yaw(t_eval)

        pos_ned = enu_to_ned_pos(
            pos_enu,
            convention=self.frame_convention,
            origin_enu=self.launch_origin_enu,
        )
        vel_ned = enu_to_ned_vel(vel_enu, convention=self.frame_convention)
        yaw_ned = enu_yaw_to_ned_yaw(yaw_enu, convention=self.frame_convention)

        sample = PX4SetpointSample(
            position_ned=pos_ned,
            velocity_ned=vel_ned,
            yaw_ned=yaw_ned,
            timestamp_us=int(now_s * 1_000_000),
            stage=decision.stage,
            branch=decision.selected_branch,
        )
        return sample, decision.reason


def decode_ros_depth_image(message) -> np.ndarray:
    """Decode a ROS `sensor_msgs/msg/Image` (`16UC1` mm or `32FC1` m) into a 2D float32 array in metres.

    Follows Chapter 03 of `dronehacks-flight-notebook.ipynb`.
    """
    h, w, step = int(message.height), int(message.width), int(message.step)
    dtype_code = {"16UC1": "u2", "32FC1": "f4"}.get(message.encoding)
    if dtype_code is None:
        raise ValueError(f"Unsupported depth encoding: {message.encoding} (expected 16UC1 or 32FC1)")
    dt = np.dtype((">" if message.is_bigendian else "<") + dtype_code)
    if step < w * dt.itemsize or len(message.data) != step * h:
        raise ValueError("Invalid depth image row step or buffer length")
    raw = np.ndarray((h, w), dtype=dt, buffer=bytes(message.data), strides=(step, dt.itemsize))
    scale = 0.001 if dtype_code == "u2" else 1.0
    return raw.astype(np.float32) * scale


def run_ros2_node(argv=None) -> int:
    """Entrypoint for running the live ROS 2 Jazzy / Humble EGO-Planner + PX4 bridge node."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parent / "config" / "course_config.json",
        help="Path to course_config.json",
    )
    parser.add_argument(
        "--prior",
        type=Path,
        default=None,
        help="Optional path to Koji's 3D map prior (.json or .obj)",
    )
    parser.add_argument("--fast-mode", action="store_true", help="Enable scaled velocity/acceleration fast mode")
    parser.add_argument("--sim-time", action="store_true", help="Enable ROS use_sim_time for FlyX Lab SITL")
    parser.add_argument(
        "--approve-flight",
        action="store_true",
        help="Explicit team flight approval required to publish setpoints onto /fmu/in/* (AGENTS.md)",
    )
    args = parser.parse_args(argv)

    cfg = PlannerConfig.from_json(args.config) if args.config.exists() else PlannerConfig()
    if args.fast_mode:
        cfg.fast_mode = True

    grid = OccupancyGrid3D(
        bounds_min_m=cfg.global_map_bounds_min_m,
        bounds_max_m=cfg.global_map_bounds_max_m,
        resolution_m=cfg.voxel_resolution_m,
        inflation_radius_m=cfg.obstacle_inflation_m,
        local_window_m=cfg.local_window_size_m,
    )
    if args.prior and args.prior.exists():
        if args.prior.suffix.lower() == ".obj":
            grid.load_prior_from_obj(args.prior)
        else:
            grid.load_prior_from_json(args.prior)

    router = GlobalCourseRouter(cfg, grid)
    tracker = PX4TrajectoryTrackerCore(router)

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import Bool
    from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleStatus

    rclpy.init()
    node = Node("ego_px4_planner_bridge")
    node.set_parameters([Parameter("use_sim_time", value=bool(args.sim_time))])

    px4_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=5,
    )

    cam_intrinsics: Dict[str, float] = {}

    def now_sec() -> float:
        return node.get_clock().now().nanoseconds * 1e-9

    def on_local_pos(msg: VehicleLocalPosition) -> None:
        tracker.update_local_position(
            now_sec(),
            msg.x,
            msg.y,
            msg.z,
            msg.vx,
            msg.vy,
            msg.vz,
            msg.heading,
            msg.xy_valid,
            msg.z_valid,
        )

    def on_status(msg: VehicleStatus) -> None:
        tracker.update_vehicle_status(now_sec(), msg.arming_state, msg.nav_state)

    def on_estimator_health(msg: Bool) -> None:
        tracker.update_estimator_health(bool(msg.data))

    def on_camera_info(msg: CameraInfo) -> None:
        if len(msg.k) >= 6:
            cam_intrinsics["fx"] = float(msg.k[0])
            cam_intrinsics["cx"] = float(msg.k[2])
            cam_intrinsics["fy"] = float(msg.k[4])
            cam_intrinsics["cy"] = float(msg.k[5])

    def on_depth(msg: Image) -> None:
        if "fx" not in cam_intrinsics or tracker.telemetry.last_local_pos_time_s is None:
            return
        try:
            depth_m = decode_ros_depth_image(msg)
            pos_enu = ned_to_enu_pos(
                tracker.telemetry.pos_ned,
                convention=tracker.frame_convention,
                origin_enu=tracker.launch_origin_enu,
            )
            yaw_enu = (0.5 * math.pi - tracker.telemetry.heading_ned_rad) if tracker.frame_convention == "enu_to_ned" else -tracker.telemetry.heading_ned_rad
            grid.update_from_depth_image(
                depth_m,
                cam_intrinsics["fx"],
                cam_intrinsics["fy"],
                cam_intrinsics["cx"],
                cam_intrinsics["cy"],
                pos_enu,
                yaw_enu,
            )
        except ValueError as err:
            node.get_logger().warning(f"Depth decode warning: {err}")

    node.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", on_local_pos, px4_qos)
    node.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", on_status, px4_qos)
    node.create_subscription(Bool, "/estimator/healthy", on_estimator_health, px4_qos)
    node.create_subscription(CameraInfo, "/camera/depth/camera_info", on_camera_info, px4_qos)
    node.create_subscription(Image, "/camera/depth/image_raw", on_depth, px4_qos)

    offboard_pub = node.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
    setpoint_pub = node.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)

    def timer_callback() -> None:
        t_now = now_sec()
        sample, status_reason = tracker.sample_setpoint(t_now)
        if sample is None:
            node.get_logger().debug(f"Planner holding publishing: {status_reason}")
            return
        if not args.approve_flight:
            # Read-only / dry-run planning mode until team explicitly approves flight (AGENTS.md)
            return

        ocm = OffboardControlMode()
        ocm.position = True
        ocm.velocity = True
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        ocm.timestamp = sample.timestamp_us
        offboard_pub.publish(ocm)

        sp = TrajectorySetpoint()
        sp.position = [float(sample.position_ned[0]), float(sample.position_ned[1]), float(sample.position_ned[2])]
        sp.velocity = [float(sample.velocity_ned[0]), float(sample.velocity_ned[1]), float(sample.velocity_ned[2])]
        sp.yaw = float(sample.yaw_ned)
        sp.timestamp = sample.timestamp_us
        setpoint_pub.publish(sp)

    period_s = 1.0 / max(10.0, cfg.tracker_rate_hz)
    timer = node.create_timer(period_s, timer_callback)
    node.get_logger().info(
        f"EGO-Planner PX4 Bridge started at {cfg.tracker_rate_hz:.1f} Hz "
        f"(fast_mode={cfg.fast_mode}, approve_flight={args.approve_flight})"
    )

    try:
        rclpy.spin(node)
    finally:
        node.destroy_timer(timer)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(run_ros2_node())
