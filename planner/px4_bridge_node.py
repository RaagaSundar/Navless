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
- Never arms without explicit team approval (`--approve-flight`). Without `--skybeta-mode`, approval
  runs `step_px4_mission`: preflight, Offboard + arm, takeoff, route, `NAV_LAND` on the pad,
  re-arm for fly-home, `NAV_LAND` at home (uses `/fmu/out/vehicle_land_detected`).
"""

from __future__ import annotations

import argparse
import json
import math
import signal
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

from .bspline import UniformBspline
from .config import PlannerConfig
from .frames import (
    clamp_altitude_enu,
    course_to_odom_pos,
    course_vel_to_odom_vel,
    enu_to_ned_pos,
    enu_to_ned_vel,
    enu_yaw_to_ned_yaw,
    enu_yaw_to_odom_quat,
    ned_to_enu_pos,
    ned_to_enu_vel,
    odom_to_course_pos,
    wrap_pi,
)
from .fsm_router import CourseStage, GlobalCourseRouter
from .grid_map import OccupancyGrid3D
from .nmpc import NMPCRollout


# Standard PX4 VehicleStatus constants
ARMING_STATE_ARMED = 2
NAVIGATION_STATE_OFFBOARD = 14
NAVIGATION_STATE_AUTO_LAND = 18
# px4_msgs VehicleCommand ids, kept here so the core stays importable without ROS
VEHICLE_CMD_NAV_LAND = 21
VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400


@dataclass
class TelemetryHealthState:
    """Snapshot of PX4 & VIO estimator readiness and safety status."""

    last_local_pos_time_s: Optional[float] = None
    last_status_time_s: Optional[float] = None
    xy_valid: bool = False
    z_valid: bool = False
    estimator_healthy: bool = False
    vslam_vo_state: Optional[int] = None
    position_jump_detected: bool = False
    geofence_breach_detected: bool = False
    last_jump_reason: str = ""
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
        """Validate fresh telemetry, position validity flags, VIO health, geofence, and offboard authority."""
        if self.geofence_breach_detected:
            return False, f"geofence_breach ({self.last_jump_reason})"
        if self.position_jump_detected:
            return False, f"estimator_position_jump ({self.last_jump_reason})"
        if self.vslam_vo_state is not None and self.vslam_vo_state != 1:
            return False, f"vslam_lost (vo_state={self.vslam_vo_state})"
        if self.last_local_pos_time_s is None:
            return False, "missing_vehicle_local_position"
        if (now_s - self.last_local_pos_time_s) > max_age_s:
            return False, "stale_vehicle_local_position"
        if self.last_status_time_s is not None and (now_s - self.last_status_time_s) > max(2.5, 4.0 * max_age_s):
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
    """Evaluated 30-50 Hz PX4 setpoint ready for `px4_msgs/msg/TrajectorySetpoint` or `/skybeta/setpoint/pose`."""

    position_ned: Tuple[float, float, float]
    velocity_ned: Tuple[float, float, float]
    yaw_ned: float
    timestamp_us: int
    stage: CourseStage
    branch: str
    yaw_enu: float = 0.0
    odom_orientation_xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class SkybetaMissionCommand:
    """Evaluated `/skybeta` command (`pose`, `twist`, or `none`) for `flight_interface`."""

    mode: str  # "pose", "twist", or "none"
    pos_odom: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    quat_xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    vel_odom: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    skybeta_state: str = "WAIT_FOR_OFFBOARD"
    stage: str = "STAGE_1_MAZE"
    reason: str = "idle"


@dataclass(frozen=True)
class PX4MissionCommand:
    """One tick of the plain-PX4 mission: an optional setpoint to stream plus VehicleCommands to send."""

    setpoint: Optional[PX4SetpointSample]
    commands: Tuple[Tuple[int, float, float], ...] = ()
    state: str = "PREFLIGHT"
    reason: str = "idle"


class PX4TrajectoryTrackerCore:
    """Testable core tracker evaluating the active EGO B-spline at 30-50 Hz for PX4 & `/skybeta`."""

    def __init__(
        self,
        router: GlobalCourseRouter,
        *,
        frame_convention: str = "enu_to_ned",
        launch_origin_enu: Optional[Tuple[float, float, float]] = None,
        lookahead_s: float = 0.25,
        start_bearing_deg: float = 0.0,
        start_height_m: float = 0.0,
        max_yaw_rate_rad_s: float = 1.05,
        max_lead_m: float = 0.48,
        touch_and_go_fly_home: bool = True,
        prearm_settle_s: float = 0.0,
        prearm_height_tol_m: float = 0.15,
        prearm_jitter_tol_m: float = 0.05,
    ) -> None:
        self.router = router
        self.config = router.config
        self.frame_convention = frame_convention
        self.lookahead_s = float(lookahead_s)
        self.start_bearing_deg = float(start_bearing_deg)
        self.start_height_m = float(start_height_m)
        self.max_yaw_rate_rad_s = float(max_yaw_rate_rad_s)
        self.max_lead_m = float(max_lead_m)
        self.carrot_t: float = 0.0
        self.last_carrot_spline: Optional[UniformBspline] = None
        self.last_carrot_time_s: Optional[float] = None
        self.last_nmpc_rollout: Optional[NMPCRollout] = None
        self.touch_and_go_fly_home = bool(touch_and_go_fly_home)
        self.prearm_settle_s = float(prearm_settle_s)
        self.prearm_height_tol_m = float(prearm_height_tol_m)
        self.prearm_jitter_tol_m = float(prearm_jitter_tol_m)
        if launch_origin_enu is None:
            sp = self.config.checkpoints.start_pad
            self.launch_origin_enu: Tuple[float, float, float] = (float(sp[0]), float(sp[1]), 0.0)
        else:
            self.launch_origin_enu = launch_origin_enu
        self.telemetry = TelemetryHealthState()
        self.raw_pos_ned: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.nav_zero_offset_ned: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.nav_zeroed: bool = False
        self.prearm_anchor_ned: Optional[Tuple[float, float, float]] = None
        self.prearm_stable_since_s: Optional[float] = None
        self.prearm_settled: bool = self.prearm_settle_s <= 0.0
        self.px4_skybeta_z_enu: Optional[float] = None
        self.px4_skybeta_z_time_s: Optional[float] = None
        self.direct_takeoff_complete: bool = False
        self.pad_land_in_progress: bool = False
        self.pad_landing_completed: bool = False
        self.unhealthy_since_s: Optional[float] = None
        self.last_sample_time_s: Optional[float] = None
        self.last_tube_offset_time_s: Optional[float] = None
        self.tube_offset_left_m: float = 0.0
        self.tube_width_m: float = 0.0
        self.last_localize_time_s: Optional[float] = None
        self.last_localize_status_time_s: Optional[float] = None
        self.localized_pos_enu: Optional[Tuple[float, float, float]] = None
        self.localized_yaw_enu_rad: float = 0.0
        self.localize_status: str = "tracking"
        self.localize_measured_xy: Tuple[float, float] = (1.0, 1.0)
        self.commanded_yaw_enu_rad: Optional[float] = None
        self.last_yaw_update_time_s: Optional[float] = None
        # `/skybeta` flight_interface state machine
        self.skybeta_state: str = "WAIT_FOR_OFFBOARD"
        self.landed_flag: bool = False
        self.pad_touchdown_since_s: Optional[float] = None
        self.last_safe_course_pos_enu: Optional[Tuple[float, float, float]] = None
        # Plain-PX4 (`/fmu/in/*`) mission state machine, see `step_px4_mission`
        self.px4_state: str = "PREFLIGHT"
        self.px4_leg: str = "course"  # "course" -> land on pad, then "home" -> land on start pad
        self.px4_state_since_s: float = 0.0
        self.px4_last_cmd_s: float = -1e9
        self.px4_unhealthy_since_s: Optional[float] = None
        self.px4_takeoff_enu: Optional[Tuple[float, float, float]] = None
        self.last_landed_time_s: Optional[float] = None
        # Operator stop (SIGINT/SIGTERM, `planner/real_flight_kill.sh`): land where we are, then finish
        self.stop_requested_s: Optional[float] = None
        self.stop_hold_xy_enu: Optional[Tuple[float, float]] = None

    def update_px4_skybeta_height(self, now_s: float, px4_z_enu: float) -> None:
        """Update PX4's `local_enu` z height from `/skybeta/state` for pre-arm height agreement."""
        if math.isfinite(px4_z_enu):
            self.px4_skybeta_z_enu = float(px4_z_enu)
            self.px4_skybeta_z_time_s = float(now_s)

    def update_vslam_status(self, now_s: float, vo_state: int) -> None:
        """Update `/visual_slam/status` (`vo_state == 1` means tracking)."""
        self.telemetry.vslam_vo_state = int(vo_state)
        self.telemetry.estimator_healthy = int(vo_state) == 1

    def check_prearm_navigation_ready(self, now_s: float) -> Tuple[bool, str]:
        """Verify 2-second pre-arm navigation stability, PX4/VIO height agreement, and zero-tare at launch."""
        if self.telemetry.last_local_pos_time_s is None:
            return False, "waiting_for_odometry"
        if (now_s - self.telemetry.last_local_pos_time_s) > self.config.safety.max_telemetry_age_s:
            self.prearm_stable_since_s = None
            self.prearm_settled = False
            return False, "stale_prearm_odometry"
        if not (self.telemetry.xy_valid and self.telemetry.z_valid and self.telemetry.estimator_healthy):
            self.prearm_stable_since_s = None
            self.prearm_settled = False
            return False, "prearm_estimator_unhealthy"
        if self.telemetry.vslam_vo_state is not None and self.telemetry.vslam_vo_state != 1:
            self.prearm_stable_since_s = None
            self.prearm_settled = False
            return False, f"prearm_vslam_not_tracking ({self.telemetry.vslam_vo_state})"

        # Check PX4 vs Odom height agreement when `/skybeta/state` provides `local_enu` z
        if (
            self.prearm_height_tol_m > 0.0
            and self.px4_skybeta_z_enu is not None
            and self.px4_skybeta_z_time_s is not None
            and (now_s - self.px4_skybeta_z_time_s) <= 1.0
        ):
            odom_z = -self.raw_pos_ned[2] - self.start_height_m
            tol = self.prearm_height_tol_m + (0.05 if self.prearm_settled else 0.0)
            if abs(self.px4_skybeta_z_enu - odom_z) > tol:
                self.prearm_stable_since_s = None
                self.prearm_settled = False
                return False, f"prearm_height_mismatch (dz={abs(self.px4_skybeta_z_enu - odom_z):.2f}m)"

        # Check stationary velocity and position stability on the pad
        vel_norm = math.hypot(*self.telemetry.vel_ned)
        if vel_norm > 0.20:
            self.prearm_anchor_ned = self.raw_pos_ned
            self.prearm_stable_since_s = now_s
            self.prearm_settled = self.prearm_settle_s <= 0.0
            return self.prearm_settled, f"prearm_velocity_not_zero ({vel_norm:.2f}m/s)"

        if self.prearm_anchor_ned is None or math.dist(self.raw_pos_ned, self.prearm_anchor_ned) > self.prearm_jitter_tol_m:
            self.prearm_anchor_ned = self.raw_pos_ned
            self.prearm_stable_since_s = now_s
            self.prearm_settled = self.prearm_settle_s <= 0.0
            if not self.prearm_settled:
                return False, "prearm_position_settling"

        if self.prearm_stable_since_s is None:
            self.prearm_stable_since_s = now_s
        if (now_s - self.prearm_stable_since_s) < self.prearm_settle_s:
            return False, f"prearm_settling ({now_s - self.prearm_stable_since_s:.1f}/{self.prearm_settle_s:.1f}s)"

        # Zero-tare the initial launch pad offset once settled before first takeoff
        if not self.nav_zeroed and not self.router.primary_landed:
            rx, ry, rz = self.raw_pos_ned
            if math.hypot(rx, ry) <= 0.45 and abs(rz) <= 0.35:
                expected_z_ned = 0.0 if abs(self.launch_origin_enu[2]) > 1e-4 else -self.start_height_m
                self.nav_zero_offset_ned = (rx, ry, rz - expected_z_ned)
                self.telemetry.pos_ned = (
                    rx - self.nav_zero_offset_ned[0],
                    ry - self.nav_zero_offset_ned[1],
                    rz - self.nav_zero_offset_ned[2],
                )
            self.nav_zeroed = True

        self.prearm_settled = True
        return True, "prearm_ready_and_zeroed"

    def update_localize_status(
        self,
        status: str,
        *,
        measured_x: float = 1.0,
        measured_y: float = 1.0,
        now_s: Optional[float] = None,
    ) -> None:
        """Update status and measurement confidence from `/team/localize/status`."""
        if status:
            self.localize_status = str(status).strip().lower()
        self.localize_measured_xy = (float(measured_x), float(measured_y))
        if now_s is not None:
            self.last_localize_status_time_s = float(now_s)

    def update_localized_course_pose(
        self,
        now_s: float,
        x_course_m: float,
        y_course_m: float,
        z_course_m: float,
        yaw_enu_rad: float = 0.0,
    ) -> None:
        """Update course-frame pose from `/team/localize/base_pose` or `/team/localize/pose` (`depthcloud/cloud_localize.py`).

        Smooths step corrections (limiting per-update jump to 8 cm) so setpoints never jump.
        """
        if not (math.isfinite(x_course_m) and math.isfinite(y_course_m) and math.isfinite(z_course_m)):
            return
        if self.localized_pos_enu is not None and self.last_localize_time_s is not None and (now_s - self.last_localize_time_s) <= 2.0:
            prev = np.asarray(self.localized_pos_enu, dtype=np.float64)
            raw = np.array([x_course_m, y_course_m, z_course_m], dtype=np.float64)
            diff = raw - prev
            dist = float(np.linalg.norm(diff))
            if dist > 0.08:
                raw = prev + (0.08 / dist) * diff
            self.localized_pos_enu = (float(raw[0]), float(raw[1]), float(raw[2]))
        else:
            self.localized_pos_enu = (float(x_course_m), float(y_course_m), float(z_course_m))
        self.localized_yaw_enu_rad = float(yaw_enu_rad)
        self.last_localize_time_s = float(now_s)

    def current_course_pose_enu(self, now_s: float) -> Tuple[np.ndarray, float]:
        """Return `(pos_enu, yaw_enu)` in the course frame, preferring `/team/localize/base_pose` when fresh and measured."""
        status_fresh = (
            self.last_localize_status_time_s is None
            or (now_s - self.last_localize_status_time_s) <= 2.0
        )
        # Only trust `/team/localize/base_pose` when status is `"tracking"` (high ICP fitness);
        # `"weak"` frames in master_sim had 45-103 cm errors (@sim 1794067145).
        status_ok = status_fresh and (self.localize_status == "tracking")
        if (
            self.localized_pos_enu is not None
            and self.last_localize_time_s is not None
            and (now_s - self.last_localize_time_s) <= 1.5
            and status_ok
        ):
            return np.asarray(self.localized_pos_enu, dtype=np.float64), float(self.localized_yaw_enu_rad)
        pos_enu = ned_to_enu_pos(
            self.telemetry.pos_ned,
            convention=self.frame_convention,
            origin_enu=self.launch_origin_enu,
        )
        yaw_enu = (
            (0.5 * math.pi - self.telemetry.heading_ned_rad)
            if self.frame_convention == "enu_to_ned"
            else -self.telemetry.heading_ned_rad
        )
        return pos_enu, float(yaw_enu)

    def update_tube_offset(self, now_s: float, offset_left_m: float, width_m: float = 0.0) -> None:
        """Update lateral tunnel-centering offset `(L - R) / 2` from `/tube/offset` (`tube/tube_node.py`)."""
        if math.isfinite(offset_left_m):
            self.last_tube_offset_time_s = float(now_s)
            self.tube_offset_left_m = float(offset_left_m)
            self.tube_width_m = float(width_m)

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
        self.raw_pos_ned = (float(x_ned), float(y_ned), float(z_ned))
        zeroed_ned = (
            float(x_ned) - self.nav_zero_offset_ned[0],
            float(y_ned) - self.nav_zero_offset_ned[1],
            float(z_ned) - self.nav_zero_offset_ned[2],
        )
        prev_t = self.telemetry.last_local_pos_time_s
        if (
            self.telemetry.arming_state == ARMING_STATE_ARMED
            and prev_t is not None
            and math.isfinite(zeroed_ned[0])
            and math.isfinite(zeroed_ned[1])
            and math.isfinite(zeroed_ned[2])
        ):
            dt = float(now_s - prev_t)
            if 0.001 <= dt <= 0.50:
                step_m = math.dist(zeroed_ned, self.telemetry.pos_ned)
                vel_norm = math.hypot(float(vx_ned), float(vy_ned), float(vz_ned))
                if step_m > max(0.85, 4.5 * dt) or vel_norm > 5.0:
                    self.telemetry.position_jump_detected = True
                    self.telemetry.last_jump_reason = f"step={step_m:.2f}m/{dt:.2f}s, vel={vel_norm:.2f}m/s"
            pos_enu_chk = ned_to_enu_pos(
                zeroed_ned,
                convention=self.frame_convention,
                origin_enu=self.launch_origin_enu,
            )
            if (
                abs(float(pos_enu_chk[0])) > 3.20
                or abs(float(pos_enu_chk[1])) > 3.20
                or float(pos_enu_chk[2]) < -0.35
                or float(pos_enu_chk[2]) > 2.65
            ):
                self.telemetry.geofence_breach_detected = True
                self.telemetry.last_jump_reason = (
                    f"pos_enu=({pos_enu_chk[0]:.2f},{pos_enu_chk[1]:.2f},{pos_enu_chk[2]:.2f})"
                )
        self.telemetry.last_local_pos_time_s = now_s
        self.telemetry.pos_ned = zeroed_ned
        self.telemetry.vel_ned = (float(vx_ned), float(vy_ned), float(vz_ned))
        self.telemetry.heading_ned_rad = float(heading_rad)
        self.telemetry.xy_valid = bool(xy_valid)
        self.telemetry.z_valid = bool(z_valid)

    def update_vehicle_status(
        self,
        now_s: float,
        arming_state: int,
        nav_state: int,
        *,
        landed: Optional[bool] = None,
    ) -> None:
        self.telemetry.last_status_time_s = now_s
        self.telemetry.arming_state = int(arming_state)
        self.telemetry.nav_state = int(nav_state)
        if self.telemetry.arming_state != ARMING_STATE_ARMED:
            self.direct_takeoff_complete = False
            self.telemetry.position_jump_detected = False
            self.telemetry.geofence_breach_detected = False
        if self.telemetry.nav_state == NAVIGATION_STATE_OFFBOARD:
            self.telemetry.was_in_offboard = True
        if landed is not None:
            self.landed_flag = bool(landed)

    def update_estimator_health(self, healthy: bool) -> None:
        self.telemetry.estimator_healthy = bool(healthy)

    def to_px4_frame(self, position_ned: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Undo the pre-arm zero-tare for an outgoing setpoint: PX4 flies in its own (raw) local frame."""
        ox, oy, oz = self.nav_zero_offset_ned
        return (float(position_ned[0]) + ox, float(position_ned[1]) + oy, float(position_ned[2]) + oz)

    def _course_to_odom(self, pos_enu) -> Tuple[float, float, float]:
        """Course ENU -> `/skybeta/setpoint/pose` odom, undoing the pre-arm zero-tare like `to_px4_frame`:
        flight_interface flies raw cuVSLAM odom, not the tared frame the tracker works in."""
        tare_enu = ned_to_enu_vel(self.nav_zero_offset_ned, convention=self.frame_convention)
        raw_enu = (float(pos_enu[0]) + tare_enu[0], float(pos_enu[1]) + tare_enu[1], float(pos_enu[2]) + tare_enu[2])
        return course_to_odom_pos(
            raw_enu, (self.launch_origin_enu[0], self.launch_origin_enu[1]), self.start_bearing_deg, self.start_height_m
        )

    def request_stop(self, now_s: float) -> None:
        """Operator stop: in the air, land where we are (skybeta: descend; plain PX4: NAV_LAND), then finish
        without re-arming; on the ground, finish now. Idempotent."""
        if self.stop_requested_s is None:
            self.stop_requested_s = float(now_s)

    def stop_finished(self) -> bool:
        """True once a requested stop has nothing left to fly."""
        return self.stop_requested_s is not None and (
            self.skybeta_state == "DONE" or self.px4_state in ("DONE", "ABORT")
        )

    def update_landed(self, now_s: float, landed: bool) -> None:
        """PX4 `vehicle_land_detected` (slow: ~0.8 Hz in the FlyX lab, so freshness allows 2.5 s)."""
        self.landed_flag = bool(landed)
        self.last_landed_time_s = float(now_s)

    def _rate_limit_yaw_enu(self, now_s: float, target_yaw_enu_rad: float) -> float:
        """Rate-limit camera yaw heading changes to `max_yaw_rate_rad_s` so cuVSLAM tracks smoothly."""
        if (
            self.commanded_yaw_enu_rad is None
            or self.last_yaw_update_time_s is None
            or (now_s - self.last_yaw_update_time_s) > 1.0
        ):
            self.commanded_yaw_enu_rad = float(wrap_pi(target_yaw_enu_rad))
            self.last_yaw_update_time_s = float(now_s)
            return self.commanded_yaw_enu_rad
        dt = max(0.0, min(0.25, float(now_s - self.last_yaw_update_time_s)))
        self.last_yaw_update_time_s = float(now_s)
        max_step = self.max_yaw_rate_rad_s * dt
        err = wrap_pi(float(target_yaw_enu_rad) - self.commanded_yaw_enu_rad)
        step = float(np.clip(err, -max_step, max_step))
        self.commanded_yaw_enu_rad = float(wrap_pi(self.commanded_yaw_enu_rad + step))
        return self.commanded_yaw_enu_rad

    def sample_setpoint(
        self,
        now_s: float,
        *,
        external_window_confidence: Optional[float] = None,
    ) -> Tuple[Optional[PX4SetpointSample], str]:
        """Run safety gate, update 1 Hz router if due, and sample 30 Hz PX4 TrajectorySetpoint."""
        dt_sample = (
            max(0.0, min(0.25, float(now_s - self.last_sample_time_s)))
            if self.last_sample_time_s is not None
            else 0.0
        )
        self.last_sample_time_s = float(now_s)

        # Transition from pad AUTO_LAND to pad re-arm once PX4 touches down and disarms on the Concentric Pad
        if self.pad_land_in_progress and self.telemetry.arming_state != ARMING_STATE_ARMED:
            self.pad_land_in_progress = False
            self.pad_landing_completed = True
            self.telemetry.was_in_offboard = False
            if not self.config.enable_fly_home:
                self.router.stage = CourseStage.MISSION_COMPLETE

        safety = self.config.safety
        ok, reason = self.telemetry.check_authority_and_health(
            now_s,
            max_age_s=safety.max_telemetry_age_s,
            require_xy_valid=safety.require_xy_valid,
            require_z_valid=safety.require_z_valid,
            require_estimator_healthy=safety.require_estimator_healthy,
        )
        # 1.0-second in-flight VIO / estimator health grace period when called in direct `--pilot-arms` mode:
        if not ok:
            if (
                self.px4_state == "PREFLIGHT"
                and self.telemetry.arming_state == ARMING_STATE_ARMED
                and self.last_safe_course_pos_enu is not None
                and reason.startswith(("vslam_lost", "estimator_unhealthy", "xy_valid_false", "z_valid_false"))
            ):
                if self.unhealthy_since_s is None:
                    self.unhealthy_since_s = float(now_s)
                if (now_s - self.unhealthy_since_s) <= 1.0:
                    self.router.spline_start_time_s += dt_sample
                    hold_enu = np.asarray(self.last_safe_course_pos_enu, dtype=np.float64)
                    yaw_enu = self.commanded_yaw_enu_rad if self.commanded_yaw_enu_rad is not None else 0.0
                    pos_ned = enu_to_ned_pos(hold_enu, convention=self.frame_convention, origin_enu=self.launch_origin_enu)
                    vel_ned = enu_to_ned_vel((0.0, 0.0, 0.0), convention=self.frame_convention)
                    yaw_ned = enu_yaw_to_ned_yaw(yaw_enu, convention=self.frame_convention)
                    odom_quat = enu_yaw_to_odom_quat(yaw_enu, self.start_bearing_deg)
                    return (
                        PX4SetpointSample(
                            position_ned=pos_ned,
                            velocity_ned=vel_ned,
                            yaw_ned=yaw_ned,
                            timestamp_us=int(now_s * 1_000_000),
                            stage=self.router.stage,
                            branch="VIO_GRACE_HOLD",
                            yaw_enu=yaw_enu,
                            odom_orientation_xyzw=odom_quat,
                        ),
                        f"vio_grace_hold ({reason}, {now_s - self.unhealthy_since_s:.2f}s)",
                    )
            return None, reason

        self.unhealthy_since_s = None
        if not self.prearm_settled and self.telemetry.arming_state == ARMING_STATE_ARMED:
            return None, "refusing_already_armed_before_prearm_settle"

        # 1. Pre-arm & pre-Offboard gate: never advance the B-spline clock while disarmed or outside Offboard!
        if (
            self.telemetry.arming_state != ARMING_STATE_ARMED
            or self.telemetry.nav_state != NAVIGATION_STATE_OFFBOARD
        ):
            if self.router.stage == CourseStage.MISSION_COMPLETE:
                return None, "mission_complete_disarmed"
            self.router.spline_start_time_s = now_s
            drone_pos_enu, current_yaw_enu = self.current_course_pose_enu(now_s)
            if not self.pad_landing_completed:
                prearm_ok, prearm_reason = self.check_prearm_navigation_ready(now_s)
                if not prearm_ok:
                    return None, prearm_reason
                takeoff_z = float(self.config.checkpoints.start_pad[2])
                takeoff_y_offset = 0.0 if self.launch_origin_enu[1] > 1.00 else 0.15
                pos_enu = np.array(
                    [float(self.launch_origin_enu[0]), float(self.launch_origin_enu[1]) + takeoff_y_offset, takeoff_z],
                    dtype=np.float64,
                )
            else:
                hop_z = float(self.config.checkpoints.fly_home_altitude_m)
                pos_enu = np.array(
                    [float(drone_pos_enu[0]), float(drone_pos_enu[1]), hop_z],
                    dtype=np.float64,
                )
            yaw_enu = self._rate_limit_yaw_enu(now_s, current_yaw_enu)
            pos_ned = enu_to_ned_pos(pos_enu, convention=self.frame_convention, origin_enu=self.launch_origin_enu)
            vel_ned = enu_to_ned_vel((0.0, 0.0, 0.0), convention=self.frame_convention)
            yaw_ned = enu_yaw_to_ned_yaw(yaw_enu, convention=self.frame_convention)
            odom_quat = enu_yaw_to_odom_quat(yaw_enu, self.start_bearing_deg)
            return (
                PX4SetpointSample(
                    position_ned=pos_ned,
                    velocity_ned=vel_ned,
                    yaw_ned=yaw_ned,
                    timestamp_us=int(now_s * 1_000_000),
                    stage=self.router.stage,
                    branch="PREARM_STREAM",
                    yaw_enu=yaw_enu,
                    odom_orientation_xyzw=odom_quat,
                ),
                "prearm_settled_and_zeroed_streaming",
            )

        drone_pos_enu, current_yaw_enu = self.current_course_pose_enu(now_s)

        # 2. Mandatory vertical takeoff climb gate in direct PX4 mode:
        # Climb straight up over the launch mark and hold initial yaw before starting lateral flight!
        force_takeoff_replan = False
        if self.router.stage == CourseStage.STAGE_1_MAZE and not self.direct_takeoff_complete:
            climb_done_z = 0.72 if self.launch_origin_enu[2] > 0.05 else 0.65
            if float(drone_pos_enu[2]) >= climb_done_z:
                self.direct_takeoff_complete = True
                force_takeoff_replan = True
            else:
                self.router.spline_start_time_s = now_s
                takeoff_z = float(self.config.checkpoints.start_pad[2])
                takeoff_y_offset = 0.0 if self.launch_origin_enu[1] > 1.00 else 0.15
                pos_enu = np.array(
                    [float(self.launch_origin_enu[0]), float(self.launch_origin_enu[1]) + takeoff_y_offset, takeoff_z],
                    dtype=np.float64,
                )
                hold_yaw = self.commanded_yaw_enu_rad if self.commanded_yaw_enu_rad is not None else current_yaw_enu
                yaw_enu = self._rate_limit_yaw_enu(now_s, hold_yaw)
                self.last_safe_course_pos_enu = (float(pos_enu[0]), float(pos_enu[1]), float(pos_enu[2]))
                pos_ned = enu_to_ned_pos(pos_enu, convention=self.frame_convention, origin_enu=self.launch_origin_enu)
                vel_ned = enu_to_ned_vel((0.0, 0.0, 0.35), convention=self.frame_convention)
                yaw_ned = enu_yaw_to_ned_yaw(yaw_enu, convention=self.frame_convention)
                odom_quat = enu_yaw_to_odom_quat(yaw_enu, self.start_bearing_deg)
                return (
                    PX4SetpointSample(
                        position_ned=pos_ned,
                        velocity_ned=vel_ned,
                        yaw_ned=yaw_ned,
                        timestamp_us=int(now_s * 1_000_000),
                        stage=CourseStage.STAGE_1_MAZE,
                        branch="TAKEOFF_CLIMB",
                        yaw_enu=yaw_enu,
                        odom_orientation_xyzw=odom_quat,
                    ),
                    "climbing_to_takeoff_altitude",
                )

        decision = self.router.step(
            now_s,
            drone_pos_enu,
            external_window_confidence=external_window_confidence,
            force_replan=force_takeoff_replan,
        )

        # 3. Autonomous Concentric Pad Landing before Stage 6 Fly-Home:
        # When the primary route reaches the Concentric Pad (`router.primary_landed`), command PX4 AUTO_LAND
        # onto the pad until disarmed, then re-arm on the pad to fly home!
        if (
            self.px4_state == "PREFLIGHT"
            and self.router.primary_landed
            and not self.pad_landing_completed
            and self.prearm_settle_s > 0.0
            and not self.touch_and_go_fly_home
        ):
            self.pad_land_in_progress = True
            self.router.spline_start_time_s = now_s
            lp_enu, _ = self.router.landing_target_enu()
            yaw_enu = self.commanded_yaw_enu_rad if self.commanded_yaw_enu_rad is not None else current_yaw_enu
            pos_ned = enu_to_ned_pos(lp_enu, convention=self.frame_convention, origin_enu=self.launch_origin_enu)
            vel_ned = enu_to_ned_vel((0.0, 0.0, 0.0), convention=self.frame_convention)
            yaw_ned = enu_yaw_to_ned_yaw(yaw_enu, convention=self.frame_convention)
            odom_quat = enu_yaw_to_odom_quat(yaw_enu, self.start_bearing_deg)
            return (
                PX4SetpointSample(
                    position_ned=pos_ned,
                    velocity_ned=vel_ned,
                    yaw_ned=yaw_ned,
                    timestamp_us=int(now_s * 1_000_000),
                    stage=CourseStage.STAGE_5_LANDING_PAD,
                    branch="PAD_LAND",
                    yaw_enu=yaw_enu,
                    odom_orientation_xyzw=odom_quat,
                ),
                "landing_on_concentric_pad",
            )

        spline = decision.spline
        if spline is None:
            return None, f"no_feasible_spline ({decision.reason})"

        profile = self.router.profile_for_stage(decision.stage)
        z_min_clamp = 0.05 if decision.stage == CourseStage.STAGE_6_FLY_HOME else profile.z_min_enu
        xy_fence = safety.arena_half_extent_xy_m - safety.net_keepout_margin_m

        # Monotonic waiting-carrot advancement along the active B-spline (matching mission.py Carrot):
        # Never jump past a corner via closest-point projection when the drone has cross-track overshoot;
        # instead, slow down and shrink lead radius approaching turns, and wait at the current carrot point
        # whenever ||carrot - drone|| >= effective_lead_m!
        new_spline = (
            self.last_carrot_spline is not spline
            or (decision.replanned and decision.reason != "periodic_1hz_route_verified_clear")
        )
        if new_spline:
            t_prev = 0.0
            dt_carrot = self.lookahead_s
        else:
            t_prev = min(spline.duration, max(0.0, self.carrot_t))
            dt_carrot = (
                max(0.0, min(0.25, now_s - self.last_carrot_time_s))
                if self.last_carrot_time_s is not None
                else self.lookahead_s
            )

        yaw_bwd = spline.evaluate_yaw(max(0.0, t_prev - 0.30), fallback_yaw=current_yaw_enu)
        yaw_fwd1 = spline.evaluate_yaw(min(spline.duration, t_prev + 0.35), fallback_yaw=current_yaw_enu)
        yaw_fwd2 = spline.evaluate_yaw(min(spline.duration, t_prev + 0.70), fallback_yaw=current_yaw_enu)
        ang1 = abs(math.atan2(math.sin(yaw_fwd1 - yaw_bwd), math.cos(yaw_fwd1 - yaw_bwd)))
        ang2 = abs(math.atan2(math.sin(yaw_fwd2 - yaw_fwd1), math.cos(yaw_fwd2 - yaw_fwd1)))
        turn_angle = max(ang1, ang2)
        turn_ratio = min(1.0, max(0.0, (turn_angle - 0.20) / 0.70))
        turn_slow = 0.28 if self.config.fast_mode else 0.50
        turn_factor = 1.0 - turn_slow * turn_ratio
        base_lead_m = self.max_lead_m * (1.75 if self.config.fast_mode else 1.0)
        effective_lead_m = base_lead_m * turn_factor if base_lead_m > 0.0 else 0.0

        p_prev = clamp_altitude_enu(
            spline.evaluate_position(t_prev),
            z_min_clamp,
            profile.z_max_enu,
            xy_limit_m=xy_fence,
        )
        d_prev = float(np.linalg.norm(p_prev - drone_pos_enu))

        if effective_lead_m > 0.0 and not new_spline and d_prev >= effective_lead_m:
            t_eval = t_prev
            pos_enu = p_prev
        else:
            step_dt = dt_carrot if new_spline else dt_carrot * turn_factor
            t_cand = min(spline.duration, t_prev + step_dt)
            p_cand = clamp_altitude_enu(
                spline.evaluate_position(t_cand),
                z_min_clamp,
                profile.z_max_enu,
                xy_limit_m=xy_fence,
            )
            if effective_lead_m > 0.0 and float(np.linalg.norm(p_cand - drone_pos_enu)) > effective_lead_m:
                t_lo, t_hi = t_prev, t_cand
                for _ in range(10):
                    t_mid = 0.5 * (t_lo + t_hi)
                    pm = clamp_altitude_enu(
                        spline.evaluate_position(t_mid),
                        z_min_clamp,
                        profile.z_max_enu,
                        xy_limit_m=xy_fence,
                    )
                    if float(np.linalg.norm(pm - drone_pos_enu)) <= effective_lead_m:
                        t_lo = t_mid
                    else:
                        t_hi = t_mid
                t_eval = t_lo
                pos_enu = clamp_altitude_enu(
                    spline.evaluate_position(t_eval),
                    z_min_clamp,
                    profile.z_max_enu,
                    xy_limit_m=xy_fence,
                )
            else:
                t_eval = t_cand
                pos_enu = p_cand

            lead_vec = pos_enu - drone_pos_enu
            lead_dist = float(np.linalg.norm(lead_vec))
            if effective_lead_m > 0.0 and lead_dist > effective_lead_m:
                pos_enu = clamp_altitude_enu(
                    drone_pos_enu + (effective_lead_m / lead_dist) * lead_vec,
                    z_min_clamp,
                    profile.z_max_enu,
                    xy_limit_m=xy_fence,
                )

        self.carrot_t = t_eval
        self.last_carrot_spline = spline
        self.last_carrot_time_s = now_s
        self.router.spline_start_time_s = now_s - max(0.0, t_eval - self.lookahead_s)
        vel_enu = spline.evaluate_velocity(t_eval) * turn_factor
        t_yaw = max(0.0, t_eval - 0.15)
        raw_yaw_enu = spline.evaluate_yaw(t_yaw, fallback_yaw=current_yaw_enu)

        # Receding-Horizon 3D NMPC / MPCC short-term trajectory optimization from live (pos_enu, vel_enu)
        if str(getattr(self.config, "short_term_planner", "nmpc")).lower() == "nmpc" and hasattr(self.router, "nmpc"):
            if new_spline:
                self.router.nmpc._warm_U = None
            drone_vel_enu = ned_to_enu_vel(self.telemetry.vel_ned, convention=self.frame_convention)
            rollout = self.router.nmpc.solve_short_horizon(
                drone_pos_enu,
                drone_vel_enu,
                current_yaw_enu,
                spline,
                t_eval,
                profile,
                z_min_override=z_min_clamp,
                xy_limit_m=xy_fence,
                max_lead_m=effective_lead_m,
            )
            self.last_nmpc_rollout = rollout
            pos_enu = rollout.target_pos_enu
            vel_enu = rollout.target_vel_enu
            raw_yaw_enu = rollout.target_yaw_enu

        yaw_enu = self._rate_limit_yaw_enu(now_s, raw_yaw_enu)

        # If inside STAGE_3_TUNNEL and fresh `/tube/offset` is available, blend lateral centering nudge
        # using actual body/odom yaw (`current_yaw_enu`) since ToF ranges are measured in body frame
        if (
            decision.stage == CourseStage.STAGE_3_TUNNEL
            and self.last_tube_offset_time_s is not None
            and (now_s - self.last_tube_offset_time_s) <= 0.15
        ):
            left_x = -math.sin(current_yaw_enu)
            left_y = math.cos(current_yaw_enu)
            nudge = float(np.clip(0.5 * self.tube_offset_left_m, -0.15, 0.15))
            pos_enu = clamp_altitude_enu(
                np.array(
                    [pos_enu[0] + nudge * left_x, pos_enu[1] + nudge * left_y, pos_enu[2]],
                    dtype=np.float64,
                ),
                z_min_clamp,
                profile.z_max_enu,
                xy_limit_m=xy_fence,
            )

        self.last_safe_course_pos_enu = (float(pos_enu[0]), float(pos_enu[1]), float(pos_enu[2]))
        pos_ned = enu_to_ned_pos(
            pos_enu,
            convention=self.frame_convention,
            origin_enu=self.launch_origin_enu,
        )
        vel_ned = enu_to_ned_vel(vel_enu, convention=self.frame_convention)
        yaw_ned = enu_yaw_to_ned_yaw(yaw_enu, convention=self.frame_convention)
        odom_quat = enu_yaw_to_odom_quat(yaw_enu, self.start_bearing_deg)

        sample = PX4SetpointSample(
            position_ned=pos_ned,
            velocity_ned=vel_ned,
            yaw_ned=yaw_ned,
            timestamp_us=int(now_s * 1_000_000),
            stage=decision.stage,
            branch=decision.selected_branch,
            yaw_enu=yaw_enu,
            odom_orientation_xyzw=odom_quat,
        )
        return sample, decision.reason

    def _compute_flare_twist_odom(
        self,
        pos_enu: np.ndarray,
        target_xy_enu: Tuple[float, float],
        *,
        fast_vz_mps: float = 0.38,
        flare_vz_mps: float = 0.18,
        flare_alt_m: float = 0.35,
        xy_max_mps: float = 0.30,
    ) -> Tuple[Tuple[float, float, float], str]:
        """Compute two-stage vertical flare descent + XY pad-centering velocity in `odom` frame."""
        ex = float(target_xy_enu[0]) - float(pos_enu[0])
        ey = float(target_xy_enu[1]) - float(pos_enu[1])
        norm = math.hypot(ex, ey)
        scale = min(1.0, xy_max_mps / norm) if norm > 1e-6 else 0.0
        vx_enu = ex * scale
        vy_enu = ey * scale
        is_flare = float(pos_enu[2]) <= flare_alt_m
        vz_down = flare_vz_mps if is_flare else fast_vz_mps
        vel_odom = course_vel_to_odom_vel((vx_enu, vy_enu, -vz_down), self.start_bearing_deg)
        sub_reason = "ground_effect_flare" if is_flare else "nav_land_descent"
        return vel_odom, sub_reason

    def step_skybeta_mission(
        self,
        now_s: float,
        *,
        external_window_confidence: Optional[float] = None,
    ) -> SkybetaMissionCommand:
        """Advance the full `/skybeta` Offboard state machine with live EGO B-spline replanning.

        States:
        - `WAIT_FOR_OFFBOARD`: Pre-streams current position on `/skybeta/setpoint/pose` so
          `flight_interface` sees an active stream and grants `offboard_active` upon arming.
        - `TAKEOFF`: Climbs straight up above the launch mark (`0.85 m`) before starting the route.
        - `ROUTE`: Tracks the live 1 Hz + collision-avoidance EGO B-spline with smooth camera yaw
          alignment, `/team/localize/pose` corrections, and `/tube/offset` centering.
        - `PAD_LAND`: Two-stage controlled flare descent (`-0.38 m/s` -> `-0.18 m/s` below `0.35 m`)
          via `/skybeta/setpoint/twist` onto the Concentric Pad bullseye.
          - If `touch_and_go_fly_home=True` and `config.enable_fly_home=True`: touches down on the
            pad (`z <= 0.10 m`) for `0.80 s` (before PX4's `2.0 s` `COM_DISARM_LAND`), then climbs
            immediately into `FLY_HOME` without losing Offboard authority.
          - Otherwise: descends until PX4 disarms (`DONE`), or resumes into `FLY_HOME` if re-armed.
        - `FLY_HOME`: Climbs to `hop_z` (`~2.03 m`) and jumps across the dividing wall to the Start Pad.
        - `HOME_LAND`: Final two-stage controlled flare descent onto the Start Pad until `COM_DISARM_LAND`.
        - `PAUSED` / `DONE`: Safe pilot takeover hold and post-mission termination.
        """
        if self.telemetry.last_local_pos_time_s is None:
            return SkybetaMissionCommand(
                mode="none",
                skybeta_state=self.skybeta_state,
                stage=self.router.stage.value,
                reason="waiting_for_odometry",
            )

        launch_xy = (self.launch_origin_enu[0], self.launch_origin_enu[1])
        pos_enu, current_yaw_enu = self.current_course_pose_enu(now_s)
        pos_odom_cur = self._course_to_odom(pos_enu)
        offboard_active = self.telemetry.nav_state == NAVIGATION_STATE_OFFBOARD
        armed = self.telemetry.arming_state == ARMING_STATE_ARMED

        # Operator stop: while we own the vehicle (armed + Offboard), descend in place until PX4 disarms
        # (COM_DISARM_LAND). Stopping at touchdown instead could hand flight_interface an old pose setpoint
        # to climb back to. Not ours (disarmed, or the pilot took over): finish now.
        if self.stop_requested_s is not None and self.skybeta_state != "DONE":
            if not (armed and offboard_active):
                self.skybeta_state = "DONE"
                return SkybetaMissionCommand(
                    mode="none", skybeta_state="DONE", stage=self.router.stage.value, reason="operator_stop"
                )
            if self.stop_hold_xy_enu is None:
                self.stop_hold_xy_enu = (float(pos_enu[0]), float(pos_enu[1]))
            self.skybeta_state = "STOP_LAND"
            vel_odom, sub_rsn = self._compute_flare_twist_odom(pos_enu, self.stop_hold_xy_enu)
            return SkybetaMissionCommand(
                mode="twist",
                vel_odom=vel_odom,
                skybeta_state="STOP_LAND",
                stage=self.router.stage.value,
                reason=f"operator_stop_landing_{sub_rsn}",
            )

        if self.skybeta_state == "DONE":
            return SkybetaMissionCommand(
                mode="none",
                skybeta_state="DONE",
                stage=CourseStage.MISSION_COMPLETE.value,
                reason="mission_done",
            )

        # Detect Offboard loss or touchdown disarm in active flight states
        if self.skybeta_state not in ("WAIT_FOR_OFFBOARD", "PAUSED", "DONE") and not offboard_active:
            if self.skybeta_state in ("PAD_LAND", "HOME_LAND") and (self.landed_flag or not armed):
                if self.skybeta_state == "PAD_LAND" and self.config.enable_fly_home and not self.touch_and_go_fly_home:
                    # Wait in PAUSED so if the operator re-arms on the Concentric Pad we fly home
                    self.router.stage = CourseStage.STAGE_6_FLY_HOME
                    self.router.primary_landed = True
                    self.skybeta_state = "PAUSED"
                    return SkybetaMissionCommand(
                        mode="pose",
                        pos_odom=pos_odom_cur,
                        quat_xyzw=(0.0, 0.0, 0.0, 0.0),
                        skybeta_state="PAUSED",
                        stage=CourseStage.STAGE_6_FLY_HOME.value,
                        reason="landed_on_pad_waiting_rearm_for_fly_home",
                    )
                self.skybeta_state = "DONE"
                self.router.stage = CourseStage.MISSION_COMPLETE
                return SkybetaMissionCommand(
                    mode="none",
                    skybeta_state="DONE",
                    stage=CourseStage.MISSION_COMPLETE.value,
                    reason="touched_down_and_disarmed",
                )
            self.skybeta_state = "PAUSED"

        if self.skybeta_state == "PAUSED":
            if not armed:
                self.skybeta_state = "WAIT_FOR_OFFBOARD"
                self.prearm_stable_since_s = None
                self.prearm_settled = self.prearm_settle_s <= 0.0
            return SkybetaMissionCommand(
                mode="pose",
                pos_odom=pos_odom_cur,
                quat_xyzw=(0.0, 0.0, 0.0, 0.0),
                skybeta_state=self.skybeta_state,
                stage=self.router.stage.value,
                reason="paused_pilot_control",
            )

        if self.skybeta_state == "REFUSED":
            if not armed:
                self.skybeta_state = "WAIT_FOR_OFFBOARD"
                self.prearm_stable_since_s = None
                self.prearm_settled = self.prearm_settle_s <= 0.0
                return SkybetaMissionCommand(
                    mode="none",
                    skybeta_state="WAIT_FOR_OFFBOARD",
                    stage=self.router.stage.value,
                    reason="disarmed_after_refused",
                )
            return SkybetaMissionCommand(
                mode="twist",
                vel_odom=(0.0, 0.0, -0.30),
                skybeta_state="REFUSED",
                stage=self.router.stage.value,
                reason="refused_unsettled_prearm_descending",
            )

        if self.skybeta_state == "WAIT_FOR_OFFBOARD":
            self.router.spline_start_time_s = now_s
            prearm_ok, prearm_reason = self.check_prearm_navigation_ready(now_s)
            pos_enu, current_yaw_enu = self.current_course_pose_enu(now_s)
            pos_odom_cur = self._course_to_odom(pos_enu)
            if not offboard_active:
                if not prearm_ok:
                    return SkybetaMissionCommand(
                        mode="none",
                        skybeta_state="WAIT_FOR_OFFBOARD",
                        stage=self.router.stage.value,
                        reason=prearm_reason,
                    )
                if self.router.recorded_start_pos_enu is None:
                    self.router.recorded_start_pos_enu = (
                        float(pos_enu[0]),
                        float(pos_enu[1]),
                        float(self.config.checkpoints.start_pad[2]),
                    )
                return SkybetaMissionCommand(
                    mode="pose",
                    pos_odom=pos_odom_cur,
                    quat_xyzw=(0.0, 0.0, 0.0, 0.0),
                    skybeta_state="WAIT_FOR_OFFBOARD",
                    stage=self.router.stage.value,
                    reason="prestreaming_current_pose_for_offboard_handover",
                )
            if not prearm_ok:
                self.skybeta_state = "REFUSED"
                return SkybetaMissionCommand(
                    mode="twist",
                    vel_odom=(0.0, 0.0, -0.30),
                    skybeta_state="REFUSED",
                    stage=self.router.stage.value,
                    reason=f"refused_takeover ({prearm_reason})",
                )
            if self.router.recorded_start_pos_enu is None:
                self.router.recorded_start_pos_enu = (
                    float(pos_enu[0]),
                    float(pos_enu[1]),
                    float(self.config.checkpoints.start_pad[2]),
                )
            # Offboard just became active! If resuming from Concentric Pad after disarm, jump to FLY_HOME
            if self.router.primary_landed and self.config.enable_fly_home:
                self.skybeta_state = "FLY_HOME"
                self.router.stage = CourseStage.STAGE_6_FLY_HOME
                self.router.step(now_s, pos_enu, force_replan=True)
            else:
                self.skybeta_state = "TAKEOFF"

        if self.skybeta_state == "TAKEOFF":
            takeoff_z = float(self.config.checkpoints.start_pad[2])
            # Lean 0.20 m North up Corridor 1 during climb-out so PX4 tilt limit never drifts toward Barrier 1
            target_enu = (float(self.launch_origin_enu[0]), float(self.launch_origin_enu[1]) + 0.20, takeoff_z)
            if float(pos_enu[2]) >= (takeoff_z - 0.15) or float(np.linalg.norm(pos_enu - np.asarray(target_enu, dtype=np.float64))) <= 0.25:
                self.skybeta_state = "ROUTE"
                self.router.spline_start_time_s = now_s
            else:
                target_odom = self._course_to_odom(target_enu)
                yaw_quat = enu_yaw_to_odom_quat(self._rate_limit_yaw_enu(now_s, current_yaw_enu), self.start_bearing_deg)
                return SkybetaMissionCommand(
                    mode="pose",
                    pos_odom=target_odom,
                    quat_xyzw=yaw_quat,
                    skybeta_state="TAKEOFF",
                    stage=self.router.stage.value,
                    reason="climbing_to_takeoff_altitude",
                )

        if self.skybeta_state == "ROUTE":
            pad_goal_enu, _ = self.router.landing_target_enu()
            dist_xy_pad = float(math.hypot(pos_enu[0] - pad_goal_enu[0], pos_enu[1] - pad_goal_enu[1]))
            if (
                self.router.stage in (CourseStage.STAGE_5_LANDING_PAD, CourseStage.STAGE_6_FLY_HOME, CourseStage.MISSION_COMPLETE)
                and dist_xy_pad <= 0.35
            ):
                self.skybeta_state = "PAD_LAND"
                self.pad_touchdown_since_s = None
            else:
                sample, reason = self.sample_setpoint(
                    now_s,
                    external_window_confidence=external_window_confidence,
                )
                # If router just transitioned to STAGE_6_FLY_HOME upon reaching the pad, intercept for PAD_LAND!
                if self.router.stage in (CourseStage.STAGE_6_FLY_HOME, CourseStage.MISSION_COMPLETE) and dist_xy_pad <= 0.45:
                    self.skybeta_state = "PAD_LAND"
                    self.pad_touchdown_since_s = None
                elif sample is not None:
                    sp_enu = ned_to_enu_pos(
                        sample.position_ned,
                        convention=self.frame_convention,
                        origin_enu=self.launch_origin_enu,
                    )
                    sp_odom = self._course_to_odom(sp_enu)
                    return SkybetaMissionCommand(
                        mode="pose",
                        pos_odom=sp_odom,
                        quat_xyzw=sample.odom_orientation_xyzw,
                        skybeta_state="ROUTE",
                        stage=sample.stage.value,
                        reason=reason,
                    )
                else:
                    if self.telemetry.geofence_breach_detected or self.telemetry.position_jump_detected:
                        return SkybetaMissionCommand(
                            mode="twist",
                            vel_odom=(0.0, 0.0, -0.35),
                            skybeta_state="EMERGENCY_LAND",
                            stage=self.router.stage.value,
                            reason=f"emergency_land ({reason})",
                        )
                    # VIO lost or temporary hold: freeze at last safe setpoint
                    hold_enu = self.last_safe_course_pos_enu or (float(pos_enu[0]), float(pos_enu[1]), float(pos_enu[2]))
                    hold_odom = self._course_to_odom(hold_enu)
                    yaw_quat = enu_yaw_to_odom_quat(self.commanded_yaw_enu_rad or current_yaw_enu, self.start_bearing_deg)
                    return SkybetaMissionCommand(
                        mode="pose",
                        pos_odom=hold_odom,
                        quat_xyzw=yaw_quat,
                        skybeta_state="ROUTE",
                        stage=self.router.stage.value,
                        reason=f"vio_or_planner_hold ({reason})",
                    )

        if self.skybeta_state == "PAD_LAND":
            pad_goal_enu, _ = self.router.landing_target_enu()
            if self.touch_and_go_fly_home and self.config.enable_fly_home:
                if float(pos_enu[2]) <= 0.12:
                    if self.pad_touchdown_since_s is None:
                        self.pad_touchdown_since_s = float(now_s)
                    elif (now_s - self.pad_touchdown_since_s) >= 0.80:
                        self.router.primary_landed = True
                        self.router.stage = CourseStage.STAGE_6_FLY_HOME
                        self.router.step(now_s, pos_enu, force_replan=True)
                        self.skybeta_state = "FLY_HOME"
                else:
                    self.pad_touchdown_since_s = None
            if self.skybeta_state == "PAD_LAND":
                vel_odom, sub_rsn = self._compute_flare_twist_odom(
                    pos_enu,
                    (pad_goal_enu[0], pad_goal_enu[1]),
                    fast_vz_mps=0.38,
                    flare_vz_mps=0.18,
                )
                return SkybetaMissionCommand(
                    mode="twist",
                    vel_odom=vel_odom,
                    skybeta_state="PAD_LAND",
                    stage="STAGE_5_LANDING",
                    reason=f"pad_landing_{sub_rsn}",
                )

        if self.skybeta_state == "FLY_HOME":
            home_goal_enu = self.router.home_target_enu()
            dist_xy_home = float(math.hypot(pos_enu[0] - home_goal_enu[0], pos_enu[1] - home_goal_enu[1]))
            if self.router.stage == CourseStage.MISSION_COMPLETE or (dist_xy_home <= 0.35 and float(pos_enu[1]) > 0.20):
                self.skybeta_state = "HOME_LAND"
            else:
                sample, reason = self.sample_setpoint(now_s)
                if self.router.stage == CourseStage.MISSION_COMPLETE:
                    self.skybeta_state = "HOME_LAND"
                elif sample is not None:
                    sp_enu = ned_to_enu_pos(
                        sample.position_ned,
                        convention=self.frame_convention,
                        origin_enu=self.launch_origin_enu,
                    )
                    sp_odom = self._course_to_odom(sp_enu)
                    return SkybetaMissionCommand(
                        mode="pose",
                        pos_odom=sp_odom,
                        quat_xyzw=sample.odom_orientation_xyzw,
                        skybeta_state="FLY_HOME",
                        stage=CourseStage.STAGE_6_FLY_HOME.value,
                        reason=reason,
                    )
                else:
                    hold_enu = self.last_safe_course_pos_enu or (float(pos_enu[0]), float(pos_enu[1]), float(pos_enu[2]))
                    hold_odom = self._course_to_odom(hold_enu)
                    yaw_quat = enu_yaw_to_odom_quat(self.commanded_yaw_enu_rad or current_yaw_enu, self.start_bearing_deg)
                    return SkybetaMissionCommand(
                        mode="pose",
                        pos_odom=hold_odom,
                        quat_xyzw=yaw_quat,
                        skybeta_state="FLY_HOME",
                        stage=CourseStage.STAGE_6_FLY_HOME.value,
                        reason=f"vio_or_planner_hold ({reason})",
                    )

        if self.skybeta_state == "HOME_LAND":
            home_goal_enu = self.router.home_target_enu()
            vel_odom, sub_rsn = self._compute_flare_twist_odom(
                pos_enu,
                (home_goal_enu[0], home_goal_enu[1]),
                fast_vz_mps=0.45,
                flare_vz_mps=0.18,
            )
            return SkybetaMissionCommand(
                mode="twist",
                vel_odom=vel_odom,
                skybeta_state="HOME_LAND",
                stage="STAGE_6_LANDING_HOME",
                reason=f"home_landing_{sub_rsn}",
            )

        return SkybetaMissionCommand(
            mode="none",
            skybeta_state=self.skybeta_state,
            stage=self.router.stage.value,
            reason="idle",
        )

    # ---------- plain PX4 (`/fmu/in/*`) mission: takeoff, route, NAV_LAND ----------

    def _px4_enter(self, state: str, now_s: float) -> None:
        self.px4_state = state
        self.px4_state_since_s = float(now_s)
        self.px4_last_cmd_s = -1e9

    def _px4_hold(self, now_s: float, pos_enu, yaw_enu: float) -> PX4SetpointSample:
        """Position-only setpoint (velocity NaN = unset) at a course-ENU point."""
        return PX4SetpointSample(
            position_ned=enu_to_ned_pos(pos_enu, convention=self.frame_convention, origin_enu=self.launch_origin_enu),
            velocity_ned=(math.nan, math.nan, math.nan),
            yaw_ned=enu_yaw_to_ned_yaw(yaw_enu, convention=self.frame_convention),
            timestamp_us=int(now_s * 1_000_000),
            stage=self.router.stage,
            branch=self.router.selected_branch,
            yaw_enu=float(yaw_enu),
        )

    def _px4_cmd(self, now_s: float, *commands: Tuple[int, float, float]) -> Tuple[Tuple[int, float, float], ...]:
        """Send `commands` now and then at most once a second while the state lasts."""
        if now_s - self.px4_last_cmd_s < 1.0:
            return ()
        self.px4_last_cmd_s = float(now_s)
        return tuple(commands)

    def step_px4_mission(
        self,
        now_s: float,
        *,
        external_window_confidence: Optional[float] = None,
    ) -> PX4MissionCommand:
        """Plain-PX4 mission for the FlyX lab / any native `px4_msgs` link (not `--skybeta-mode`).

        `PREFLIGHT` (fresh healthy telemetry, landed, disarmed) -> `PRIME` (stream a ground hold >= 1 s,
        then request Offboard + arm) -> `ARMING` -> `TAKEOFF` (climb to the start-pad height, leaning
        0.20 m north as in `step_skybeta_mission`) -> `ROUTE` (live EGO B-spline via `sample_setpoint`) ->
        `LAND` (PX4 `NAV_LAND` over the Concentric Pad). With fly-home enabled, the disarmed drone on the
        pad is re-armed and flies `STAGE_6_FLY_HOME` straight from the pad, then `LAND`s at home -> `DONE`.
        Never takes over an already-armed vehicle; yields (`ABORT`, no more publishing) if PX4 leaves
        Offboard; lands if planner/estimator health is lost for more than 1 s in flight.
        """
        tel = self.telemetry
        state = self.px4_state
        if state in ("DONE", "ABORT"):
            return PX4MissionCommand(None, (), state, "finished")
        if self.stop_requested_s is not None and state != "LAND":  # operator stop
            if tel.arming_state != ARMING_STATE_ARMED:
                self._px4_enter("ABORT", now_s)
                return PX4MissionCommand(None, (), "ABORT", "operator_stop_on_ground")
            self._px4_enter("LAND", now_s)
            state = "LAND"
        if tel.last_local_pos_time_s is None or tel.last_status_time_s is None:
            return PX4MissionCommand(None, (), state, "waiting_for_px4_telemetry")

        safety = self.config.safety
        healthy, why = tel.check_authority_and_health(
            now_s,
            max_age_s=safety.max_telemetry_age_s,
            require_xy_valid=safety.require_xy_valid,
            require_z_valid=safety.require_z_valid,
            require_estimator_healthy=safety.require_estimator_healthy,
        )
        armed = tel.arming_state == ARMING_STATE_ARMED
        offboard = tel.nav_state == NAVIGATION_STATE_OFFBOARD
        landed = (
            self.landed_flag
            and self.last_landed_time_s is not None
            and (now_s - self.last_landed_time_s) <= 2.5
        )
        pos_enu, yaw_enu = self.current_course_pose_enu(now_s)
        land_cmd = (VEHICLE_CMD_NAV_LAND, 0.0, 0.0)

        if state == "PREFLIGHT":
            if armed:
                self._px4_enter("ABORT", now_s)
                return PX4MissionCommand(None, (), "ABORT", "armed_before_our_request")
            if not healthy or not landed:
                return PX4MissionCommand(None, (), state, f"preflight_wait ({why if not healthy else 'not_landed'})")
            settled, settle_why = self.check_prearm_navigation_ready(now_s)  # stationary + zero-tare
            if not settled:
                return PX4MissionCommand(None, (), state, f"preflight_wait ({settle_why})")
            pos_enu, yaw_enu = self.current_course_pose_enu(now_s)  # after any zero-tare
            self.px4_takeoff_enu = (float(pos_enu[0]), float(pos_enu[1]), float(pos_enu[2]))
            if self.router.recorded_start_pos_enu is None:
                self.router.recorded_start_pos_enu = (
                    float(pos_enu[0]),
                    float(pos_enu[1]),
                    float(self.config.checkpoints.start_pad[2]),
                )
            self.commanded_yaw_enu_rad = float(yaw_enu)
            self._px4_enter("PRIME", now_s)
            state = "PRIME"

        if state in ("PRIME", "ARMING"):
            hold = self._px4_hold(now_s, self.px4_takeoff_enu, self.commanded_yaw_enu_rad or yaw_enu)
            if state == "PRIME":
                if armed or not healthy or not landed:  # nothing sent yet: just start over
                    self._px4_enter("PREFLIGHT", now_s)
                    return PX4MissionCommand(None, (), "PREFLIGHT", f"prime_cancelled ({why})")
                if now_s - self.px4_state_since_s < 1.0:  # PX4 needs a live stream before Offboard
                    return PX4MissionCommand(hold, (), state, "prestreaming_ground_hold")
                self._px4_enter("ARMING", now_s)
            if armed and offboard:
                if self.px4_leg == "course":
                    self._px4_enter("TAKEOFF", now_s)
                    return PX4MissionCommand(hold, (), "TAKEOFF", "armed_offboard")
                self._px4_enter("ROUTE", now_s)
                self.router.spline_start_time_s = now_s
                self.router.step(now_s, pos_enu, force_replan=True)
                return PX4MissionCommand(hold, (), "ROUTE", "armed_offboard_fly_home")
            if now_s - self.px4_state_since_s > 8.0:
                nxt = "LAND" if armed else "ABORT"
                self._px4_enter(nxt, now_s)
                return PX4MissionCommand(None, (land_cmd,) if armed else (), nxt, "offboard_or_arm_refused")
            cmds = self._px4_cmd(
                now_s,
                (VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0),
                (VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0),
            )
            return PX4MissionCommand(hold, cmds, "ARMING", "offboard_and_arm_requested")

        if state in ("TAKEOFF", "ROUTE"):
            if not armed:
                self._px4_enter("ABORT", now_s)
                return PX4MissionCommand(None, (), "ABORT", "disarmed_in_flight")
            if not offboard:  # pilot or failsafe took over: never fight it
                self._px4_enter("ABORT", now_s)
                return PX4MissionCommand(None, (), "ABORT", f"left_offboard (nav_state={tel.nav_state})")
            if not healthy:
                if self.px4_unhealthy_since_s is None:
                    self.px4_unhealthy_since_s = float(now_s)
                if now_s - self.px4_unhealthy_since_s > 1.0:
                    self._px4_enter("LAND", now_s)
                    return PX4MissionCommand(None, self._px4_cmd(now_s, land_cmd), "LAND", f"unhealthy ({why})")
                if self.last_sample_time_s is not None:
                    self.router.spline_start_time_s += max(0.0, min(0.25, float(now_s - self.last_sample_time_s)))
                self.last_sample_time_s = float(now_s)
                hold_enu = self.last_safe_course_pos_enu or self.px4_takeoff_enu or tuple(pos_enu)
                return PX4MissionCommand(self._px4_hold(now_s, hold_enu, self.commanded_yaw_enu_rad or yaw_enu),
                                         (), state, f"hold ({why})")
            self.px4_unhealthy_since_s = None

        if state == "TAKEOFF":
            assert self.px4_takeoff_enu is not None
            takeoff_z = float(self.config.checkpoints.start_pad[2])
            north_nudge = 0.0 if self.launch_origin_enu[1] > 1.00 else 0.20
            target = (self.px4_takeoff_enu[0], self.px4_takeoff_enu[1] + north_nudge, takeoff_z)
            if float(pos_enu[2]) >= takeoff_z - 0.15:
                self._px4_enter("ROUTE", now_s)
                self.router.spline_start_time_s = now_s
                self.router.step(now_s, pos_enu, force_replan=True)
                state = "ROUTE"
            else:
                yaw = self._rate_limit_yaw_enu(now_s, self.commanded_yaw_enu_rad or yaw_enu)
                return PX4MissionCommand(self._px4_hold(now_s, target, yaw), (), state, "climbing_to_takeoff_altitude")

        if state == "ROUTE":
            if self.px4_leg == "course":
                pad_goal_enu, _ = self.router.landing_target_enu()
                d_land = float(math.hypot(pos_enu[0] - pad_goal_enu[0], pos_enu[1] - pad_goal_enu[1]))
                at_goal = self.router.stage in (
                    CourseStage.STAGE_5_LANDING_PAD,
                    CourseStage.STAGE_6_FLY_HOME,
                    CourseStage.MISSION_COMPLETE,
                ) and d_land <= 0.35
            else:
                home = self.router.home_target_enu()
                d_land = float(math.hypot(pos_enu[0] - home[0], pos_enu[1] - home[1]))
                at_goal = self.router.stage == CourseStage.MISSION_COMPLETE or (d_land <= 0.35 and float(pos_enu[1]) > 0.20)
            if at_goal:
                self._px4_enter("LAND", now_s)
                return PX4MissionCommand(None, self._px4_cmd(now_s, land_cmd), "LAND", f"over_{self.px4_leg}_landing_target")
            sample, reason = self.sample_setpoint(now_s, external_window_confidence=external_window_confidence)
            if sample is None:  # no feasible spline: hold, and land if it lasts
                if self.px4_unhealthy_since_s is None:
                    self.px4_unhealthy_since_s = float(now_s)
                if now_s - self.px4_unhealthy_since_s > 1.0:
                    self._px4_enter("LAND", now_s)
                    return PX4MissionCommand(None, self._px4_cmd(now_s, land_cmd), "LAND", f"planner_hold_timeout ({reason})")
                hold_enu = self.last_safe_course_pos_enu or tuple(pos_enu)
                return PX4MissionCommand(self._px4_hold(now_s, hold_enu, self.commanded_yaw_enu_rad or yaw_enu),
                                         (), state, f"hold ({reason})")
            return PX4MissionCommand(sample, (), state, reason)

        if state == "LAND":  # PX4 Land mode owns the vehicle; we only (re)send NAV_LAND and watch
            if armed:
                cmds = self._px4_cmd(now_s, land_cmd) if tel.nav_state != NAVIGATION_STATE_AUTO_LAND else ()
                # Until PX4 accepts NAV_LAND, keep the Offboard stream alive (as takeoff_land.py does)
                hold = self._px4_hold(now_s, tuple(pos_enu), self.commanded_yaw_enu_rad or yaw_enu) if offboard else None
                return PX4MissionCommand(hold, cmds, state, "landing")
            if not landed:
                if now_s - self.px4_state_since_s > 30.0:
                    self._px4_enter("ABORT", now_s)
                    return PX4MissionCommand(None, (), "ABORT", "disarmed_without_ground_confirmation")
                return PX4MissionCommand(None, (), state, "disarmed_waiting_for_land_detected")
            if self.px4_leg == "course" and self.config.enable_fly_home and self.stop_requested_s is None:
                self.px4_leg = "home"
                self.router.primary_landed = True
                self.router.stage = CourseStage.STAGE_6_FLY_HOME
                self._px4_enter("PREFLIGHT", now_s)
                return PX4MissionCommand(None, (), "PREFLIGHT", "landed_on_pad_rearming_for_fly_home")
            self.router.stage = CourseStage.MISSION_COMPLETE
            self._px4_enter("DONE", now_s)
            return PX4MissionCommand(None, (), "DONE", "landed_and_disarmed")

        return PX4MissionCommand(None, (), state, "idle")


def decode_ros_depth_image(message) -> np.ndarray:
    """Decode a ROS `sensor_msgs/msg/Image` (`16UC1` mm or `32FC1` m) into a 2D float32 array in metres.

    Follows Chapter 03 of `dronehacks-flight-notebook.ipynb` and supports `@depthcloud`'s
    `/team/stereo/depth` (`32FC1` metres, 512x320) as well as `/camera/depth/image_raw`.
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


def decode_ros_pointcloud2_optical(
    message,
    drone_pos_enu: np.ndarray | Tuple[float, float, float],
    drone_yaw_rad: float,
    *,
    stride: int = 8,
    min_depth_m: float = 0.30,
    max_depth_m: float = 3.50,
) -> np.ndarray:
    """Decode `@depthcloud`'s `/team/stereo/points` (`PointCloud2` in left camera optical frame) to world ENU.

    Optical frame: +Z forward, +X right, +Y down -> Body FLU: +X forward, +Y left (-X_opt), +Z up (-Y_opt).
    """
    point_step = int(message.point_step)
    if point_step < 12 or len(message.data) < point_step:
        return np.zeros((0, 3), dtype=np.float64)
    n_pts = len(message.data) // point_step
    endian = ">" if getattr(message, "is_bigendian", False) else "<"

    # Extract x, y, z float32 offsets from PointField list if present, otherwise standard (0, 4, 8)
    offsets = {"x": 0, "y": 4, "z": 8}
    for f in getattr(message, "fields", []) or []:
        if f.name in offsets:
            offsets[f.name] = int(f.offset)

    buf = bytes(message.data)
    idx = np.arange(0, n_pts, max(1, stride), dtype=np.int64)
    if idx.size == 0:
        return np.zeros((0, 3), dtype=np.float64)

    dt_f4 = np.dtype(endian + "f4")
    x_opt = np.ndarray((n_pts,), dtype=dt_f4, buffer=buf, offset=offsets["x"], strides=(point_step,))[idx].astype(np.float64)
    y_opt = np.ndarray((n_pts,), dtype=dt_f4, buffer=buf, offset=offsets["y"], strides=(point_step,))[idx].astype(np.float64)
    z_opt = np.ndarray((n_pts,), dtype=dt_f4, buffer=buf, offset=offsets["z"], strides=(point_step,))[idx].astype(np.float64)

    valid = np.isfinite(x_opt) & np.isfinite(y_opt) & np.isfinite(z_opt) & (z_opt >= min_depth_m) & (z_opt <= max_depth_m)
    if not np.any(valid):
        return np.zeros((0, 3), dtype=np.float64)

    x_flu = z_opt[valid]
    y_flu = -x_opt[valid]
    z_flu = -y_opt[valid]

    c, s = math.cos(drone_yaw_rad), math.sin(drone_yaw_rad)
    pos = np.asarray(drone_pos_enu, dtype=np.float64)
    x_w = pos[0] + c * x_flu - s * y_flu
    y_w = pos[1] + s * x_flu + c * y_flu
    z_w = pos[2] + z_flu
    pts_world = np.stack([x_w, y_w, z_w], axis=-1)
    in_course = (
        (pts_world[:, 2] >= 0.15)
        & (pts_world[:, 2] <= 1.70)
        & (np.abs(pts_world[:, 0]) <= 2.85)
        & (np.abs(pts_world[:, 1]) <= 2.85)
    )
    return pts_world[in_course]


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
        help="Explicit team flight approval required to publish setpoints (AGENTS.md)",
    )
    parser.add_argument(
        "--skybeta-mode",
        action="store_true",
        help="Publish to /skybeta/setpoint/{pose,twist} and subscribe to /visual_slam/tracking/odometry & /skybeta/state",
    )
    parser.add_argument(
        "--start-bearing-deg",
        type=float,
        default=0.0,
        help="Initial nose bearing clockwise from course North at power-up on the launch mark",
    )
    parser.add_argument(
        "--start-height-m",
        type=float,
        default=0.07,
        help="Height of cuVSLAM odom origin above the floor at power-up (m)",
    )
    parser.add_argument(
        "--stop-on-pad",
        action="store_true",
        help="Land and disarm on the Concentric Pad instead of touch-and-go Stage 6 Fly-Home",
    )
    parser.add_argument(
        "--touch-and-go",
        action="store_true",
        help="Perform 0.8 s touch-and-go on the Concentric Pad instead of full disarm + pilot re-arm",
    )
    parser.add_argument(
        "--use-cloud-localize",
        action="store_true",
        help="Fuse /team/localize/base_pose corrections into the live course frame",
    )
    parser.add_argument(
        "--live-depth-mapping",
        action="store_true",
        help="Insert live stereo depth/pointcloud into OccupancyGrid3D even when --prior mesh is loaded",
    )
    parser.add_argument(
        "--prearm-settle-s",
        type=float,
        default=2.0,
        help="Seconds to verify stationary, valid, height-matched navigation and zero-tare before arming",
    )
    parser.add_argument(
        "--launch-origin",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=None,
        help="Course ENU coordinates of PX4 local origin (e.g. -2.515 1.114 0.126 in FlyX Lab)",
    )
    parser.add_argument(
        "--pilot-arms",
        action="store_true",
        help="Direct PX4 mode: wait for an external pilot/script to switch to Offboard and arm, instead of "
        "the autonomous preflight -> arm -> takeoff -> route -> NAV_LAND sequence (step_px4_mission)",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=None,
        help="Optional override for horizontal stage cruise speed v_max (m/s)",
    )
    parser.add_argument(
        "--planner",
        choices=("nmpc", "ego"),
        default=None,
        help="Short-term trajectory planner mode: 'nmpc' (3D Receding-Horizon MPCC/NMPC) or 'ego' (EGO B-spline)",
    )
    args, unknown_args = parser.parse_known_args(argv)
    # Support `-p fly_home:=true` passed by master_sim/run_master.sh
    fly_home_arg = any("fly_home:=true" in str(u).lower() for u in unknown_args)

    cfg = PlannerConfig.from_json(args.config) if args.config.exists() else PlannerConfig()
    if args.planner is not None:
        cfg.short_term_planner = str(args.planner)
    if args.fast_mode:
        cfg.fast_mode = True
    if args.speed is not None and args.speed > 0.0:
        from dataclasses import replace

        v_override = float(args.speed)
        cfg.maze_profile = replace(cfg.maze_profile, v_max=v_override)
        cfg.tunnel_profile = replace(cfg.tunnel_profile, v_max=min(v_override, cfg.tunnel_profile.v_max))
        cfg.chicane_profile = replace(cfg.chicane_profile, v_max=v_override)
    if args.stop_on_pad and not fly_home_arg:
        cfg.enable_fly_home = False

    grid = OccupancyGrid3D(
        bounds_min_m=cfg.global_map_bounds_min_m,
        bounds_max_m=cfg.global_map_bounds_max_m,
        resolution_m=cfg.voxel_resolution_m,
        inflation_radius_m=cfg.obstacle_inflation_m,
        local_window_m=cfg.local_window_size_m,
    )
    prior_loaded = False
    default_json = Path(__file__).resolve().parent / "config" / "prior_course_map.json"
    default_obj = Path(__file__).resolve().parent.parent / "dronehacks-course-v3.obj"
    prior_path = (
        args.prior
        if (args.prior and args.prior.exists())
        else (default_json if default_json.exists() else (default_obj if default_obj.exists() else None))
    )
    if prior_path is not None and prior_path.exists():
        if prior_path.suffix.lower() == ".obj":
            grid.load_prior_from_obj(prior_path)
        else:
            grid.load_prior_from_json(prior_path)
        prior_loaded = True

    router = GlobalCourseRouter(cfg, grid)
    tracker = PX4TrajectoryTrackerCore(
        router,
        launch_origin_enu=(
            (float(args.launch_origin[0]), float(args.launch_origin[1]), float(args.launch_origin[2]))
            if args.launch_origin is not None
            else None
        ),
        start_bearing_deg=args.start_bearing_deg,
        start_height_m=(args.start_height_m if args.skybeta_mode else 0.0),
        touch_and_go_fly_home=bool(args.touch_and_go and not args.stop_on_pad),
        prearm_settle_s=args.prearm_settle_s,
    )

    import rclpy
    from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from std_msgs.msg import Bool, String
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleCommand,
        VehicleLandDetected,
        VehicleLocalPosition,
        VehicleStatus,
    )

    # Signals are ours, not rclpy's: its handler shuts the context down, after which nothing (no landing
    # setpoint, no NAV_LAND) can be published. First SIGINT/SIGTERM = controlled stop (land, then exit);
    # a second one = stop publishing now (KeyboardInterrupt below).
    try:
        from rclpy.signals import SignalHandlerOptions

        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    stop_signals: List[int] = []

    def _on_stop_signal(signum, _frame) -> None:
        stop_signals.append(int(signum))
        if len(stop_signals) >= 2:
            raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _on_stop_signal)
    signal.signal(signal.SIGTERM, _on_stop_signal)
    node = Node("ego_px4_planner_bridge")
    node.set_parameters([Parameter("use_sim_time", value=bool(args.sim_time))])

    px4_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=5,
    )

    cam_intrinsics: Dict[str, float] = {}
    last_fmu_pos_rx_s: List[float] = [-1e9]
    last_nav_land_cmd_s: List[float] = [-1e9]

    def now_sec() -> float:
        return node.get_clock().now().nanoseconds * 1e-9

    def on_local_pos(msg: VehicleLocalPosition) -> None:
        last_fmu_pos_rx_s[0] = now_sec()
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

    def on_land_detected(msg: VehicleLandDetected) -> None:
        tracker.update_landed(now_sec(), bool(msg.landed))

    def on_camera_info(msg: CameraInfo) -> None:
        if len(msg.k) >= 6:
            cam_intrinsics["fx"] = float(msg.k[0])
            cam_intrinsics["cx"] = float(msg.k[2])
            cam_intrinsics["fy"] = float(msg.k[4])
            cam_intrinsics["cy"] = float(msg.k[5])
            cam_intrinsics["width"] = float(msg.width) if msg.width > 0 else 0.0
            cam_intrinsics["height"] = float(msg.height) if msg.height > 0 else 0.0

    # With a prior map, live depth stays out of the planning grid unless asked for: its projection uses
    # yaw only (no tilt, mount offset or frame latency), so in flight it paints phantom walls into the
    # corridors -> constant "collision_avoidance_replan" and detours (FlyX lab, 2026-11-07).
    def on_depth(msg: Image) -> None:
        if (prior_loaded and not args.live_depth_mapping) or "fx" not in cam_intrinsics or tracker.telemetry.last_local_pos_time_s is None:
            return
        try:
            depth_m = decode_ros_depth_image(msg)
            # Support half-resolution depth images (e.g. 512x320 `/team/stereo/depth` vs 1024x640 `/oak/left/camera_info`)
            sx = (float(msg.width) / cam_intrinsics["width"]) if cam_intrinsics.get("width", 0.0) > 0 else 1.0
            sy = (float(msg.height) / cam_intrinsics["height"]) if cam_intrinsics.get("height", 0.0) > 0 else 1.0
            pos_enu, yaw_enu = tracker.current_course_pose_enu(now_sec())
            grid.update_from_depth_image(
                depth_m,
                cam_intrinsics["fx"] * sx,
                cam_intrinsics["fy"] * sy,
                cam_intrinsics["cx"] * sx,
                cam_intrinsics["cy"] * sy,
                pos_enu,
                yaw_enu,
                max_depth_m=2.80,
                min_live_hits=2,
            )
        except ValueError as err:
            node.get_logger().warning(f"Depth decode warning: {err}")

    def on_pointcloud(msg: PointCloud2) -> None:
        if (prior_loaded and not args.live_depth_mapping) or (
            tracker.telemetry.last_local_pos_time_s is None and tracker.last_localize_time_s is None
        ):
            return
        pos_enu, yaw_enu = tracker.current_course_pose_enu(now_sec())
        pts_world = decode_ros_pointcloud2_optical(msg, pos_enu, yaw_enu, max_depth_m=2.80)
        if pts_world.size > 0:
            grid.insert_point_cloud(pts_world, sensor_origin_m=pos_enu, min_live_hits=2)

    def on_tube_offset(msg: PointStamped) -> None:
        tracker.update_tube_offset(now_sec(), float(msg.point.y), float(msg.point.z))

    last_base_pose_rx_s: List[float] = [-1e9]
    last_validated_odom_rx_s: List[float] = [-1e9]
    last_vslam_status_rx_s: List[float] = [-1e9]

    def _apply_localize_msg(msg: PoseStamped) -> None:
        if prior_loaded and not args.use_cloud_localize:
            return
        p = msg.pose.position
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_enu = math.atan2(siny_cosp, cosy_cosp)
        tracker.update_localized_course_pose(now_sec(), float(p.x), float(p.y), float(p.z), yaw_enu)

    def on_localize_base_pose(msg: PoseStamped) -> None:
        last_base_pose_rx_s[0] = now_sec()
        _apply_localize_msg(msg)

    def on_localize_pose(msg: PoseStamped) -> None:
        # Prefer `/team/localize/base_pose` (drone base_link center) when active
        if (now_sec() - last_base_pose_rx_s[0]) <= 1.5:
            return
        _apply_localize_msg(msg)

    def on_localize_status(msg: String) -> None:
        try:
            st = json.loads(msg.data)
            meas = st.get("measured") or {}
            tracker.update_localize_status(
                str(st.get("status", "tracking")),
                measured_x=float(meas.get("x", 1.0)),
                measured_y=float(meas.get("y", 1.0)),
                now_s=now_sec(),
            )
        except (ValueError, TypeError):
            pass

    def _apply_odom_msg(msg: Odometry, *, from_validated_gate: bool) -> None:
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_odom = math.atan2(siny_cosp, cosy_cosp)
        launch_xy = (tracker.launch_origin_enu[0], tracker.launch_origin_enu[1])
        pos_enu = odom_to_course_pos(
            (float(p.x), float(p.y), float(p.z)),
            launch_xy,
            tracker.start_bearing_deg,
            tracker.start_height_m,
        )
        pos_ned = enu_to_ned_pos(pos_enu, origin_enu=tracker.launch_origin_enu)
        yaw_enu_0 = 0.5 * math.pi - math.radians(tracker.start_bearing_deg)
        yaw_enu = wrap_pi(yaw_enu_0 + yaw_odom)
        heading_ned = enu_yaw_to_ned_yaw(yaw_enu)
        tracker.update_local_position(
            now_sec(),
            float(pos_ned[0]),
            float(pos_ned[1]),
            float(pos_ned[2]),
            float(v.x),
            float(-v.y),
            float(-v.z),
            float(heading_ned),
            True,
            True,
        )
        # Take estimator health from Koji's gate or `/visual_slam/status` (`vo_state == 1`),
        # falling back to healthy only when `/visual_slam/status` is absent (e.g. SITL)
        if from_validated_gate or (now_sec() - last_vslam_status_rx_s[0]) > 2.0:
            tracker.update_estimator_health(True)

    def on_validated_odom(msg: Odometry) -> None:
        last_validated_odom_rx_s[0] = now_sec()
        if not args.skybeta_mode and (now_sec() - last_fmu_pos_rx_s[0]) <= 1.0:
            return
        _apply_odom_msg(msg, from_validated_gate=True)

    def on_visual_slam_odom(msg: Odometry) -> None:
        # Prefer Koji's fail-closed `/mapping/validated_odometry` gate when active,
        # and in direct PX4 mode do not overwrite `/fmu/out/vehicle_local_position_v1`
        if (now_sec() - last_validated_odom_rx_s[0]) <= 1.0:
            return
        if not args.skybeta_mode and (now_sec() - last_fmu_pos_rx_s[0]) <= 1.0:
            return
        _apply_odom_msg(msg, from_validated_gate=False)

    def on_vslam_status(msg: object) -> None:
        last_vslam_status_rx_s[0] = now_sec()
        vo_state = int(getattr(msg, "vo_state", 1))
        tracker.update_vslam_status(now_sec(), vo_state)

    def on_skybeta_state(msg: String) -> None:
        try:
            st = json.loads(msg.data)
            armed = ARMING_STATE_ARMED if st.get("armed", False) else 1
            nav = NAVIGATION_STATE_OFFBOARD if st.get("offboard_active", False) else 2
            landed = bool(st.get("landed", False))
            tracker.update_vehicle_status(now_sec(), armed, nav, landed=landed)
            local = st.get("local_enu")
            if isinstance(local, (list, tuple)) and len(local) >= 3:
                try:
                    tracker.update_px4_skybeta_height(now_sec(), float(local[2]))
                except (TypeError, ValueError):
                    pass
        except ValueError:
            pass

    if not args.skybeta_mode:
        # Never subscribe to `/fmu/out/vehicle_local_position_v1` in `--skybeta-mode` because
        # PX4's local frame on `skybeta-19` is rotated 90 deg relative to VIO odom
        node.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", on_local_pos, px4_qos)
        node.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", on_status, px4_qos)
        land_version = int(getattr(VehicleLandDetected, "MESSAGE_VERSION", 0))
        land_topic = "/fmu/out/vehicle_land_detected" + (f"_v{land_version}" if land_version else "")
        node.create_subscription(VehicleLandDetected, land_topic, on_land_detected, px4_qos)
    node.create_subscription(Bool, "/estimator/healthy", on_estimator_health, px4_qos)
    node.create_subscription(CameraInfo, "/camera/depth/camera_info", on_camera_info, px4_qos)
    node.create_subscription(CameraInfo, "/oak/left/camera_info", on_camera_info, px4_qos)
    node.create_subscription(Image, "/camera/depth/image_raw", on_depth, px4_qos)
    node.create_subscription(Image, "/team/stereo/depth", on_depth, px4_qos)
    node.create_subscription(PointCloud2, "/team/stereo/points", on_pointcloud, px4_qos)
    node.create_subscription(PointStamped, "/tube/offset", on_tube_offset, px4_qos)
    node.create_subscription(PoseStamped, "/team/localize/base_pose", on_localize_base_pose, 10)
    node.create_subscription(PoseStamped, "/team/localize/pose", on_localize_pose, 10)
    node.create_subscription(String, "/team/localize/status", on_localize_status, 10)
    node.create_subscription(Odometry, "/mapping/validated_odometry", on_validated_odom, 10)
    node.create_subscription(Odometry, "/visual_slam/tracking/odometry", on_visual_slam_odom, 10)
    node.create_subscription(String, "/skybeta/state", on_skybeta_state, 10)
    try:
        from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus

        node.create_subscription(VisualSlamStatus, "/visual_slam/status", on_vslam_status, 10)
    except ImportError:
        pass

    offboard_pub = (
        None if args.skybeta_mode else node.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
    )
    setpoint_pub = (
        None if args.skybeta_mode else node.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
    )
    cmd_pub = (
        None if args.skybeta_mode else node.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
    )
    skybeta_pose_pub = node.create_publisher(PoseStamped, "/skybeta/setpoint/pose", 10)
    skybeta_twist_pub = node.create_publisher(TwistStamped, "/skybeta/setpoint/twist", 10)
    # Autonomous plain-PX4 mission (arms, takes off, lands itself) unless a pilot arms (`--pilot-arms`)
    px4_mission = (not args.skybeta_mode) and args.approve_flight and not args.pilot_arms
    last_logged = [None]

    def publish_px4_setpoint(sample: PX4SetpointSample) -> None:
        ocm = OffboardControlMode()
        ocm.position = True
        ocm.velocity = False
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        ocm.timestamp = sample.timestamp_us
        offboard_pub.publish(ocm)

        sp = TrajectorySetpoint()
        sp.position = list(tracker.to_px4_frame(sample.position_ned))  # tracker works in the zero-tared frame
        sp.velocity = [math.nan, math.nan, math.nan]
        sp.acceleration = [math.nan, math.nan, math.nan]
        sp.yaw = float(sample.yaw_ned)
        sp.timestamp = sample.timestamp_us
        setpoint_pub.publish(sp)

    abort_landing_active: List[bool] = [False]
    exit_code_holder: List[Optional[int]] = [None]

    def _send_vehicle_cmd(t_now: float, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        if cmd_pub is None or not args.approve_flight:
            return
        vc = VehicleCommand()
        vc.command = int(command)
        vc.param1 = float(param1)
        vc.param2 = float(param2)
        vc.target_system = 1
        vc.target_component = 1
        vc.source_system = 1
        vc.source_component = 1
        vc.from_external = True
        vc.timestamp = int(t_now * 1_000_000)
        cmd_pub.publish(vc)

    def _send_nav_land(t_now: float, why: str) -> None:
        if cmd_pub is None or not args.approve_flight:
            return
        if (t_now - last_nav_land_cmd_s[0]) < 1.0:
            return
        last_nav_land_cmd_s[0] = t_now
        node.get_logger().warning(f"Sending VEHICLE_CMD_NAV_LAND: {why}")
        _send_vehicle_cmd(t_now, int(getattr(VehicleCommand, "VEHICLE_CMD_NAV_LAND", 21)))

    def timer_callback() -> None:
        t_now = now_sec()
        if t_now <= 0.0:
            return
        if stop_signals and tracker.stop_requested_s is None:
            tracker.request_stop(t_now)
            node.get_logger().warning("Stop requested: landing where we are if flying, then exiting.")
        if tracker.stop_requested_s is not None and not args.approve_flight:
            raise SystemExit(0)  # never published a setpoint: nothing to land
        if args.skybeta_mode:
            if not args.approve_flight:
                return
            cmd = tracker.step_skybeta_mission(t_now)
            # Log state/stage changes and holds; ROUTE's tracking reasons change at 30 Hz, so not those
            quiet = cmd.skybeta_state == "ROUTE" and not cmd.reason.startswith("vio_or_planner_hold")
            key = (cmd.skybeta_state, cmd.stage, "" if quiet else cmd.reason.split(" (")[0])
            if key != last_logged[0]:
                last_logged[0] = key
                here, _ = tracker.current_course_pose_enu(t_now)
                node.get_logger().info(
                    f"[{cmd.skybeta_state}] stage={cmd.stage} {cmd.reason} "
                    f"at course ({here[0]:.2f}, {here[1]:.2f}, {here[2]:.2f})"
                )
            if tracker.stop_finished():
                node.get_logger().info("Stopped: disarmed or pilot in control. Exiting.")
                raise SystemExit(0)
            if cmd.mode == "pose":
                pose_msg = PoseStamped()
                pose_msg.header.stamp = node.get_clock().now().to_msg()
                pose_msg.header.frame_id = "odom"
                pose_msg.pose.position.x = float(cmd.pos_odom[0])
                pose_msg.pose.position.y = float(cmd.pos_odom[1])
                pose_msg.pose.position.z = float(cmd.pos_odom[2])
                pose_msg.pose.orientation.x = float(cmd.quat_xyzw[0])
                pose_msg.pose.orientation.y = float(cmd.quat_xyzw[1])
                pose_msg.pose.orientation.z = float(cmd.quat_xyzw[2])
                pose_msg.pose.orientation.w = float(cmd.quat_xyzw[3])
                skybeta_pose_pub.publish(pose_msg)
            elif cmd.mode == "twist":
                twist_msg = TwistStamped()
                twist_msg.header.stamp = node.get_clock().now().to_msg()
                twist_msg.header.frame_id = "odom"
                twist_msg.twist.linear.x = float(cmd.vel_odom[0])
                twist_msg.twist.linear.y = float(cmd.vel_odom[1])
                twist_msg.twist.linear.z = float(cmd.vel_odom[2])
                twist_msg.twist.angular.z = 0.0
                skybeta_twist_pub.publish(twist_msg)
            return

        if px4_mission:
            mcmd = tracker.step_px4_mission(t_now)
            quiet = ("tracking_active_spline", "periodic_1hz_route_verified_clear")
            key = (mcmd.state, tracker.router.stage.value, mcmd.reason.split(" (")[0],
                   t_now if mcmd.state == "ROUTE" and mcmd.reason not in quiet else None)
            if mcmd.state == "ROUTE" and mcmd.reason in quiet:
                key = (mcmd.state, tracker.router.stage.value, "", None)
            if key != last_logged[0]:
                last_logged[0] = key
                here, _ = tracker.current_course_pose_enu(t_now)
                node.get_logger().info(
                    f"[{mcmd.state}] stage={tracker.router.stage.value} {mcmd.reason} "
                    f"at course ({here[0]:.2f}, {here[1]:.2f}, {here[2]:.2f})"
                )
            for command, p1, p2 in mcmd.commands:
                _send_vehicle_cmd(t_now, command, p1, p2)
            if mcmd.setpoint is not None:
                publish_px4_setpoint(mcmd.setpoint)
            return

        if tracker.stop_requested_s is not None:  # --pilot-arms: PX4 Land owns it; exit once disarmed
            if tracker.telemetry.arming_state == ARMING_STATE_ARMED:
                _send_nav_land(t_now, "operator_stop")
                return
            node.get_logger().info("Stopped: disarmed. Exiting.")
            raise SystemExit(0)
        sample, status_reason = tracker.sample_setpoint(t_now)
        if sample is None:
            if status_reason == "mission_complete_disarmed":
                node.get_logger().info("Mission complete: landed and disarmed.")
                exit_code_holder[0] = 0
                raise SystemExit(0)
            if abort_landing_active[0] and tracker.telemetry.arming_state != ARMING_STATE_ARMED:
                node.get_logger().info("Emergency land complete: disarmed.")
                exit_code_holder[0] = 2
                raise SystemExit(2)
            if tracker.telemetry.arming_state == ARMING_STATE_ARMED and (
                "geofence_breach" in status_reason
                or "estimator_position_jump" in status_reason
                or "vslam_lost" in status_reason
                or "estimator_unhealthy" in status_reason
                or "xy_valid_false" in status_reason
                or "z_valid_false" in status_reason
                or "stale_" in status_reason
            ):
                abort_landing_active[0] = True
                _send_nav_land(t_now, status_reason)
            node.get_logger().debug(f"Planner holding publishing: {status_reason}")
            return
        if sample.branch == "PAD_LAND":
            if tracker.telemetry.arming_state == ARMING_STATE_ARMED:
                _send_nav_land(t_now, "landing_on_concentric_pad")
            return
        if sample.stage == CourseStage.MISSION_COMPLETE:
            if tracker.telemetry.arming_state == ARMING_STATE_ARMED:
                _send_nav_land(t_now, "mission_complete")
            else:
                node.get_logger().info("Mission complete: landed and disarmed.")
                exit_code_holder[0] = 0
                raise SystemExit(0)
            return
        if not args.approve_flight:
            # Read-only / dry-run planning mode until team explicitly approves flight (AGENTS.md)
            return
        publish_px4_setpoint(sample)
        # Auto-arming lives in `step_px4_mission` (the default with --approve-flight): it arms once per
        # leg and ends in DONE/ABORT, so a landed drone is never re-armed by a still-streaming node.

    period_s = 1.0 / max(10.0, cfg.tracker_rate_hz)
    timer = node.create_timer(period_s, timer_callback)
    node.get_logger().info(
        f"EGO-Planner PX4 Bridge started at {cfg.tracker_rate_hz:.1f} Hz "
        f"(fast_mode={cfg.fast_mode}, approve_flight={args.approve_flight})"
    )

    try:
        if px4_mission:
            while rclpy.ok() and tracker.px4_state not in ("DONE", "ABORT"):
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            rclpy.spin(node)
    except SystemExit as exc:
        return int(exc.code) if isinstance(exc.code, int) else 0
    except KeyboardInterrupt:  # second stop signal: stop publishing now
        if tracker.telemetry.arming_state == ARMING_STATE_ARMED:
            if cmd_pub is not None:
                for _ in range(5):
                    _send_vehicle_cmd(now_sec(), VEHICLE_CMD_NAV_LAND)
                node.get_logger().warning("Second stop signal: VEHICLE_CMD_NAV_LAND sent, exiting.")
            else:
                node.get_logger().warning(
                    "Second stop signal while armed: exiting. flight_interface holds the last setpoint; pilot takes over."
                )
        return 130
    finally:
        node.destroy_timer(timer)
        node.destroy_node()
        rclpy.try_shutdown()
    if exit_code_holder[0] is not None:
        return exit_code_holder[0]
    return 1 if px4_mission and tracker.px4_state == "ABORT" and tracker.stop_requested_s is None else 0


if __name__ == "__main__":
    raise SystemExit(run_ros2_node())
