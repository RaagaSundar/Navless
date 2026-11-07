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
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

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
    odom_to_course_pos,
    wrap_pi,
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
        max_yaw_rate_rad_s: float = 1.40,
        max_lead_m: float = 0.55,
        touch_and_go_fly_home: bool = True,
    ) -> None:
        self.router = router
        self.config = router.config
        self.frame_convention = frame_convention
        self.lookahead_s = float(lookahead_s)
        self.start_bearing_deg = float(start_bearing_deg)
        self.start_height_m = float(start_height_m)
        self.max_yaw_rate_rad_s = float(max_yaw_rate_rad_s)
        self.max_lead_m = float(max_lead_m)
        self.touch_and_go_fly_home = bool(touch_and_go_fly_home)
        if launch_origin_enu is None:
            sp = self.config.checkpoints.start_pad
            self.launch_origin_enu: Tuple[float, float, float] = (float(sp[0]), float(sp[1]), 0.0)
        else:
            self.launch_origin_enu = launch_origin_enu
        self.telemetry = TelemetryHealthState()
        self.last_tube_offset_time_s: Optional[float] = None
        self.tube_offset_left_m: float = 0.0
        self.tube_width_m: float = 0.0
        self.last_localize_time_s: Optional[float] = None
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

    def update_localize_status(
        self,
        status: str,
        *,
        measured_x: float = 1.0,
        measured_y: float = 1.0,
    ) -> None:
        """Update status and measurement confidence from `/team/localize/status`."""
        if status:
            self.localize_status = str(status).strip().lower()
        self.localize_measured_xy = (float(measured_x), float(measured_y))

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
        status_ok = self.localize_status == "tracking" or (
            self.localize_status == "weak"
            and self.localize_measured_xy[0] >= 0.7
            and self.localize_measured_xy[1] >= 0.7
        )
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
        self.telemetry.last_local_pos_time_s = now_s
        self.telemetry.pos_ned = (float(x_ned), float(y_ned), float(z_ned))
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
        if self.telemetry.nav_state == NAVIGATION_STATE_OFFBOARD:
            self.telemetry.was_in_offboard = True
        if landed is not None:
            self.landed_flag = bool(landed)

    def update_estimator_health(self, healthy: bool) -> None:
        self.telemetry.estimator_healthy = bool(healthy)

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

        drone_pos_enu, current_yaw_enu = self.current_course_pose_enu(now_s)
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
        z_min_clamp = 0.05 if decision.stage == CourseStage.STAGE_6_FLY_HOME else profile.z_min_enu
        xy_fence = safety.arena_half_extent_xy_m - safety.net_keepout_margin_m
        pos_enu = clamp_altitude_enu(
            spline.evaluate_position(t_eval),
            z_min_clamp,
            profile.z_max_enu,
            xy_limit_m=xy_fence,
        )
        # Cap carrot lead distance so a lagging drone never sees a setpoint > max_lead_m ahead
        if self.max_lead_m > 0.0 and decision.stage != CourseStage.STAGE_6_FLY_HOME:
            lead_vec = pos_enu - drone_pos_enu
            lead_dist = float(np.linalg.norm(lead_vec))
            if lead_dist > self.max_lead_m:
                pos_enu = clamp_altitude_enu(
                    drone_pos_enu + (self.max_lead_m / lead_dist) * lead_vec,
                    z_min_clamp,
                    profile.z_max_enu,
                    xy_limit_m=xy_fence,
                )
        vel_enu = spline.evaluate_velocity(t_eval)
        raw_yaw_enu = spline.evaluate_yaw(t_eval, fallback_yaw=current_yaw_enu)
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
        pos_odom_cur = course_to_odom_pos(pos_enu, launch_xy, self.start_bearing_deg, self.start_height_m)
        offboard_active = self.telemetry.nav_state == NAVIGATION_STATE_OFFBOARD
        armed = self.telemetry.arming_state == ARMING_STATE_ARMED

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
            return SkybetaMissionCommand(
                mode="pose",
                pos_odom=pos_odom_cur,
                quat_xyzw=(0.0, 0.0, 0.0, 0.0),
                skybeta_state=self.skybeta_state,
                stage=self.router.stage.value,
                reason="paused_pilot_control",
            )

        if self.skybeta_state == "WAIT_FOR_OFFBOARD":
            if self.router.recorded_start_pos_enu is None:
                self.router.recorded_start_pos_enu = (
                    float(pos_enu[0]),
                    float(pos_enu[1]),
                    float(self.config.checkpoints.start_pad[2]),
                )
            if not offboard_active:
                return SkybetaMissionCommand(
                    mode="pose",
                    pos_odom=pos_odom_cur,
                    quat_xyzw=(0.0, 0.0, 0.0, 0.0),
                    skybeta_state="WAIT_FOR_OFFBOARD",
                    stage=self.router.stage.value,
                    reason="prestreaming_current_pose_for_offboard_handover",
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
            target_enu = (float(self.launch_origin_enu[0]), float(self.launch_origin_enu[1]), takeoff_z)
            if float(np.linalg.norm(pos_enu - np.asarray(target_enu, dtype=np.float64))) <= 0.25:
                self.skybeta_state = "ROUTE"
            else:
                target_odom = course_to_odom_pos(target_enu, launch_xy, self.start_bearing_deg, self.start_height_m)
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
                    sp_odom = course_to_odom_pos(sp_enu, launch_xy, self.start_bearing_deg, self.start_height_m)
                    return SkybetaMissionCommand(
                        mode="pose",
                        pos_odom=sp_odom,
                        quat_xyzw=sample.odom_orientation_xyzw,
                        skybeta_state="ROUTE",
                        stage=sample.stage.value,
                        reason=reason,
                    )
                else:
                    # VIO lost or temporary hold: freeze at last safe setpoint
                    hold_enu = self.last_safe_course_pos_enu or (float(pos_enu[0]), float(pos_enu[1]), float(pos_enu[2]))
                    hold_odom = course_to_odom_pos(hold_enu, launch_xy, self.start_bearing_deg, self.start_height_m)
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
                    sp_odom = course_to_odom_pos(sp_enu, launch_xy, self.start_bearing_deg, self.start_height_m)
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
                    hold_odom = course_to_odom_pos(hold_enu, launch_xy, self.start_bearing_deg, self.start_height_m)
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
    return np.stack([x_w, y_w, z_w], axis=-1)


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
    args = parser.parse_args(argv)

    cfg = PlannerConfig.from_json(args.config) if args.config.exists() else PlannerConfig()
    if args.fast_mode:
        cfg.fast_mode = True
    if args.stop_on_pad:
        cfg.enable_fly_home = False

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
    tracker = PX4TrajectoryTrackerCore(
        router,
        start_bearing_deg=args.start_bearing_deg,
        start_height_m=(args.start_height_m if args.skybeta_mode else 0.0),
        touch_and_go_fly_home=not args.stop_on_pad,
    )

    import rclpy
    from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from std_msgs.msg import Bool, String
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
            cam_intrinsics["width"] = float(msg.width) if msg.width > 0 else 0.0
            cam_intrinsics["height"] = float(msg.height) if msg.height > 0 else 0.0

    def on_depth(msg: Image) -> None:
        if "fx" not in cam_intrinsics or tracker.telemetry.last_local_pos_time_s is None:
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
            )
        except ValueError as err:
            node.get_logger().warning(f"Depth decode warning: {err}")

    def on_pointcloud(msg: PointCloud2) -> None:
        if tracker.telemetry.last_local_pos_time_s is None and tracker.last_localize_time_s is None:
            return
        pos_enu, yaw_enu = tracker.current_course_pose_enu(now_sec())
        pts_world = decode_ros_pointcloud2_optical(msg, pos_enu, yaw_enu)
        if pts_world.size > 0:
            grid.insert_point_cloud(pts_world, sensor_origin_m=pos_enu)

    def on_tube_offset(msg: PointStamped) -> None:
        tracker.update_tube_offset(now_sec(), float(msg.point.y), float(msg.point.z))

    last_base_pose_rx_s: List[float] = [-1e9]
    last_validated_odom_rx_s: List[float] = [-1e9]
    last_vslam_status_rx_s: List[float] = [-1e9]

    def _apply_localize_msg(msg: PoseStamped) -> None:
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
        _apply_odom_msg(msg, from_validated_gate=True)

    def on_visual_slam_odom(msg: Odometry) -> None:
        # Prefer Koji's fail-closed `/mapping/validated_odometry` gate when active
        if (now_sec() - last_validated_odom_rx_s[0]) <= 1.0:
            return
        _apply_odom_msg(msg, from_validated_gate=False)

    def on_vslam_status(msg: object) -> None:
        last_vslam_status_rx_s[0] = now_sec()
        vo_state = int(getattr(msg, "vo_state", 1))
        tracker.update_estimator_health(vo_state == 1)

    def on_skybeta_state(msg: String) -> None:
        try:
            st = json.loads(msg.data)
            armed = ARMING_STATE_ARMED if st.get("armed", False) else 1
            nav = NAVIGATION_STATE_OFFBOARD if st.get("offboard_active", False) else 2
            landed = bool(st.get("landed", False))
            tracker.update_vehicle_status(now_sec(), armed, nav, landed=landed)
        except ValueError:
            pass

    if not args.skybeta_mode:
        # Never subscribe to `/fmu/out/vehicle_local_position_v1` in `--skybeta-mode` because
        # PX4's local frame on `skybeta-11` is rotated 90 deg relative to VIO odom
        node.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", on_local_pos, px4_qos)
        node.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", on_status, px4_qos)
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
    skybeta_pose_pub = node.create_publisher(PoseStamped, "/skybeta/setpoint/pose", 10)
    skybeta_twist_pub = node.create_publisher(TwistStamped, "/skybeta/setpoint/twist", 10)

    def timer_callback() -> None:
        t_now = now_sec()
        if args.skybeta_mode:
            if not args.approve_flight:
                return
            cmd = tracker.step_skybeta_mission(t_now)
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
