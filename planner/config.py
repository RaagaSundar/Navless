"""Configuration dataclasses for EGO-Planner, physical constraints, and course checkpoints."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, List, Tuple


@dataclass(frozen=True)
class StageProfile:
    """Dynamic velocity and acceleration bounds for a mission stage."""

    v_max: float
    a_max: float
    j_max: float = 4.0
    z_min_enu: float = 0.5
    z_max_enu: float = 1.1  # Strictly below 68-inch (1.73 m) wall ceiling

    def scaled(self, multiplier: float) -> "StageProfile":
        """Return a speed-scaled profile (e.g., for fast_mode) while preserving Z bounds."""
        if multiplier <= 0.0:
            raise ValueError("Speed multiplier must be positive")
        return StageProfile(
            v_max=self.v_max * multiplier,
            a_max=self.a_max * multiplier,
            j_max=self.j_max * multiplier,
            z_min_enu=self.z_min_enu,
            z_max_enu=self.z_max_enu,
        )


@dataclass
class CourseCheckpoints:
    """High-level topological stage anchors in ENU meters (+X East, +Y North, +Z Up).

    Note: Intermediate maze turns, tunnel entrance/exit, Window cutout centroid/normal,
    and the Gate B chicane path are discovered dynamically from the 3D occupancy map
    and live depth sensor data rather than hardcoded.
    """

    start_pad: Tuple[float, float, float] = (-2.515, 0.914, 0.85)
    corridor_bend: Tuple[float, float, float] = (-2.515, 2.585, 0.85)
    gate_a_exit: Tuple[float, float, float] = (0.900, 2.585, 0.85)
    # Fallback priors used only when running on an empty synthetic test grid
    window_inspect_point: Tuple[float, float, float] = (0.350, -2.500, 0.78)
    window_aperture: Tuple[float, float, float] = (-0.471, -2.272, 0.762)
    window_exit: Tuple[float, float, float] = (-1.250, -1.900, 0.80)
    landing_pad: Tuple[float, float, float] = (-2.042, -0.792, 0.85)
    # Hardcoded over-the-wall return altitude (walls are 68 in = 1.727 m)
    fly_home_altitude_m: float = 2.00


@dataclass
class SafetyConfig:
    """Telemetry, estimator, and physical safety gates required by AGENTS.md & AI_CONTEXT.md."""

    max_telemetry_age_s: float = 0.50
    require_xy_valid: bool = True
    require_z_valid: bool = True
    require_estimator_healthy: bool = True
    wall_height_m: float = 1.7272  # 68 inches in meters
    course_z_min_m: float = 0.50
    course_z_max_m: float = 1.10
    fly_home_z_min_m: float = 1.85
    fly_home_z_max_m: float = 2.30


@dataclass
class PlannerConfig:
    """Master configuration for the EGO-Planner + FSM + PX4 Tracker pipeline."""

    # Map & voxel settings (Step 1)
    voxel_resolution_m: float = 0.10
    local_window_size_m: Tuple[float, float, float] = (6.0, 6.0, 2.0)
    global_map_bounds_min_m: Tuple[float, float, float] = (-3.1, -3.1, 0.0)
    global_map_bounds_max_m: Tuple[float, float, float] = (3.1, 3.1, 2.6)
    obstacle_inflation_m: float = 0.22  # 0.22 - 0.28 m safety inflation for 6x6 m course

    # Goal sequencer & replanning settings (Step 2 & Goal 3)
    acceptance_radius_m: float = 0.40
    window_inspect_distance_m: float = 1.00
    window_confidence_threshold: float = 0.80
    max_window_plan_time_ms: float = 100.0
    replan_period_s: float = 1.00  # Re-check route every 1.0 second
    tracker_rate_hz: float = 30.0  # 30-50 Hz PX4 TrajectorySetpoint streaming
    fast_mode: bool = False
    fast_mode_multiplier: float = 1.30
    enable_fly_home: bool = True

    # Stage kinematic profiles (Step 3)
    maze_profile: StageProfile = field(
        default_factory=lambda: StageProfile(v_max=1.1, a_max=1.5, j_max=4.0, z_min_enu=0.5, z_max_enu=1.1)
    )
    tunnel_profile: StageProfile = field(
        default_factory=lambda: StageProfile(v_max=0.8, a_max=1.0, j_max=3.0, z_min_enu=0.5, z_max_enu=1.1)
    )
    chicane_profile: StageProfile = field(
        default_factory=lambda: StageProfile(v_max=1.1, a_max=1.5, j_max=4.0, z_min_enu=0.5, z_max_enu=1.1)
    )
    fly_home_profile: StageProfile = field(
        default_factory=lambda: StageProfile(v_max=2.0, a_max=2.0, j_max=6.0, z_min_enu=1.85, z_max_enu=2.3)
    )

    checkpoints: CourseCheckpoints = field(default_factory=CourseCheckpoints)
    safety: SafetyConfig = field(default_factory=SafetyConfig)

    def active_profile(self, base: StageProfile, *, allow_fast_mode: bool = True) -> StageProfile:
        """Return the effective StageProfile accounting for fast_mode."""
        if self.fast_mode and allow_fast_mode:
            return base.scaled(self.fast_mode_multiplier)
        return base

    def to_dict(self) -> Dict:
        return asdict(self)

    def save_json(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=2), encoding="utf-8")

    @classmethod
    def from_json(cls, path: str | Path) -> "PlannerConfig":
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        cp = CourseCheckpoints(**{
            k: tuple(v) if isinstance(v, list) else v
            for k, v in raw.get("checkpoints", {}).items()
        })
        safety = SafetyConfig(**raw.get("safety", {}))
        cfg = cls(
            voxel_resolution_m=raw.get("voxel_resolution_m", 0.10),
            local_window_size_m=tuple(raw.get("local_window_size_m", (6.0, 6.0, 2.0))),
            global_map_bounds_min_m=tuple(raw.get("global_map_bounds_min_m", (-2.0, -3.0, 0.0))),
            global_map_bounds_max_m=tuple(raw.get("global_map_bounds_max_m", (15.0, 5.0, 2.6))),
            obstacle_inflation_m=raw.get("obstacle_inflation_m", 0.28),
            acceptance_radius_m=raw.get("acceptance_radius_m", 0.40),
            window_inspect_distance_m=raw.get("window_inspect_distance_m", 1.00),
            window_confidence_threshold=raw.get("window_confidence_threshold", 0.80),
            max_window_plan_time_ms=raw.get("max_window_plan_time_ms", 100.0),
            replan_period_s=raw.get("replan_period_s", 1.00),
            tracker_rate_hz=raw.get("tracker_rate_hz", 30.0),
            fast_mode=raw.get("fast_mode", False),
            fast_mode_multiplier=raw.get("fast_mode_multiplier", 1.30),
            enable_fly_home=raw.get("enable_fly_home", True),
            checkpoints=cp,
            safety=safety,
        )
        return cfg
