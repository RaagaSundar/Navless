"""Global Waypoint Router & Dynamic Goal Sequencer (Topological FSM).

Manages the full DroneHacks course lifecycle:
- Stage 1: Maze Corridor Bend
- Stage 2: Maze Exit at Gate A
- Stage 3: Tunnel Exit & Window Approach (~1 m inspection pose)
- Stage 4: Chicane (Window 5-pt bonus route if high confidence & feasible within 100 ms;
           automatic fallback to Gate B otherwise)
- Stage 5: Landing Pad approach & primary touchdown
- Stage 6: Fly Home (bonus return leg: climb to z = 2.0 m above the 68-inch walls,
           sprint over the barrier at v_max = 2.0 m/s, and land back at start)
- Enforces 1 Hz periodic route re-checking (`replan_period_s = 1.0`).
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from .bspline import EgoBsplineOptimizer, UniformBspline
from .config import PlannerConfig, StageProfile
from .frames import clamp_altitude_enu, enu_to_ned_pos
from .grid_map import OccupancyGrid3D, WindowInspectResult
from .nmpc import NonlinearMPCPlanner


class CourseStage(str, Enum):
    """Finite-state machine stages for the DroneHacks course."""

    STAGE_1_MAZE = "STAGE_1_MAZE"
    STAGE_2_GATE_A = "STAGE_2_GATE_A"
    STAGE_3_TUNNEL = "STAGE_3_TUNNEL"
    STAGE_4_CHICANE = "STAGE_4_CHICANE"
    STAGE_5_LANDING_PAD = "STAGE_5_LANDING_PAD"
    STAGE_6_FLY_HOME = "STAGE_6_FLY_HOME"
    MISSION_COMPLETE = "MISSION_COMPLETE"


@dataclass
class PlannerDecision:
    """Output snapshot of the GlobalCourseRouter at a planning tick."""

    stage: CourseStage
    selected_branch: str  # "PENDING", "WINDOW", or "GATE_B"
    goal_enu: Tuple[float, float, float]
    spline: Optional[UniformBspline]
    replanned: bool
    plan_time_ms: float
    window_inspection: Optional[WindowInspectResult]
    reason: str


class GlobalCourseRouter:
    """Topological Finite-State Machine & 1 Hz EGO-Planner Route Sequencer."""

    def __init__(
        self,
        config: PlannerConfig,
        grid: OccupancyGrid3D,
        optimizer: Optional[EgoBsplineOptimizer | NonlinearMPCPlanner] = None,
    ) -> None:
        self.config = config
        self.grid = grid
        self.nmpc = NonlinearMPCPlanner(grid)
        self.ego_optimizer = EgoBsplineOptimizer(grid)
        if optimizer is not None:
            self.optimizer = optimizer
        elif str(getattr(config, "short_term_planner", "nmpc")).lower() == "nmpc":
            self.optimizer = self.nmpc
        else:
            self.optimizer = self.ego_optimizer

        self.stage: CourseStage = CourseStage.STAGE_1_MAZE
        self.selected_branch: str = "PENDING"
        self.active_spline: Optional[UniformBspline] = None
        self.spline_start_time_s: float = 0.0
        self.last_replan_time_s: float = -1e9
        self.last_window_inspection: Optional[WindowInspectResult] = None
        self.primary_landed: bool = False
        self.recorded_start_pos_enu: Optional[Tuple[float, float, float]] = None

    def home_target_enu(self, hop_z_m: Optional[float] = None) -> Tuple[float, float, float]:
        """Return the dynamically recorded start XY (where the drone initialized) at hop altitude."""
        base = self.recorded_start_pos_enu or self.config.checkpoints.start_pad
        z_target = hop_z_m if hop_z_m is not None else self.config.checkpoints.fly_home_altitude_m
        return (float(base[0]), float(base[1]), float(z_target))

    def profile_for_stage(self, stage: Optional[CourseStage] = None) -> StageProfile:
        """Return the velocity/acceleration/ceiling profile for the given stage."""
        st = stage or self.stage
        if st in (CourseStage.STAGE_1_MAZE, CourseStage.STAGE_2_GATE_A):
            return self.config.active_profile(self.config.maze_profile)
        if st == CourseStage.STAGE_3_TUNNEL:
            return self.config.active_profile(self.config.tunnel_profile)
        if st in (CourseStage.STAGE_4_CHICANE, CourseStage.STAGE_5_LANDING_PAD):
            return self.config.active_profile(self.config.chicane_profile)
        if st in (CourseStage.STAGE_6_FLY_HOME, CourseStage.MISSION_COMPLETE):
            # Fly-home already runs at 2.0 m/s over the wall
            return self.config.active_profile(self.config.fly_home_profile, allow_fast_mode=False)
        return self.config.active_profile(self.config.maze_profile)

    def landing_target_enu(self, features=None) -> Tuple[Tuple[float, float, float], float]:
        """Return (bullseye_target_enu, concentric_outer_radius_m) from discovered concentric rings."""
        cp = self.config.checkpoints
        if features is None:
            features = self.grid.discover_course_features(
                standoff_distance_m=self.config.window_inspect_distance_m,
            )
        if features.concentric_pad is not None:
            cx, cy = features.concentric_pad.center_xy_m
            return (float(cx), float(cy), float(cp.landing_pad[2])), float(features.concentric_pad.outer_radius_m)
        return cp.landing_pad, 0.762

    def current_goal_enu(self) -> Tuple[float, float, float]:
        """Return the current stage's target goal dynamically from map features & concentric target."""
        cp = self.config.checkpoints
        features = self.grid.discover_course_features(
            standoff_distance_m=self.config.window_inspect_distance_m,
        )
        landing_goal, _ = self.landing_target_enu(features)
        # When the Concentric Circle Target is present on the 3D course map, the entire primary run
        # (Stage 1 -> Stage 5) targets the Concentric Circle Bullseye in ONE continuous unsectioned A* path!
        if features.concentric_pad is not None and self.stage in (
            CourseStage.STAGE_1_MAZE,
            CourseStage.STAGE_2_GATE_A,
            CourseStage.STAGE_3_TUNNEL,
            CourseStage.STAGE_4_CHICANE,
            CourseStage.STAGE_5_LANDING_PAD,
        ):
            return landing_goal
        if self.stage in (CourseStage.STAGE_1_MAZE, CourseStage.STAGE_2_GATE_A):
            return cp.corridor_bend if self.stage == CourseStage.STAGE_1_MAZE else cp.gate_a_exit
        if self.stage == CourseStage.STAGE_3_TUNNEL:
            return features.window_approach_m or cp.window_inspect_point
        if self.stage == CourseStage.STAGE_4_CHICANE:
            if self.selected_branch == "WINDOW":
                return features.window_exit_m or cp.window_exit
            return landing_goal
        if self.stage == CourseStage.STAGE_5_LANDING_PAD:
            return landing_goal
        if self.stage == CourseStage.STAGE_6_FLY_HOME:
            return self.home_target_enu()
        sx, sy, sz = self.recorded_start_pos_enu or cp.start_pad
        return (sx, sy, sz)

    def _select_window_or_gate_b(
        self,
        current_pos_enu: np.ndarray,
        external_vision_confidence: Optional[float] = None,
    ) -> Tuple[str, UniformBspline, float, WindowInspectResult, str]:
        """Dynamically evaluate Window vs. Gate B from the 3D map & sensor data (no hardcoded chicane points)."""
        cp = self.config.checkpoints
        profile = self.profile_for_stage(CourseStage.STAGE_4_CHICANE)

        # 1. Dynamically detect Window cutout centroid, normal, and exit from 3D map/depth voxels
        features = self.grid.discover_course_features(
            standoff_distance_m=self.config.window_inspect_distance_m,
        )
        win_center = features.window_center_m or cp.window_aperture
        win_exit = features.window_exit_m or cp.window_exit

        inspect_res = self.grid.inspect_window_aperture(
            win_center,
            confidence_threshold=self.config.window_confidence_threshold,
        )
        effective_conf = inspect_res.confidence
        if external_vision_confidence is not None:
            effective_conf = min(effective_conf, float(external_vision_confidence))
            inspect_res = WindowInspectResult(
                confidence=effective_conf,
                clear_ratio=inspect_res.clear_ratio,
                wall_detected_ratio=inspect_res.wall_detected_ratio,
                is_passable=(
                    inspect_res.is_passable
                    and effective_conf >= self.config.window_confidence_threshold
                ),
                reason=(
                    inspect_res.reason
                    if effective_conf >= self.config.window_confidence_threshold
                    else f"vision_confidence_low ({effective_conf:.2f})"
                ),
                aperture_center_m=win_center,
                approach_point_m=features.window_approach_m,
                exit_point_m=win_exit,
            )
        self.last_window_inspection = inspect_res

        # 2. If confident, route straight through the detected Window normal within 100 ms budget
        landing_goal, concentric_r = self.landing_target_enu(features)
        if inspect_res.is_passable:
            win_target = landing_goal if features.concentric_pad is not None else win_exit
            win_mids = None if features.concentric_pad is not None else [win_center]
            win_spline, elapsed_ms = self.optimizer.plan_spline(
                current_pos_enu,
                win_target,
                profile,
                intermediate_waypoints=win_mids,
                max_plan_time_ms=self.config.max_window_plan_time_ms,
                use_clearance_cost=(features.concentric_pad is not None),
                goal_radius_m=(concentric_r if features.concentric_pad is not None else 0.0),
            )
            if win_spline is not None and self.optimizer.is_spline_collision_free(win_spline):
                if elapsed_ms <= self.config.max_window_plan_time_ms or float(current_pos_enu[1]) < -0.50:
                    return "WINDOW", win_spline, elapsed_ms, inspect_res, "window_confident_and_feasible"

        # 3. Fallback: mask out the Window opening and let 3D A* + concentric-circle target field
        #    automatically discover the Gate B path around the chicane walls to the Concentric Pad!
        gate_b_spline, elapsed_ms = self.optimizer.plan_spline(
            current_pos_enu,
            landing_goal,
            profile,
            intermediate_waypoints=None,
            avoid_spheres=[(win_center, 0.55)],
            goal_radius_m=concentric_r,
        )
        if gate_b_spline is None:
            pts = np.vstack([current_pos_enu, np.asarray(landing_goal)])
            gate_b_spline = UniformBspline.from_waypoints(pts, 0.6).reallocate_time(profile.v_max, profile.a_max)
        return "GATE_B", gate_b_spline, elapsed_ms, inspect_res, f"fallback_to_gate_b ({inspect_res.reason})"

    def step(
        self,
        now_s: float,
        drone_pos_enu: np.ndarray | Tuple[float, float, float],
        *,
        external_window_confidence: Optional[float] = None,
        force_replan: bool = False,
    ) -> PlannerDecision:
        """Advance the topological FSM and trigger 1 Hz (or collision-driven) B-spline replanning."""
        pos = np.asarray(drone_pos_enu, dtype=np.float64)
        if self.recorded_start_pos_enu is None:
            self.recorded_start_pos_enu = (float(pos[0]), float(pos[1]), float(pos[2]))
        stage_changed = False

        if self.stage != CourseStage.MISSION_COMPLETE:
            if self.grid.detected_concentric_pad is not None and self.stage in (
                CourseStage.STAGE_1_MAZE,
                CourseStage.STAGE_2_GATE_A,
                CourseStage.STAGE_3_TUNNEL,
                CourseStage.STAGE_4_CHICANE,
                CourseStage.STAGE_5_LANDING_PAD,
            ):
                # Continuous unsectioned A* tracking toward the Concentric Circle Target:
                # Update topological stage label smoothly based on spatial progress without cutting the spline!
                if self.stage == CourseStage.STAGE_1_MAZE and float(pos[0]) > -0.60:
                    self.stage = CourseStage.STAGE_2_GATE_A
                elif self.stage == CourseStage.STAGE_2_GATE_A and float(pos[0]) > 1.65 and float(pos[1]) < 1.65:
                    self.stage = CourseStage.STAGE_3_TUNNEL
                elif self.stage == CourseStage.STAGE_3_TUNNEL and float(pos[1]) < -0.85:
                    self.stage = CourseStage.STAGE_4_CHICANE
                    if self.selected_branch == "PENDING":
                        # Evaluate Window confidence/clearance as we exit the tunnel:
                        # Only force a replan if we need to switch to Gate B!
                        features = self.grid.discover_course_features(
                            standoff_distance_m=self.config.window_inspect_distance_m,
                        )
                        win_c = features.window_center_m or self.config.checkpoints.window_aperture
                        insp = self.grid.inspect_window_aperture(
                            win_c,
                            confidence_threshold=self.config.window_confidence_threshold,
                        )
                        eff_c = (
                            min(insp.confidence, float(external_window_confidence))
                            if external_window_confidence is not None
                            else insp.confidence
                        )
                        if insp.is_passable and eff_c >= self.config.window_confidence_threshold:
                            self.selected_branch = "WINDOW"
                            self.last_window_inspection = insp
                        else:
                            stage_changed = True
                elif self.stage == CourseStage.STAGE_4_CHICANE and float(pos[0]) < -0.90:
                    self.stage = CourseStage.STAGE_5_LANDING_PAD

                goal = np.asarray(self.current_goal_enu(), dtype=np.float64)
                if float(np.linalg.norm(pos - goal)) <= self.config.acceptance_radius_m:
                    stage_changed = True
                    self.primary_landed = True
                    if self.config.enable_fly_home:
                        self.stage = CourseStage.STAGE_6_FLY_HOME
                    else:
                        self.stage = CourseStage.MISSION_COMPLETE
            else:
                goal = np.asarray(self.current_goal_enu(), dtype=np.float64)
                dist_to_goal = float(np.linalg.norm(pos - goal))

                if dist_to_goal <= self.config.acceptance_radius_m:
                    stage_changed = True
                    if self.stage == CourseStage.STAGE_1_MAZE:
                        self.stage = CourseStage.STAGE_2_GATE_A
                    elif self.stage == CourseStage.STAGE_2_GATE_A:
                        self.stage = CourseStage.STAGE_3_TUNNEL
                    elif self.stage == CourseStage.STAGE_3_TUNNEL:
                        self.stage = CourseStage.STAGE_4_CHICANE
                    elif self.stage == CourseStage.STAGE_4_CHICANE:
                        self.stage = CourseStage.STAGE_5_LANDING_PAD
                    elif self.stage == CourseStage.STAGE_5_LANDING_PAD:
                        self.primary_landed = True
                        if self.config.enable_fly_home:
                            self.stage = CourseStage.STAGE_6_FLY_HOME
                        else:
                            self.stage = CourseStage.MISSION_COMPLETE
                    elif self.stage == CourseStage.STAGE_6_FLY_HOME:
                        self.stage = CourseStage.MISSION_COMPLETE

        if self.stage == CourseStage.MISSION_COMPLETE:
            return PlannerDecision(
                stage=self.stage,
                selected_branch=self.selected_branch,
                goal_enu=self.current_goal_enu(),
                spline=self.active_spline,
                replanned=False,
                plan_time_ms=0.0,
                window_inspection=self.last_window_inspection,
                reason="mission_complete",
            )

        # Check 1 Hz periodic replanning condition or collision along current spline
        periodic_due = (now_s - self.last_replan_time_s) >= self.config.replan_period_s
        spline_blocked = False
        spline_finished = False
        if self.active_spline is not None:
            t_rel = max(0.0, now_s - self.spline_start_time_s)
            if (now_s - self.last_replan_time_s) >= 0.40 and self.stage != CourseStage.STAGE_6_FLY_HOME:
                spline_blocked = not self.optimizer.is_spline_collision_free(self.active_spline, t_start=t_rel)
            end_pos = self.active_spline.evaluate_position(self.active_spline.duration)
            end_at_goal = (
                float(np.linalg.norm(end_pos - np.asarray(self.current_goal_enu(), dtype=np.float64)))
                <= self.config.acceptance_radius_m
            )
            spline_finished = t_rel >= (self.active_spline.duration + (1.5 if end_at_goal else -0.05))

        needs_new_spline = (
            force_replan
            or stage_changed
            or self.active_spline is None
            or spline_blocked
            or spline_finished
        )

        if not needs_new_spline:
            if periodic_due:
                self.last_replan_time_s = now_s
                return PlannerDecision(
                    stage=self.stage,
                    selected_branch=self.selected_branch,
                    goal_enu=self.current_goal_enu(),
                    spline=self.active_spline,
                    replanned=True,
                    plan_time_ms=0.0,
                    window_inspection=self.last_window_inspection,
                    reason="periodic_1hz_route_verified_clear",
                )
            return PlannerDecision(
                stage=self.stage,
                selected_branch=self.selected_branch,
                goal_enu=self.current_goal_enu(),
                spline=self.active_spline,
                replanned=False,
                plan_time_ms=0.0,
                window_inspection=self.last_window_inspection,
                reason="tracking_active_spline",
            )

        # Replan for the current stage
        self.last_replan_time_s = now_s
        self.spline_start_time_s = now_s
        profile = self.profile_for_stage(self.stage)

        if self.stage == CourseStage.STAGE_4_CHICANE:
            branch, spline, elapsed_ms, inspect_res, reason = self._select_window_or_gate_b(
                pos, external_vision_confidence=external_window_confidence
            )
            self.selected_branch = branch
            self.active_spline = spline
            return PlannerDecision(
                stage=self.stage,
                selected_branch=self.selected_branch,
                goal_enu=self.current_goal_enu(),
                spline=self.active_spline,
                replanned=True,
                plan_time_ms=elapsed_ms,
                window_inspection=inspect_res,
                reason=reason,
            )

        if self.stage == CourseStage.STAGE_6_FLY_HOME:
            # Goal 4: Dynamically detect the barrier between where we landed and where we started,
            # climb above the measured wall top, and jump across the wall back to the start origin!
            start_origin = self.home_target_enu()
            barrier = self.grid.detect_barrier_between(pos, start_origin)
            hop_z = barrier.safe_hop_z_m
            home_wp = self.home_target_enu(hop_z_m=hop_z)
            climb_wp = (float(pos[0]), float(pos[1]), hop_z)
            dist_xy_home = float(np.linalg.norm(pos[:2] - np.asarray(home_wp[:2], dtype=np.float64)))
            fly_home_mids = [barrier.apex_waypoint_m] if dist_xy_home > 0.75 else None

            spline, elapsed_ms = self.optimizer.plan_spline(
                climb_wp,
                home_wp,
                profile,
                intermediate_waypoints=fly_home_mids,
                use_clearance_cost=False,
                goal_radius_m=self.config.acceptance_radius_m,
            )
            if spline is not None:
                z_start = max(0.05, float(pos[2]))
                if z_start < hop_z - 0.25:
                    p_bottom = np.array([float(pos[0]), float(pos[1]), z_start], dtype=np.float64)
                    p_mid = np.array([float(pos[0]), float(pos[1]), 0.5 * (z_start + hop_z)], dtype=np.float64)
                    ctrl_with_climb = np.vstack([
                        p_bottom,
                        p_bottom,
                        p_bottom,
                        p_mid,
                        spline.control_points[2:],
                    ])
                    spline = UniformBspline(ctrl_with_climb, spline.dt).reallocate_time(profile.v_max, profile.a_max)
            elif self.active_spline is not None:
                spline = self.active_spline
            self.active_spline = spline
            return PlannerDecision(
                stage=self.stage,
                selected_branch=self.selected_branch,
                goal_enu=home_wp,
                spline=self.active_spline,
                replanned=True,
                plan_time_ms=elapsed_ms,
                window_inspection=self.last_window_inspection,
                reason=f"fly_home_dynamic_wall_jump (wall_detected={barrier.wall_detected}, hop_z={hop_z:.2f}m)",
            )

        goal_enu = self.current_goal_enu()
        mids: Optional[List[Tuple[float, float, float]]] = None
        use_clearance = True
        goal_rad = 0.0
        avoid_list: Optional[List[Tuple[Tuple[float, float, float], float]]] = None
        if self.grid.detected_concentric_pad is not None or self.stage == CourseStage.STAGE_5_LANDING_PAD:
            _, goal_rad = self.landing_target_enu()
        if self.selected_branch == "GATE_B" and float(pos[0]) > -0.45:
            features = self.grid.discover_course_features(
                standoff_distance_m=self.config.window_inspect_distance_m,
            )
            win_c = features.window_center_m or self.config.checkpoints.window_aperture
            avoid_list = [(win_c, 0.55)]
        if self.stage == CourseStage.STAGE_3_TUNNEL and self.grid.detected_concentric_pad is None:
            features = self.grid.discover_course_features(
                standoff_distance_m=self.config.window_inspect_distance_m,
            )
            all_tunnel_wps = [features.tunnel_entry_m, features.tunnel_mid_m, features.tunnel_exit_m]
            # Only include tunnel waypoints that are still ahead (South) of the drone's current Y position
            ahead_wps = [wp for wp in all_tunnel_wps if float(pos[1]) > float(wp[1]) + 0.25]
            if ahead_wps:
                mids = ahead_wps
                use_clearance = False
        plan_start = np.array(
            [
                float(pos[0]),
                float(pos[1]),
                max(float(pos[2]), float(self.config.checkpoints.start_pad[2]))
                if self.stage == CourseStage.STAGE_1_MAZE
                else float(pos[2]),
            ],
            dtype=np.float64,
        )
        spline, elapsed_ms = self.optimizer.plan_spline(
            plan_start,
            goal_enu,
            profile,
            intermediate_waypoints=mids,
            use_clearance_cost=use_clearance,
            avoid_spheres=avoid_list,
            goal_radius_m=goal_rad,
        )
        if spline is None and self.active_spline is not None:
            spline = self.active_spline
        self.active_spline = spline
        reason = "collision_avoidance_replan" if spline_blocked else ("stage_transition" if stage_changed else "periodic_1hz_replan")
        return PlannerDecision(
            stage=self.stage,
            selected_branch=self.selected_branch,
            goal_enu=goal_enu,
            spline=self.active_spline,
            replanned=True,
            plan_time_ms=elapsed_ms,
            window_inspection=self.last_window_inspection,
            reason=reason,
        )

    def plan_full_course_route(
        self,
        *,
        use_window: bool = False,
        include_fly_home: bool = True,
        samples_per_stage: int = 20,
    ) -> Dict:
        """Generate a complete end-to-end course trajectory dynamically from the 3D map.

        - Maze & Chicane routes are discovered via 3D A* + clearance field (zero hardcoded turns).
        - Tunnel and Window aperture are discovered from 3D voxel geometry (`discover_course_features`).
        - Fly-home uses the deterministic over-the-wall climb-and-hop plan after landing.
        """
        cp = self.config.checkpoints
        features = self.grid.discover_course_features(
            standoff_distance_m=self.config.window_inspect_distance_m,
        )
        win_center = features.window_center_m or cp.window_aperture
        win_approach = features.window_approach_m or cp.window_inspect_point
        win_exit = features.window_exit_m or cp.window_exit
        landing_goal, concentric_r = self.landing_target_enu(features)
        branch_name = "WINDOW" if use_window else "GATE_B"

        # Discover Maze + Gate A path directly via 3D A* toward the Tunnel entrance (zero hardcoded anchors!)
        if features.concentric_pad is not None:
            from .bspline import astar_search_3d
            soft_cost = self.grid.build_soft_clearance_cost()
            maze_guiding = astar_search_3d(
                self.grid,
                cp.start_pad,
                features.tunnel_entry_m,
                z_min_m=self.config.maze_profile.z_min_enu,
                z_max_m=self.config.maze_profile.z_max_enu,
                soft_clearance_cost=soft_cost,
            )
            if maze_guiding is not None and len(maze_guiding) >= 4:
                mid_pt = maze_guiding[len(maze_guiding) // 2]
                maze_split_wp = (float(mid_pt[0]), float(mid_pt[1]), float(cp.start_pad[2]))
                gate_a_end_wp = features.tunnel_entry_m
                tunnel_mids = [features.tunnel_mid_m, features.tunnel_exit_m]
            else:
                maze_split_wp = cp.corridor_bend
                gate_a_end_wp = cp.gate_a_exit
                tunnel_mids = [features.tunnel_entry_m, features.tunnel_mid_m, features.tunnel_exit_m]
        else:
            maze_split_wp = cp.corridor_bend
            gate_a_end_wp = cp.gate_a_exit
            tunnel_mids = [features.tunnel_entry_m, features.tunnel_mid_m, features.tunnel_exit_m]

        # Stage 1 -> Stage 2 -> Stage 3 discovered dynamically via 3D A* on the map
        stages_spec: List[Tuple[str, Tuple[float, float, float], Tuple[float, float, float], List[Tuple[float, float, float]], StageProfile, Optional[List[Tuple[Tuple[float, float, float], float]]]]] = [
            (
                CourseStage.STAGE_1_MAZE.value,
                cp.start_pad,
                maze_split_wp,
                [],
                self.profile_for_stage(CourseStage.STAGE_1_MAZE),
                None,
            ),
            (
                CourseStage.STAGE_2_GATE_A.value,
                maze_split_wp,
                gate_a_end_wp,
                [],  # No hardcoded maze turns: 3D A* discovers the serpentine maze path from the map!
                self.profile_for_stage(CourseStage.STAGE_2_GATE_A),
                None,
            ),
            (
                CourseStage.STAGE_3_TUNNEL.value,
                gate_a_end_wp,
                win_approach,
                tunnel_mids,
                self.profile_for_stage(CourseStage.STAGE_3_TUNNEL),
                None,
            ),
        ]

        if use_window:
            stages_spec.append(
                (
                    CourseStage.STAGE_4_CHICANE.value,
                    win_approach,
                    win_exit,
                    [win_center],
                    self.profile_for_stage(CourseStage.STAGE_4_CHICANE),
                    None,
                )
            )
            stages_spec.append(
                (
                    CourseStage.STAGE_5_LANDING_PAD.value,
                    win_exit,
                    landing_goal,
                    [],
                    self.profile_for_stage(CourseStage.STAGE_5_LANDING_PAD),
                    None,
                )
            )
        else:
            # Gate B route: mask out the Window opening so 3D A* discovers the Gate B passage around chicane walls!
            stages_spec.append(
                (
                    CourseStage.STAGE_4_CHICANE.value,
                    win_approach,
                    landing_goal,
                    [],
                    self.profile_for_stage(CourseStage.STAGE_4_CHICANE),
                    [(win_center, 0.55)],
                )
            )

        launch_origin_enu: Tuple[float, float, float] = (
            float(cp.start_pad[0]),
            float(cp.start_pad[1]),
            0.0,
        )
        waypoints_enu: List[List[float]] = []
        waypoints_local_enu: List[List[float]] = []
        waypoints_ned: List[List[float]] = []
        waypoints_course_ned: List[List[float]] = []
        stage_summaries: List[Dict] = []

        def _append_point(pt: np.ndarray | Tuple[float, float, float]) -> None:
            enu_list = [round(float(pt[0]), 4), round(float(pt[1]), 4), round(float(pt[2]), 4)]
            local_enu_list = [
                round(float(pt[0] - launch_origin_enu[0]), 4),
                round(float(pt[1] - launch_origin_enu[1]), 4),
                round(float(pt[2] - launch_origin_enu[2]), 4),
            ]
            ned_local = enu_to_ned_pos(pt, convention="enu_to_ned", origin_enu=launch_origin_enu)
            ned_course = enu_to_ned_pos(pt, convention="enu_to_ned", origin_enu=None)
            waypoints_enu.append(enu_list)
            waypoints_local_enu.append(local_enu_list)
            waypoints_ned.append([round(ned_local[0], 4), round(ned_local[1], 4), round(ned_local[2], 4)])
            waypoints_course_ned.append([round(ned_course[0], 4), round(ned_course[1], 4), round(ned_course[2], 4)])

        if features.concentric_pad is not None:
            # Plan ONE continuous unsectioned 3D A* + B-spline trajectory from Launch Mark to Concentric Target
            prof_main = self.profile_for_stage(CourseStage.STAGE_1_MAZE)
            avoid_main = None if use_window else [(win_center, 0.55)]
            spline_main, elapsed_main_ms = self.optimizer.plan_spline(
                cp.start_pad,
                landing_goal,
                prof_main,
                use_clearance_cost=True,
                avoid_spheres=avoid_main,
                goal_radius_m=concentric_r,
            )
            if spline_main is None:
                raise RuntimeError("Failed to plan continuous unsectioned 3D A* + B-spline trajectory")
            n_total_samples = (5 if use_window else 4) * samples_per_stage
            pts_main = spline_main.sample_points(n_total_samples)
            for pt in pts_main:
                _append_point(pt)
            stage_list = [
                CourseStage.STAGE_1_MAZE.value,
                CourseStage.STAGE_2_GATE_A.value,
                CourseStage.STAGE_3_TUNNEL.value,
                CourseStage.STAGE_4_CHICANE.value,
            ]
            if use_window:
                stage_list.append(CourseStage.STAGE_5_LANDING_PAD.value)
            for st_name in stage_list:
                stage_summaries.append(
                    {
                        "stage": st_name,
                        "duration_s": round(spline_main.duration / len(stage_list), 3),
                        "v_max_mps": prof_main.v_max,
                        "a_max_mps2": prof_main.a_max,
                        "plan_time_ms": round(elapsed_main_ms / len(stage_list), 2),
                    }
                )
        else:
            for stage_name, p_start, p_goal, mids, prof, avoid_list in stages_spec:
                spline, elapsed_ms = self.optimizer.plan_spline(
                    p_start,
                    p_goal,
                    prof,
                    intermediate_waypoints=mids,
                    use_clearance_cost=(
                        not (use_window and stage_name == CourseStage.STAGE_4_CHICANE.value)
                        and stage_name != CourseStage.STAGE_3_TUNNEL.value
                    ),
                    avoid_spheres=avoid_list,
                )
                if spline is None:
                    raise RuntimeError(f"Failed to plan collision-free spline for stage {stage_name}")
                pts = spline.sample_points(samples_per_stage)
                if waypoints_enu:
                    pts = pts[1:]
                for pt in pts:
                    _append_point(pt)
                stage_summaries.append(
                    {
                        "stage": stage_name,
                        "duration_s": round(spline.duration, 3),
                        "v_max_mps": prof.v_max,
                        "a_max_mps2": prof.a_max,
                        "plan_time_ms": round(elapsed_ms, 2),
                    }
                )

        if include_fly_home and self.config.enable_fly_home:
            prof_home = self.profile_for_stage(CourseStage.STAGE_6_FLY_HOME)
            start_origin = self.home_target_enu()
            barrier = self.grid.detect_barrier_between(cp.landing_pad, start_origin)
            hop_z = barrier.safe_hop_z_m
            climb_start = (cp.landing_pad[0], cp.landing_pad[1], hop_z)
            home_above = self.home_target_enu(hop_z_m=hop_z)
            spline_home, elapsed_home_ms = self.optimizer.plan_spline(
                climb_start,
                home_above,
                prof_home,
                intermediate_waypoints=[barrier.apex_waypoint_m],
                use_clearance_cost=False,
            )
            if spline_home is None:
                raise RuntimeError("Failed to plan dynamic over-wall fly-home spline")
            ctrl_home = np.vstack([
                np.asarray(cp.landing_pad, dtype=np.float64),
                spline_home.control_points,
            ])
            spline_home = UniformBspline(ctrl_home, spline_home.dt).reallocate_time(prof_home.v_max, prof_home.a_max)
            for pt in spline_home.sample_points(samples_per_stage)[1:]:
                _append_point(pt)
            stage_summaries.append(
                {
                    "stage": CourseStage.STAGE_6_FLY_HOME.value,
                    "duration_s": round(spline_home.duration, 3),
                    "v_max_mps": prof_home.v_max,
                    "a_max_mps2": prof_home.a_max,
                    "plan_time_ms": round(elapsed_home_ms, 2),
                    "detected_wall_xy_m": [round(barrier.wall_xy_m[0], 3), round(barrier.wall_xy_m[1], 3)],
                    "detected_wall_top_z_m": round(barrier.wall_top_z_m, 3),
                }
            )

        return {
            "route_type": branch_name,
            "fast_mode": self.config.fast_mode,
            "include_fly_home": include_fly_home and self.config.enable_fly_home,
            "frame_convention": "enu_to_ned",
            "launch_origin_enu": [round(launch_origin_enu[0], 4), round(launch_origin_enu[1], 4), round(launch_origin_enu[2], 4)],
            "stages": stage_summaries,
            "waypoints_enu": waypoints_enu,
            "waypoints_local_enu": waypoints_local_enu,
            "waypoints_ned": waypoints_ned,
            "waypoints_course_ned": waypoints_course_ned,
        }

    def export_route_json(
        self,
        path: str | Path,
        *,
        use_window: bool = False,
        include_fly_home: bool = True,
    ) -> Dict:
        """Export the planned route to JSON for sharing with Bryan's simulator / `refnav`."""
        data = self.plan_full_course_route(use_window=use_window, include_fly_home=include_fly_home)
        Path(path).write_text(json.dumps(data, indent=2), encoding="utf-8")
        return data

    def export_all_routes(self, out_dir: str | Path) -> Dict[str, Dict]:
        """Export the three scored run route files requested by `@sim (Bryan)`:
        - `route_gate_b.json`: Gate B route ending at the Landing Pad (`include_fly_home=False`)
        - `route_window.json`: Window route ending at the Landing Pad (`include_fly_home=False`)
        - `route_fly_home.json`: Full run including post-landing over-the-wall Fly-Home (`include_fly_home=True`)
        - `route.json`: Alias for `route_fly_home.json`
        """
        base = Path(out_dir)
        base.mkdir(parents=True, exist_ok=True)
        gate_b = self.export_route_json(base / "route_gate_b.json", use_window=False, include_fly_home=False)
        window = self.export_route_json(base / "route_window.json", use_window=True, include_fly_home=False)
        fly_home = self.export_route_json(base / "route_fly_home.json", use_window=False, include_fly_home=True)
        (base / "route.json").write_text(json.dumps(fly_home, indent=2), encoding="utf-8")
        return {
            "route_gate_b": gate_b,
            "route_window": window,
            "route_fly_home": fly_home,
        }
