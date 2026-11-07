"""Unit tests for the DroneHacks 2026 EGO-Planner, Topological FSM, and PX4 Bridge."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from planner.bspline import EgoBsplineOptimizer, UniformBspline, astar_search_3d
from planner.config import PlannerConfig
from planner.frames import (
    clamp_altitude_enu,
    enu_to_ned_pos,
    enu_to_ned_vel,
    enu_yaw_to_ned_yaw,
    ned_to_enu_pos,
)
from planner.fsm_router import CourseStage, GlobalCourseRouter
from planner.grid_map import OccupancyGrid3D
from planner.px4_bridge_node import (
    ARMING_STATE_ARMED,
    NAVIGATION_STATE_OFFBOARD,
    PX4TrajectoryTrackerCore,
)


class TestPlannerScaffold(unittest.TestCase):
    def test_frame_conversions_and_z_clamp(self) -> None:
        # ENU [x_east=2.0, y_north=1.5, z_up=0.9] -> NED [x_north=1.5, y_east=2.0, z_down=-0.9]
        pos_enu = (2.0, 1.5, 0.9)
        ned = enu_to_ned_pos(pos_enu)
        self.assertAlmostEqual(ned[0], 1.5)
        self.assertAlmostEqual(ned[1], 2.0)
        self.assertAlmostEqual(ned[2], -0.9)

        recovered = ned_to_enu_pos(ned)
        np.testing.assert_allclose(recovered, pos_enu, atol=1e-6)

        # With launch pad origin offset: launch pad itself maps to NED (0, 0, -z)
        launch_origin = (-2.515, 0.914, 0.0)
        ned_launch = enu_to_ned_pos((-2.515, 0.914, 0.85), origin_enu=launch_origin)
        np.testing.assert_allclose(ned_launch, (0.0, 0.0, -0.85), atol=1e-6)
        np.testing.assert_allclose(
            ned_to_enu_pos(ned_launch, origin_enu=launch_origin),
            (-2.515, 0.914, 0.85),
            atol=1e-6,
        )

        # Verify Z-ceiling clamp enforces [0.5, 1.1] m (below 68-inch 1.73 m wall)
        clamped_high = clamp_altitude_enu((1.0, 2.0, 1.65), z_min=0.5, z_max=1.1)
        self.assertAlmostEqual(float(clamped_high[2]), 1.1)
        clamped_low = clamp_altitude_enu((1.0, 2.0, 0.2), z_min=0.5, z_max=1.1)
        self.assertAlmostEqual(float(clamped_low[2]), 0.5)

    def test_astar_and_bspline_avoid_obstacle_and_respect_limits(self) -> None:
        cfg = PlannerConfig()
        grid = OccupancyGrid3D(
            bounds_min_m=(-1.0, -2.0, 0.0),
            bounds_max_m=(6.0, 3.0, 2.2),
            resolution_m=0.10,
            inflation_radius_m=0.25,
        )
        # Place a boulder/pillar blocking the direct straight line from (0,0,0.9) to (4,0,0.9)
        grid.add_aabb_obstacle((1.8, -0.3, 0.0), (2.2, 0.3, 1.7))

        optimizer = EgoBsplineOptimizer(grid)
        spline, elapsed_ms = optimizer.plan_spline(
            (0.0, 0.0, 0.9),
            (4.0, 0.0, 0.9),
            cfg.maze_profile,
        )
        self.assertIsNotNone(spline)
        assert spline is not None
        self.assertTrue(optimizer.is_spline_collision_free(spline))

        # Check all sampled points obey the [0.5, 1.1] m Z-ceiling clamp
        pts = spline.sample_points(40)
        self.assertTrue(np.all(pts[:, 2] >= 0.5 - 1e-6))
        self.assertTrue(np.all(pts[:, 2] <= 1.1 + 1e-6))

        # Check time reallocation respects stage v_max
        v_peak, _ = spline.max_kinematics()
        self.assertLessEqual(v_peak, cfg.maze_profile.v_max + 1e-3)

    def test_window_vs_gate_b_branching_and_fallback(self) -> None:
        cfg = PlannerConfig()
        grid = OccupancyGrid3D(
            bounds_min_m=cfg.global_map_bounds_min_m,
            bounds_max_m=cfg.global_map_bounds_max_m,
            resolution_m=cfg.voxel_resolution_m,
            inflation_radius_m=cfg.obstacle_inflation_m,
        )
        router = GlobalCourseRouter(cfg, grid)
        router.stage = CourseStage.STAGE_4_CHICANE

        # Case 1: Window is clear and external confidence is high (0.95) -> chooses WINDOW
        dec_win = router.step(
            10.0,
            cfg.checkpoints.window_inspect_point,
            external_window_confidence=0.95,
            force_replan=True,
        )
        self.assertEqual(dec_win.selected_branch, "WINDOW")
        self.assertEqual(dec_win.goal_enu, cfg.checkpoints.window_exit)
        self.assertIsNotNone(dec_win.spline)

        # Case 2: Low confidence (0.45) -> falls back to GATE_B (dynamically routing to landing_pad)
        dec_low_conf = router.step(
            12.0,
            cfg.checkpoints.window_inspect_point,
            external_window_confidence=0.45,
            force_replan=True,
        )
        self.assertEqual(dec_low_conf.selected_branch, "GATE_B")
        self.assertEqual(dec_low_conf.goal_enu, cfg.checkpoints.landing_pad)

        # Case 3: Window aperture physically blocked in voxel grid -> falls back to GATE_B
        wx, wy, wz = cfg.checkpoints.window_aperture
        grid.add_aabb_obstacle((wx - 0.1, wy - 0.2, wz - 0.2), (wx + 0.1, wy + 0.2, wz + 0.2))
        dec_blocked = router.step(
            14.0,
            cfg.checkpoints.window_inspect_point,
            external_window_confidence=0.99,
            force_replan=True,
        )
        self.assertEqual(dec_blocked.selected_branch, "GATE_B")

    def test_1hz_replanning_and_fly_home_over_wall(self) -> None:
        cfg = PlannerConfig(fast_mode=True)
        grid = OccupancyGrid3D(
            bounds_min_m=cfg.global_map_bounds_min_m,
            bounds_max_m=cfg.global_map_bounds_max_m,
            resolution_m=cfg.voxel_resolution_m,
            inflation_radius_m=cfg.obstacle_inflation_m,
        )
        router = GlobalCourseRouter(cfg, grid)

        # Initial plan at t = 0.0 s
        d0 = router.step(0.0, cfg.checkpoints.start_pad)
        self.assertTrue(d0.replanned)

        # At t = 0.4 s (< 1.0 s replan period), should not replan
        d1 = router.step(0.4, cfg.checkpoints.start_pad)
        self.assertFalse(d1.replanned)

        # At t = 1.05 s (>= 1.0 s replan period), triggers 1 Hz re-check
        d2 = router.step(1.05, cfg.checkpoints.start_pad)
        self.assertTrue(d2.replanned)

        # Verify Stage 6 Fly Home dynamically jumps over the barrier back to recorded_start_pos_enu
        router.stage = CourseStage.STAGE_5_LANDING_PAD
        d_fly_home = router.step(20.0, cfg.checkpoints.landing_pad)
        self.assertEqual(d_fly_home.stage, CourseStage.STAGE_6_FLY_HOME)
        self.assertEqual(d_fly_home.goal_enu[0], cfg.checkpoints.start_pad[0])
        self.assertEqual(d_fly_home.goal_enu[1], cfg.checkpoints.start_pad[1])
        self.assertGreaterEqual(d_fly_home.goal_enu[2], 1.85)

    def test_px4_tracker_safety_and_authority_gates(self) -> None:
        cfg = PlannerConfig()
        grid = OccupancyGrid3D()
        router = GlobalCourseRouter(cfg, grid)
        tracker = PX4TrajectoryTrackerCore(router)

        # Without telemetry -> refuses to produce a setpoint
        sample, reason = tracker.sample_setpoint(1.0)
        self.assertIsNone(sample)
        self.assertEqual(reason, "missing_vehicle_local_position")

        # Provide valid telemetry and healthy estimator
        tracker.update_local_position(1.0, 0.0, 0.0, -0.9, 0.0, 0.0, 0.0, 0.0, True, True)
        tracker.update_vehicle_status(1.0, ARMING_STATE_ARMED, NAVIGATION_STATE_OFFBOARD)
        tracker.update_estimator_health(True)

        sample_ok, reason_ok = tracker.sample_setpoint(1.1)
        self.assertIsNotNone(sample_ok)
        assert sample_ok is not None
        # NED z must be negative (altitude above floor in [0.5, 1.1] m -> z_ned in [-1.1, -0.5])
        self.assertLessEqual(sample_ok.position_ned[2], -0.5)
        self.assertGreaterEqual(sample_ok.position_ned[2], -1.1)

        # Verify /tube/offset lateral centering nudge in STAGE_3_TUNNEL
        tracker.router.stage = CourseStage.STAGE_3_TUNNEL
        sample_before_tube, _ = tracker.sample_setpoint(1.1)
        assert sample_before_tube is not None
        tracker.update_tube_offset(1.1, offset_left_m=0.10, width_m=0.76)
        sample_tube, reason_tube = tracker.sample_setpoint(1.1)
        self.assertIsNotNone(sample_tube)
        self.assertIn("spline", reason_tube)
        assert sample_tube is not None
        self.assertGreater(
            float(
                np.linalg.norm(
                    np.asarray(sample_tube.position_ned[:2])
                    - np.asarray(sample_before_tube.position_ned[:2])
                )
            ),
            0.01,
        )

        # Simulate pilot RC takeover out of OFFBOARD -> immediately yields authority
        tracker.update_vehicle_status(1.2, ARMING_STATE_ARMED, nav_state=2)
        sample_takeover, reason_takeover = tracker.sample_setpoint(1.2)
        self.assertIsNone(sample_takeover)
        self.assertEqual(reason_takeover, "pilot_takeover_or_offboard_lost")

        # Simulate stale telemetry -> refuses to publish
        tracker.update_vehicle_status(1.2, ARMING_STATE_ARMED, NAVIGATION_STATE_OFFBOARD)
        sample_stale, reason_stale = tracker.sample_setpoint(2.0)
        self.assertIsNone(sample_stale)
        self.assertEqual(reason_stale, "stale_vehicle_local_position")

    def test_export_route_json_for_gate_b(self) -> None:
        cfg = PlannerConfig()
        grid = OccupancyGrid3D()
        router = GlobalCourseRouter(cfg, grid)
        with tempfile.TemporaryDirectory() as tmp:
            routes = router.export_all_routes(tmp)
            self.assertEqual(routes["route_gate_b"]["route_type"], "GATE_B")
            self.assertFalse(routes["route_gate_b"]["include_fly_home"])
            self.assertEqual(routes["route_window"]["route_type"], "WINDOW")
            self.assertFalse(routes["route_window"]["include_fly_home"])
            self.assertTrue(routes["route_fly_home"]["include_fly_home"])
            self.assertTrue((Path(tmp) / "route_gate_b.json").exists())
            self.assertTrue((Path(tmp) / "route_window.json").exists())
            self.assertTrue((Path(tmp) / "route_fly_home.json").exists())
            # First NED waypoint at launch pad must be (0.0, 0.0, -0.85)
            first_ned = routes["route_gate_b"]["waypoints_ned"][0]
            np.testing.assert_allclose(first_ned, [0.0, 0.0, -0.85], atol=1e-3)

    def test_real_course_v3_obj_gate_b_and_window_routes(self) -> None:
        obj_path = Path(__file__).resolve().parent.parent / "dronehacks-course-v3.obj"
        if not obj_path.exists():
            self.skipTest("dronehacks-course-v3.obj not present")
        cfg = PlannerConfig()
        grid = OccupancyGrid3D(
            bounds_min_m=cfg.global_map_bounds_min_m,
            bounds_max_m=cfg.global_map_bounds_max_m,
            resolution_m=cfg.voxel_resolution_m,
            inflation_radius_m=cfg.obstacle_inflation_m,
        )
        n_vox = grid.load_prior_from_obj(obj_path)
        self.assertGreater(n_vox, 5000)

        # Verify dynamic discovery of Tunnel, Window cutout, and Concentric Landing Rings from 3D geometry
        features = grid.discover_course_features()
        self.assertIsNotNone(features.window_center_m)
        self.assertIsNotNone(features.concentric_pad)
        assert features.window_center_m is not None
        assert features.concentric_pad is not None
        self.assertAlmostEqual(features.window_center_m[0], -0.47, delta=0.20)
        self.assertAlmostEqual(features.window_center_m[1], -2.27, delta=0.20)
        self.assertAlmostEqual(features.concentric_pad.center_xy_m[0], -2.042, delta=0.02)
        self.assertAlmostEqual(features.concentric_pad.center_xy_m[1], -0.792, delta=0.02)
        self.assertAlmostEqual(features.concentric_pad.outer_radius_m, 0.762, delta=0.03)

        # Verify that even if the exact bullseye center of the concentric rings is blocked,
        # 3D A* with goal_radius_m=0.762 ("closer to the goal is better") reaches the innermost clear ring!
        cx, cy = features.concentric_pad.center_xy_m
        grid_blocked_bullseye = OccupancyGrid3D(
            bounds_min_m=cfg.global_map_bounds_min_m,
            bounds_max_m=cfg.global_map_bounds_max_m,
            resolution_m=cfg.voxel_resolution_m,
            inflation_radius_m=cfg.obstacle_inflation_m,
        )
        grid_blocked_bullseye.add_aabb_obstacle((cx - 0.05, cy - 0.05, 0.0), (cx + 0.05, cy + 0.05, 1.5))
        path_concentric = astar_search_3d(
            grid_blocked_bullseye,
            (-1.2, -1.5, 0.85),
            (cx, cy, 0.85),
            goal_radius_m=features.concentric_pad.outer_radius_m,
        )
        self.assertIsNotNone(path_concentric)
        assert path_concentric is not None
        dist_to_bullseye = float(np.linalg.norm(path_concentric[-1][:2] - np.array([cx, cy])))
        self.assertLessEqual(dist_to_bullseye, features.concentric_pad.outer_radius_m)

        router = GlobalCourseRouter(cfg, grid)
        gate_b_route = router.plan_full_course_route(use_window=False)
        self.assertEqual(gate_b_route["route_type"], "GATE_B")
        self.assertGreaterEqual(len(gate_b_route["stages"]), 5)

        window_route = router.plan_full_course_route(use_window=True)
        self.assertEqual(window_route["route_type"], "WINDOW")
        self.assertEqual(len(window_route["stages"]), 6)

    def test_perimeter_net_border_prevents_outside_dodging(self) -> None:
        cfg = PlannerConfig()
        # Even on an empty grid (no .obj loaded, invisible net), the perimeter net border is active
        grid = OccupancyGrid3D(
            bounds_min_m=cfg.global_map_bounds_min_m,
            bounds_max_m=cfg.global_map_bounds_max_m,
            resolution_m=cfg.voxel_resolution_m,
            inflation_radius_m=cfg.obstacle_inflation_m,
        )
        # Points near the ±3.048m perimeter net must be occupied in the inflated grid
        self.assertTrue(grid.is_occupied((-3.02, 0.0, 0.85), inflated=False))
        self.assertTrue(grid.is_occupied((0.0, -2.80, 0.85), inflated=True))
        self.assertTrue(grid.is_occupied((0.0, 2.80, 0.85), inflated=True))

        # Place a boulder near the South net at y = -2.40 m and plan past it:
        # Planner must dodge NORTH (toward interior y > -2.40), never SOUTH into the invisible net!
        grid.add_aabb_obstacle((-0.25, -2.65, 0.0), (0.25, -2.20, 1.5))
        opt = EgoBsplineOptimizer(grid)
        spline, _ = opt.plan_spline((1.0, -2.40, 0.85), (-1.0, -2.40, 0.85), cfg.chicane_profile)
        self.assertIsNotNone(spline)
        assert spline is not None
        pts = spline.sample_points(40)
        # Every point must stay >= 0.35 m inside the -3.048 m South net (y >= -2.70 m) and dodge North (y > -2.0 m)
        self.assertTrue(np.all(pts[:, 1] >= -2.70))
        self.assertGreater(float(np.max(pts[:, 1])), -2.00)

    def test_skybeta_mission_yaw_alignment_and_touch_and_go_fly_home(self) -> None:
        cfg = PlannerConfig()
        grid = OccupancyGrid3D(
            bounds_min_m=cfg.global_map_bounds_min_m,
            bounds_max_m=cfg.global_map_bounds_max_m,
            resolution_m=cfg.voxel_resolution_m,
            inflation_radius_m=cfg.obstacle_inflation_m,
        )
        router = GlobalCourseRouter(cfg, grid)
        tracker = PX4TrajectoryTrackerCore(router, start_bearing_deg=0.0, start_height_m=0.07, touch_and_go_fly_home=True)

        # 1. WAIT_FOR_OFFBOARD pre-streams current odom position before arming/Offboard
        tracker.update_local_position(0.0, 0.0, 0.0, -0.07, 0.0, 0.0, 0.0, 0.0, True, True)
        tracker.update_estimator_health(True)
        tracker.update_vehicle_status(0.0, arming_state=1, nav_state=2, landed=True)
        cmd_wait = tracker.step_skybeta_mission(0.0)
        self.assertEqual(cmd_wait.skybeta_state, "WAIT_FOR_OFFBOARD")
        self.assertEqual(cmd_wait.mode, "pose")

        # 2. Enter Offboard -> immediately transitions to TAKEOFF
        tracker.update_vehicle_status(0.1, ARMING_STATE_ARMED, NAVIGATION_STATE_OFFBOARD, landed=False)
        cmd_to = tracker.step_skybeta_mission(0.1)
        self.assertEqual(cmd_to.skybeta_state, "TAKEOFF")
        self.assertEqual(cmd_to.mode, "pose")

        # 3. Reach takeoff altitude (z = 0.85 m -> z_ned = -0.85 m) -> transitions to ROUTE with valid unit quaternion
        tracker.update_local_position(1.0, 0.0, 0.0, -0.85, 0.0, 0.0, 0.0, 0.0, True, True)
        cmd_route = tracker.step_skybeta_mission(1.0)
        self.assertEqual(cmd_route.skybeta_state, "ROUTE")
        self.assertEqual(cmd_route.mode, "pose")
        q_norm = float(np.linalg.norm(cmd_route.quat_xyzw))
        self.assertAlmostEqual(q_norm, 1.0, places=5)

        # 4. Arrive over Concentric Landing Pad -> enters PAD_LAND two-stage TwistStamped flare
        router.stage = CourseStage.STAGE_5_LANDING_PAD
        lp_ned = enu_to_ned_pos(cfg.checkpoints.landing_pad, origin_enu=tracker.launch_origin_enu)
        tracker.update_local_position(10.0, lp_ned[0], lp_ned[1], -0.60, 0.0, 0.0, 0.0, 0.0, True, True)
        cmd_pad_fast = tracker.step_skybeta_mission(10.0)
        self.assertEqual(cmd_pad_fast.skybeta_state, "PAD_LAND")
        self.assertEqual(cmd_pad_fast.mode, "twist")
        self.assertAlmostEqual(cmd_pad_fast.vel_odom[2], -0.38, places=3)

        # Below z = 0.35 m -> ground-effect flare slows vertical descent to -0.18 m/s
        tracker.update_local_position(10.5, lp_ned[0], lp_ned[1], -0.25, 0.0, 0.0, 0.0, 0.0, True, True)
        cmd_pad_flare = tracker.step_skybeta_mission(10.5)
        self.assertEqual(cmd_pad_flare.mode, "twist")
        self.assertAlmostEqual(cmd_pad_flare.vel_odom[2], -0.18, places=3)

        # 5. Touchdown on Concentric Pad (z = 0.08 m) for 0.85 s -> Touch-and-Go transitions to FLY_HOME!
        tracker.update_local_position(11.0, lp_ned[0], lp_ned[1], -0.08, 0.0, 0.0, 0.0, 0.0, True, True)
        tracker.update_vehicle_status(11.0, ARMING_STATE_ARMED, NAVIGATION_STATE_OFFBOARD, landed=False)
        tracker.step_skybeta_mission(11.0)
        tracker.update_local_position(11.9, lp_ned[0], lp_ned[1], -0.08, 0.0, 0.0, 0.0, 0.0, True, True)
        tracker.update_vehicle_status(11.9, ARMING_STATE_ARMED, NAVIGATION_STATE_OFFBOARD, landed=False)
        cmd_fly_home = tracker.step_skybeta_mission(11.9)
        self.assertEqual(cmd_fly_home.skybeta_state, "FLY_HOME")
        self.assertEqual(cmd_fly_home.mode, "pose")
        self.assertEqual(router.stage, CourseStage.STAGE_6_FLY_HOME)
        self.assertAlmostEqual(float(np.linalg.norm(cmd_fly_home.quat_xyzw)), 1.0, places=5)

        # 6. Arrive over Start Pad after wall hop -> enters HOME_LAND flare -> disarms to DONE
        tracker.update_local_position(18.0, 0.0, 0.0, -0.30, 0.0, 0.0, 0.0, 0.0, True, True)
        tracker.update_vehicle_status(18.0, ARMING_STATE_ARMED, NAVIGATION_STATE_OFFBOARD, landed=False)
        cmd_home_land = tracker.step_skybeta_mission(18.0)
        self.assertEqual(cmd_home_land.skybeta_state, "HOME_LAND")
        self.assertEqual(cmd_home_land.mode, "twist")
        self.assertAlmostEqual(cmd_home_land.vel_odom[2], -0.18, places=3)

        tracker.update_vehicle_status(19.0, arming_state=1, nav_state=2, landed=True)
        cmd_done = tracker.step_skybeta_mission(19.0)
        self.assertEqual(cmd_done.skybeta_state, "DONE")
        self.assertEqual(cmd_done.mode, "none")


if __name__ == "__main__":
    unittest.main()
