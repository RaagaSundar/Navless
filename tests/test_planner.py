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
    NAVIGATION_STATE_AUTO_LAND,
    NAVIGATION_STATE_OFFBOARD,
    VEHICLE_CMD_COMPONENT_ARM_DISARM,
    VEHICLE_CMD_DO_SET_MODE,
    VEHICLE_CMD_NAV_LAND,
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

        # Verify pre-arm navigation settle + launch-pad zero-tare + vertical TAKEOFF_CLIMB gate
        tracker2 = PX4TrajectoryTrackerCore(GlobalCourseRouter(cfg, grid), prearm_settle_s=2.0)
        tracker2.update_estimator_health(True)
        # Disarmed on launch pad with a small non-zero spawn offset (0.12, -0.05, -0.02) NED
        tracker2.update_vehicle_status(0.0, 1, nav_state=2)
        tracker2.update_local_position(0.0, 0.12, -0.05, -0.02, 0.0, 0.0, 0.0, 0.0, True, True)
        sp_settle, r_settle = tracker2.sample_setpoint(0.5)
        self.assertIsNone(sp_settle)
        self.assertIn("prearm_position_settling", r_settle)
        # After 2.0 s stationary on the pad, pre-arm zero-tare locks in and pre-offboard stream begins
        tracker2.update_vehicle_status(2.6, 1, nav_state=2)
        tracker2.update_local_position(2.6, 0.12, -0.05, -0.02, 0.0, 0.0, 0.0, 0.0, True, True)
        sp_ready, r_ready = tracker2.sample_setpoint(2.6)
        self.assertIsNotNone(sp_ready)
        self.assertTrue(tracker2.nav_zeroed)
        np.testing.assert_allclose(tracker2.nav_zero_offset_ned, (0.12, -0.05, -0.02), atol=1e-6)
        # Arm in Offboard while still near the floor (z_enu = 0.10 m < 0.65 m): must hold XY at (0,0) in TAKEOFF_CLIMB
        tracker2.update_vehicle_status(5.0, ARMING_STATE_ARMED, NAVIGATION_STATE_OFFBOARD)
        tracker2.update_local_position(5.0, 0.12, -0.05, -0.12, 0.0, 0.0, 0.0, 0.0, True, True)
        sp_climb, r_climb = tracker2.sample_setpoint(5.0)
        self.assertIsNotNone(sp_climb)
        self.assertIn("climbing_to_takeoff_altitude", r_climb)
        assert sp_climb is not None
        self.assertEqual(sp_climb.branch, "TAKEOFF_CLIMB")
        np.testing.assert_allclose(sp_climb.position_ned[:2], (0.15, 0.0), atol=1e-3)
        self.assertAlmostEqual(tracker2.router.spline_start_time_s, 5.0, places=3)
        # Impossible position jump (> 0.85 m in 0.05 s) must trip position_jump_detected and refuse setpoints
        tracker2.update_local_position(5.05, -15.0, 12.0, -3.5, 0.0, 0.0, 0.0, 0.0, True, True)
        sp_jump, r_jump = tracker2.sample_setpoint(5.05)
        self.assertIsNone(sp_jump)
        self.assertTrue("position_jump" in r_jump or "geofence_breach" in r_jump)

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

    def test_px4_mission_takeoff_route_pad_land_fly_home_land(self) -> None:
        cfg = PlannerConfig()
        router = GlobalCourseRouter(cfg, OccupancyGrid3D())
        tracker = PX4TrajectoryTrackerCore(router)
        origin = tracker.launch_origin_enu
        DISARMED, MANUAL = 1, 2

        def tick(t, ned, arming=DISARMED, nav=MANUAL, landed=True):
            tracker.update_local_position(t, ned[0], ned[1], ned[2], 0.0, 0.0, 0.0, 0.0, True, True)
            tracker.update_vehicle_status(t, arming, nav)
            tracker.update_landed(t, landed)
            tracker.update_estimator_health(True)
            return tracker.step_px4_mission(t)

        ground = (0.0, 0.0, -0.07)
        # PREFLIGHT -> PRIME: streams a ground hold, sends nothing until PX4 has >= 1 s of stream
        c = tick(0.0, ground)
        self.assertEqual((c.state, c.commands), ("PRIME", ()))
        self.assertIsNotNone(c.setpoint)
        self.assertEqual(tick(0.5, ground).commands, ())
        c = tick(1.1, ground)
        self.assertEqual(c.state, "ARMING")
        self.assertEqual(c.commands, ((VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0), (VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)))
        self.assertEqual(tick(1.5, ground).commands, ())  # re-sent at most once a second

        # Armed + Offboard -> TAKEOFF: climb to the start-pad height, leaning 0.20 m north
        armed = dict(arming=ARMING_STATE_ARMED, nav=NAVIGATION_STATE_OFFBOARD, landed=False)
        self.assertEqual(tick(2.0, ground, **armed).state, "TAKEOFF")
        c = tick(2.1, (0.0, 0.0, -0.3), **armed)
        assert c.setpoint is not None
        self.assertAlmostEqual(c.setpoint.position_ned[0], 0.20, places=3)
        self.assertAlmostEqual(c.setpoint.position_ned[2], -cfg.checkpoints.start_pad[2], places=3)
        # The pre-arm zero-tare made the resting -0.07 m read as 0; PX4 must get its own frame back
        self.assertAlmostEqual(tracker.to_px4_frame(c.setpoint.position_ned)[2], -cfg.checkpoints.start_pad[2] - 0.07, places=3)

        # At height (raw -0.85 -> 0.78 m above the tared ground) -> ROUTE streams the live spline
        c = tick(3.0, (0.2, 0.0, -0.85), **armed)
        self.assertEqual(c.state, "ROUTE")
        c = tick(3.1, (0.2, 0.0, -0.85), **armed)
        self.assertEqual(c.state, "ROUTE")
        self.assertIsNotNone(c.setpoint)

        # Over the Concentric Pad -> LAND with PX4 NAV_LAND, re-sent once a second until PX4 is in Land
        router.stage = CourseStage.STAGE_5_LANDING_PAD
        pad = enu_to_ned_pos(cfg.checkpoints.landing_pad, origin_enu=origin)
        c = tick(20.0, (pad[0], pad[1], -0.8), **armed)
        self.assertEqual((c.state, c.commands), ("LAND", ((VEHICLE_CMD_NAV_LAND, 0.0, 0.0),)))
        c = tick(21.5, (pad[0], pad[1], -0.4), arming=ARMING_STATE_ARMED, nav=NAVIGATION_STATE_AUTO_LAND, landed=False)
        self.assertEqual((c.state, c.commands, c.setpoint), ("LAND", (), None))

        # Disarmed on the pad -> fly-home leg: re-arm from the pad, then straight into ROUTE
        pad_ground = (pad[0], pad[1], -0.07)
        c = tick(25.0, pad_ground)
        self.assertEqual((c.state, c.reason), ("PREFLIGHT", "landed_on_pad_rearming_for_fly_home"))
        self.assertEqual(tick(25.1, pad_ground).state, "PRIME")
        self.assertEqual(tick(26.2, pad_ground).state, "ARMING")
        c = tick(27.0, pad_ground, **armed)
        self.assertEqual(c.state, "ROUTE")
        self.assertEqual(router.stage, CourseStage.STAGE_6_FLY_HOME)

        # Back over the start pad -> LAND -> disarmed and landed -> DONE
        c = tick(40.0, (0.0, 0.0, -1.0), **armed)
        self.assertEqual(c.state, "LAND")
        c = tick(45.0, ground)
        self.assertEqual((c.state, c.reason), ("DONE", "landed_and_disarmed"))
        self.assertEqual(tick(46.0, ground).setpoint, None)

    def test_px4_mission_safety_exits(self) -> None:
        def fresh():
            t = PX4TrajectoryTrackerCore(GlobalCourseRouter(PlannerConfig(), OccupancyGrid3D()))
            t.update_estimator_health(True)
            return t

        def feed(t, now, ned, arming, nav, landed):
            t.update_local_position(now, ned[0], ned[1], ned[2], 0.0, 0.0, 0.0, 0.0, True, True)
            t.update_vehicle_status(now, arming, nav)
            t.update_landed(now, landed)
            return t.step_px4_mission(now)

        # Never takes over a vehicle that is already armed
        t = fresh()
        c = feed(t, 0.0, (0.0, 0.0, -0.07), ARMING_STATE_ARMED, 2, True)
        self.assertEqual((c.state, c.commands, c.setpoint), ("ABORT", (), None))

        # Not landed -> stays in PREFLIGHT, publishes nothing
        t = fresh()
        c = feed(t, 0.0, (0.0, 0.0, -1.0), 1, 2, False)
        self.assertEqual((c.state, c.setpoint), ("PREFLIGHT", None))

        # In flight: PX4 leaves Offboard (pilot/failsafe) -> yields, publishes nothing more
        t = fresh()
        for now, arming, nav, landed in ((0.0, 1, 2, True), (1.1, 1, 2, True), (2.0, ARMING_STATE_ARMED, NAVIGATION_STATE_OFFBOARD, False)):
            feed(t, now, (0.0, 0.0, -0.07), arming, nav, landed)
        self.assertEqual(t.px4_state, "TAKEOFF")
        c = feed(t, 2.5, (0.0, 0.0, -0.4), ARMING_STATE_ARMED, 4, False)
        self.assertEqual((c.state, c.setpoint), ("ABORT", None))

        # In flight: estimator unhealthy -> hold for 1 s, then NAV_LAND
        t = fresh()
        for now, arming, nav, landed in ((0.0, 1, 2, True), (1.1, 1, 2, True), (2.0, ARMING_STATE_ARMED, NAVIGATION_STATE_OFFBOARD, False)):
            feed(t, now, (0.0, 0.0, -0.07), arming, nav, landed)
        t.update_estimator_health(False)
        c = feed(t, 2.5, (0.0, 0.0, -0.4), ARMING_STATE_ARMED, NAVIGATION_STATE_OFFBOARD, False)
        self.assertEqual(c.state, "TAKEOFF")
        self.assertIsNotNone(c.setpoint)
        c = feed(t, 3.6, (0.0, 0.0, -0.4), ARMING_STATE_ARMED, NAVIGATION_STATE_OFFBOARD, False)
        self.assertEqual((c.state, c.commands), ("LAND", ((VEHICLE_CMD_NAV_LAND, 0.0, 0.0),)))

    def test_direct_px4_vio_grace_and_pad_land_fly_home_sequence(self) -> None:
        cfg = PlannerConfig()
        grid = OccupancyGrid3D(
            bounds_min_m=cfg.global_map_bounds_min_m,
            bounds_max_m=cfg.global_map_bounds_max_m,
            resolution_m=cfg.voxel_resolution_m,
            inflation_radius_m=cfg.obstacle_inflation_m,
        )
        router = GlobalCourseRouter(cfg, grid)
        tracker = PX4TrajectoryTrackerCore(
            router,
            launch_origin_enu=(-2.515, 1.114, 0.126),
            touch_and_go_fly_home=False,
            prearm_settle_s=2.0,
        )

        # 1. Refuse to take over if already armed before pre-arm settle completes
        tracker.update_vslam_status(0.0, 1)
        tracker.update_vehicle_status(0.0, ARMING_STATE_ARMED, NAVIGATION_STATE_OFFBOARD)
        tracker.update_local_position(0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0, True, True)
        sp_armed, r_armed = tracker.sample_setpoint(0.0)
        self.assertIsNone(sp_armed)
        self.assertEqual(r_armed, "refusing_already_armed_before_prearm_settle")

        # 2. Disarmed 2.0 s settle -> PREARM_STREAM -> TAKEOFF_CLIMB -> active route
        tracker.update_vehicle_status(0.1, 1, 2)
        tracker.update_local_position(0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, True, True)
        tracker.sample_setpoint(0.1)
        tracker.update_vehicle_status(2.2, 1, 2)
        tracker.update_local_position(2.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, True, True)
        sp_stream, _ = tracker.sample_setpoint(2.2)
        self.assertIsNotNone(sp_stream)
        assert sp_stream is not None
        self.assertEqual(sp_stream.branch, "PREARM_STREAM")

        tracker.update_vehicle_status(3.5, ARMING_STATE_ARMED, NAVIGATION_STATE_OFFBOARD)
        tracker.update_local_position(3.5, 0.0, 0.0, -0.75, 0.0, 0.0, 0.0, 0.0, True, True)
        sp_fly, _ = tracker.sample_setpoint(3.5)
        self.assertIsNotNone(sp_fly)

        # 3. Transient < 1.0 s VIO dropout (vo_state=2 for 0.3 s) holds safe position instead of aborting
        tracker.update_vslam_status(4.0, 2)
        sp_grace, r_grace = tracker.sample_setpoint(4.2)
        self.assertIsNotNone(sp_grace)
        assert sp_grace is not None
        self.assertEqual(sp_grace.branch, "VIO_GRACE_HOLD")
        self.assertIn("vio_grace_hold", r_grace)

        # Persistent > 1.0 s VIO dropout (1.2 s) returns None and triggers emergency land
        tracker.update_local_position(5.25, 0.0, 0.0, -0.75, 0.0, 0.0, 0.0, 0.0, True, True)
        sp_abort, r_abort = tracker.sample_setpoint(5.25)
        self.assertIsNone(sp_abort)
        self.assertIn("vslam_lost", r_abort)

        # Recover VIO -> resumes tracking
        tracker.update_vslam_status(5.5, 1)
        tracker.update_local_position(5.5, 0.0, 0.0, -0.75, 0.0, 0.0, 0.0, 0.0, True, True)
        sp_rec, _ = tracker.sample_setpoint(5.5)
        self.assertIsNotNone(sp_rec)

        # 4. Arrive at Concentric Pad -> commands PAD_LAND -> disarms -> streams pad re-arm -> flies home
        router.stage = CourseStage.STAGE_5_LANDING_PAD
        lp_ned = enu_to_ned_pos(cfg.checkpoints.landing_pad, origin_enu=tracker.launch_origin_enu)
        tracker.update_vehicle_status(10.0, ARMING_STATE_ARMED, NAVIGATION_STATE_OFFBOARD)
        tracker.update_local_position(10.0, lp_ned[0], lp_ned[1], lp_ned[2], 0.0, 0.0, 0.0, 0.0, True, True)
        sp_pad_land, r_pad_land = tracker.sample_setpoint(10.0)
        self.assertIsNotNone(sp_pad_land)
        assert sp_pad_land is not None
        self.assertEqual(sp_pad_land.branch, "PAD_LAND")
        self.assertEqual(r_pad_land, "landing_on_concentric_pad")

        # Disarm on Concentric Pad -> transitions to PREARM_STREAM for Stage 6 Fly-Home
        tracker.update_vehicle_status(12.0, 1, 2)
        tracker.update_local_position(12.0, lp_ned[0], lp_ned[1], 0.0, 0.0, 0.0, 0.0, 0.0, True, True)
        sp_pad_rearm, _ = tracker.sample_setpoint(12.0)
        self.assertIsNotNone(sp_pad_rearm)
        assert sp_pad_rearm is not None
        self.assertEqual(sp_pad_rearm.branch, "PREARM_STREAM")
        self.assertEqual(sp_pad_rearm.stage, CourseStage.STAGE_6_FLY_HOME)


if __name__ == "__main__":
    unittest.main()
