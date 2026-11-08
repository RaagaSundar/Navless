"""19 flies the window route planned on the as-built course map, and its launch settings let it.

The plan's route passes 3.6 cm from a real wall (Shining's phone scan, 2026-11-06). The as-built route
crosses the ~0.6 m gaps between M1's and M3's north ends and the north net 0.256 m from the net. With
navless.launch.py's 0.4 m net margin, course_mission would pull those setpoints into the wall ends.
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_course_launch as launch    # noqa: E402  (launch files parsed with ast)
import test_tube_blind as tb           # noqa: E402  (stub ROS, point-mass drone)
from navless_nav.route import NET_HALF_M, REPO_ROOT, course_to_odom    # noqa: E402

ARGS = launch._arguments(launch.COURSE)
ROUTE = ARGS['route_file']


def test_course_launch_flies_the_asbuilt_route():
    r = json.load(open(os.path.join(REPO_ROOT, ROUTE)))
    assert r['course_file'] == 'gazebo_sim/course/course_asbuilt.json'
    assert r['min_clearance_m_nose_north'] >= 0.08


def test_net_margin_leaves_every_route_point_alone():
    assert 'net_margin_m' in launch._forwarded(launch.COURSE)
    pts = json.load(open(os.path.join(REPO_ROOT, ROUTE)))['waypoints_enu']
    assert max(max(abs(p[0]), abs(p[1])) for p in pts) <= NET_HALF_M - float(ARGS['net_margin_m'])


def fly(**params):
    """The whole route on the stub drone at the slow launch's speeds; the node after its route."""
    h, cm = tb.h, tb.cm
    h.T[0] = 0.0
    h.Z_BIAS[0] = 0.0
    h.OVERRIDES.clear()
    h.OVERRIDES.update(route_file=ROUTE, cruise_speed_mps=0.5, max_lead_m=0.6, max_lat_accel_mps2=0.25,
                       **params)
    node = cm.CourseMission()
    px4 = {'armed': False, 'offboard_active': False, 'landed': True}
    pos = [0.0, 0.0, 0.0]
    for _ in range(30):
        tb.step(node, px4, pos)
    px4.update(armed=True, offboard_active=True)
    for _ in range(1500):
        tb.step(node, px4, pos)
        if node.state == 'LAND':
            break
    return node


def test_flies_the_whole_route_without_a_setpoint_pulled_off_it():
    node = fly(net_margin_m=float(ARGS['net_margin_m']))
    assert node.state == 'LAND' and 'fenced' not in node.log_once
    assert 'fenced' in fly(net_margin_m=0.6).log_once      # overly tight net margin would pull setpoints off route


def test_asbuilt_route_uses_active_ego_planner(monkeypatch):
    called = []
    real_plan = tb.cm.plan_dynamic_course_route

    def tracked_plan(*args, **kwargs):
        called.append((args, kwargs))
        return real_plan(*args, **kwargs)

    monkeypatch.setattr(tb.cm, 'plan_dynamic_course_route', tracked_plan)
    tb.h.OVERRIDES.clear()
    tb.h.OVERRIDES.update(route_file=ROUTE)
    node = tb.cm.CourseMission()
    assert not node.static_asbuilt_route
    assert node.replan_on_obstacles
    assert node.planner_router is not None
    assert len(called) == 1
    assert len(node.route) > 20
    # node.route is in odom (launch mark at the origin): check it inside the net in the course frame
    course = [tb.cm.odom_to_course(p, node.frame[0], node.frame[1], node.start_height) for p in node.route]
    assert max(max(abs(p[0]), abs(p[1])) for p in course) <= NET_HALF_M


def test_asbuilt_world_replanning_ignores_known_walls_and_detours_around_old_m3():
    import numpy as np
    from navless_nav.route import odom_to_course

    tb.h.OVERRIDES.clear()
    tb.h.OVERRIDES.update(route_file=ROUTE, replan_on_obstacles=True)
    node = tb.cm.CourseMission()
    node.position = (0.0, 0.0, 0.78)
    node.start_route()
    initial_route = list(node.route)

    # 1. As-built world: stereo points on as-built MD, MN, MT, and North Net must not trigger false detours
    asbuilt_pts = np.array([
        [-1.08, 1.28, 0.85],
        [-1.50, 2.98, 0.85],
        [0.68, 0.93, 0.85],
        [0.00, 3.03, 0.85],
    ], dtype=np.float64)
    for _ in range(3):
        n_added = node.planner_grid.insert_point_cloud(
            asbuilt_pts, sensor_origin_m=(-1.00, 1.80, 0.85), min_live_hits=2
        )
        assert n_added == 0
    node.live_grid_updated = True
    node.maybe_replan_for_discrepancies(10.0)
    assert node.route == initial_route

    # 2. Generality demo: an unmapped discrepancy wall/pillar at x = +0.55 m, y in [1.95, 2.35] blocks the route
    # and triggers an online 3D A* + EGO B-spline detour around it
    ys = np.linspace(1.95, 2.35, 6)
    zs = np.linspace(0.30, 1.40, 6)
    old_m3_pts = np.array([[0.55, y, z] for y in ys for z in zs], dtype=np.float64)
    for _ in range(3):
        node.planner_grid.insert_point_cloud(
            old_m3_pts, sensor_origin_m=(-0.20, 2.20, 0.85), min_live_hits=2
        )
    node.live_grid_updated = True
    node.maybe_replan_for_discrepancies(20.0)
    assert node.route != initial_route
    # Verify every point on the replanned route clears the discrepancy obstacle
    for q in node.route:
        c_pt = odom_to_course(q, node.frame[0], node.frame[1], node.start_height)
        assert not node.planner_grid.is_discrepancy_occupied(c_pt, clearance_m=0.20)


def test_multithreaded_background_planner_and_forward_travel_heading():
    import time
    import numpy as np
    from navless_nav.route import odom_to_course

    tb.h.OVERRIDES.clear()
    tb.h.OVERRIDES.update(route_file=ROUTE, replan_on_obstacles=True, heading_mode='travel')
    node = tb.cm.CourseMission()
    node.start_background_planner()
    try:
        assert node._bg_worker_enabled
        assert node._cloud_thread.is_alive() and node._bg_thread.is_alive()
        node.position = (0.0, 0.0, 0.78)
        node.start_route()
        initial_route = list(node.route)

        # Insert discrepancy obstacle and trigger non-blocking background replan
        ys = np.linspace(1.95, 2.35, 6)
        zs = np.linspace(0.30, 1.40, 6)
        obs_pts = np.array([[0.55, y, z] for y in ys for z in zs], dtype=np.float64)
        with node._planner_lock:
            for _ in range(3):
                node.planner_grid.insert_point_cloud(
                    obs_pts, sensor_origin_m=(-0.20, 2.20, 0.85), min_live_hits=2
                )
            node.live_grid_updated = True

        # First call dispatches to background thread without blocking
        node.maybe_replan_for_discrepancies(10.0)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and node._bg_replan_result is None:
            time.sleep(0.02)
        assert node._bg_replan_result is not None
        # Next tick swaps in the replanned route in O(1)
        node.maybe_replan_for_discrepancies(10.2)
        assert node.route != initial_route
        assert node.replans == 1
        for q in node.route:
            c_pt = odom_to_course(q, node.frame[0], node.frame[1], node.start_height)
            assert not node.planner_grid.is_discrepancy_occupied(c_pt, clearance_m=0.20)

        # Verify travel_heading always faces the direction of movement along the planner route (never spins backwards in the tube when ToF is off)
        while not node.carrot.at_end:
            want_yaw, is_tube_north = node.travel_heading(node.carrot)
            assert not is_tube_north
            node.carrot.advance(0.25)
    finally:
        node.stop_background_planner()
        assert not node._bg_worker_enabled


def test_disarmed_smoke_test_and_live_map_visualizer():
    import time
    from navless_nav.live_map_viz import LiveMapVisualizer

    tb.h.OVERRIDES.clear()
    tb.h.OVERRIDES.update(route_file=ROUTE, replan_on_obstacles=True, heading_mode='travel')
    node = tb.cm.CourseMission()
    node.start_background_planner(start_http=False)
    viz = LiveMapVisualizer(node, port=0)
    try:
        # Disarmed on launch mark in WAIT_FOR_OFFBOARD
        node.position = (0.0, 0.0, 0.0)
        snap0 = viz.snapshot()
        assert snap0['state'] == 'WAIT_FOR_OFFBOARD'
        assert not snap0['armed']
        assert snap0['cloud_thread_alive'] and snap0['replan_thread_alive']
        assert len(snap0['prior_xy']) > 100
        assert len(snap0['route_enu']) > 20
        assert snap0['discrepancy_xy'] == []

        # Smoke test: inject obstacle 1.2 m ahead while disarmed
        res = viz.inject_test_obstacle()
        assert res['ok']
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and node._bg_replan_result is None:
            time.sleep(0.02)
        node.maybe_replan_for_discrepancies(node.now_s() + 2.0)

        snap1 = viz.snapshot()
        assert len(snap1['discrepancy_xy']) > 0
        assert snap1['replans'] == 1
        assert snap1['route_enu'] != snap0['initial_route_enu']
        # Takeoff altitude of route[0] is preserved even when replanned while disarmed on floor
        assert abs(node.route[0][2] - node.initial_route_odom[0][2]) < 1e-6

        # Reset before arming restores clean map and initial route
        rst = viz.reset_map()
        assert rst['ok']
        snap2 = viz.snapshot()
        assert snap2['discrepancy_xy'] == []
        assert snap2['route_enu'] == snap0['initial_route_enu']
    finally:
        node.stop_background_planner()

