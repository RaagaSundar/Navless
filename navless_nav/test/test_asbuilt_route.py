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
    assert 'fenced' in fly(net_margin_m=0.4).log_once      # navless.launch.py's margin would have


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
    assert max(max(abs(p[0]), abs(p[1])) for p in node.route) <= NET_HALF_M


def test_asbuilt_world_replanning_ignores_known_walls_and_detours_around_old_m3():
    import numpy as np
    from navless_nav.route import odom_to_course

    tb.h.OVERRIDES.clear()
    tb.h.OVERRIDES.update(route_file=ROUTE, replan_on_obstacles=True)
    node = tb.cm.CourseMission()
    node.position = (0.0, 0.0, 0.78)
    node.start_route()
    initial_route = list(node.route)

    # 1. As-built world: stereo points on as-built M1, M3, and North Net must not trigger false detours
    asbuilt_pts = np.array([
        [-1.90, 2.00, 0.85],
        [-1.90, 2.40, 0.85],
        [1.10, 2.00, 0.85],
        [0.00, 3.03, 0.85],
    ], dtype=np.float64)
    for _ in range(3):
        n_added = node.planner_grid.insert_point_cloud(
            asbuilt_pts, sensor_origin_m=(-1.90, 1.50, 0.85), min_live_hits=2
        )
        assert n_added == 0
    node.live_grid_updated = True
    node.maybe_replan_for_discrepancies(10.0)
    assert node.route == initial_route

    # 2. Generality demo: FlyX/old-course M3 wall at x = +0.25 m, y in [1.22, 2.44] blocks the route
    # and triggers an online 3D A* + EGO B-spline detour around old M3
    ys = np.linspace(1.22, 2.44, 14)
    zs = np.linspace(0.30, 1.40, 6)
    old_m3_pts = np.array([[0.25, y, z] for y in ys for z in zs], dtype=np.float64)
    for _ in range(3):
        node.planner_grid.insert_point_cloud(
            old_m3_pts, sensor_origin_m=(-0.20, 2.20, 0.85), min_live_hits=2
        )
    node.live_grid_updated = True
    node.maybe_replan_for_discrepancies(20.0)
    assert node.route != initial_route
    # Verify every point on the replanned route clears old M3
    for q in node.route:
        c_pt = odom_to_course(q, node.frame[0], node.frame[1], node.start_height)
        assert not node.planner_grid.is_discrepancy_occupied(c_pt, clearance_m=0.20)
