"""The pieces added when 19's flight features were merged onto main's planner course_mission:
route_source, the wall correction's target under main's odom_zero_offset, and its easing.
On test_fly_home.py's stub ROS (via test_wall_correction's helpers)."""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_wall_correction as wc    # noqa: E402  (stubs ROS, imports course_mission)

h, cm = wc.h, wc.cm
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
WINDOW = 'gazebo_sim/course/route_centreline_window_asbuilt.json'


def node_with(**overrides):
    h.T[0] = 0.0
    h.SUBS.clear()
    h.OVERRIDES.clear()
    h.OVERRIDES.update(**overrides)
    return cm.CourseMission()


def test_route_source_planner_is_the_default():
    node = node_with(route_file=WINDOW)
    assert not node.static_asbuilt_route
    assert node.route


def test_route_source_file_flies_the_exported_route():
    node = node_with(route_file=WINDOW, route_source='file')
    assert node.static_asbuilt_route
    with open(os.path.join(REPO, WINDOW)) as f:
        data = json.load(f)
    wps = data['waypoints_enu']
    assert len(node.route) == len(wps)
    launch = data['launch_origin_enu']
    # course -> odom with bearing 0: odom x = course north, y = course west; start height 0
    first = node.route[0]
    assert math.isclose(first[0], wps[0][1] - launch[1], abs_tol=1e-9)
    assert math.isclose(first[1], -(wps[0][0] - launch[0]), abs_tol=1e-9)


def test_a_bad_route_source_falls_back_to_the_planner():
    node = node_with(route_file=WINDOW, route_source='bogus')
    assert not node.static_asbuilt_route


def test_correction_target_accounts_for_the_arming_zero_offset():
    node = node_with(route_file=WINDOW, wall_correction=True)
    node.odom_zero_offset = (0.10, -0.05, 0.02)
    node.corr_rx = (0.0, 0.0, 0.0)
    assert node.correction_target() == (-0.10, 0.05, 0.0)
    node.corr_rx = (0.30, -0.20, math.radians(4.0))
    tx, ty, ta = node.correction_target()
    assert math.isclose(tx, 0.20) and math.isclose(ty, -0.15) and ta == math.radians(4.0)


def test_with_no_zero_offset_the_target_is_d():
    node = node_with(route_file=WINDOW, wall_correction=True)
    node.odom_zero_offset = (0.0, 0.0, 0.0)
    node.corr_rx = (0.30, -0.20, math.radians(4.0))
    assert node.correction_target() == node.corr_rx


def _place(e, p):
    c, s = math.cos(e[2]), math.sin(e[2])
    return (c * p[0] - s * p[1] + e[0], s * p[0] + c * p[1] + e[1])


def test_easing_limits_how_fast_the_drone_point_moves_not_dx_dy_alone():
    """A 5 deg correction whose shift keeps a drone 5 m from the launch mark where it is: the old
    separate limits moved that point ~0.18 m/s; one fraction keeps it under the rate."""
    node = node_with(route_file=WINDOW, wall_correction=True)
    p = (5.0, 0.0, 0.0)
    a = math.radians(5.0)
    tgt = (p[0] - math.cos(a) * p[0], -math.sin(a) * p[0], a)     # D*p == p
    cur, dt, worst = (0.0, 0.0, 0.0), 0.1, 0.0
    for _ in range(300):
        node.corr = cur
        node.position = _place(cur, p) + (0.0,)                    # odom position of the drone at p
        nxt = node.ease_correction(cur, tgt, dt)
        worst = max(worst, math.dist(_place(nxt, p), _place(cur, p)) / dt)
        assert abs(h.wrap(nxt[2] - cur[2])) <= node.corr_rate_rad * dt + 1e-12
        cur = nxt
    assert worst <= node.corr_rate_m + 1e-9
    assert all(math.isclose(c, t, abs_tol=1e-6) for c, t in zip(cur, tgt))


def test_easing_reaches_a_pure_shift_at_the_linear_rate():
    node = node_with(route_file=WINDOW, wall_correction=True)
    node.position = (1.0, 2.0, 0.0)
    cur = node.ease_correction((0.0, 0.0, 0.0), (0.5, 0.0, 0.0), 1.0)
    assert math.isclose(cur[0], node.corr_rate_m) and cur[1] == 0.0 and cur[2] == 0.0
