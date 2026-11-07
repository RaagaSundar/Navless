"""Off-drone tests for navless_nav/route.py:  python3 -m pytest -q navless_nav/test"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from navless_nav.route import (Carrot, NET_HALF_M, REPO_ROOT,  # noqa: E402
                               clamp_to_net, course_to_odom, load_route,
                               odom_to_course, resolve_route_path,
                               speed_profile, turn_radii)

LAUNCH = (-2.515, 0.914)


def close(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def test_launch_mark_is_the_odom_origin():
    for bearing in (0.0, 37.0, 90.0, 180.0, 270.0):
        assert close(course_to_odom((*LAUNCH, 0.0), LAUNCH, bearing), (0, 0, 0))


def test_nose_north_maps_north_to_forward_and_west_to_left():
    north = (LAUNCH[0], LAUNCH[1] + 1.0, 0.85)
    west = (LAUNCH[0] - 1.0, LAUNCH[1], 0.85)
    assert close(course_to_odom(north, LAUNCH, 0.0), (1.0, 0.0, 0.85))
    assert close(course_to_odom(west, LAUNCH, 0.0), (0.0, 1.0, 0.85))


def test_nose_east_maps_east_to_forward_and_north_to_left():
    east = (LAUNCH[0] + 1.0, LAUNCH[1], 0.85)
    north = (LAUNCH[0], LAUNCH[1] + 1.0, 0.85)
    assert close(course_to_odom(east, LAUNCH, 90.0), (1.0, 0.0, 0.85))
    assert close(course_to_odom(north, LAUNCH, 90.0), (0.0, 1.0, 0.85))


def test_start_height_shifts_z_only():
    p = (0.3, -1.2, 0.85)
    a = course_to_odom(p, LAUNCH, 20.0, 0.0)
    b = course_to_odom(p, LAUNCH, 20.0, 0.1)
    assert close(a[:2], b[:2]) and abs((a[2] - b[2]) - 0.1) < 1e-12


def test_round_trip():
    for p in ((0.9, 2.585, 0.85), (-0.471, -2.272, 0.762), (2.31, 1.0, 0.8)):
        for bearing in (0.0, 15.0, 90.0, 200.0):
            q = course_to_odom(p, LAUNCH, bearing, 0.05)
            assert close(odom_to_course(q, LAUNCH, bearing, 0.05), p, 1e-9)


def test_carrot_moves_at_speed_and_ends():
    path = [(0, 0, 1), (1, 0, 1), (1, 1, 1)]
    c = Carrot(path, speed=0.5, max_lead=10.0)
    p = c.step((0, 0, 1), 1.0)
    assert close(p, (0.5, 0, 1))
    p = c.step(p, 1.0)                       # exactly reaches the corner
    assert close(p, (1, 0, 1))
    p = c.step(p, 1.0)                       # turns it
    assert close(p, (1, 0.5, 1))
    for _ in range(10):
        p = c.step(p, 1.0)
    assert close(p, (1, 1, 1)) and c.at_end
    assert c.done((1, 1.1, 1), 0.25) and not c.done((1, 0.5, 1), 0.25)


def test_carrot_waits_for_a_lagging_drone():
    c = Carrot([(0, 0, 1), (5, 0, 1)], speed=1.0, max_lead=0.6)
    drone = (0, 0, 1)
    for _ in range(20):
        p = c.step(drone, 0.1)
    assert math.dist(drone, p) <= 0.6 + 0.1 + 1e-9
    held = c.step(drone, 0.1)
    assert close(held, c.step(drone, 0.1))   # no further while we lag


def _arc(radius, n=40):
    """Straight 2 m east, a quarter circle of `radius` turning north, 2 m north."""
    pts = [(x * 0.1, 0.0, 1.0) for x in range(20)]
    for k in range(n + 1):
        a = -math.pi / 2 + (math.pi / 2) * k / n
        pts.append((2.0 + radius * math.cos(a), radius + radius * math.sin(a), 1.0))
    pts += [(2.0 + radius, radius + 0.1 * y, 1.0) for y in range(1, 21)]
    return pts


def test_carrot_advance_slides_along_the_path_and_stops_at_its_end():
    c = Carrot([(0, 0, 0), (1, 0, 0), (1, 1, 0)], speed=0.1, max_lead=0.01)
    assert close(c.advance(1.5), (1.0, 0.5, 0.0)) and c.index == 1
    assert close(c.advance(10.0), (1.0, 1.0, 0.0)) and c.at_end
    assert close(c.advance(-1.0), (1.0, 1.0, 0.0))


def test_carrot_catch_up_moves_forward_to_the_drone_only():
    path = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (2, 2, 0)]
    c = Carrot(path, speed=0.1, max_lead=0.6)
    c.advance(0.5)
    assert abs(c.catch_up((1.4, 0.1, 0.0)) - 0.9) < 1e-9          # ahead: moves up
    assert close(c.point, (1.4, 0.0, 0.0))
    assert c.catch_up((0.2, 0.0, 0.0)) == 0.0                      # behind: never back
    assert close(c.point, (1.4, 0.0, 0.0))
    assert c.catch_up((2.6, 1.0, 0.0)) == 0.0                      # 0.6 m off the path
    assert abs(c.catch_up((2.1, 1.0, 0.0)) - 1.6) < 1e-9          # round the corner
    assert close(c.point, (2.0, 1.0, 0.0)) and c.index == 2


def _l_turn(r=0.5, ds=0.05):
    """1.5 m east, a quarter circle of radius r turning north, 1.5 m north."""
    pts = [(x * ds, 0.0, 1.0) for x in range(int(1.5 / ds) + 1)]
    cx, cy = 1.5, r
    n = int(math.pi / 2 * r / ds)
    for k in range(1, n + 1):
        th = -math.pi / 2 + (math.pi / 2) * k / n
        pts.append((cx + r * math.cos(th), cy + r * math.sin(th), 1.0))
    x0, y0 = pts[-1][0], pts[-1][1]
    pts += [(x0, y0 + j * ds, 1.0) for j in range(1, int(1.5 / ds) + 1)]
    return pts


def _dist_to_path(q, path):
    best = math.inf
    for a, b in zip(path, path[1:]):
        ab = [y - x for x, y in zip(a, b)]
        t = sum((qq - x) * d for qq, x, d in zip(q, a, ab)) / max(sum(d * d for d in ab), 1e-12)
        t = max(0.0, min(1.0, t))
        best = min(best, math.dist(q, [x + d * t for x, d in zip(a, ab)]))
    return best


def test_carrot_lead_shrinks_in_tight_turns_only():
    path = _l_turn(0.5)
    c = Carrot(path, speed=0.5, max_lead=0.6, max_sagitta=0.02)
    assert c.lead_limit() == 0.6                                  # straight
    c.advance(1.5 + math.pi / 4 * 0.5)                            # mid-arc
    assert abs(c.lead_limit() - 0.15) < 1e-9          # sqrt(2 R s - s^2) = 0.14 -> floor
    c.advance(1.5)                                                # straight again
    assert c.lead_limit() == 0.6
    assert Carrot(path, 0.5, 0.6).lead_limit() == 0.6            # off by default


def test_sagitta_cap_reduces_the_cut_inside_a_turn():
    # Drone ~ PX4 position control: velocity = 0.95 x (carrot - drone), <= 1 m/s.
    path = _l_turn(0.5)

    def worst_cut(sagitta):
        c = Carrot(path, speed=0.5, max_lead=0.6, max_sagitta=sagitta)
        drone, worst = list(path[0]), 0.0
        for _ in range(3000):
            goal = c.step(drone, 0.05)
            v = [0.95 * (g - d) for g, d in zip(goal, drone)]
            n = math.hypot(*v)
            if n > 1.0:
                v = [x / n for x in v]
            drone = [d + x * 0.05 for d, x in zip(drone, v)]
            worst = max(worst, _dist_to_path(drone, path))
            if c.at_end and math.dist(drone, c.point) < 0.05:
                break
        return worst
    off, on = worst_cut(0.0), worst_cut(0.02)
    assert off > 0.06 and on < 0.04 and on < off / 2, (off, on)


def test_sagitta_cap_is_horizontal_so_a_height_offset_cannot_stall_it():
    # Review of #20: PX4 can hold height up to ~0.2 m off odom; with a 3D
    # check, a 0.18 m offset exceeds a ~0.16 m cap and the carrot never moves.
    path = _l_turn(0.32)
    c = Carrot(path, speed=0.5, max_lead=0.6, max_sagitta=0.04)
    for _ in range(4000):
        p = c.point
        drone = (p[0], p[1], p[2] - 0.18)        # on the path, 0.18 m low
        c.step(drone, 0.05)
        if c.at_end:
            break
    assert c.at_end


def test_turn_radius_is_measured():
    path = _arc(0.5)
    r = turn_radii(path)
    assert math.isinf(r[5]) or r[5] > 50          # straight
    assert abs(r[40] - 0.5) < 0.02                # middle of the arc


def test_speed_profile_slows_through_bends_and_before_them():
    path = _arc(0.5)
    v = speed_profile(path, cruise=0.5, max_lat_accel=0.25, max_accel=0.5)
    assert abs(v[0] - 0.5) < 1e-9 and abs(v[-1] - 0.5) < 1e-9     # straights at cruise
    assert abs(v[40] - math.sqrt(0.25 * 0.5)) < 0.01                 # 0.354 m/s in the bend
    assert v[20] < 0.5                                                # braking from the arc entry
    assert all(a >= b - 1e-12 for a, b in zip(v[15:40], v[16:41]))    # and only slowing into it
    assert speed_profile(path, 0.5, 0.0) == [0.5] * len(path)         # off


def test_carrot_takes_longer_through_a_bend_when_limited():
    def time_to_end(lat):
        c = Carrot(_arc(0.5), speed=0.5, max_lead=10.0, max_lat_accel=lat)
        t = 0.0
        while not c.at_end and t < 100:
            c.step(c.point, 0.1)
            t += 0.1
        return t
    free, limited = time_to_end(0.0), time_to_end(0.25)
    assert limited > free + 0.3


def test_shipped_routes_load_and_start_on_the_launch_mark():
    names = [n for n in ('route_gate_b.json', 'route_window.json', 'route.json')
             if os.path.isfile(os.path.join(REPO_ROOT, n))]
    if not names:
        pytest.skip('no route files next to the repo')
    for name in names:
        waypoints, launch, _ = load_route(resolve_route_path(name))
        odom = [course_to_odom(p, launch, 0.0) for p in waypoints]
        assert math.dist(odom[0][:2], (0, 0)) < 0.05, name
        steps = [math.dist(a, b) for a, b in zip(odom, odom[1:])]
        assert max(steps) < 1.5, (name, max(steps))   # mission.py's check too
        assert all(p[2] > 0.3 for p in odom), name   # above flight_interface's floor


def test_net_fence_leaves_inside_points_alone():
    q = course_to_odom((0.9, 2.585, 0.85), LAUNCH, 30.0)   # Gate A exit
    out, moved = clamp_to_net(q, LAUNCH, 30.0, 0.4)
    assert not moved and out == q


def test_net_fence_pulls_outside_points_in_and_keeps_z():
    lim = NET_HALF_M - 0.4
    for bearing in (0.0, 90.0, 225.0):
        for course in ((5.0, 0.0, 1.0), (-4.0, 4.0, 1.7), (0.0, -9.0, 0.5)):
            q = course_to_odom(course, LAUNCH, bearing)
            out, moved = clamp_to_net(q, LAUNCH, bearing, 0.4)
            c = odom_to_course(out, LAUNCH, bearing)
            assert moved
            assert abs(c[0]) <= lim + 1e-9 and abs(c[1]) <= lim + 1e-9
            assert abs(out[2] - course[2]) < 1e-12


def test_shipped_routes_clear_the_net_fence():
    for name in ('route_gate_b.json', 'route_window.json', 'route.json'):
        if not os.path.isfile(os.path.join(REPO_ROOT, name)):
            continue
        waypoints, launch, _ = load_route(resolve_route_path(name))
        for p in waypoints:
            q = course_to_odom(p, launch, 0.0)
            assert not clamp_to_net(q, launch, 0.0, 0.4)[1], (name, p)


def test_missing_route_is_an_error():
    with pytest.raises(FileNotFoundError):
        resolve_route_path('no_such_route.json')
