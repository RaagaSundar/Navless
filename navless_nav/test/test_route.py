"""Off-drone tests for navless_nav/route.py:  python3 -m pytest -q navless_nav/test"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from navless_nav.route import (Carrot, REPO_ROOT, course_to_odom,  # noqa: E402
                               load_route, odom_to_course, resolve_route_path)

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


def test_missing_route_is_an_error():
    with pytest.raises(FileNotFoundError):
        resolve_route_path('no_such_route.json')
