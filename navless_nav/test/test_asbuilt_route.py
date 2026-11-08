"""19 flies the window route planned on the as-built course map, and its launch settings let it.

Since the organisers' 2026-11-07 maze change (M1-M3 out; MN, MD and MT in), both of 19's launches fly routes
planned from the new walls: north-east up corridor 1 beside MD, round MD's tip through Gate A, then the tube.
The plan's route passes 3.6 cm from a real wall (Shining's phone scan, 2026-11-06).
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_course_launch as launch    # noqa: E402  (launch files parsed with ast)
import test_tube_blind as tb           # noqa: E402  (stub ROS, point-mass drone)
from navless_nav.route import NET_HALF_M, REPO_ROOT, odom_to_course    # noqa: E402

ARGS = launch._arguments(launch.COURSE)
ROUTE = ARGS['route_file']
GATE_B = launch._gate_b_values()       # the backup launch: Gate B route, base speeds, turn cap
PROP_REACH_SIDE, PROP_REACH_BACK = 0.2375, 0.2275    # prop tips from the body centre, m (nose north)
BARRIER_1_NORTH_FACE_Y = 0.508                       # course frame, m (138 in + 2 in half thickness)


def _route(path):
    return json.load(open(os.path.join(REPO_ROOT, path)))


def test_course_launch_flies_the_asbuilt_route():
    r = _route(ROUTE)
    assert r['course_file'] == 'gazebo_sim/course/course_asbuilt.json'
    assert r['maze'].startswith('2026-11-07'), ROUTE       # planned from the new maze's walls
    assert r['min_clearance_m_nose_north'] >= 0.08


def test_gate_b_backup_is_planned_on_the_asbuilt_course():
    r = _route(GATE_B['route_file'])
    assert r['route_type'] == 'GATE_B_CENTRELINE'
    assert r['course_file'] == 'gazebo_sim/course/course_asbuilt.json'
    assert r['maze'].startswith('2026-11-07'), GATE_B['route_file']
    assert r['min_clearance_m_nose_north'] >= 0.08


def test_both_routes_share_the_maze_part():
    # The Gate B backup reuses the window route's head as far as the turn into the tube (dh/route.py).
    win, gb = _route(ROUTE), _route(GATE_B['route_file'])
    assert (win['tip_radius_m'], win['tube_turn_y_in']) == (gb['tip_radius_m'], gb['tube_turn_y_in'])

    def off(p, path):          # horizontal distance from p to a polyline
        best = float('inf')
        for a, b in zip(path, path[1:]):
            ab = (b[0] - a[0], b[1] - a[1])
            t = ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / max(ab[0] ** 2 + ab[1] ** 2, 1e-12)
            t = min(1.0, max(0.0, t))
            best = min(best, math.hypot(p[0] - a[0] - t * ab[0], p[1] - a[1] - t * ab[1]))
        return best

    pts = win['waypoints_enu']
    n = next(i for i, p in enumerate(pts) if p[0] > 1.5 and p[1] < 1.6)   # nearing the tube's mouth (y 1.219)
    # (to within 2 mm: the 5 cm samples fall differently, and the turn into the tube rounds off towards each
    # route's next corner, 1.2 deg apart)
    assert max(off(p, gb['waypoints_enu'][:n + 5]) for p in pts[:n]) <= 0.002


def test_net_margin_leaves_every_route_point_alone():
    # The clamp acts only on setpoints past NET_HALF_M - net_margin_m. The routes peak 2.633 m out (the arc
    # over MD's tip), so the clamp cannot pull a setpoint off the route, into a wall or not.
    assert 'net_margin_m' in launch._forwarded(launch.COURSE)
    for route in (ROUTE, GATE_B['route_file']):
        pts = _route(route)['waypoints_enu']
        top = max(max(abs(p[0]), abs(p[1])) for p in pts)
        assert top <= NET_HALF_M - float(ARGS['net_margin_m']) - 0.15, (route, top)


def fly(**params):
    """The whole route on the stub drone at the slow launch's speeds; the node after its route."""
    h, cm = tb.h, tb.cm
    h.T[0] = 0.0
    h.Z_BIAS[0] = 0.0
    h.OVERRIDES.clear()
    h.OVERRIDES.update({'route_file': ROUTE, 'cruise_speed_mps': 0.5, 'max_lead_m': 0.6,
                        'max_lat_accel_mps2': 0.25}, **params)
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
    # (Before the 2026-11-07 maze this also checked that navless.launch.py's 0.4 m would have fenced the old
    # route at M1's and M3's north ends. The new routes stay inside both margins.)
    node = fly(net_margin_m=float(ARGS['net_margin_m']))
    assert node.state == 'LAND' and 'fenced' not in node.log_once


def test_gate_b_backup_flies_the_whole_route_at_its_launch_settings():
    settings = {k: (v if k == 'route_file' else float(v)) for k, v in GATE_B.items()}
    node = fly(net_margin_m=float(ARGS['net_margin_m']), **settings)
    assert node.state == 'LAND' and 'fenced' not in node.log_once


def test_takeoff_leans_away_from_the_west_net():
    # course_mission climbs to a point takeoff_lead_m along the route's first leg. On the launch mark the prop
    # tips are 29.6 cm from the west net, and 19's liftoffs slid 25-33 cm west and 10-16 cm back (2026-11-07
    # test flights) before recovering. The route's first leg points north-east, so the climb target leaves
    # >= 10 cm to the west net and to Barrier 1 even after the worst of that slide.
    h, cm = tb.h, tb.cm
    h.T[0] = 0.0
    h.Z_BIAS[0] = 0.0
    h.OVERRIDES.clear()
    h.OVERRIDES.update({'route_file': ROUTE, 'cruise_speed_mps': 0.5, 'max_lead_m': 0.6,
                        'max_lat_accel_mps2': 0.25, 'net_margin_m': float(ARGS['net_margin_m'])})
    node = cm.CourseMission()
    px4 = {'armed': False, 'offboard_active': False, 'landed': True}
    pos = [0.0, 0.0, 0.0]
    for _ in range(30):
        tb.step(node, px4, pos)
    px4.update(armed=True, offboard_active=True)
    while node.state != 'TAKEOFF':
        tb.step(node, px4, pos)
    x, y, _ = odom_to_course(node.target, node.frame[0], node.frame[1])
    west_room = x + NET_HALF_M - PROP_REACH_SIDE
    barrier_room = y - PROP_REACH_BACK - BARRIER_1_NORTH_FACE_Y
    assert x - node.frame[0][0] >= 0.15                     # leaned east, away from the west net
    assert west_room - 0.33 >= 0.10, west_room
    assert barrier_room - 0.16 >= 0.10, barrier_room
