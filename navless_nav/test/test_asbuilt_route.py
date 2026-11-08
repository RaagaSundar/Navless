"""19 flies the window route planned on the as-built course map, and its launch settings let it.

The plan's route passes 3.6 cm from a real wall (Shining's phone scan, 2026-11-06), and the organisers
replaced the plan's maze on 2026-11-07. The as-built route on the new maze (MN, MD, MT) keeps >= 0.5 m
from the net, so course_mission's net clamp leaves it alone at the course launch's margin and at
navless.launch.py's 0.4 m. (On the old maze it crossed M1's and M3's north ends 0.256 m from the net,
which is why the course launch's margin is 0.25.)
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_course_launch as launch    # noqa: E402  (launch files parsed with ast)
import test_tube_blind as tb           # noqa: E402  (stub ROS, point-mass drone)
from navless_nav.route import NET_HALF_M, REPO_ROOT    # noqa: E402

ARGS = launch._arguments(launch.COURSE)
ROUTE = ARGS['route_file']
GATE_B = launch._gate_b_values()       # the backup launch: Gate B route, base speeds, turn cap


def test_course_launch_flies_the_asbuilt_route():
    r = json.load(open(os.path.join(REPO_ROOT, ROUTE)))
    assert r['course_file'] == 'gazebo_sim/course/course_asbuilt.json'
    assert r['min_clearance_m_nose_north'] >= 0.08


def test_gate_b_backup_is_planned_on_the_asbuilt_course():
    r = json.load(open(os.path.join(REPO_ROOT, GATE_B['route_file'])))
    assert r['route_type'] == 'GATE_B_CENTRELINE'
    assert r['course_file'] == 'gazebo_sim/course/course_asbuilt.json'
    assert r['min_clearance_m_nose_north'] >= 0.08


def test_net_margin_leaves_every_route_point_alone():
    assert 'net_margin_m' in launch._forwarded(launch.COURSE)
    for route in (ROUTE, GATE_B['route_file']):
        pts = json.load(open(os.path.join(REPO_ROOT, route)))['waypoints_enu']
        assert max(max(abs(p[0]), abs(p[1])) for p in pts) <= NET_HALF_M - float(ARGS['net_margin_m']), route


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
    # Neither the course launch's margin nor navless.launch.py's 0.4 m touches the new maze's route
    # (>= 0.502 m from the net); a margin wider than that gap still does, so the fence is live.
    for margin in (float(ARGS['net_margin_m']), 0.4):
        node = fly(net_margin_m=margin)
        assert node.state == 'LAND' and 'fenced' not in node.log_once, margin
    assert 'fenced' in fly(net_margin_m=0.55).log_once


def test_gate_b_backup_flies_the_whole_route_at_its_launch_settings():
    settings = {k: (v if k == 'route_file' else float(v)) for k, v in GATE_B.items()}
    node = fly(net_margin_m=float(ARGS['net_margin_m']), **settings)
    assert node.state == 'LAND' and 'fenced' not in node.log_once
