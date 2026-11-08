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
from navless_nav.route import NET_HALF_M, REPO_ROOT    # noqa: E402

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
