"""takeoff_lead_m in course_mission.py, on test_fly_home.py's stub ROS.

The climb leans into the route's first leg (north up the first corridor,
away from Barrier 1 behind the launch mark), and the route then starts from
that point instead of pulling the drone back to the launch mark.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_fly_home as h    # noqa: E402  (stubs ROS, imports course_mission)


def climb(lead):
    h.T[0] = 0.0
    h.Z_BIAS[0] = 0.0
    h.OVERRIDES.clear()
    h.OVERRIDES.update(route_file='route_gate_b.json', cruise_speed_mps=1.0,
                       takeoff_lead_m=lead)
    node = h.course_mission.CourseMission()
    px4 = {'armed': False, 'offboard_active': False, 'landed': True}
    pos = [0.0, 0.0, 0.0]
    h.fly(node, px4, pos, lambda: h.T[0] >= 2.5)    # past the pre-arm settle
    h.arm(px4)
    h.fly(node, px4, pos, lambda: node.state == 'TAKEOFF')
    return node, px4, pos


def test_default_climbs_straight_up():
    node, _, _ = climb(0.0)
    assert math.dist(node.target[:2], (0.0, 0.0)) < 1e-9
    assert node.target[2] == node.route[0][2]


def test_lead_leans_the_climb_up_the_first_corridor():
    node, _, _ = climb(0.2)
    # nose north on the launch mark: odom +x is north, up the first corridor
    # (this file's first leg heads ~7 deg east of north)
    assert abs(math.hypot(*node.target[:2]) - 0.2) < 0.005
    assert node.target[0] > 0.19
    assert node.target[2] == node.route[0][2]


def test_route_starts_from_the_lead_point_and_still_lands():
    node, px4, pos = climb(0.2)
    h.fly(node, px4, pos, lambda: node.state == 'ROUTE')
    assert node.carrot.point[0] >= 0.2 - 0.01        # never pulled back south
    h.fly(node, px4, pos, lambda: not px4['armed'])
    assert node.state == 'DONE'
    assert math.dist(pos[:2], node.route[-1][:2]) < 0.1 and pos[2] == 0.0


def test_fly_home_takeoff_leans_toward_the_launch_mark():
    node, px4, pad = h.run(fly_home=True, takeoff_lead_m=0.2)
    assert node.state == 'WAIT_FOR_OFFBOARD' and node.going_home
    t0 = h.T[0]
    h.fly(node, px4, pad, lambda: h.T[0] - t0 >= 2.5, max_s=5.0)   # pre-arm settle
    pos = list(pad)
    h.arm(px4)
    h.fly(node, px4, pos, lambda: node.state == 'TAKEOFF')
    lean = (node.target[0] - pad[0], node.target[1] - pad[1])
    home = (-pad[0], -pad[1])                   # the launch mark is the odom origin
    along = (lean[0] * home[0] + lean[1] * home[1]) / math.hypot(*home)
    assert abs(math.hypot(*lean) - 0.2) < 0.01 and along > 0.199
    assert abs(node.target[2] - node.home_z) < 1e-9
    h.fly(node, px4, pos, lambda: not px4['armed'])
    assert node.state == 'DONE'
    assert math.dist(pos[:2], (0.0, 0.0)) < 0.1 and pos[2] == 0.0


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
    print('ok')
