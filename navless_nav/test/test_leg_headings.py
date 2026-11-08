"""Per-leg headings (route_headings) in course_mission.py, on test_fly_home.py's stub ROS.

Prototype (2026-11-08, Shining's question): hold each leg's heading, turn on the spot between legs. The route
gazebo_sim/course/route_centreline_window_asbuilt_legheading.json is 19's NE-first-leg window route with
headings_deg: north to waypoint 69 (just past MD's tip), 99 deg (east, along the stretch to the tube turn) to
waypoint 106, north from there through the tube, the window and the pad.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_fly_home as h    # noqa: E402  (stubs ROS, imports course_mission)

cm = h.course_mission
LEGS = 'gazebo_sim/course/route_centreline_window_asbuilt_legheading.json'
PLAIN = 'gazebo_sim/course/route_centreline_window_asbuilt.json'
EAST = math.radians(-99.0)       # 99 deg clockwise from north, as an odom ENU yaw (nose north at power-up)


def start(route, **overrides):
    h.T[0] = 0.0
    h.Z_BIAS[0] = 0.0
    h.reset_heading()
    h.OVERRIDES.clear()
    h.OVERRIDES.update(route_file=route, cruise_speed_mps=0.5, max_lead_m=0.6, net_margin_m=0.25, **overrides)
    node = cm.CourseMission()
    px4 = {'armed': False, 'offboard_active': False, 'landed': True}
    pos = [0.0, 0.0, 0.0]
    h.fly(node, px4, pos, lambda: h.T[0] >= 2.5)
    h.arm(px4)
    return node, px4, pos


def fly(node, px4, pos, hook=None):
    """To touchdown. Per tick: (t, state, carrot index, commanded yaw or None, drone yaw, drone xy, turning)."""
    rows = []

    def record():
        if hook is not None:
            hook(node)
        if h.SENT:
            topic, m = h.SENT[-1]
            cmd = h.pose_yaw(m) if topic == cm.POSE_TOPIC else None
            idx = node.carrot.index if node.carrot is not None else -1
            rows.append((h.T[0], node.state, idx, cmd, h.YAW[0], tuple(pos[:2]), node.turn is not None))
        return not px4['armed']

    h.fly(node, px4, pos, record)
    return rows


def test_legs_are_flown_at_their_headings_with_turns_on_the_spot():
    try:
        node, px4, pos = start(LEGS)
        assert node.route_yaw is not None
        rows = fly(node, px4, pos)
        assert node.state == 'DONE'
        route = [r for r in rows if r[1] == 'ROUTE' and r[3] is not None and not r[6]]
        before = [r for r in route if r[2] < 69]
        east = [r for r in route if 69 <= r[2] < 106]
        after = [r for r in route if r[2] >= 106]
        assert before and east and after
        assert all(abs(h.wrap(r[3])) < 1e-6 for r in before)
        assert all(abs(h.wrap(r[3] - EAST)) < 1e-6 for r in east)
        assert all(abs(h.wrap(r[3])) < 1e-6 for r in after)
        # the drone really faced each way (stub PX4 lags the command by a few degrees)
        assert max(abs(h.wrap(r[4] - EAST)) for r in east) < math.radians(6)
        # each turn: stopped at its waypoint while turning, the command ramping <= 15 deg/s
        for j in (69, 106):
            turning = [r for r in rows if r[6] and r[2] >= j - 1 and r[2] <= j + 13]
            assert turning, j
            p = node.route[j]
            assert max(math.dist(r[5], p[:2]) for r in turning[-5:]) < 0.11, j
            cmds = [r[3] for r in turning if r[3] is not None]
            steps = [abs(h.wrap(b - a)) for a, b in zip(cmds, cmds[1:])]
            assert max(steps) <= math.radians(15.0) * 0.1 + 1e-9, (j, max(steps))
    finally:
        h.reset_heading()


def test_route_without_headings_holds_the_takeoff_heading():
    try:
        node, px4, pos = start(PLAIN)
        assert node.route_yaw is None
        rows = fly(node, px4, pos)
        assert node.state == 'DONE'
        cmds = [r[3] for r in rows if r[1] == 'ROUTE' and r[3] is not None]
        assert cmds and all(abs(h.wrap(c)) < 1e-6 for c in cmds)
        assert not any(r[6] for r in rows)
    finally:
        h.reset_heading()


def test_route_headings_off_ignores_them():
    try:
        node, px4, pos = start(LEGS, route_headings=False)
        assert node.route_yaw is None
        rows = fly(node, px4, pos)
        assert node.state == 'DONE'
        assert all(abs(h.wrap(r[3])) < 1e-6 for r in rows if r[1] == 'ROUTE' and r[3] is not None)
    finally:
        h.reset_heading()


def test_a_turn_that_cannot_finish_gives_up_and_goes_on():
    # Something holds the nose north (a stuck yaw): after the timeout the leg is flown at the heading the
    # nose has, and the route goes on to the pad instead of hovering at the turn point.
    try:
        node, px4, pos = start(LEGS, heading_turn_timeout_s=3.0)

        def stuck(node_):
            h.YAW[0] = 0.0

        rows = fly(node, px4, pos, hook=stuck)
        assert node.state == 'DONE'
        turns = [r for r in rows if r[6]]
        span = turns[-1][0] - turns[0][0]
        assert span < 2 * (3.0 + 2.0)          # two turns, each given up after ~3 s (plus the approach)
        assert not node.heading_suspended
    finally:
        h.reset_heading()


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
    print('ok')
