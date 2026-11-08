"""Heading hold, the heading guard and the takeoff settle in course_mission.py,
on test_fly_home.py's stub ROS (a point-mass drone with a heading).

Why, from skybeta-19's test flights on 2026-11-07 (navless_flight_tests logs):
- course_mission sent a zero quaternion, which flight_interface turns into yaw
  NaN, and PX4's PositionControl then sets its yaw setpoint to the current
  heading every cycle: nothing turns the nose back. With a commanded heading
  19 held within 1.4 deg; without one (the twist descents) it wandered 6 deg
  in 10 s.
- After 0.4 s without vision, PX4 reset its heading to the magnetometer, ~100
  deg off (four times on the floor after the yaw test). A held heading must
  then be let go, or PX4 spins the drone to it.
- Both liftoffs slid 25-33 cm sideways and took 10-15 s to come back.
"""
import math
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_fly_home as h    # noqa: E402  (stubs ROS, imports course_mission)

cm = h.course_mission
DRIFT_19 = math.radians(6.0) / 10.0     # 19's unheld descents: 6 deg in 10 s
FLYING = ('TAKEOFF', 'ROUTE', 'HOLD', 'LAND')


def start(yaw0=0.0, drift=0.0, hook=None, offset=None, **overrides):
    """A node on the launch mark with heading yaw0, past the pre-arm settle,
    then armed (the next tick takes off)."""
    h.T[0] = 0.0
    h.Z_BIAS[0] = 0.0
    h.reset_heading()
    h.YAW[0] = yaw0
    h.YAW_DRIFT[0] = drift
    h.PX4_HOOK[0] = hook
    h.ODOM_OFFSET[0] = offset
    h.OVERRIDES.clear()
    h.OVERRIDES.update(route_file='route_gate_b.json', cruise_speed_mps=1.0, heading_mode='hold', **overrides)
    node = cm.CourseMission()
    px4 = {'armed': False, 'offboard_active': False, 'landed': True}
    pos = [0.0, 0.0, 0.0]
    h.fly(node, px4, pos, lambda: h.T[0] >= 2.5)
    h.arm(px4)
    return node, px4, pos


def watch(node, px4, pos, until, max_s=600.0):
    """Fly until `until()`. Per tick: (state, drone yaw, commanded yaw or None
    (pose setpoints), twist yaw rate or None)."""
    rows = []

    def record():
        if h.SENT:
            topic, m = h.SENT[-1]
            if topic == cm.POSE_TOPIC:
                rows.append((node.state, h.YAW[0], h.pose_yaw(m), None))
            else:
                rows.append((node.state, h.YAW[0], None, m.twist.angular.z))
        return until()

    h.fly(node, px4, pos, record, max_s=max_s)
    return rows


def tick(node, px4, pos):
    h.fly(node, px4, pos, lambda: True, max_s=1.0)
    return list(h.SENT)


class Px4Heading:
    """PX4's heading for PX4_HOOK: the drone's, plus `off` (rad), with a reset counter."""

    def __init__(self):
        self.off, self.resets = 0.0, 3

    def __call__(self, node):
        node.px4_local_cb(types.SimpleNamespace(
            heading=math.pi / 2.0 - (h.YAW[0] + self.off),
            heading_reset_counter=self.resets, heading_good_for_control=True))


def push_nose_until_suspended(node, px4, pos, rate=math.radians(140.0)):
    """A yaw disturbance PX4 cannot turn back (it turns at 1.5 rad/s here) until the guard lets
    go, at a rate odometry can really show (under odom_max_angular_rate_rps); then it stops."""
    h.YAW_DRIFT[0] = rate
    h.fly(node, px4, pos, lambda: node.heading_suspended, max_s=2.0)
    h.YAW_DRIFT[0] = 0.0


def test_every_flight_setpoint_carries_the_takeoff_heading():
    try:
        node, px4, pos = start(yaw0=math.radians(5.0))
        rows = watch(node, px4, pos, lambda: not px4['armed'])
        assert node.state == 'DONE'
        poses = [r for r in rows if r[0] in FLYING and r[3] is None]
        assert len(poses) > 100
        assert all(r[2] is not None and abs(h.wrap(r[2] - math.radians(5.0))) < 1e-9
                   for r in poses)
        assert any(r[3] is not None for r in rows)          # it landed on twists
    finally:
        h.reset_heading()


def test_a_steady_yaw_disturbance_is_turned_back():
    # 19's measured wander as a steady push: held, the nose stays within a
    # degree; as course_mission flew it before (no heading), it walks off.
    try:
        worst = {}
        for hold in (True, False):
            node, px4, pos = start(drift=DRIFT_19, hold_heading=hold)
            rows = watch(node, px4, pos, lambda: not px4['armed'])
            assert node.state == 'DONE'
            worst[hold] = max(abs(h.wrap(r[1])) for r in rows if r[0] in FLYING)
        print(f'worst heading error: held {math.degrees(worst[True]):.2f} deg, '
              f'unheld {math.degrees(worst[False]):.1f} deg')
        assert worst[True] < math.radians(1.0)
        assert worst[False] > math.radians(10.0)
    finally:
        h.reset_heading()


def test_landing_turns_the_nose_back_with_yaw_rate():
    try:
        node, px4, pos = start()
        h.fly(node, px4, pos, lambda: node.state == 'LAND')
        h.YAW[0] = math.radians(10.0)
        sent = tick(node, px4, pos)
        assert sent[-1][0] == cm.TWIST_TOPIC
        assert abs(sent[-1][1].twist.angular.z + math.radians(10.0) * 1.0) < 1e-6
        push_nose_until_suspended(node, px4, pos)     # past the guard: let go, no yaw rate
        sent = tick(node, px4, pos)
        assert node.state == 'LAND'
        assert node.heading_suspended and sent[-1][1].twist.angular.z == 0.0
    finally:
        h.reset_heading()


def test_px4_heading_reset_in_flight_latches_fault_and_never_resumes():
    # A reset turns PX4's local frame under every odom setpoint flight_interface converts.
    # Before: let go of the heading, then resume once PX4 agreed again. Now: FAULT, for good.
    try:
        px4_heading = Px4Heading()
        node, px4, pos = start(hook=px4_heading)
        h.fly(node, px4, pos, lambda: node.state == 'ROUTE')
        assert node.px4_yaw_ref is not None and abs(node.px4_yaw_ref) < 1e-9
        t = h.T[0]
        h.fly(node, px4, pos, lambda: h.T[0] >= t + 2.0)
        # Vision gap: PX4 jumps to the magnetometer, 100 deg off, counter +1.
        px4_heading.off, px4_heading.resets = math.radians(100.0), 4
        h.SENT.clear()
        px4_heading(node)
        assert node.state == 'FAULT' and 'PX4 reset its heading' in node.navigation_fault
        assert h.SENT == []
        # Vision back and PX4 agreeing again: still nothing commanded.
        px4_heading.off, px4_heading.resets = 0.0, 5
        t = h.T[0]
        rows = watch(node, px4, pos, lambda: h.T[0] >= t + 3.0)
        assert rows == [] and node.state == 'FAULT'
        assert h.STATUS[-1]['fault_latched'] and h.STATUS[-1]['state'] == 'FAULT'
    finally:
        h.reset_heading()


def test_px4_heading_reset_on_the_ground_is_not_a_fault():
    try:
        px4_heading = Px4Heading()
        node, px4, pos = start(hook=px4_heading)
        px4.update(armed=False, offboard_active=False)   # disarmed before the takeoff tick
        px4_heading.resets = 4
        h.fly(node, px4, pos, lambda: h.T[0] >= 4.0)
        assert node.navigation_fault is None and node.state == 'WAIT_FOR_OFFBOARD'
    finally:
        h.reset_heading()


def test_px4_heading_disagreeing_without_a_reset_also_lets_go():
    try:
        px4_heading = Px4Heading()
        node, px4, pos = start(hook=px4_heading)
        h.fly(node, px4, pos, lambda: node.state == 'ROUTE')
        px4_heading.off = math.radians(20.0)      # counter unchanged
        sent = tick(node, px4, pos)
        assert node.heading_suspended and h.pose_yaw(sent[-1][1]) is None
        px4_heading.off = math.radians(12.0)      # still past a third of the guard
        t = h.T[0]
        h.fly(node, px4, pos, lambda: h.T[0] >= t + 2.0)
        assert node.heading_suspended
    finally:
        h.reset_heading()


def test_nose_pushed_far_off_lets_go_with_no_px4_heading():
    try:
        # PX4's heading only on the ground (the pre-arm check needs it), none in flight
        node, px4, pos = start(hook=lambda n: None if n.armed else h.px4_heading(n))
        h.fly(node, px4, pos, lambda: node.state == 'ROUTE')
        push_nose_until_suspended(node, px4, pos)
        pushed = h.YAW[0]
        assert abs(h.wrap(pushed)) > node.heading_guard
        sent = tick(node, px4, pos)
        assert node.heading_suspended and h.pose_yaw(sent[-1][1]) is None
        t = h.T[0]
        h.fly(node, px4, pos, lambda: not node.heading_suspended, max_s=3.0)
        assert h.T[0] - t <= 1.3 and node.state == 'ROUTE'
        assert abs(h.wrap(node.hold_yaw - pushed)) < math.radians(0.5)    # where the nose is
    finally:
        h.reset_heading()


def test_prearm_waits_while_px4_heading_disagrees():
    try:
        px4_heading = Px4Heading()
        px4_heading.off = math.radians(100.0)     # PX4 on the magnetometer
        h.T[0] = 0.0
        h.Z_BIAS[0] = 0.0
        h.reset_heading()
        h.PX4_HOOK[0] = px4_heading
        h.OVERRIDES.clear()
        h.OVERRIDES.update(route_file='route_gate_b.json', cruise_speed_mps=1.0, heading_mode='hold')
        node = cm.CourseMission()
        px4 = {'armed': False, 'offboard_active': False, 'landed': True}
        pos = [0.0, 0.0, 0.0]
        streamed = []
        for _ in range(40):
            streamed += tick(node, px4, pos)
        assert streamed == []                     # no stream: no takeover
        px4_heading.off = math.radians(2.0)
        for _ in range(5):
            streamed += tick(node, px4, pos)
        assert streamed and all(t == cm.POSE_TOPIC for t, _ in streamed)
        # Offboard anyway while they disagree: refuse, descend in place.
        px4_heading.off = math.radians(100.0)
        h.arm(px4)
        tick(node, px4, pos)
        assert node.state == 'REFUSED'
    finally:
        h.reset_heading()


def slide(t0, size=0.30, tau=4.0, rise=0.3):
    """19's liftoff slide: 30 cm sideways within `rise` s of liftoff (1 m/s: odometry
    stepping 30 cm in one sample would be the jump FAULT), PX4 taking it back with a 4 s
    time constant (both 2026-11-07 flights: 25-33 cm, 10-15 s)."""
    return lambda t: ((0.0, size * min(1.0, (t - t0) / rise) * math.exp(-(t - t0) / tau), 0.0)
                      if t >= t0 else (0.0, 0.0, 0.0))


def test_route_waits_for_the_liftoff_slide_to_die_out():
    # start() arms at t = 2.6 s; the takeoff target is set on that tick, and
    # the slide starts at liftoff, the tick after.
    try:
        node, px4, pos = start(offset=slide(2.7), drift_correction=False)
        t = h.T[0]
        h.fly(node, px4, pos, lambda: node.state == 'ROUTE')
        waited = h.T[0] - t
        # 30 cm * exp(-dt/4) < 5 cm after 7.2 s, then 2 s held
        assert 9.0 <= waited <= 10.5, waited
        h.fly(node, px4, pos, lambda: not px4['armed'])
        assert node.state == 'DONE'
        # Before: the route started as soon as it was within 25 cm.
        node, px4, pos = start(offset=slide(2.7), takeoff_settle_s=0.0, drift_correction=False)
        t = h.T[0]
        h.fly(node, px4, pos, lambda: node.state == 'ROUTE')
        assert h.T[0] - t < 2.0
    finally:
        h.reset_heading()


def test_never_settling_holds_and_never_starts_the_route():
    # Before: the route started anyway at the timeout. An unsettled takeoff (the first
    # course crash slid 0.41 m left) is no place to start a 6-14 cm-margin route from.
    try:
        node, px4, pos = start(offset=lambda t: (0.0, 0.08, 0.0) if t >= 2.7 else (0.0, 0.0, 0.0),
                               takeoff_timeout_s=6.0, drift_correction=False)
        t = h.T[0]
        h.fly(node, px4, pos, lambda: node.state not in ('WAIT_FOR_OFFBOARD', 'TAKEOFF'))
        assert node.state == 'HOLD' and 6.0 <= h.T[0] - t <= 6.5, (node.state, h.T[0] - t)
        h.fly(node, px4, pos, lambda: h.T[0] - t >= 20.0)
        assert node.state == 'HOLD' and node.carrot is None
    finally:
        h.reset_heading()


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
    print('ok')
