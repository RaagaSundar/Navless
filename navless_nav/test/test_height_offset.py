"""PX4's height offset on the floor (match_px4_height), on test_fly_home.py's stub ROS.

skybeta-19, 2026-11-07: on the floor PX4's height read 0.185-0.198 m above
odom in all three navless_flight_tests logs. The old 0.15 m pre-arm limit
would never have let course_mission take over. Allowing it without more
would fly every height that far low: flight_interface passes odom z to PX4
unchanged, and PX4 holds its own z at it. So the offset is measured on the
floor and added to every height sent, and the limit is 0.30 m, which still
refuses the ~0.6 m boot error the check exists for.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_fly_home as h    # noqa: E402  (stubs ROS, imports course_mission)

cm = h.course_mission
BIAS_19 = 0.19


def start(bias, **overrides):
    """On the launch mark with PX4 reading `bias` above odom, past the pre-arm
    settle, then armed (Offboard comes with the next tick)."""
    h.T[0] = 0.0
    h.reset_heading()
    h.Z_BIAS[0] = bias
    h.PX4_OWN_Z[0] = True
    h.OVERRIDES.clear()
    h.OVERRIDES.update(route_file='route_gate_b.json', cruise_speed_mps=1.0, **overrides)
    node = cm.CourseMission()
    px4 = {'armed': False, 'offboard_active': False, 'landed': True}
    pos = [0.0, 0.0, 0.0]
    h.fly(node, px4, pos, lambda: h.T[0] >= 2.5)
    h.arm(px4)
    return node, px4, pos


def route_heights(node, px4, pos):
    """Fly to touchdown. Per ROUTE tick: (true z, the carrot's z)."""
    rows = []

    def record():
        if node.state == 'ROUTE' and node.carrot is not None:
            rows.append((pos[2], node.carrot.point[2]))
        return not px4['armed']

    h.fly(node, px4, pos, record)
    return rows


def finish():
    h.reset_heading()
    h.Z_BIAS[0] = 0.0


def test_19s_offset_is_flown_out():
    try:
        node, px4, pos = start(BIAS_19)
        rows = route_heights(node, px4, pos)
        assert node.state == 'DONE'
        assert abs(node.z_offset - BIAS_19) < 1e-9
        # steady flight (the carrot moves 0.1 m a tick at 1 m/s, so compare
        # where it is not climbing or descending)
        level = [(z, c) for z, c in rows[20:]]
        worst = max(abs(z - c) for z, c in level)
        print(f'worst height error with the offset matched: {worst * 100:.1f} cm')
        assert worst < 0.03
    finally:
        finish()


def test_without_matching_it_flies_that_far_low():
    try:
        node, px4, pos = start(BIAS_19, match_px4_height=False)
        rows = route_heights(node, px4, pos)
        assert node.state == 'DONE'
        lows = [c - z for z, c in rows[20:]]
        mean_low = sum(lows) / len(lows)
        print(f'mean height error unmatched: {mean_low * 100:.1f} cm low')
        assert abs(mean_low - BIAS_19) < 0.02
    finally:
        finish()


def test_old_limit_never_takes_over_on_19():
    # What tonight's code would have done: no stream, so Offboard is refused.
    try:
        node, px4, pos = start(BIAS_19, prearm_height_tol_m=0.15)
        h.fly(node, px4, pos, lambda: True, max_s=1.0)
        assert node.state == 'REFUSED'
    finally:
        finish()


def test_a_bad_px4_boot_height_is_still_refused():
    try:
        node, px4, pos = start(0.6)
        h.fly(node, px4, pos, lambda: True, max_s=1.0)
        assert node.state == 'REFUSED'
    finally:
        finish()


def test_the_offset_is_the_median_of_the_floor_samples():
    # One wild sample in the last 2 s must not set it.
    try:
        h.T[0] = 0.0
        h.reset_heading()
        h.PX4_OWN_Z[0] = True
        h.OVERRIDES.clear()
        h.OVERRIDES.update(route_file='route_gate_b.json', cruise_speed_mps=1.0)
        node = cm.CourseMission()
        px4 = {'armed': False, 'offboard_active': False, 'landed': True}
        pos = [0.0, 0.0, 0.0]
        for i in range(30):
            h.Z_BIAS[0] = 0.27 if i == 25 else (0.18, 0.19, 0.20)[i % 3]
            h.fly(node, px4, pos, lambda: True, max_s=1.0)
        assert abs(node.z_offset - 0.19) < 1e-9
        h.arm(px4)
        h.fly(node, px4, pos, lambda: True, max_s=1.0)
        assert node.state == 'TAKEOFF'
        sent = [m for t, m in h.SENT if t == cm.POSE_TOPIC]
        assert sent and abs(sent[-1].pose.position.z - (node.target[2] + 0.19)) < 1e-9
    finally:
        finish()


def test_offset_is_measured_again_for_each_arming():
    # The fly_home re-arm on the pad re-measures rather than reusing it.
    try:
        node, px4, pad = h.run(fly_home=True)
        assert node.state == 'WAIT_FOR_OFFBOARD' and node.going_home
        assert node.z_offset == 0.0 and node.z_samples == []
        h.PX4_OWN_Z[0] = True
        h.Z_BIAS[0] = 0.12
        t0 = h.T[0]
        h.fly(node, px4, pad, lambda: h.T[0] - t0 >= 2.5, max_s=5.0)
        assert abs(node.z_offset - 0.12) < 1e-9
    finally:
        finish()


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
    print('ok')
