"""Pre-arm height check of course_mission.py, on test_fly_home.py's stub ROS.

PX4 can boot with its height ~0.6 m off (EKF2 parks the error in its terrain
estimate; with EV_CTRL 9 nothing pulls it back), and flight_interface passes
odom z to PX4 unchanged. The node must then withhold its setpoint stream, so
flight_interface refuses the takeover, and must never climb if Offboard comes
anyway.
"""
import json
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_fly_home as h    # noqa: E402  (stubs ROS, imports course_mission)

cm = h.course_mission


def make(**overrides):
    h.T[0] = 0.0
    h.Z_BIAS[0] = 0.0
    h.OVERRIDES.clear()
    h.OVERRIDES.update(route_file='route_gate_b.json', cruise_speed_mps=1.0, **overrides)
    node = cm.CourseMission()
    px4 = {'armed': False, 'offboard_active': False, 'landed': True}
    return node, px4, [0.0, 0.0, 0.0]


def tick(node, px4, pos, local=True):
    """One 10 Hz tick with the drone sitting at pos -> what the node published."""
    node.odometry_cb(types.SimpleNamespace(pose=types.SimpleNamespace(
        pose=types.SimpleNamespace(position=types.SimpleNamespace(
            x=pos[0], y=pos[1], z=pos[2])))))
    state = dict(px4)
    if local:
        state['local_enu'] = [pos[0], pos[1], pos[2] + h.Z_BIAS[0]]
    node.state_cb(types.SimpleNamespace(data=json.dumps(state)))
    h.SENT.clear()
    node.tick()
    h.T[0] += 0.1
    return list(h.SENT)


def ticks(node, px4, pos, seconds, **kw):
    sent = []
    for _ in range(int(round(seconds / 0.1))):
        sent += tick(node, px4, pos, **kw)
    return sent


def poses(sent):
    return [m for topic, m in sent if topic == cm.POSE_TOPIC]


def test_streams_only_after_the_heights_settle():
    node, px4, pos = make()
    h.Z_BIAS[0] = 0.07        # the Gazebo stand-in reports world z: start height above odom
    assert ticks(node, px4, pos, 1.9) == []
    assert poses(ticks(node, px4, pos, 0.4))


def test_px4_height_off_blocks_the_takeover_and_never_climbs():
    node, px4, pos = make()
    h.Z_BIAS[0] = -0.6        # PX4 believes it is 0.6 m low: it would climb 0.6 m too high
    assert ticks(node, px4, pos, 10.0) == []
    # Offboard arrives anyway (stream stopped < setpoint_timeout_s before the switch)
    h.arm(px4)
    sent = tick(node, px4, pos)
    assert node.state == 'REFUSED'
    assert not poses(sent)
    assert sent and all(m.twist.linear.z < 0 for _, m in sent)
    top = h.fly(node, px4, pos, lambda: not px4['armed'])     # PX4 disarms on the floor
    assert top == 0.0 and pos[2] == 0.0
    tick(node, px4, pos)
    assert node.state == 'WAIT_FOR_OFFBOARD'
    assert ticks(node, px4, pos, 5.0) == []                   # still off: still silent


def test_no_local_position_from_px4_blocks_the_stream():
    node, px4, pos = make()
    assert ticks(node, px4, pos, 5.0, local=False) == []


def test_a_glitch_restarts_the_settle():
    node, px4, pos = make()
    assert ticks(node, px4, pos, 1.5) == []
    h.Z_BIAS[0] = 0.4                           # past prearm_height_tol_m (0.30)
    assert tick(node, px4, pos) == []
    h.Z_BIAS[0] = 0.0
    assert ticks(node, px4, pos, 1.9) == []
    assert poses(ticks(node, px4, pos, 0.4))


def test_noise_at_the_threshold_after_passing_keeps_streaming():
    node, px4, pos = make()
    h.Z_BIAS[0] = 0.29
    ticks(node, px4, pos, 2.5)                  # passed (limit 0.30)
    for i in range(30):                         # 0.29 / 0.32 / 0.34 m, alternating
        h.Z_BIAS[0] = (0.29, 0.32, 0.34)[i % 3]   # under the 0.35 revoke
        assert poses(tick(node, px4, pos))


def test_takeover_right_after_a_threshold_blip_still_takes_off():
    # Gazebo, simulated pilot: it armed at a 0.15 m boot offset, one noisy
    # sample dropped the old check, and the takeover became a REFUSED descent.
    # Same at today's limit (0.30): armed at it, one sample just over.
    node, px4, pos = make()
    h.Z_BIAS[0] = 0.30
    ticks(node, px4, pos, 2.5)
    h.Z_BIAS[0] = 0.31
    tick(node, px4, pos)
    h.arm(px4)
    tick(node, px4, pos)
    assert node.state == 'TAKEOFF'


def test_a_clear_disagreement_after_passing_still_revokes():
    node, px4, pos = make()
    ticks(node, px4, pos, 2.5)
    h.Z_BIAS[0] = 0.45                          # past 0.30 + the 0.05 hysteresis
    tick(node, px4, pos)
    assert ticks(node, px4, pos, 2.0) == []


def test_one_missing_px4_position_is_tolerated_but_not_a_second_of_none():
    node, px4, pos = make()
    ticks(node, px4, pos, 2.5)
    assert poses(tick(node, px4, pos, local=False))
    ticks(node, px4, pos, 1.5, local=False)
    assert tick(node, px4, pos, local=False) == []


def test_check_off_streams_at_once():
    node, px4, pos = make(prearm_height_tol_m=0.0)
    h.Z_BIAS[0] = 0.6
    assert poses(tick(node, px4, pos))


def test_fly_home_rearm_is_checked_again_on_the_pad():
    node, px4, pad = h.run(fly_home=True)
    assert node.state == 'WAIT_FOR_OFFBOARD' and node.going_home
    h.Z_BIAS[0] = 0.5         # heights drifted apart during the flight
    assert ticks(node, px4, pad, 5.0) == []
    h.Z_BIAS[0] = 0.0
    assert ticks(node, px4, pad, 1.9) == []
    assert poses(ticks(node, px4, pad, 0.4))


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
    print('ok')
