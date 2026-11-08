"""blind_carry in course_mission.py, on test_fly_home.py's stub ROS.

The tube is dark inside (course_v3.json), so cuVSLAM may lose tracking there.
Freezing the carrot in the tube would hover blind until PX4's dead reckoning
times out (EKF2_NOAID_TOUT, 5 s). With blind_carry on, the carrot keeps
pacing PX4's own estimate along the route while it is in the tube zone, for
at most blind_carry_max_s per loss; anywhere else a VIO loss still freezes it.
"""
import json
import math
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_fly_home as h    # noqa: E402  (stubs ROS, imports course_mission)
from navless_nav.route import odom_to_course    # noqa: E402

cm = h.course_mission
TUBE_MOUTH_Y = 1.2192        # course frame: the tube's north mouth


def step(node, px4, pos, vo_state=1, dt=0.1, frozen=None, moving=False, yaw=0.0):
    """One tick: odometry, state, and the point-mass drone. While lost
    (vo_state 2) odometry stops, or with `frozen` keeps repeating that pose,
    or with `moving` keeps following the drone (IMU-propagated). The drone
    holds heading `yaw` (odom ENU)."""
    h.vslam(node, vo_state)
    h.px4_heading(node, yaw)
    if vo_state != 2 or frozen is not None or moving:
        p = frozen if (vo_state == 2 and frozen is not None) else pos
        h.send_odom(node, *p, yaw=yaw)
    px4['local_enu'] = list(pos)          # PX4's own estimate (perfect here)
    node.state_cb(types.SimpleNamespace(data=json.dumps(px4)))
    h.SENT.clear()
    node.tick()
    if px4['armed'] and h.SENT:
        topic, m = h.SENT[-1]
        if topic == cm.POSE_TOPIC:
            goal = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
            d = math.dist(pos, goal)
            f = min(1.0, 1.0 * dt / d) if d > 0 else 0.0
            pos[:] = [a + (b - a) * f for a, b in zip(pos, goal)]
        else:
            v = m.twist.linear
            pos[:] = [pos[0] + v.x * dt, pos[1] + v.y * dt, max(0.0, pos[2] + v.z * dt)]
    h.T[0] += dt


def course(node, p):
    return odom_to_course(p, node.frame[0], node.frame[1])


def fly_to_tube(**overrides):
    h.T[0] = 0.0
    h.Z_BIAS[0] = 0.0
    h.OVERRIDES.clear()
    h.OVERRIDES.update(route_file='gazebo_sim/course/route_centreline_window.json',
                       cruise_speed_mps=0.5, **overrides)
    node = cm.CourseMission()
    px4 = {'armed': False, 'offboard_active': False, 'landed': True}
    pos = [0.0, 0.0, 0.0]
    for _ in range(30):                       # past the pre-arm settle
        step(node, px4, pos)
    px4.update(armed=True, offboard_active=True)
    for _ in range(3000):
        step(node, px4, pos)
        if node.state == 'ROUTE':
            c = course(node, node.carrot.point)
            if c[0] > 2.0 and c[1] < TUBE_MOUTH_Y - 0.2:     # carrot inside the tube
                return node, px4, pos
    raise AssertionError(f'never reached the tube ({node.state})')


def lose_vio(node, px4, pos, seconds):
    s0 = node.carrot.point
    for _ in range(int(round(seconds / 0.1))):
        step(node, px4, pos, vo_state=2)
    return math.dist(s0, node.carrot.point)


def test_default_freezes_in_the_tube():
    node, px4, pos = fly_to_tube()
    assert lose_vio(node, px4, pos, 2.0) == 0.0


def test_blind_carry_keeps_moving_through_the_tube_then_stops():
    node, px4, pos = fly_to_tube(blind_carry=True, blind_carry_max_s=3.0)
    moved = lose_vio(node, px4, pos, 2.0)
    assert 0.6 < moved < 1.2                       # ~0.5 m/s, paced by PX4's estimate
    p = node.carrot.point
    lose_vio(node, px4, pos, 3.0)                  # past blind_carry_max_s: frozen again
    assert math.dist(p, node.carrot.point) < 0.6
    q = node.carrot.point
    assert lose_vio(node, px4, pos, 1.0) == 0.0 and node.carrot.point == q


def last_pose(node):
    poses = [m for topic, m in h.SENT if topic == cm.POSE_TOPIC]
    m = poses[-1]
    return (m.pose.position.x, m.pose.position.y, m.pose.position.z)


def frozen_loss(node, px4, pos, seconds):
    loss = tuple(pos)
    for _ in range(int(round(seconds / 0.1))):
        step(node, px4, pos, vo_state=2, frozen=loss)
    return loss


def test_frozen_odometry_holds_where_vio_was_lost():
    # Gazebo 'freeze': cuVSLAM repeating its last pose (and PX4 fusing it)
    # with the carrot ahead made PX4 push on into the south net.
    node, px4, pos = fly_to_tube()
    carrot = node.carrot.advance(0.3)                      # carrot 0.3 m ahead
    loss = frozen_loss(node, px4, pos, 1.0)
    assert math.dist(last_pose(node), loss) < 1e-9
    assert math.dist(last_pose(node), carrot) > 0.25      # not the carrot ahead
    step(node, px4, pos)                                   # VIO back
    assert node.state == 'ROUTE'


def test_blind_carry_does_not_run_on_frozen_odometry():
    node, px4, pos = fly_to_tube(blind_carry=True)
    s0 = node.carrot.point
    loss = frozen_loss(node, px4, pos, 2.0)
    # the freeze shows after a message >= 0.1 s into the loss: <= 2 ticks of carry
    assert math.dist(node.carrot.point, s0) < 0.11
    assert math.dist(last_pose(node), loss) < 1e-9


def test_any_odometry_during_a_loss_holds_at_its_latest_pose():
    # cuVSLAM has IMU fusion off on skybeta, so odometry that keeps coming
    # while it reports lost is frozen or unreliable, and PX4 fuses it: hold at
    # the latest pose (zero error), and never carry on it.
    node, px4, pos = fly_to_tube(blind_carry=True)
    s0 = node.carrot.point
    for _ in range(10):
        step(node, px4, pos, vo_state=2, moving=True)
    assert math.dist(node.carrot.point, s0) < 0.11      # <= the 2 ticks to see it
    assert math.dist(last_pose(node), node.position) < 1e-9


def test_moving_then_frozen_odometry_holds_at_the_frozen_pose():
    # Review of #18: odometry that moves for a tick or two after the loss and
    # then freezes must still be held, not sent back to the carrot ahead.
    for moving_ticks in (0, 1, 2):
        node, px4, pos = fly_to_tube()
        node.carrot.advance(0.3)
        for _ in range(moving_ticks):
            step(node, px4, pos, vo_state=2, moving=True)
        frozen_at = tuple(pos)
        for _ in range(10):
            step(node, px4, pos, vo_state=2, frozen=frozen_at)
        assert math.dist(last_pose(node), frozen_at) < 1e-9, moving_ticks


def test_odometry_topic_is_configurable():
    # Read the same odometry that feeds PX4 (e.g. Koji's gated topic, which
    # drops samples while cuVSLAM is not tracking), so a loss looks the same.
    h.SUBS.clear()
    h.OVERRIDES.clear()
    h.OVERRIDES.update(odometry_topic='/mapping/validated_odometry')
    cm.CourseMission()
    assert '/mapping/validated_odometry' in h.SUBS
    assert '/visual_slam/tracking/odometry' not in h.SUBS
    h.SUBS.clear()
    h.OVERRIDES.clear()
    cm.CourseMission()
    assert '/visual_slam/tracking/odometry' in h.SUBS          # the default


def test_blind_carry_never_runs_outside_the_tube_zone():
    h.T[0] = 0.0
    h.OVERRIDES.clear()
    h.OVERRIDES.update(route_file='gazebo_sim/course/route_centreline_window.json',
                       cruise_speed_mps=0.5, blind_carry=True)
    node = cm.CourseMission()
    px4 = {'armed': False, 'offboard_active': False, 'landed': True}
    pos = [0.0, 0.0, 0.0]
    for _ in range(30):
        step(node, px4, pos)
    px4.update(armed=True, offboard_active=True)
    for _ in range(100):                      # into the first corridor
        step(node, px4, pos)
    assert node.state == 'ROUTE'
    assert lose_vio(node, px4, pos, 2.0) == 0.0


def test_blind_carry_needs_px4_position():
    node, px4, pos = fly_to_tube(blind_carry=True)
    s0 = node.carrot.point
    for _ in range(10):
        node.status_cb(types.SimpleNamespace(vo_state=2))
        node.state_cb(types.SimpleNamespace(data=json.dumps(
            {'armed': True, 'offboard_active': True, 'landed': False, 'local_enu': None})))
        node.tick()
        h.T[0] += 0.1
    assert node.carrot.point == s0


def test_vio_back_moves_the_carrot_up_to_a_drone_that_got_ahead():
    # Frozen carrot (blind_carry off), but the drone drifted 0.8 m on down
    # the tube while blind: on VIO back the route continues from the drone,
    # instead of flying it back into the dark.
    node, px4, pos = fly_to_tube()
    frozen = node.carrot.point
    for _ in range(10):
        step(node, px4, pos, vo_state=2)
    assert node.carrot.point == frozen
    # put the drone 0.8 m further along the route (down the tube)
    ahead = cm.Carrot(node.route, 1.0, math.inf)
    ahead.index, ahead.point = node.carrot.index, node.carrot.point
    pos[:] = list(ahead.advance(0.8))
    # PX4 dead-reckoned that drift (its estimate arrives before the first sample back,
    # which must agree with it: see test_navigation_faults' post-gap checks)
    px4['local_enu'] = list(pos)
    node.state_cb(types.SimpleNamespace(data=json.dumps(px4)))
    step(node, px4, pos)                               # VIO back
    assert math.dist(node.carrot.point, pos) < 0.15
    assert math.dist(node.carrot.point, frozen) > 0.65


def test_vio_back_resumes_normally():
    node, px4, pos = fly_to_tube(blind_carry=True)
    lose_vio(node, px4, pos, 1.0)
    for _ in range(1500):
        step(node, px4, pos)
        if not px4['armed'] or node.state in ('DONE', 'HOLD'):
            break
        if node.state == 'LAND' and pos[2] <= 0.0:
            px4.update(armed=False, offboard_active=False)
    assert node.state in ('LAND', 'DONE')


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
    print('ok')
