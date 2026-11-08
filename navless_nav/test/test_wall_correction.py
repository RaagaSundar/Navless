"""wall_correction in course_mission.py, on test_fly_home.py's stub ROS (a point-mass drone).

depthcloud's cloud_localize (main 239ca00) publishes /team/localize/odom_correction while it is valid:
D, a planar transform in odom with corrected = D * course_to_odom(p), so the route stays put against
the walls while cuVSLAM's odometry frame drifts. Here a perfect localizer publishes the true drift D,
and the drone's true position in the route's (walls') frame is D^-1 * its odometry position.
"""
import math
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_fly_home as h    # noqa: E402  (stubs ROS, imports course_mission)

cm = h.course_mission
TOPIC = '/team/localize/odom_correction'
DRIFT = (0.30, -0.20, 4.0)                  # dx, dy (m), dyaw (deg)
RATE_M, RATE_DEG = 0.12, 4.5                # course_mission's wall_correction_rate_* defaults


def corr_msg(dx, dy, dyaw_deg, frame='odom'):
    a = math.radians(dyaw_deg)
    return types.SimpleNamespace(
        header=types.SimpleNamespace(frame_id=frame),
        pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=dx, y=dy, z=0.0),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(a / 2), w=math.cos(a / 2))))


def unapply(d, p):
    """D^-1 * p (planar): where an odometry point really is, in the route's frame."""
    a = math.radians(d[2])
    c, s = math.cos(a), math.sin(a)
    x, y = p[0] - d[0], p[1] - d[1]
    return (c * x + s * y, -s * x + c * y)


def off_route(route, q):
    """Distance (m) from the point q to the route polyline, horizontally."""
    best = math.inf
    for a, b in zip(route, route[1:]):
        vx, vy = b[0] - a[0], b[1] - a[1]
        n2 = vx * vx + vy * vy
        t = 0.0 if n2 == 0.0 else max(0.0, min(1.0, ((q[0] - a[0]) * vx + (q[1] - a[1]) * vy) / n2))
        best = min(best, math.hypot(q[0] - a[0] - t * vx, q[1] - a[1] - t * vy))
    return best


def start(feed=None, **overrides):
    """A node on the launch mark past the pre-arm settle, then armed. feed(t, node) returns the
    correction published that tick, (dx, dy, dyaw_deg), or None (cloud_localize is silent)."""
    h.T[0] = 0.0
    h.Z_BIAS[0] = 0.0
    h.reset_heading()
    h.SUBS.clear()
    if feed is not None:
        def hook(node):
            d = feed(h.T[0], node)
            if d is not None:
                node.odom_correction_cb(corr_msg(*d))
        h.PX4_HOOK[0] = hook
    h.OVERRIDES.clear()
    h.OVERRIDES.update(route_file='route_gate_b.json', cruise_speed_mps=1.0, heading_mode='hold', **overrides)
    node = cm.CourseMission()
    px4 = {'armed': False, 'offboard_active': False, 'landed': True}
    pos = [0.0, 0.0, 0.0]
    h.fly(node, px4, pos, lambda: h.T[0] >= 2.5)
    h.arm(px4)
    return node, px4, pos


def fly_route(node, px4, pos, drift=(0.0, 0.0, 0.0)):
    """Fly until disarmed. Per ROUTE tick: (t, applied D, carrot in the tube zone, how far the drone
    really is off the route when the walls have drifted by `drift`)."""
    rows = []

    def record():
        if node.state == 'ROUTE':
            rows.append((h.T[0], node.corr, node.in_blind_zone(node.carrot.point),
                         off_route(node.route, unapply(drift, pos))))
        return not px4['armed']

    h.fly(node, px4, pos, record)
    return rows


def median(xs):
    xs = sorted(xs)
    return xs[len(xs) // 2]


def test_off_by_default():
    try:
        node, px4, pos = start(feed=lambda t, n: DRIFT)      # even if messages were delivered
        assert TOPIC not in h.SUBS
        fly_route(node, px4, pos, DRIFT)
        assert node.state == 'DONE' and node.corr == (0.0, 0.0, 0.0)
    finally:
        h.reset_heading()


def test_constant_drift_keeps_the_route_on_the_walls():
    try:
        node, px4, pos = start()
        base = [r[3] for r in fly_route(node, px4, pos)]               # no drift: the stub's own tracking
        node, px4, pos = start(feed=lambda t, n: DRIFT)
        raw = [r[3] for r in fly_route(node, px4, pos, DRIFT)]         # drift, correction off
        node, px4, pos = start(feed=lambda t, n: DRIFT, wall_correction=True)
        assert TOPIC in h.SUBS
        rows = fly_route(node, px4, pos, DRIFT)
        assert node.state == 'DONE'
        cor = [r[3] for r in rows]
        settled = [r[3] for r in rows if r[0] - rows[0][0] > 5.0]   # D ramped in (0.36 m at 0.12 m/s)
        print(f"off the route against the walls: no drift {median(base) * 100:.1f} cm median, "
              f"{max(base) * 100:.1f} max; drift uncorrected {median(raw) * 100:.1f} median, "
              f"{max(raw) * 100:.1f} max; corrected {median(cor) * 100:.1f} median, "
              f"{max(settled) * 100:.1f} max after the ramp")
        assert median(raw) > median(base) + 0.15
        assert median(cor) < median(base) + 0.02
        assert max(settled) < max(base) + 0.02
        want = (DRIFT[0], DRIFT[1], math.radians(DRIFT[2]))
        assert all(abs(a - b) < 1e-9 for a, b in zip(node.corr, want))
    finally:
        h.reset_heading()


def test_a_jump_in_d_is_ramped_never_stepped():
    jump = (0.5, 0.0, 10.0)
    try:
        node, px4, pos = start(feed=lambda t, n: jump if n.state == 'ROUTE' and t - n.state_since > 3.0
                               else (0.0, 0.0, 0.0), wall_correction=True)
        rows = fly_route(node, px4, pos)
        assert node.state == 'DONE'
        for (t0, a, *_), (t1, b, *_) in zip(rows, rows[1:]):
            assert math.hypot(b[0] - a[0], b[1] - a[1]) <= RATE_M * (t1 - t0) + 1e-9
            assert abs(h.wrap(b[2] - a[2])) <= math.radians(RATE_DEG) * (t1 - t0) + 1e-9
        assert rows[0][1] == (0.0, 0.0, 0.0)
        end = rows[-1][1]
        assert abs(end[0] - 0.5) < 1e-9 and abs(end[2] - math.radians(10.0)) < 1e-9
    finally:
        h.reset_heading()


def test_d_holds_still_in_the_tube_zone():
    try:
        node, px4, pos = start(feed=lambda t, n: (0.02 * t, 0.0, 0.0), wall_correction=True)
        rows = fly_route(node, px4, pos)
        steps = list(zip(rows, rows[1:]))
        in_zone = [(a, b) for a, b in steps if a[2]]       # the carrot in the zone as the tick began
        assert len(in_zone) > 10                            # the route goes through the tube
        assert all(b[1] == a[1] for a, b in in_zone)
        assert any(b[1] != a[1] for a, b in steps if not a[2])
    finally:
        h.reset_heading()


def test_when_messages_stop_the_last_d_stays():
    d = (0.2, 0.1, 2.0)
    try:
        node, px4, pos = start(feed=lambda t, n: None if n.state == 'ROUTE' and t - n.state_since > 6.0
                               else d, wall_correction=True)
        rows = fly_route(node, px4, pos)
        late = [r[1] for r in rows if r[0] - rows[0][0] > 10.0]   # silent for over 3 s by then
        assert late
        assert all(abs(c[0] - 0.2) < 1e-9 and abs(c[1] - 0.1) < 1e-9 for c in late)
    finally:
        h.reset_heading()


def test_the_held_heading_turns_with_dyaw():
    try:
        node, px4, pos = start(feed=lambda t, n: (0.0, 0.0, 5.0), wall_correction=True)
        rows = []

        def record():
            if node.state == 'ROUTE' and h.SENT and h.SENT[-1][0] == cm.POSE_TOPIC:
                rows.append((h.pose_yaw(h.SENT[-1][1]), node.corr[2]))
            return not px4['armed']

        h.fly(node, px4, pos, record)
        assert len(rows) > 100
        # every route setpoint: the takeoff heading (north, 0) turned by the dyaw applied that tick
        assert all(yaw is not None and abs(h.wrap(yaw - dyaw)) < 1e-9 for yaw, dyaw in rows)
        assert abs(rows[-1][1] - math.radians(5.0)) < 1e-9
    finally:
        h.reset_heading()


def test_bad_messages_are_ignored():
    try:
        node, px4, pos = start(wall_correction=True)
        for bad in (corr_msg(0.1, 0.0, 0.0, frame='map'), corr_msg(float('nan'), 0.0, 0.0),
                    corr_msg(3.5, 0.0, 0.0), corr_msg(0.0, 0.0, 45.0)):
            node.odom_correction_cb(bad)
            assert node.corr_rx is None
        node.odom_correction_cb(corr_msg(0.1, -0.1, 2.0))
        assert node.corr_rx is not None and abs(node.corr_rx[2] - math.radians(2.0)) < 1e-12
    finally:
        h.reset_heading()


def test_fly_home_starts_where_the_drone_is_and_lands_where_the_walls_put_the_mark():
    d = (0.25, 0.15, 3.0)
    try:
        node, px4, pos = start(feed=lambda t, n: d, wall_correction=True, fly_home=True)
        h.fly(node, px4, pos, lambda: not px4['armed'])
        assert node.state == 'WAIT_FOR_OFFBOARD' and node.going_home
        assert math.dist(node.route[0][:2], unapply(d, pos)) < 1e-9   # no step when the hop starts
        t0 = h.T[0]
        h.fly(node, px4, pos, lambda: h.T[0] - t0 >= 2.5, max_s=5.0)
        h.arm(px4)
        h.fly(node, px4, pos, lambda: not px4['armed'])
        assert node.state == 'DONE'
        # The launch mark where the walls put it is D * (0, 0) in odom, not the odom origin.
        assert math.dist(pos[:2], d[:2]) < 0.12
    finally:
        h.reset_heading()
