"""heading_mode 'travel' (nose along the route, north through the tube), the
hold drift correction and the flight recorder, on test_fly_home.py's stub ROS
and point-mass drone."""
import csv
import glob
import json
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_fly_home as h    # noqa: E402  (stubs ROS, imports course_mission)

cm = h.course_mission
WINDOW = 'gazebo_sim/course/route_centreline_window_asbuilt.json'   # the new-maze route, through the tube


def start(offset=None, **overrides):
    h.T[0] = 0.0
    h.Z_BIAS[0] = 0.0
    h.reset_heading()
    h.PX4_HOOK[0] = None
    h.ODOM_OFFSET[0] = offset
    h.OVERRIDES.clear()
    h.OVERRIDES.update(route_file=WINDOW, route_source='file', cruise_speed_mps=0.8, **overrides)
    node = cm.CourseMission()
    px4 = {'armed': False, 'offboard_active': False, 'landed': True}
    pos = [0.0, 0.0, 0.0]
    h.fly(node, px4, pos, lambda: h.T[0] >= 2.5)
    h.arm(px4)
    return node, px4, pos


def record(node, px4, pos, until, max_s=600.0):
    rows = []

    def done():
        if node.state == 'ROUTE':
            c = node.carrot
            rows.append({'t': h.T[0], 'cmd': node.last_cmd_yaw, 'want': node.want_yaw,
                         'tube': node.tube_heading, 'carrot': c.point,
                         'in_tube': node.in_tube_footprint(c.point), 'drift': node.drift_body,
                         'learning': node.drift_learning})
        return until()
    h.fly(node, px4, pos, done, max_s=max_s)
    return rows


def deg(a):
    return math.degrees(a)


def test_travel_heading_follows_the_route_and_faces_north_in_the_tube():
    try:
        node, px4, pos = start()
        assert node.heading_mode == 'travel'
        rows = record(node, px4, pos, lambda: not px4['armed'])
        assert node.state == 'DONE'
        cmds = [r for r in rows if r['cmd'] is not None]
        assert len(cmds) > 100
        # inside the tube's footprint the nose is held north (odom yaw 0 with start bearing 0)
        tube = [r for r in cmds if r['in_tube']]
        assert len(tube) > 10
        assert max(abs(deg(h.wrap(r['cmd']))) for r in tube) < 3.0
        # the north-east first leg (bearing ~51 deg = odom yaw ~ -51 deg) is flown nose first
        assert any(abs(deg(h.wrap(r['cmd'])) + 51.0) < 8.0 for r in cmds[:80])
        # after the tube it turns away from north again
        after = [r for r in cmds if r['t'] > max(x['t'] for x in tube) + 3.0]
        assert after and any(abs(deg(h.wrap(r['cmd']))) > 30.0 for r in after)
        # never faster than travel_heading_rate_dps (+1 tick of slack)
        rate = node.travel_rate
        for a, b in zip(cmds, cmds[1:]):
            assert abs(h.wrap(b['cmd'] - a['cmd'])) <= rate * (b['t'] - a['t']) + 1e-6
    finally:
        h.reset_heading()


def test_the_turn_to_north_starts_before_the_tube_mouth():
    try:
        node, px4, pos = start()
        rows = record(node, px4, pos, lambda: not px4['armed'])
        first_in = next(i for i, r in enumerate(rows) if r['in_tube'])
        assert rows[first_in - 1]['tube'] or rows[first_in]['tube']
        assert any(r['tube'] for r in rows[:first_in])          # announced before the footprint
    finally:
        h.reset_heading()


def test_hold_mode_keeps_the_takeoff_heading():
    try:
        node, px4, pos = start(heading_mode='hold')
        rows = record(node, px4, pos, lambda: not px4['armed'])
        cmds = [r['cmd'] for r in rows if r['cmd'] is not None]
        assert cmds and max(abs(deg(h.wrap(c))) for c in cmds) < 1e-6
    finally:
        h.reset_heading()


def test_a_bad_heading_mode_falls_back_to_travel():
    h.OVERRIDES.clear()
    h.OVERRIDES.update(route_file=WINDOW, route_source='file', heading_mode='sideways')
    assert cm.CourseMission().heading_mode == 'travel'


def steady_push(t0, dy=0.10, rise=0.3):
    """A push PX4 doesn't take out, fixed to the drone's body: it sits dy to its left of whatever it
    is told (19's level / centre-of-gravity bias; on 2026-11-07 the liftoff slide flipped with a
    180 deg yaw), so in odom it turns with the nose. It builds up over `rise` s (a 30 cm step in
    one odometry sample would be the jump FAULT)."""
    def off(t):
        if t < t0:
            return (0.0, 0.0, 0.0)
        c, s = math.cos(h.YAW[0]), math.sin(h.YAW[0])
        k = dy * min(1.0, (t - t0) / rise)
        return (-s * k, c * k, 0.0)
    return off


def test_drift_correction_cancels_a_steady_push_during_the_takeoff_settle():
    try:
        node, px4, pos = start(offset=steady_push(2.7), takeoff_timeout_s=30.0)
        t0 = h.T[0]
        h.fly(node, px4, pos, lambda: node.state not in ('WAIT_FOR_OFFBOARD', 'TAKEOFF') or h.T[0] - t0 > 31)
        assert node.state == 'ROUTE'
        assert h.T[0] - t0 < 25.0                              # settled, not the timeout fallback
        assert abs(math.hypot(*node.drift_body) - 0.10) < 0.03  # learned the 10 cm push
        # without it the same push never settles within 5 cm: the timeout holds, no route
        node, px4, pos = start(offset=steady_push(2.7), takeoff_timeout_s=8.0, drift_correction=False)
        t0 = h.T[0]
        h.fly(node, px4, pos, lambda: node.state not in ('WAIT_FOR_OFFBOARD', 'TAKEOFF'))
        assert h.T[0] - t0 >= 8.0 and node.drift_body == (0.0, 0.0) and node.state == 'HOLD'
    finally:
        h.reset_heading()


def test_drift_is_never_learned_while_the_target_moves():
    try:
        node, px4, pos = start(offset=steady_push(2.7))
        rows = record(node, px4, pos, lambda: not px4['armed'])
        assert not any(r['learning'] for r in rows)
        assert len({r['drift'] for r in rows}) == 1             # kept, unchanged, along the route
    finally:
        h.reset_heading()


def test_drift_correction_turns_with_the_nose():
    h.OVERRIDES.clear()
    h.OVERRIDES.update(route_file=WINDOW, route_source='file')
    node = cm.CourseMission()
    node.drift_body = (0.10, 0.0)
    node.yaw = math.pi / 2
    x, y = node.drift_world()
    assert abs(x) < 1e-9 and abs(y - 0.10) < 1e-9


def test_drift_correction_is_not_applied_without_live_odometry():
    try:
        node, px4, pos = start(offset=steady_push(2.7))
        h.fly(node, px4, pos, lambda: node.state == 'ROUTE')
        assert node.drift_applies()
        node.vo_state = cm.VO_LOST
        assert not node.drift_applies()
        node.vo_state = 1
        node.blind_carrying = True
        assert not node.drift_applies()
    finally:
        h.reset_heading()


def test_flight_log_records_ticks_events_and_parameters():
    d = tempfile.mkdtemp(prefix='navless_flog_')
    try:
        node, px4, pos = start(flight_log_dir=d)
        h.fly(node, px4, pos, lambda: node.state == 'ROUTE')
        node.flog.close()
        csvs = glob.glob(os.path.join(d, '*_course.csv'))
        params = glob.glob(os.path.join(d, '*_course_params.json'))
        assert len(csvs) == 1 and len(params) == 1
        assert json.load(open(params[0]))['heading_mode'] == 'travel'
        rows = list(csv.DictReader(open(csvs[0])))
        ticks = [r for r in rows if r['kind'] == 'tick']
        events = [r for r in rows if r['kind'] in ('info', 'warn', 'error')]
        assert len(ticks) > 20
        assert any('takeoff settled' in r['event'] for r in events)
        assert {r['state'] for r in ticks} >= {'WAIT_FOR_OFFBOARD', 'TAKEOFF'}
        assert all(r['x'] != '' and r['cmd_yaw_deg'] != '' for r in ticks if r['state'] == 'TAKEOFF')
    finally:
        h.reset_heading()


def test_flight_log_off_writes_nothing():
    d = tempfile.mkdtemp(prefix='navless_flog_')
    h.OVERRIDES.clear()
    h.OVERRIDES.update(route_file=WINDOW, route_source='file', flight_log=False, flight_log_dir=d)
    node = cm.CourseMission()
    assert node.flog is None and os.listdir(d) == []


def test_with_a_body_fixed_push_the_route_is_flown_closer_to_its_targets():
    """The learned correction keeps cancelling 19's body-fixed push along the route as the nose turns."""
    def median_error(**kw):
        # A 15 cm settle radius, so the uncorrected flight (pushed 10 cm off) settles and flies
        # the route too: an unsettled takeoff now holds instead of starting it.
        node, px4, pos = start(offset=steady_push(2.7), takeoff_settle_radius_m=0.15, **kw)
        errs = []

        def until():
            if node.state == 'ROUTE' and node.last_target is not None and node.vio_ok():
                errs.append(math.dist(node.position[:2], node.last_target[:2]))
            return not px4['armed']
        h.fly(node, px4, pos, until)
        assert node.state == 'DONE'
        errs.sort()
        return errs[len(errs) // 2]
    try:
        on, off = median_error(), median_error(drift_correction=False)
        assert on < off - 0.04, (on, off)
    finally:
        h.reset_heading()


SEED = dict(drift_seed_body_y_m=-0.28)     # the course launch's value for skybeta-19


def climb_error(**kw):
    """Sideways (north-west/left) error from the true takeoff target near the top of the climb (0.65 m, before the drift
    correction starts learning),
    with 19's liftoff slide: a 30 cm body-left push from the instant of liftoff. (Not the distance
    to the target: that includes the planned takeoff lean, which is flown on purpose.)"""
    node, px4, pos = start(offset=steady_push(2.7, dy=0.30), **kw)
    err = [None]

    def until():
        if node.state == 'TAKEOFF' and err[0] is None and node.position[2] >= 0.65:
            err[0] = abs(node.position[1] - node.target[1])            # odom y = left, nose north
        return err[0] is not None or node.state not in ('WAIT_FOR_OFFBOARD', 'TAKEOFF')
    h.fly(node, px4, pos, until)
    return node, err[0]


def test_seeded_takeoff_holds_against_the_liftoff_slide():
    try:
        node, seeded = climb_error(**SEED)
        assert node.drift_seeded and abs(node.drift_body[1] + 0.28) < 0.02
        _, bare = climb_error()
        # without the seed nothing counters the push below 0.4 m: ~30 cm to the left (the west net
        # from the launch mark); with it, a few cm
        assert bare > 0.25, bare
        assert seeded < bare - 0.15, (seeded, bare)
    finally:
        h.reset_heading()


def test_seed_without_a_slide_is_learned_away_and_still_settles():
    try:
        node, px4, pos = start(takeoff_timeout_s=30.0, **SEED)
        t0 = h.T[0]
        h.fly(node, px4, pos, lambda: node.state not in ('WAIT_FOR_OFFBOARD', 'TAKEOFF') or h.T[0] - t0 > 31)
        assert node.state == 'ROUTE' and h.T[0] - t0 < 25.0
        assert math.hypot(*node.drift_body) < 0.08            # learned back toward zero
    finally:
        h.reset_heading()


def test_seed_is_off_by_default_and_capped():
    h.OVERRIDES.clear()
    h.OVERRIDES.update(route_file=WINDOW, route_source='file')
    node = cm.CourseMission()
    node.seed_drift()
    assert node.drift_body == (0.0, 0.0)
    h.OVERRIDES.update(drift_seed_body_y_m=-0.9)
    node = cm.CourseMission()
    node.seed_drift()
    assert abs(math.hypot(*node.drift_body) - node.drift_max) < 1e-9
    node.drift_body = (0.0, 0.05)
    node.seed_drift()                                          # only the first takeoff is seeded
    assert node.drift_body == (0.0, 0.05)
