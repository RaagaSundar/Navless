"""Combined navigation-integrity behaviour of course_mission.py, on test_fly_home.py's stub ROS.

Synthetic closed-loop checks of the one commanding node, not flight or sensor qualification:
odometry discontinuities, stale or missing inputs, the localizer's correction contract, the
pre-takeoff registration, capture-time map projection and what FAULT does (and never does).
"""
import json
import math
import os
import sys
import types

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_fly_home as h               # noqa: E402  (stubs ROS, imports course_mission)
import test_wall_correction as wc       # noqa: E402
from test_cloud_transform import FRAME, cloud    # noqa: E402
from navless_nav.route import Carrot, odom_to_course    # noqa: E402

cm = h.course_mission
NS = types.SimpleNamespace
COMMANDS = (cm.POSE_TOPIC, cm.TWIST_TOPIC)


def make(start=(0.0, 0.0, 0.0), **overrides):
    h.T[0] = 0.0
    h.Z_BIAS[0] = 0.0
    h.reset_heading()
    h.OVERRIDES.clear()
    h.OVERRIDES.update(route_file='route_gate_b.json', cruise_speed_mps=1.0, heading_mode='hold',
                       **overrides)
    node = cm.CourseMission()
    px4 = {'armed': False, 'offboard_active': False, 'landed': True}
    return node, px4, list(start)


def collect(node, px4, pos, seconds):
    """Fly `seconds`; every (topic, msg) the node published on its command topics."""
    out, t0 = [], h.T[0]
    h.SENT.clear()

    def until():
        out.extend(h.SENT)
        return h.T[0] - t0 >= seconds - 1e-9
    h.fly(node, px4, pos, until, max_s=seconds + 1.0)
    return out


def wait(node, px4, pos, seconds):
    """Fly for `seconds` (fly()'s max_s is only a timeout)."""
    t0 = h.T[0]
    h.fly(node, px4, pos, lambda: h.T[0] - t0 >= seconds - 1e-9, max_s=seconds + 1.0)


def armed(**overrides):
    node, px4, pos = make(**overrides)
    h.fly(node, px4, pos, lambda: h.T[0] >= 3.0)
    h.arm(px4)
    return node, px4, pos


def ramp(t0, vec, rise=0.5, until=math.inf):
    """An odometry offset building up to `vec` over `rise` s from t0 (and back after `until`)."""
    def off(t):
        if t < t0:
            return (0.0, 0.0, 0.0)
        k = min(1.0, (t - t0) / rise) if t < until else max(0.0, 1.0 - (t - until) / rise)
        return tuple(k * v for v in vec)
    return off


# -- odometry discontinuities -------------------------------------------------------------------

def test_a_jump_latches_fault_and_nothing_is_commanded_after_even_when_healthy_again():
    node, px4, pos = armed()
    h.fly(node, px4, pos, lambda: node.state == 'ROUTE')
    h.fly(node, px4, pos, lambda: node.carrot.index >= 3)
    before = node.position
    tj = h.T[0]
    # the second 2026-11-08 course CSV: 1.140 m between ticks 0.102 s apart, vo_state 1
    h.ODOM_OFFSET[0] = lambda t: (1.14, 0.0, 0.0) if t >= tj else (0.0, 0.0, 0.0)
    wait(node, px4, pos, 0.2)
    assert node.state == 'FAULT' and node.navigation_fault == 'position_jump'
    assert node.position == before                  # the jumped sample never became the pose
    h.ODOM_OFFSET[0] = None                         # tracking "healthy" again
    assert collect(node, px4, pos, 3.0) == []
    assert node.state == 'FAULT' and node.position == before
    assert h.STATUS[-1]['fault_latched'] and h.STATUS[-1]['reason'] == 'position_jump'
    assert not h.STATUS[-1]['fault_descending']


def test_a_fault_while_landing_keeps_descending_straight_down_then_stops():
    node, px4, pos = armed()
    h.fly(node, px4, pos, lambda: node.state == 'LAND')
    wait(node, px4, pos, 0.3)
    tj = h.T[0]
    h.ODOM_OFFSET[0] = lambda t: (0.0, 0.9, 0.0) if t >= tj else (0.0, 0.0, 0.0)
    sent = collect(node, px4, pos, 1.0)
    assert node.state == 'FAULT' and node.fault_descending
    assert sent and all(topic == cm.TWIST_TOPIC for topic, _ in sent)
    for _, m in sent:
        t = m.twist
        assert (t.linear.x, t.linear.y, t.angular.z) == (0.0, 0.0, 0.0)
        assert t.linear.z == -node.land_speed
    h.fly(node, px4, pos, lambda: pos[2] == 0.0, max_s=10.0)
    px4.update(armed=False, offboard_active=False)  # PX4's land detector disarms
    assert collect(node, px4, pos, 2.0) == [] and node.state == 'FAULT'


def test_stale_source_with_a_fresh_receipt_is_never_a_pose():
    node, px4, pos = make()
    h.ODOM_AGE[0] = 0.5                     # arrives now, captured half a second ago
    assert [t for t, _ in collect(node, px4, pos, 3.0) if t in COMMANDS] == []
    assert node.position is None and node.navigation_fault is None
    h.ODOM_AGE[0] = 0.0
    # Hovering in flight, a pipeline falls behind: stamps still advance, but half as fast as
    # time. (Hovering: this stub reports the CURRENT position under the late stamp.)
    node, px4, pos = armed(takeoff_settle_s=30.0, takeoff_timeout_s=60.0)
    h.fly(node, px4, pos, lambda: node.state == 'TAKEOFF' and node.settle_since is not None
          and h.T[0] - node.settle_since > 1.0)
    seen = []

    def lagging():
        seen.append(node.position)
        h.ODOM_AGE[0] += 0.05
        return node.state == 'FAULT'
    h.fly(node, px4, pos, lagging, max_s=2.0)
    assert 'VIO tracking lost' in node.navigation_fault
    assert seen[-1] == seen[-8]          # the last stale samples never moved the pose
    # A source clock that goes backwards is itself a discontinuity.
    node, px4, pos = armed()
    h.fly(node, px4, pos, lambda: node.state == 'TAKEOFF')
    h.ODOM_AGE[0] = 0.2
    wait(node, px4, pos, 0.1)
    assert node.navigation_fault == 'nonmonotonic_stamp'


def test_odometry_without_a_header_is_dropped_on_the_ground_and_a_fault_in_flight():
    bare = NS(pose=NS(pose=NS(position=NS(x=0.0, y=0.0, z=0.0),
                              orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))))
    node, px4, pos = make()
    h.fly(node, px4, pos, lambda: h.T[0] >= 1.0)
    p = node.position
    node.odometry_cb(bare)
    assert node.navigation_fault is None and node.position == p
    node, px4, pos = armed()
    h.fly(node, px4, pos, lambda: node.state == 'TAKEOFF')
    node.odometry_cb(bare)
    assert node.state == 'FAULT' and 'malformed' in node.navigation_fault


def test_short_vio_gaps_ride_through_but_a_loss_outside_route_ends_the_mission():
    node, px4, pos = armed()
    h.fly(node, px4, pos, lambda: node.state == 'TAKEOFF' and pos[2] > 0.3)
    h.VO_STATE[0] = 2
    wait(node, px4, pos, 0.4)
    h.VO_STATE[0] = 1
    wait(node, px4, pos, 0.3)
    assert node.navigation_fault is None
    h.VO_STATE[0] = 2
    h.fly(node, px4, pos, lambda: node.state == 'FAULT', max_s=1.0)
    assert 'VIO tracking lost' in node.navigation_fault


def test_a_vio_loss_while_landing_keeps_landing():
    node, px4, pos = armed()
    h.fly(node, px4, pos, lambda: node.state == 'LAND')
    h.VO_STATE[0] = None                    # no status at all: not tracking
    h.fly(node, px4, pos, lambda: not px4['armed'], max_s=20.0)
    assert node.state == 'DONE' and node.navigation_fault is None


def test_sustained_lateral_error_in_route_is_a_fault_a_brief_one_is_not():
    def sideways(node):
        i = node.carrot.index
        a, b = node.route[i], node.route[i + 1]
        n = math.hypot(b[0] - a[0], b[1] - a[1])
        return (-(b[1] - a[1]) / n * 0.45, (b[0] - a[0]) / n * 0.45, 0.0)
    node, px4, pos = armed()
    h.fly(node, px4, pos, lambda: node.state == 'ROUTE')
    t0 = h.T[0]
    h.ODOM_OFFSET[0] = ramp(t0, sideways(node), rise=0.3, until=t0 + 0.6)   # 0.45 m for ~0.6 s
    h.fly(node, px4, pos, lambda: h.T[0] - t0 >= 2.0)
    assert node.navigation_fault is None and node.state == 'ROUTE'
    t0 = h.T[0]
    h.ODOM_OFFSET[0] = ramp(t0, sideways(node), rise=0.3)
    h.fly(node, px4, pos, lambda: node.state == 'FAULT', max_s=2.0)
    assert 'lateral tracking error' in node.navigation_fault
    assert 0.75 <= h.T[0] - t0 <= 1.5


# -- required readiness inputs ------------------------------------------------------------------

def poses_streamed(node, px4, pos, seconds=3.5):
    return [m for t, m in collect(node, px4, pos, seconds) if t == cm.POSE_TOPIC]


def test_no_vslam_status_never_streams():
    node, px4, pos = make()
    h.VO_STATE[0] = None
    assert poses_streamed(node, px4, pos) == []


def test_no_usable_px4_heading_never_streams():
    for hook in (lambda n: None,                                   # none at all
                 lambda n: h.px4_heading(n, good=False)):          # not good for control
        node, px4, pos = make()
        h.PX4_HOOK[0] = hook
        assert poses_streamed(node, px4, pos) == []
    node, px4, pos = make(prearm_heading_tol_deg=0.0)               # "off" refuses, never skips
    assert poses_streamed(node, px4, pos) == []
    node, px4, pos = make()
    assert poses_streamed(node, px4, pos)                           # the baseline does stream


# -- the localizer's correction contract --------------------------------------------------------

D = (0.25, 0.10, 6.0)            # dx, dy (m), dyaw (deg)


def feed(d=D, **fields):
    def hook(node):
        h.px4_heading(node)
        wc.localize(node, d, **fields)
    return hook


def test_registration_is_established_before_takeoff_and_kept_through_the_arming_tare():
    z0 = (0.40, -0.20, 0.0)          # odometry's origin is not where the drone sits
    node, px4, pos = make(start=z0, wall_correction=True)
    h.PX4_HOOK[0] = feed()
    a = math.radians(D[2])
    want = (D[0] - z0[0], D[1] - z0[1], a)
    h.fly(node, px4, pos, lambda: node.alignment_ready, max_s=5.0)
    for _ in range(10):               # many pre-arm ticks: still the same registration
        wait(node, px4, pos, 0.2)
        assert node.alignment_ready and np.allclose(node.corr, want, atol=1e-12)
    assert node.odom_zero_offset == z0
    # Where the walls put the drone: D^-1 * z0, in the route frame (launch mark at 0)
    c, s = math.cos(a), math.sin(a)
    gx, gy = z0[0] - D[0], z0[1] - D[1]
    assert np.allclose(node.corr_to_route((0.0, 0.0, 0.0))[:2], (c * gx + s * gy, -s * gx + c * gy))
    h.arm(px4)
    h.SENT.clear()
    h.fly(node, px4, pos, lambda: node.state == 'TAKEOFF', max_s=1.0)
    assert np.allclose(node.corr, want, atol=1e-12)                 # the tare kept it
    lead = np.subtract(Carrot(node.route, 1.0, math.inf).advance(node.takeoff_lead)[:2], node.route[0][:2])
    assert np.allclose(node.target[:2], (c * lead[0] - s * lead[1], s * lead[0] + c * lead[1]))
    m = [m for t, m in h.SENT if t == cm.POSE_TOPIC][-1]
    sent = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
    # commands go out in RAW odom: the zero offset is added back
    assert np.allclose(sent, np.add(node.last_pose, z0))
    assert abs(h.wrap(node.hold_yaw - (0.0 - a))) < 1e-9            # nose held where it is


def test_unqualified_corrections_never_register_or_stream():
    stamp = lambda: h.T[0] - 1.0     # noqa: E731
    for fields in (dict(correction_status='weak'),
                   dict(correction_measured={'x': 1.0, 'y': 1.0, 'heading': 0.5}),
                   dict(correction_measured={'x': True, 'y': True, 'heading': True}),
                   dict(correction_measured={'x': 1.0, 'y': 1.0}),
                   dict(odometry_valid=False), dict(correction_valid=False),
                   dict(correction_frame='map'), dict(t=stamp)):
        node, px4, pos = make(wall_correction=True)
        if 't' in fields:
            def hook(node, f=fields):
                h.px4_heading(node)
                wc.localize(node, D, t=f['t']())
        else:
            hook = feed(**fields)
        h.PX4_HOOK[0] = hook
        assert poses_streamed(node, px4, pos) == [], fields
        assert node.corr_rx is None and not node.alignment_ready, fields


def corr_and_status(t, **fields):
    m = wc.corr_msg(*D, t=t)
    return m, wc.status_msg(m, **fields)


def test_status_and_correction_pair_in_either_order_and_never_backwards():
    node, _, _ = make(wall_correction=True)
    h.T[0] = 10.0
    m, s = corr_and_status(9.95)
    node.odom_correction_cb(m)                       # correction first: held, not used
    assert node.corr_rx is None
    node.localizer_status_cb(s)                      # its status: now it pairs
    assert node.corr_rx is not None and node.corr_rx_at == 9.95
    old, old_s = corr_and_status(9.90)               # older than the one accepted
    node.localizer_status_cb(old_s)
    node.odom_correction_cb(old)
    assert node.corr_rx_at == 9.95
    m, s = corr_and_status(9.40)                     # 0.6 s old on arrival: never applied
    node2, _, _ = make(wall_correction=True)
    h.T[0] = 10.0
    node2.localizer_status_cb(s)
    node2.odom_correction_cb(m)
    assert node2.corr_rx is None


def test_the_newest_invalid_status_blocks_an_older_pending_correction():
    node, _, _ = make(wall_correction=True)
    h.T[0] = 10.0
    m, s = corr_and_status(9.95)
    node.localizer_status_cb(s)
    # cloud_localize's next status, for a stale cloud: no correction identity, not valid
    _, later = corr_and_status(9.98, correction_valid=False, correction_source_stamp_ns=None)
    node.localizer_status_cb(later)
    node.odom_correction_cb(m)                       # the valid correction, delivered late
    assert node.corr_rx is None
    # The same old metadata, now marked invalid by a later status, is also refused.
    node, _, _ = make(wall_correction=True)
    h.T[0] = 10.0
    m, s = corr_and_status(9.95)
    node.localizer_status_cb(s)
    _, again = corr_and_status(9.95, correction_valid=False)
    node.localizer_status_cb(again)
    node.odom_correction_cb(m)
    assert node.corr_rx is None


def test_a_localizer_epoch_change_realigns_on_the_ground_and_is_a_fault_in_flight():
    node, _, _ = make(wall_correction=True)
    h.T[0] = 10.0
    for i, epoch in enumerate((0, 1)):
        m, s = corr_and_status(9.9 + 0.05 * i, correction_epoch=epoch)
        node.localizer_status_cb(s)
        node.odom_correction_cb(m)
    assert node.navigation_fault is None and node.corr_rx is None and not node.alignment_ready
    node, px4, pos = make(wall_correction=True)
    h.PX4_HOOK[0] = feed()
    h.fly(node, px4, pos, lambda: h.T[0] >= 3.0)
    h.arm(px4)
    h.fly(node, px4, pos, lambda: node.state == 'TAKEOFF')
    h.PX4_HOOK[0] = feed(correction_epoch=1)
    h.fly(node, px4, pos, lambda: node.state == 'FAULT', max_s=1.0)
    assert node.navigation_fault == 'localization epoch changed'


def test_a_localizer_source_epoch_fault_is_a_fault():
    node, px4, pos = make(wall_correction=True)
    h.PX4_HOOK[0] = feed(source_epoch_fault='source timestamp regressed; realignment required')
    h.fly(node, px4, pos, lambda: node.state == 'FAULT', max_s=1.0)
    assert 'regressed' in node.navigation_fault


def test_a_stale_correction_in_flight_holds_the_last_d_and_is_not_a_fault():
    node, px4, pos = make(wall_correction=True)
    h.PX4_HOOK[0] = feed()
    h.fly(node, px4, pos, lambda: h.T[0] >= 3.0)
    h.arm(px4)
    h.fly(node, px4, pos, lambda: node.state == 'ROUTE')
    h.PX4_HOOK[0] = None                              # the localizer goes silent
    held = node.corr
    wait(node, px4, pos, 5.0)
    assert node.navigation_fault is None and node.state == 'ROUTE' and node.corr == held


# -- capture-time map projection ----------------------------------------------------------------

def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
                     [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
                     [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)]])


def test_clouds_project_through_the_capture_time_tf_zero_and_correction():
    node, _, _ = make(start_bearing_deg=30.0)
    z0 = (0.4, -0.3, 0.05)
    d = (0.2, -0.1, math.radians(7.0))
    h.T[0] = 20.0
    node.raw_position = z0
    node.zero_launch_origin()
    node.corr = d
    node.remember_map_frame()
    # optical (z ahead, x right, y down) -> FLU, then pitched down 10 deg and yawed 25 deg
    optical = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
    R = Rotation.from_euler('zy', [25.0, 10.0], degrees=True).as_matrix() @ optical
    q = Rotation.from_matrix(R).as_quat()                       # x, y, z, w
    t = np.array([0.55, -0.15, 0.62])
    pts_c = np.array([[0.1, -0.05, 1.0], [-0.2, 0.1, 2.0], [0.0, 0.0, 1.5]])
    msg = cloud([tuple(p) for p in pts_c])
    msg.header.stamp = h.stamp(20.05)
    tf = NS(header=NS(stamp=h.stamp(20.05), frame_id='odom'), child_frame_id=FRAME,
            transform=NS(translation=NS(x=t[0], y=t[1], z=t[2]),
                         rotation=NS(x=q[0], y=q[1], z=q[2], w=q[3])))
    h.T[0] = 20.1
    assert np.allclose(quat_to_matrix(q), R)
    points, origin = node.project_cloud(msg, tf, 20.05)

    def expected(pc):
        raw = R @ pc + t
        m = raw - np.array(z0)
        c, s = math.cos(d[2]), math.sin(d[2])
        x, y = m[0] - d[0], m[1] - d[1]
        route = (c * x + s * y, -s * x + c * y, m[2])
        return odom_to_course(route, node.frame[0], node.frame[1], node.start_height)

    want = np.array([expected(p) for p in pts_c])
    keep = (want[:, 2] >= node.obstacle_min_height) & (want[:, 2] <= 1.70)
    assert keep.sum() >= 2
    assert np.allclose(points, want[keep], atol=1e-9)
    assert np.allclose(origin, expected(np.zeros(3)), atol=1e-9)    # the camera, not the body

    # Captured before this epoch's alignment (the arming tare): never projected.
    msg.header.stamp = tf.header.stamp = h.stamp(19.9)
    try:
        node.project_cloud(msg, tf, 19.9)
        raise AssertionError('projected without a capture-time alignment')
    except cm.CloudTransformError:
        pass
    # A TF for another frame is not this cloud's.
    msg.header.stamp = tf.header.stamp = h.stamp(20.05)
    tf.child_frame_id = 'base_link'
    try:
        node.project_cloud(msg, tf, 20.05)
        raise AssertionError('projected through the wrong TF')
    except cm.CloudTransformError:
        pass


def test_no_tf_means_no_map_insertion():
    node, px4, pos = make()
    h.fly(node, px4, pos, lambda: h.T[0] >= 1.0)
    assert node.cloud_tf is None                     # no tf2 here, as with a missing TF tree
    msg = cloud([(0.0, 0.0, 1.0)])
    msg.header.stamp = h.stamp()
    node.pointcloud_cb(msg)
    assert not node.live_grid_updated and 'cloud_tf_missing' in node.log_once


def test_background_results_from_an_old_epoch_are_discarded():
    node, _, _ = make()
    route = list(node.route)
    node._bg_worker_enabled = True                    # results only; no threads started
    try:
        node._bg_replan_result = (0.0, [(9.0, 9.0, 0.8), (9.5, 9.0, 0.8)], 'STALE',
                                  node._mapping_epoch - 1)
        node.maybe_replan_for_discrepancies(1.0)
        assert node.route == route and node.route_kind != 'STALE'
        node._bg_replan_result = (0.0, [(1.0, 0.0, 0.8), (1.5, 0.0, 0.8)], 'FRESH',
                                  node._mapping_epoch)
        node.maybe_replan_for_discrepancies(1.0)
        assert node.route_kind == 'FRESH'
    finally:
        node._bg_worker_enabled = False


def test_navigation_status_is_published_every_tick():
    node, px4, pos = make(wall_correction=True)
    h.fly(node, px4, pos, lambda: h.T[0] >= 1.0)
    assert len(h.STATUS) >= 9
    last = h.STATUS[-1]
    assert last['state'] == 'WAIT_FOR_OFFBOARD' and last['fault_latched'] is False
    assert last['vio_ok'] is True and last['alignment_ready'] is False
    json.dumps(last)



# -- independent-review regressions -------------------------------------------------------------

def test_one_invalid_localizer_status_during_the_settle_restarts_it_without_crashing():
    node, px4, pos = make(wall_correction=True)
    bad = [False]

    def hook(n):
        h.px4_heading(n)
        if bad[0]:
            bad[0] = False
            wc.localize(n, D, correction_valid=False)       # one rejected cloud
        else:
            wc.localize(n, D)
    h.PX4_HOOK[0] = hook
    h.fly(node, px4, pos, lambda: h.T[0] >= 1.0)
    bad[0] = True
    wait(node, px4, pos, 0.3)          # the TypeError crashed here before
    t = h.T[0]
    h.fly(node, px4, pos, lambda: node.alignment_ready, max_s=4.0)
    assert h.T[0] - t >= node.initial_alignment_settle - 0.15   # settled again from scratch


def gap_then(node, px4, pos, gap_s, jump):
    """In ROUTE: odometry stops for gap_s (VIO lost), then returns `jump` off the truth."""
    h.fly(node, px4, pos, lambda: node.state == 'ROUTE')
    h.fly(node, px4, pos, lambda: node.carrot.index >= 3)
    h.VO_STATE[0] = 2
    send = h.send_odom
    h.send_odom = lambda *a, **k: None
    try:
        wait(node, px4, pos, gap_s)
    finally:
        h.send_odom = send
    h.VO_STATE[0] = 1
    h.ODOM_OFFSET[0] = lambda t: jump
    wait(node, px4, pos, 0.3)
    h.ODOM_OFFSET[0] = None


def test_a_relocalised_pose_after_a_gap_is_a_fault_not_a_resume():
    # 1.5 s gap: the speed bound alone (2 m/s * 1.5 s) would let this 1 m step through
    node, px4, pos = armed()
    gap_then(node, px4, pos, 1.5, (1.0, 0.0, 0.0))
    assert node.state == 'FAULT' and 'after a gap' in node.navigation_fault


def test_a_genuine_gap_that_agrees_with_px4_resumes():
    node, px4, pos = armed()
    gap_then(node, px4, pos, 1.5, (0.05, 0.0, 0.0))
    assert node.navigation_fault is None and node.state == 'ROUTE'


def test_a_route_vio_loss_is_bounded():
    node, px4, pos = armed()
    h.fly(node, px4, pos, lambda: node.state == 'ROUTE')
    h.VO_STATE[0] = 2
    t = h.T[0]
    h.fly(node, px4, pos, lambda: node.state == 'FAULT', max_s=6.0)
    assert node.blind_max_s <= h.T[0] - t <= node.blind_max_s + node.odom_timeout + 0.3


def test_a_descending_fault_never_resumes_twists_after_disarm():
    node, px4, pos = armed()
    h.fly(node, px4, pos, lambda: node.state == 'LAND')
    tj = h.T[0]
    h.ODOM_OFFSET[0] = lambda t: (0.0, 0.9, 0.0) if t >= tj else (0.0, 0.0, 0.0)
    h.fly(node, px4, pos, lambda: node.state == 'FAULT', max_s=1.0)
    px4.update(armed=False, offboard_active=False)
    assert collect(node, px4, pos, 1.0) == []
    px4.update(armed=True)                         # pilot re-arms (manual, no offboard)
    assert collect(node, px4, pos, 1.0) == [] and not node.fault_descending


def test_status_jitter_does_not_step_the_drift_offset():
    node, px4, pos = armed(drift_seed_body_y_m=-0.30)
    h.fly(node, px4, pos, lambda: node.state == 'TAKEOFF')
    h.SENT.clear()
    xs = []

    def until():
        if h.SENT and h.SENT[-1][0] == cm.POSE_TOPIC:
            xs.append(h.SENT[-1][1].pose.position.y)
        return len(xs) >= 12
    h.VO_STATE[0] = None                           # status stops for 0.3 s: past 0.25, under 0.5
    wait(node, px4, pos, 0.3)
    h.VO_STATE[0] = 1
    h.fly(node, px4, pos, until, max_s=2.0)
    assert node.navigation_fault is None and node.drift_seeded
    # the 0.30 m lean stays applied: no setpoint step between ticks (learning moves it slowly)
    assert max(abs(b - a) for a, b in zip(xs, xs[1:])) < 0.03



def test_takeover_readiness_is_logged_once_per_change_without_commas():
    node, px4, pos = make()
    lines = []
    logger = node.get_logger
    node.get_logger = lambda: types.SimpleNamespace(
        info=lambda m: lines.append(m), warn=lambda m: lines.append(m), error=lambda m: lines.append(m))
    h.fly(node, px4, pos, lambda: h.T[0] >= 3.0)
    h.VO_STATE[0] = 2
    wait(node, px4, pos, 0.5)
    node.get_logger = logger
    ready = [m for m in lines if 'ready for takeover' in m]
    assert ready[0].startswith('ready for takeover:') and ready[1].startswith('not ready for takeover:') and 'VIO' in ready[1]
    assert len(ready) == 2 and not any(',' in m or 'pre-arm' in m for m in ready)



def test_a_drone_registered_beyond_the_tracking_limit_never_starts_the_route():
    node, px4, pos = make(wall_correction=True)
    h.PX4_HOOK[0] = feed(d=(0.30, 0.20, 0.0))       # 0.36 m off the launch mark, by the walls
    h.fly(node, px4, pos, lambda: h.T[0] >= 3.0)
    h.arm(px4)
    h.fly(node, px4, pos, lambda: node.state not in ('WAIT_FOR_OFFBOARD', 'TAKEOFF'), max_s=40.0)
    assert node.state == 'HOLD' and node.carrot is None


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
    print('ok')
