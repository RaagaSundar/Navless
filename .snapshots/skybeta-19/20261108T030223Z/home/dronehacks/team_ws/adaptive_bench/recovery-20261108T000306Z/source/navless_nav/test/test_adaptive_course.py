"""ROS boundary/worker tests with message/TF stand-ins, never flight evidence."""
from concurrent.futures import Future
import json
import math
from pathlib import Path
import sys
import time
import types

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navless_nav.adaptive_course import AdaptiveCourse
from navless_nav.adaptive_route import RouteUpdate
from navless_nav.route import Carrot, course_to_odom

ROOT = Path(__file__).resolve().parents[2]
NS = types.SimpleNamespace


def stamp(t):
    return NS(sec=int(t), nanosec=round((t-int(t))*1e9))


class Mission:
    def __init__(self):
        data = json.loads((ROOT/'gazebo_sim/course/route_centreline_window_asbuilt.json').read_text())
        self.frame = (tuple(data['launch_origin_enu'][:2]), 0.)
        self.start_height = .07
        self.route = [course_to_odom(p, self.frame[0], 0, .07) for p in data['waypoints_enu']]
        self.position = self.route[0]
        self.odom_zero_offset, self.odom_zeroed = (0., 0., 0.), True
        self.now, self.vo_at, self.state_at = 1.2, 1.2, 1.2
        self.vo_state, self.armed, self.offboard_active, self.landed = 1, True, True, False
        self.state = 'ROUTE'
        self.cruise_speed, self.max_lead, self.max_lat_accel = .3, .1, .12
        self.carrot = Carrot(self.route, .3, .1)
        self.last_tick, self.land_at_end = 1.1, True
        self.sent, self.messages = [], []

    def get_logger(self):
        return NS(info=self.messages.append, warn=self.messages.append, error=self.messages.append)

    def now_s(self):
        return self.now

    def create_timer(self, *_):
        pass

    def send(self, *p):
        self.sent.append(tuple(p))

    def goto(self, state, *_):
        self.state = state

    def odom(self, t, position=None, q=(0., 0., 0., 1.)):
        p = self.position if position is None else position
        return NS(header=NS(stamp=stamp(t), frame_id='odom'), child_frame_id='base_link',
                  pose=NS(pose=NS(position=NS(x=p[0], y=p[1], z=p[2]),
                                  orientation=NS(x=q[0], y=q[1], z=q[2], w=q[3]))))


@pytest.fixture
def session(monkeypatch):
    transform = NS(transform=NS(translation=NS(x=0., y=0., z=0.),
                               rotation=NS(x=0., y=0., z=0., w=1.)))
    tf = NS(lookup_transform=lambda *_: transform)
    monkeypatch.setitem(sys.modules, 'tf2_ros', NS(Buffer=lambda: tf,
        TransformListener=lambda *_: None, TransformException=RuntimeError))
    monkeypatch.setitem(sys.modules, 'rclpy.time', NS(Time=NS(from_msg=lambda x: x)))
    monkeypatch.setitem(sys.modules, 'sensor_msgs_py', NS(point_cloud2=NS(
        read_points_numpy=lambda msg, **_: msg.points)))
    n = Mission()
    a = AdaptiveCourse(n, str(ROOT/'gazebo_sim/course/course_asbuilt.json'),
                       str(ROOT/'gazebo_sim/course/route_centreline_window_asbuilt.json'))
    n.carrot = Carrot(n.route, .3, .1)  # real mission creates it after preflight setup
    for t in (1., 1.1, 1.2):
        a.observe_odom(n.odom(t))
    yield n, a
    a.close()


def test_acquisition_age_and_authority_loss_stop_publication(session):
    n, a = session
    assert a.healthy(n.now)
    n.now = 1.5
    a.route_tick(n.now)
    assert not n.sent
    n.now, n.vo_at, n.state_at = 1.3, 1.3, 1.3
    a.observe_odom(n.odom(1.3))
    n.offboard_active = False
    a.route_tick(n.now)
    assert not n.sent and 'authority' in a.fault
    n.offboard_active = True
    assert not a.healthy(n.now)  # no automatic takeover after a lost-authority episode


def test_missing_depth_holds_a_fixed_point_instead_of_chasing_drifting_odom(session):
    n, a = session
    a.route_tick(n.now)
    first = n.sent[-1]
    n.position = (n.position[0]+.01, n.position[1], n.position[2])
    n.now, n.vo_at, n.state_at = 1.3, 1.3, 1.3
    a.observe_odom(n.odom(1.3))
    a.route_tick(n.now)
    assert n.sent[-1] == first
    assert a.future is None


def test_odometry_reset_latches_fault_and_never_reuses_old_obstacle_map(session):
    n, a = session
    n.now = 1.3
    a.observe_odom(n.odom(1.3, (n.position[0]+1, n.position[1], n.position[2])))
    assert 'discontinuity' in a.fault
    a.route_tick(n.now)
    assert not n.sent


def test_attitude_outside_explicit_vehicle_envelope_refuses_control(session):
    n, a = session
    # A 6-degree yaw change in .1 seconds is continuous but exceeds the 5-degree envelope.
    n.now = 1.3
    angle = math.radians(6)
    a.observe_odom(n.odom(1.3, q=(0, 0, math.sin(angle/2), math.cos(angle/2))))
    assert 'attitude' in a.fault


def test_source_frame_mismatch_refuses_control(session):
    n, a = session
    msg = n.odom(1.3)
    msg.header.frame_id = 'map'
    n.now = 1.3
    a.observe_odom(msg)
    assert 'frame' in a.fault


def test_height_above_range_beam_assumption_refuses_control(session):
    n, a = session
    # Smooth climb under the history jump limit but over the model's 1.1m bound.
    for t, z in ((1.3, .93), (1.4, 1.04)):
        n.now = t
        a.observe_odom(n.odom(t, (n.position[0], n.position[1], z)))
    assert 'height' in a.fault


def test_cloud_needs_bracketing_odometry_and_measured_tf(session):
    n, a = session
    # Camera stand-in aligned with base; real TF is read at the cloud's source stamp.
    message = NS(header=NS(stamp=stamp(1.15), frame_id='left_optical'),
                 points=np.repeat([[.6, 0, 0]], 40, axis=0))
    a.cloud(message)
    a.process_cloud()
    assert a.valid_cloud_at == pytest.approx(1.15)
    assert len(a.depth.pending) == 1
    # Repeating the same dense cloud does not confirm an obstacle.
    a.cloud(message)
    a.process_cloud()
    assert a.valid_cloud_at is None and not a.depth.confirmed


def test_unbracketed_or_missing_tf_cloud_cannot_enable_motion(session):
    n, a = session
    message = NS(header=NS(stamp=stamp(1.21), frame_id='left_optical'),
                 points=np.repeat([[.6, 0, 0]], 40, axis=0))
    n.now = 1.22
    a.cloud(message)
    a.process_cloud()
    assert a.valid_cloud_at is None
    message.header.stamp = stamp(1.16)
    a.last_cloud = -math.inf
    a.tf.lookup_transform = lambda *_: (_ for _ in ()).throw(RuntimeError('no TF'))
    a.cloud(message)
    a.process_cloud()
    assert a.valid_cloud_at is None


def test_registration_change_after_mapping_latches_fault(session):
    n, a = session
    a.registration_offset = (0, 0, 0)
    n.odom_zero_offset = (.01, 0, 0)
    a.pending_cloud = NS(header=NS(stamp=stamp(1.15)))
    a.process_cloud()
    assert 'registration' in a.fault


def completed(update):
    f = Future()
    f.set_result(update)
    return f


def test_pending_worker_holds_without_blocking_and_stale_result_is_discarded(session):
    n, a = session
    a.valid_cloud_at = 1.2
    a.future = Future()  # intentionally not completed: controller must not wait on it
    begin = time.monotonic()
    a.route_tick(n.now)
    assert time.monotonic()-begin < .10
    assert n.sent[-1] == n.position
    a.future.set_result(RouteUpdate(list(a.route), a.anchors, True))
    a.request_revision, a.request_epoch = -1, 0
    a.request_position, a.request_wall = a.to_course([n.position])[0], time.monotonic()
    old = n.carrot
    a.route_tick(n.now)
    assert n.carrot is old


def test_preflight_checked_prior_does_not_start_redundant_whole_course_worker(session):
    n, a = session
    a.valid_cloud_at = n.now
    here = np.asarray(a.to_course([n.position])[0])
    a.camera_origin = here
    a.camera_returns = np.array([here+[x, 1., z] for x in np.linspace(-.05, .05, 21)
                                 for z in np.linspace(-.05, .05, 9)])
    old = n.carrot.point
    a.route_tick(n.now)
    assert a.checked_revision == 0 and a.future is None
    assert n.carrot.point != old


def test_fresh_worker_route_is_installed_without_skipping_to_pad(session):
    n, a = session
    a.valid_cloud_at = 1.2
    a.future = completed(RouteUpdate(list(a.route), a.anchors, True))
    a.request_revision, a.request_epoch = a.depth.revision, a.route_epoch
    a.request_position, a.request_wall = a.to_course([n.position])[0], time.monotonic()
    old = n.carrot
    a.route_tick(n.now)
    assert n.carrot is not old and n.carrot.index < 2
    assert len(a.anchors) >= 12 and a.checked_revision == a.depth.revision


def test_no_detour_result_holds_and_does_not_continue_old_route(session):
    n, a = session
    a.valid_cloud_at = 1.2
    a.future = Future()
    a.future.set_exception(ValueError('blocked required opening'))
    before = n.carrot.point
    a.route_tick(n.now)
    assert n.carrot.point == before and n.sent[-1] == n.position
    assert a.failed_revision == a.depth.revision


def test_launch_and_survey_source_identity_must_match(monkeypatch, session, tmp_path):
    n, a = session
    survey = tmp_path/'changed.json'
    survey.write_text((ROOT/'gazebo_sim/course/course_asbuilt.json').read_text()+'\n')
    with pytest.raises(ValueError, match='exact as-built'):
        AdaptiveCourse(n, str(survey), str(ROOT/'gazebo_sim/course/route_centreline_window_asbuilt.json'))
