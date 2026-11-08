"""Geometry, odometry reset and bounded guidance regression tests (no aircraft)."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navless_nav.stereo_tube import OdomHistory, fit_tube, rotation
from navless_nav.tube_guidance import TubeGuidance


def walls(offset=0.12, yaw=0.0, width=1.45):
    rng = np.random.default_rng(3)
    x = np.tile(np.linspace(-1.0, 1.0, 160), 2)
    y = np.repeat([-width/2, width/2], 160) + offset
    p = np.column_stack((x, y, rng.uniform(-0.08, 0.08, len(x))))
    p[:, :2] += rng.normal(0, 0.003, (len(x), 2))
    r = rotation((0, 0, math.sin(yaw/2), math.cos(yaw/2)))
    return p @ r.T


@pytest.mark.parametrize('yaw', [0, 0.5, -1.2, math.pi])
def test_two_wall_center_with_rotated_odom_and_outliers(yaw):
    pts = walls(yaw=yaw)
    rng = np.random.default_rng(4)
    pts = np.vstack((pts, rng.uniform(-1, 1, (40, 3)), [[np.nan]*3]))
    body = np.array([3, -2, 0.85])
    fit = fit_tube(pts+body, body, yaw)
    normal = np.array([-math.sin(yaw), math.cos(yaw), 0])
    assert fit is not None
    assert np.linalg.norm(fit.center - body - 0.12*normal) < 0.01
    assert fit.width == pytest.approx(1.45, abs=0.01)


@pytest.mark.parametrize('case', ['one_side', 'floor', 'narrow', 'rim', 'random', 'nonparallel'])
def test_rejects_unobservable_or_inconsistent_geometry(case):
    pts = walls()
    if case == 'one_side':
        pts = pts[:160]
    elif case == 'floor':
        pts[:, 2] = -0.7
    elif case == 'narrow':
        pts = walls(width=0.8)
    elif case == 'rim':
        pts[:, 0] *= 0.02
    elif case == 'random':
        pts = np.random.default_rng(7).uniform(-1, 1, (1500, 3))
    else:
        pts[160:, 1] += pts[160:, 0]*0.15
    assert fit_tube(pts, [0, 0, 0]) is None


def test_odom_interpolates_but_never_bridges_a_pose_reset():
    history = OdomHistory()
    for t in np.arange(0, 0.21, 0.02):
        assert history.add(t, [t, 0, 0], [0, 0, 0, 1])
    p, r = history.at(0.15)
    assert p[0] == pytest.approx(0.15)
    assert np.allclose(r, np.eye(3))
    assert history.at(0.22) is None
    assert not history.add(0.22, [2, 0, 0], [0, 0, 0, 1])
    assert history.epoch == 1 and history.at(0.15) is None
    for t in np.arange(0.24, 0.41, 0.02):
        assert history.add(t, [2+t, 0, 0], [0, 0, 0, 1])
    assert history.at(0.35) is not None
    assert not history.add(0.01, [0, 0, 0], [0, 0, 0, 1])


def test_odom_rejects_heading_jump_gap_and_bad_quaternion():
    for stamp, q in [(0.02, [0, 0, 1, 0]), (0.4, [0, 0, 0, 1]), (0.02, [0, 0, 0, 0])]:
        history = OdomHistory()
        history.add(0, [0, 0, 0], [0, 0, 0, 1])
        assert not history.add(stamp, [0, 0, 0], q)
        assert history.at(0) is None


def test_guidance_rate_limit_frame_and_stale_rejection():
    g = TubeGuidance()
    for i in range(21):
        t = i*0.1
        assert g.update(t, [0, 0.12, 0.85], 0, t)
        out = g.apply([0.4, 0, 0.85], [0, 0, 0.85], 0, t, enabled=True, max_lead=0.6)
        assert out[1] <= min(0.12, t*0.10)+1e-9
    assert out[1] == pytest.approx(0.12)
    assert out[0] == 0.4 and out[2] == 0.85
    assert g.apply([0.4, 0, 0.85], [0, 0, 0.85], 0, 2.4, enabled=True, max_lead=0.6)[1] == 0
    assert not g.update(3.1, [0, 0.12, 0.85], 0, 3.0)


def test_guidance_does_not_double_count_body_error_or_exceed_lead():
    g = TubeGuidance()
    for t in (0, 0.1, 0.2):
        g.update(t, [0, 0, 0.85], 0, t)
        out = g.apply([0.4, 0, 0.85], [0, 0.15, 0.85], 0, t, enabled=True, max_lead=0.3)
    assert math.dist(out, [0, 0.15, 0.85]) <= 0.3+1e-9
    assert out[1] >= 0  # no extra correction past the measured center


def test_guidance_drops_outside_zone_or_on_heading_mismatch():
    g = TubeGuidance()
    for yaw, enabled in [(0, False), (math.pi/2, True)]:
        g.update(1, [0, 0.12, 0.85], yaw, 1)
        assert g.apply([0.4, 0, 0.85], [0, 0, 0.85], 0, 1, enabled=enabled, max_lead=0.6) == (0.4, 0, 0.85)


def test_anchored_guidance_expires_without_restamping_and_preserves_height():
    g = TubeGuidance(timeout=3.0)
    assert g.update(0, [0, 0.12, 0.85], 0, 0)
    for t in (0, 0.1, 0.2, 0.3):
        out = g.apply([0.59, 0, 0.95], [0, 0, 0.85], 0, t, enabled=True, max_lead=0.6)
        assert out[2] == 0.95
        assert math.dist(out, [0, 0, 0.85]) <= 0.6+1e-9
    assert g.measurement[0] == 0
    assert g.apply([0.4, 0, 0.85], [0, 0, 0.85], 0, 3.01, enabled=True, max_lead=0.6)[1] == 0
    assert g.update(4, [0, 0.12, 0.85], 0, 4)
    assert g.apply([1.9, 0, 0.85], [1.6, 0, 0.85], 0, 4.1, enabled=True, max_lead=0.6)[1] == 0


def test_mission_rejects_delayed_or_wrong_frame_measurements():
    import test_tube_blind as h
    from types import SimpleNamespace as NS
    node, px4, pos = h.fly_to_tube(stereo_tube_centering=True)
    now = node.now_s()
    def message(stamp, frame='odom'):
        sec = int(stamp)
        return NS(header=NS(stamp=NS(sec=sec, nanosec=int((stamp-sec)*1e9)), frame_id=frame),
                  pose=NS(position=NS(x=pos[0], y=pos[1], z=pos[2]),
                          orientation=NS(x=0, y=0, z=0, w=1)))
    node.tube_center_cb(message(now, 'map'))
    assert node.tube_guidance.measurement is None
    node.tube_center_cb(message(now-3.1))
    assert node.tube_guidance.measurement is None
    node.tube_center_cb(message(now))
    assert node.tube_guidance.measurement is not None
    node.tube_status_cb(NS(data='{"valid": false}'))
    node.tube_center_cb(message(now-0.01))
    assert node.tube_guidance.measurement is None


def test_mission_stale_telemetry_stops_publication():
    import test_tube_blind as h
    node, px4, pos = h.fly_to_tube(stereo_tube_centering=True)
    h.h.T[0] += 0.6
    h.h.SENT.clear()
    node.tick()
    assert not h.h.SENT


def test_ros_adapter_uses_camera_tf_source_stamps_and_invalidates_on_loss(monkeypatch):
    """Exercise the real callbacks on an isolated fake ROS transport, no DDS."""
    from types import SimpleNamespace as NS
    from navless_nav import stereo_tube
    now = [1.0]
    published = {}
    def message():
        return NS(header=NS(stamp=None, frame_id=''),
                  pose=NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=0.)))
    class FakeNode:
        def __init__(self, *args):
            pass
        def declare_parameter(self, name, default):
            return NS(value=default)
        def create_publisher(self, cls, topic, *args):
            published[topic] = []
            return NS(publish=published[topic].append)
        def create_subscription(self, *args):
            pass
        def create_timer(self, *args):
            pass
        def get_clock(self):
            return NS(now=lambda: NS(nanoseconds=int(now[0]*1e9)))
        def destroy_node(self):
            pass
    # Optical axes: +x right, +y down, +z forward -> body FLU.
    qtf = (-0.5, 0.5, -0.5, 0.5)
    rot = rotation(qtf)
    mount = np.array([0.1053, 0., 0.0304])
    class Buffer:
        def lookup_transform(self, target, source, stamp):
            assert target == 'base_link' and source == 'left_optical'
            return NS(transform=NS(translation=NS(x=mount[0], y=mount[1], z=mount[2]),
                                   rotation=NS(x=qtf[0], y=qtf[1], z=qtf[2], w=qtf[3])))
    def stamp(t):
        return NS(sec=int(t), nanosec=round((t-int(t))*1e9))
    def exercise(node):
        node.status(NS(vo_state=1))  # real VisualSlamStatus has no header
        def odom(t, x=3):
            node.odom(NS(header=NS(stamp=stamp(t), frame_id='odom'), child_frame_id='base_link',
                         pose=NS(pose=NS(position=NS(x=x, y=-2., z=.85),
                                         orientation=NS(x=0., y=0., z=0., w=1.)))))
        for t in np.arange(.85, 1.001, .025):
            odom(t)
        raw = (walls() - mount) @ rot
        node.cloud(NS(header=NS(stamp=stamp(.96), frame_id='left_optical'), points=raw))
        node.process()
        result = published['/tube/stereo/centerline']
        assert len(result) == 1
        assert result[0].header.stamp.nanosec == 960000000
        assert result[0].pose.position.y == pytest.approx(-1.88, abs=.01)
        # Fresh images without two walls can retain the original anchor in
        # continuous odom. They must not publish a new "measured" center.
        now[0] = 1.05
        node.status(NS(vo_state=1))
        odom(1.025)
        odom(1.05)
        node.cloud(NS(header=NS(stamp=stamp(1.025), frame_id='left_optical'), points=np.empty((0, 3))))
        node.process()
        assert len(result) == 1 and node.anchor_stamp == pytest.approx(.96)
        assert '"observed": false' in published['/tube/stereo/status'][-1].data
        # Reset: immediately invalidate; do not transform old clouds with new pose.
        odom(1.06, x=5)
        assert node.anchor is None and len(result) == 1
        node.status(NS(vo_state=2))
        odom(1.07, x=5)  # repeated/frozen stream while tracking is lost
        assert len(node.history.samples) == 0 and len(result) == 1
    modules = {
        'rclpy': NS(init=lambda **kw: None, spin=exercise, ok=lambda: False),
        'rclpy.node': NS(Node=FakeNode),
        'rclpy.qos': NS(qos_profile_sensor_data=object()),
        'rclpy.time': NS(Time=NS(from_msg=lambda s: s)),
        'geometry_msgs.msg': NS(PoseStamped=message), 'nav_msgs.msg': NS(Odometry=object),
        'sensor_msgs.msg': NS(PointCloud2=object),
        'sensor_msgs_py': NS(point_cloud2=NS(read_points_numpy=lambda m, **kw: m.points)),
        'std_msgs.msg': NS(String=lambda: NS(data='')),
        'isaac_ros_visual_slam_interfaces.msg': NS(VisualSlamStatus=object),
        'tf2_ros': NS(Buffer=Buffer, TransformListener=lambda *a: None, TransformException=LookupError),
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    stereo_tube.main()


def test_stereo_launch_is_opt_in_and_passes_the_clock_and_odom_source():
    import test_course_launch as launch
    assert launch._arguments(launch.BASE)['stereo_tube_centering'] == 'false'
    assert launch._arguments(launch.COURSE)['stereo_tube_centering'] == 'false'
    assert {'stereo_tube_centering', 'use_sim_time', 'odometry_topic'} <= launch._forwarded(launch.COURSE)
    source = Path(launch.BASE).read_text()
    assert source.count("'use_sim_time': LaunchConfiguration('use_sim_time')") == 3


def test_real_mission_applies_only_in_tube_and_stops_on_authority_loss():
    import test_tube_blind as h
    node, px4, pos = h.fly_to_tube(stereo_tube_centering=True)
    assert '/tube/stereo/centerline' in h.h.SUBS
    # Carrot is in the tube; actual aircraft must also be inside before corrections.
    pos[:] = node.carrot.point
    for _ in range(5):
        t = h.h.T[0]
        center = [pos[0], pos[1]+0.1, pos[2]]
        node.tube_guidance.update(t, center, 0, t)
        node.tube_status_cb(type('D', (), {'data': '{"valid": true}'})())
        h.step(node, px4, pos)
    assert node.tube_guidance.shift > 0
    node.status_cb(type('S', (), {'vo_state': 2})())
    assert node.tube_guidance.measurement is None
    px4.update(offboard_active=False, landed=False)
    h.step(node, px4, pos)
    assert not h.h.SENT and node.state == 'PAUSED'
