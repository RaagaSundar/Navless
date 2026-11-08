"""Software unit tests only. ROS and publishers are stubs; no vehicle or flight is run."""
import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest


class Message:
    ARMING_STATE_ARMED = 2
    NAVIGATION_STATE_OFFBOARD = 14
    NAVIGATION_STATE_AUTO_LAND = 18
    VEHICLE_CMD_NAV_LAND = 21
    VEHICLE_CMD_DO_SET_MODE = 176
    VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
    MESSAGE_VERSION = 1
    POSE_FRAME_NED = 1
    POSE_FRAME_FRD = 2


class Node:
    def __init__(self, name):
        self.now = 100.0

    def create_publisher(self, *args):
        messages = []
        return NS(messages=messages, publish=messages.append)

    def create_subscription(self, *args):
        pass

    def create_timer(self, *args):
        pass

    def declare_parameter(self, name, value):
        return NS(value=value)

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=int(self.now * 1e9)))

    def get_logger(self):
        return NS(info=lambda *a: None, error=lambda *a: None, warning=lambda *a: None)


@pytest.fixture(params=['hover_yaw_180_land', 'forward_2m_return_land', 'hover_land'])
def example(request, monkeypatch):
    for name in ['rclpy', 'rclpy.node', 'rclpy.qos', 'px4_msgs', 'px4_msgs.msg', 'nav_msgs', 'nav_msgs.msg', 'std_msgs', 'std_msgs.msg']:
        monkeypatch.setitem(sys.modules, name, ModuleType(name))
    sys.modules['rclpy.node'].Node = Node
    sys.modules['nav_msgs.msg'].Odometry = Message
    sys.modules['std_msgs.msg'].Bool = Message
    qos = sys.modules['rclpy.qos']
    qos.QoSProfile = lambda **kw: kw
    for name in ['ReliabilityPolicy', 'DurabilityPolicy', 'HistoryPolicy']:
        setattr(qos, name, NS(BEST_EFFORT=1, TRANSIENT_LOCAL=1, VOLATILE=2, KEEP_LAST=1))
    for name in ['OffboardControlMode', 'TrajectorySetpoint', 'VehicleCommand',
                 'VehicleLocalPosition', 'VehicleStatus', 'VehicleLandDetected', 'EstimatorStatusFlags', 'VehicleOdometry']:
        setattr(sys.modules['px4_msgs.msg'], name, Message)
    path = Path(__file__).parents[1] / 'examples' / (request.param + '.py')
    spec = importlib.util.spec_from_file_location(request.param, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def fusion_flags(**overrides):
    values = dict(timestamp=100_000_000, cs_yaw_align=True, cs_ev_pos=True,
                  cs_ev_yaw=True, cs_ev_yaw_fault=False, cs_inertial_dead_reckoning=False,
                  reject_hor_pos=False, reject_yaw=False)
    return NS(**(values | overrides))


def slam_sample(stamp=100.0):
    sec = int(stamp)
    return NS(header=NS(stamp=NS(sec=sec, nanosec=round((stamp-sec)*1e9)), frame_id='odom'),
              child_frame_id='base_link', pose=NS(pose=NS(
                  position=NS(x=1.0, y=2.0, z=3.0), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))))


def vision_sample(stamp=100.0):
    return NS(timestamp_sample=round(stamp*1e6), timestamp=round(stamp*1e6),
              pose_frame=1, position=[1.0, 2.0, 3.0], q=[1.0, 0.0, 0.0, 0.0], reset_counter=0)


def ready(module, phase='WAIT'):
    node = module.TestFlight()
    node.on_position(NS(x=4.0, y=-3.0, z=0.3, heading=math.pi / 2,
                        xy_valid=True, z_valid=True, heading_good_for_control=True, dead_reckoning=False,
                        timestamp=100_000_000, xy_reset_counter=0, z_reset_counter=0,
                        heading_reset_counter=0))
    node.on_status(NS(arming_state=0 if phase in ('WAIT', 'PRIME') else 2, nav_state=14))
    node.on_landed(NS(landed=phase in ('WAIT', 'PRIME')))
    node.on_estimator_flags(fusion_flags())
    node.on_slam_odometry(slam_sample())
    node.on_vision_odometry(vision_sample())
    node.on_vision_health(NS(data=True))
    node.phase = phase
    node.x, node.y, node.target_z = 4.0, -3.0, -0.7
    node.home_x, node.home_y, node.initial_yaw, node.yaw = 4.0, -3.0, math.pi / 2, math.pi / 2
    node.last_motion_time = node.now
    return node


def test_preflight_ground_and_climb_target(example):
    n = ready(example)
    n.landed = None
    n.tick()
    assert n.phase == 'WAIT' and not n.command_pub.messages and not n.setpoint_pub.messages
    n.on_landed(NS(landed=True))
    n.tick()
    assert n.phase == 'PRIME'
    assert n.target_z == pytest.approx(-0.7)
    assert (n.home_x, n.home_y, n.initial_yaw) == (4.0, -3.0, math.pi / 2)


@pytest.mark.parametrize('phase', ['CLIMB', 'HOVER', 'YAW', 'OUTBOUND', 'FORWARD_HOLD', 'RETURN', 'LAND'])
@pytest.mark.parametrize('failure', ['stale', 'takeover'])
def test_yield_without_publication(example, phase, failure):
    n = ready(example, phase)
    if failure == 'stale':
        n.status_time = n.now - 1.1
    else:
        n.status.nav_state = 3
    n.tick()
    assert n.phase == 'ABORT'
    assert not n.setpoint_pub.messages and not n.offboard_pub.messages and not n.command_pub.messages


def test_invalid_estimate_during_maneuver(example):
    n = ready(example, 'OUTBOUND')
    n.pos.xy_valid = False
    n.tick()
    assert n.phase == 'ABORT' and not n.setpoint_pub.messages


def test_estimator_reset_yields(example):
    n = ready(example, 'RETURN')
    n.pos.xy_reset_counter = 1
    n.on_position(n.pos)
    n.tick()
    assert n.phase == 'ABORT' and not n.setpoint_pub.messages


def test_forward_geometry_wait_and_return(example):
    n = ready(example, 'OUTBOUND')
    n.pos.z = n.target_z
    for i in range(1, 62):
        n.now = 100.0 + i * .1
        n.advance_maneuver(n.now)
        n.pos.x, n.pos.y = n.x, n.y
    assert n.x == pytest.approx(4.0)
    assert n.y == pytest.approx(-1.0)
    assert n.phase == 'FORWARD_HOLD'
    # The hold is based on measured arrival, and restarts after leaving tolerance.
    n.advance_maneuver(107.0)
    n.pos.x += .3
    n.advance_maneuver(108.0)
    n.pos.x = n.x
    n.advance_maneuver(109.0)
    n.advance_maneuver(118.99)
    assert n.phase == 'FORWARD_HOLD'
    n.advance_maneuver(119.0)
    assert n.phase == 'RETURN'
    for i in range(1, 63):
        n.advance_maneuver(119.0 + i * .1)
        n.pos.x, n.pos.y = n.x, n.y
    assert n.phase == 'LAND'
    assert n.x == pytest.approx(4.0) and n.y == pytest.approx(-3.0)
    assert n.yaw == math.pi / 2


def test_yaw_wrap_and_measured_completion(example):
    n = ready(example, 'YAW')
    n.initial_yaw = math.radians(179)
    n.yaw = n.pos.heading = n.initial_yaw
    n.pos.z = n.target_z
    previous = n.yaw
    ramp_ticks = math.ceil(math.pi / example.YAW_RATE * 10) + 2
    for i in range(1, ramp_ticks):
        n.advance_maneuver(100.0 + i * .1)
        delta = math.atan2(math.sin(n.yaw - previous), math.cos(n.yaw - previous))
        assert -1e-12 <= delta <= example.YAW_RATE * .1 + 1e-9
        previous = n.yaw
    assert n.yaw == pytest.approx(math.radians(-1.0))
    assert n.phase == 'YAW'  # A completed command ramp does not prove aircraft yaw.
    n.pos.heading = n.yaw
    settle_start = math.ceil(100.0 + ramp_ticks * .1)
    n.advance_maneuver(settle_start)
    n.advance_maneuver(settle_start + example.HOVER_TIME - .01)
    assert n.phase == 'YAW'
    n.advance_maneuver(settle_start + example.HOVER_TIME)
    assert n.phase == 'LAND'


def test_landing_requires_fresh_ground(example):
    n = ready(example, 'LAND')
    n.status.arming_state = 0
    n.status.nav_state = 18
    n.landed = True
    n.landed_time = 90.0
    n.tick()
    assert n.phase == 'LAND' and not n.command_pub.messages
    n.on_landed(NS(landed=True))
    n.tick()
    assert n.phase == 'DONE'


@pytest.mark.parametrize('missing', ['slam', 'bridge', 'health', 'flags'])
@pytest.mark.parametrize('phase', ['WAIT', 'PRIME', 'ACTIVATE', 'YAW', 'OUTBOUND', 'LAND'])
def test_vision_gate_blocks_before_any_publication(example, missing, phase):
    n = ready(example, phase)
    if missing == 'slam':
        n.slam_time = None
    elif missing == 'bridge':
        n.vision_time = None
    elif missing == 'health':
        n.vision_healthy = False
    else:
        n.estimator_flags = None
    n.tick()
    assert n.phase == ('WAIT' if phase in ('WAIT', 'PRIME') else 'ABORT')
    assert not n.setpoint_pub.messages and not n.offboard_pub.messages and not n.command_pub.messages


@pytest.mark.parametrize('flag,value', [('cs_ev_pos', False), ('cs_ev_yaw', False),
                                      ('cs_ev_yaw_fault', True), ('cs_inertial_dead_reckoning', True),
                                      ('reject_hor_pos', True), ('reject_yaw', True)])
def test_px4_fusion_required_even_with_valid_local_position(example, flag, value):
    n = ready(example)
    n.on_estimator_flags(fusion_flags(**{flag: value}))
    assert n.pos.xy_valid and n.pos.heading_good_for_control
    assert n.vision_problem(n.now) is not None
    n.tick()
    assert n.phase == 'WAIT' and not n.command_pub.messages


@pytest.mark.parametrize('which', ['slam', 'bridge'])
def test_duplicate_capture_does_not_refresh_vision(example, which):
    n = ready(example, 'OUTBOUND')
    n.now = 100.6
    if which == 'slam':
        n.on_slam_odometry(slam_sample())
        assert n.slam_time == 100.0
    else:
        n.on_vision_odometry(vision_sample())
        assert n.vision_time == 100.0
    assert n.vision_problem(n.now) is not None
    n.tick()
    assert n.phase == 'ABORT' and not n.setpoint_pub.messages


@pytest.mark.parametrize('age', [-0.1, 0.6, 100.0])
@pytest.mark.parametrize('which', ['slam', 'bridge'])
def test_future_stale_zero_capture_rejected(example, which, age):
    n = ready(example)
    stamp = n.now - age
    if which == 'slam':
        n.on_slam_odometry(slam_sample(stamp))
    else:
        n.on_vision_odometry(vision_sample(stamp))
    n.tick()
    assert n.phase == 'WAIT' and not n.setpoint_pub.messages


@pytest.mark.parametrize('which', ['slam', 'bridge'])
def test_invalid_pose_rejected(example, which):
    n = ready(example)
    if which == 'slam':
        sample = slam_sample()
        sample.pose.pose.position.x = math.nan
        n.on_slam_odometry(sample)
    else:
        sample = vision_sample()
        sample.q = [0.0] * 4
        n.on_vision_odometry(sample)
    n.tick()
    assert n.phase == 'WAIT' and not n.command_pub.messages


def test_stale_flags_not_bypassed_by_good_heading(example):
    n = ready(example)
    n.yaw_alignment_time = 97.0
    n.tick()
    assert n.phase == 'WAIT' and not n.setpoint_pub.messages
    n.on_estimator_flags(fusion_flags(timestamp=97_000_000))
    n.tick()
    assert n.phase == 'WAIT' and not n.setpoint_pub.messages


def test_bridge_reset_or_slam_frame_change_aborts(example):
    for which in ('slam', 'bridge'):
        n = ready(example, 'RETURN')
        n.now = 100.1
        if which == 'slam':
            sample = slam_sample(100.1)
            sample.header.frame_id = 'new_odom'
            n.on_slam_odometry(sample)
        else:
            sample = vision_sample(100.1)
            sample.reset_counter = 1
            n.on_vision_odometry(sample)
        assert n.phase == 'ABORT'
        n.tick()
        assert not n.setpoint_pub.messages


def test_gate_recovery_requires_new_priming_interval(example):
    n = ready(example)
    n.tick()
    assert n.phase == 'PRIME'
    n.vision_healthy = False
    n.tick()
    assert n.phase == 'WAIT'
    n.now = 100.2
    n.on_vision_health(NS(data=True))
    n.tick()
    assert n.phase == 'PRIME' and n.phase_start == pytest.approx(100.2)
    assert not n.command_pub.messages


def test_gate_does_not_interrupt_px4_auto_landing(example):
    n = ready(example, 'LAND')
    n.status.nav_state = 18
    n.vision_healthy = False
    n.tick()
    assert n.phase == 'LAND' and n.auto_land_observed
    assert not n.setpoint_pub.messages and not n.command_pub.messages


def test_valid_pipeline_allows_mode_and_arm_request(example):
    n = ready(example)
    n.tick()
    for i in range(1, 22):
        n.now = 100.0 + i * .1
        stamp = round(n.now * 1e6)
        n.pos.timestamp = stamp
        n.on_position(n.pos)
        n.on_status(n.status)
        n.on_landed(NS(landed=True))
        n.on_estimator_flags(fusion_flags(timestamp=stamp))
        n.on_slam_odometry(slam_sample(n.now))
        n.on_vision_odometry(vision_sample(n.now))
        n.on_vision_health(NS(data=True))
        n.tick()
    assert n.phase == 'ACTIVATE'
    assert [m.command for m in n.command_pub.messages] == [176, 400]


@pytest.mark.parametrize('which', ['slam', 'bridge', 'position'])
def test_reset_during_prime_recaptures_home(example, which):
    n = ready(example)
    n.tick()
    assert n.phase == 'PRIME'
    n.now = 100.1
    if which == 'slam':
        sample = slam_sample(100.1)
        sample.header.frame_id = 'new_odom'
        n.on_slam_odometry(sample)
    elif which == 'bridge':
        sample = vision_sample(100.1)
        sample.reset_counter = 1
        n.on_vision_odometry(sample)
    else:
        n.pos.xy_reset_counter = 1
        n.pos.x = 7.0
        n.on_position(n.pos)
    assert n.phase == 'WAIT'
    n.tick()
    assert n.phase == 'PRIME' and n.phase_start == pytest.approx(100.1)
    assert n.home_x == n.pos.x
    assert not n.command_pub.messages


def test_requested_hover_plan(example):
    n = ready(example, 'HOVER')
    n.pos.z = n.target_z
    original = (n.x, n.y, n.yaw)
    for i in range(round(example.HOVER_TIME * 10) + 1):
        n.now = 100.0 + i * .1
        stamp = round(n.now * 1e6)
        n.pos.timestamp = stamp
        n.on_position(n.pos)
        n.on_status(n.status)
        n.on_estimator_flags(fusion_flags(timestamp=stamp))
        n.on_slam_odometry(slam_sample(n.now))
        n.on_vision_odometry(vision_sample(n.now))
        n.on_vision_health(NS(data=True))
        n.tick()
    assert (n.x, n.y, n.yaw) == original
    assert n.phase == {'hover': 'LAND', 'yaw': 'YAW', 'out_back': 'OUTBOUND'}[example.MISSION]


def test_diagnostic_distinguishes_future_sample_from_lost_stream(example):
    import json
    n = ready(example)
    n.on_slam_odometry(slam_sample(100.1))
    detail = json.loads(n.vision_diagnostics(n.now))
    assert detail['slam_valid'] is False
    assert detail['latest_slam_capture_age_s'] == pytest.approx(-.1)
    assert detail['latest_slam_receipt_age_s'] == 0
    n.now = 100.6
    detail = json.loads(n.vision_diagnostics(n.now))
    assert detail['accepted_slam_capture_age_s'] == pytest.approx(.6)
    assert n.vision_problem(n.now) is not None


def test_passive_checker_has_no_control_publishers(example, monkeypatch):
    import importlib.util
    path = Path(__file__).parents[1] / 'check_odometry.py'
    spec = importlib.util.spec_from_file_location('passive_checker', path)
    checker = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checker)
    node = checker.OdometryCheck(30.)
    for publisher in (node.offboard_pub, node.setpoint_pub, node.command_pub):
        assert isinstance(publisher, checker.NoPublish)
        with pytest.raises(RuntimeError, match='Passive checker'):
            publisher.publish(Message())
    events = node.create_publisher(Message, '/parameter_events', 10)
    events.publish(Message())
    assert len(events.messages) == 1  # ROS node construction must remain usable.
    assert isinstance(node.create_publisher(Message, '/fmu/in/vehicle_visual_odometry', 10), checker.NoPublish)
    assert node.problem() == 'PX4 telemetry missing'
    node.tick()
    assert node.phase == 'WAIT'
    monkeypatch.setattr(checker.time, "monotonic", lambda: node.started + 30.1)
    node.problem = lambda: None
    node.ready_since = node.started
    node.last_ros_time = node.clock_seconds() - .1
    node.tick()
    assert node.passed and node.phase == 'DONE'
    node.phase = 'WAIT'
    node.ready_since = node.started
    monkeypatch.setattr(checker.time, "monotonic", lambda: node.started + 33.0)
    node.tick()
    assert not node.passed and node.phase == 'ABORT'


def test_nominal_one_hz_flags_survive_delivery_jitter(example):
    n = ready(example, 'CLIMB')
    # Keep all high-rate sources fresh while the next 1 Hz flags packet is in transit.
    n.now = 101.125
    n.pos.timestamp = 101_125_000
    n.on_position(n.pos)
    n.on_status(n.status)
    n.on_slam_odometry(slam_sample(n.now))
    n.on_vision_odometry(vision_sample(n.now))
    n.on_vision_health(NS(data=True))
    assert n.vision_problem(n.now) is None
    n.tick()
    assert n.phase == 'CLIMB'
    assert n.setpoint_pub.messages


@pytest.mark.parametrize('which', ['slam', 'bridge'])
def test_existing_bridge_five_ms_clock_skew_budget(example, which):
    n = ready(example)
    if which == 'slam':
        n.on_slam_odometry(slam_sample(100.004))
    else:
        msg = vision_sample(100.004)
        msg.timestamp = 100_000_000
        n.on_vision_odometry(msg)
    assert n.vision_problem(n.now) is None


def test_flags_silence_still_expires_and_duplicate_does_not_renew(example):
    n = ready(example)
    n.now = 101.1
    n.on_estimator_flags(fusion_flags())
    assert n.yaw_alignment_time == 100.
    n.now = 102.625
    n.pos.timestamp = 102_625_000
    n.on_position(n.pos)
    n.on_status(n.status)
    n.on_slam_odometry(slam_sample(n.now))
    n.on_vision_odometry(vision_sample(n.now))
    n.on_vision_health(NS(data=True))
    assert 'fusion status' in n.vision_problem(n.now)
    n.tick()
    assert n.phase == 'WAIT'
    assert not n.command_pub.messages


def test_actual_position_dead_reckoning_remains_a_hard_failure(example):
    n = ready(example, 'OUTBOUND')
    n.pos.dead_reckoning = True
    n.tick()
    assert n.phase == 'ABORT'
    assert not n.setpoint_pub.messages and not n.command_pub.messages


@pytest.mark.parametrize('which', ['slam', 'bridge'])
def test_skew_outside_bridge_budget_still_rejected(example, which):
    n = ready(example)
    if which == 'slam': n.on_slam_odometry(slam_sample(100.006))
    else: n.on_vision_odometry(vision_sample(100.006))
    assert n.vision_problem(n.now) is not None
    n.tick()
    assert not n.command_pub.messages
