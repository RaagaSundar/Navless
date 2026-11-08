#!/usr/bin/env python3
"""Receive-only navigation timing bench. Never arms, publishes, or starts a stack.

The JSONL distinguishes ROS capture age, source-clock gaps and wall receipt gaps.
PX4 boot timestamps are never subtracted from ROS time. A healthy observation is
not flight authorization, a motion test, or proof of map accuracy.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time


STREAMS = ('raw_odometry', 'validated_odometry', 'vio_status',
           'localizer_status', 'px4_local_position')
SAMPLE_LIMIT = 100_000


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def distribution(values):
    """Exact bounded-sample percentiles; missing measurements stay unknown."""
    values = sorted(values)
    if not values:
        return dict(count=0, max=None, p50=None, p95=None, p99=None)

    def percentile(p):
        index = (len(values) - 1) * p
        low = int(index)
        return values[low] + (values[min(low + 1, len(values) - 1)] - values[low]) * (index - low)

    return dict(count=len(values), max=values[-1], p50=percentile(.5),
                p95=percentile(.95), p99=percentile(.99))


def json_safe(value):
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def header_stamp(msg):
    """ROS header only: never guess units or replace a missing stamp with arrival."""
    stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
    sec, ns = getattr(stamp, 'sec', None), getattr(stamp, 'nanosec', None)
    if (not isinstance(sec, int) or isinstance(sec, bool) or sec < 0
            or not isinstance(ns, int) or isinstance(ns, bool) or not 0 <= ns < 1_000_000_000):
        return None
    result = sec + ns * 1e-9
    return result if result > 0 else None


def odometry_observation(msg):
    pose = msg.pose.pose
    p, q = pose.position, pose.orientation
    numbers = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    frame = getattr(msg.header, 'frame_id', None)
    valid = all(finite(v) for v in numbers) and isinstance(frame, str) and bool(frame)
    valid = valid and abs(sum(v*v for v in numbers[3:]) - 1) <= .02
    return header_stamp(msg), dict(position=numbers[:3], quaternion_xyzw=numbers[3:],
                                   frame_id=frame, child_frame_id=getattr(msg, 'child_frame_id', None)), valid, 'invalid odometry pose/frame'


def vio_observation(msg):
    state = getattr(msg, 'vo_state', None)
    valid = isinstance(state, int) and not isinstance(state, bool) and state == 1
    return header_stamp(msg), dict(vo_state=state), valid, 'VIO status missing or not tracking'


def localizer_observation(msg):
    data = json.loads(msg.data)
    if not isinstance(data, dict):
        raise ValueError('localizer status is not an object')
    # The cloud's capture stamp, exact integer ns only (cloud_localize's status schema). Older
    # statuses without it stay explicitly unknown, never a guessed or receipt time.
    ns = data.get('source_stamp_ns')
    stamp = ns * 1e-9 if isinstance(ns, int) and not isinstance(ns, bool) and ns > 0 else None
    values = {k: data.get(k) for k in ('status', 'correction_valid', 'correction_reason',
                                      'odometry_valid', 'correction_epoch', 'input_odom_epoch',
                                      'source_epoch_fault')}
    valid = (values['status'] == 'tracking' and values['odometry_valid'] is True
             and not values['source_epoch_fault'])
    return stamp, values, valid, 'localizer missing/not tracking, untrusted odometry or epoch fault'


def px4_observation(msg):
    # timestamp_sample is fused-state acquisition in PX4's own boot clock.
    stamp_us = getattr(msg, 'timestamp_sample', None)
    values = {k: getattr(msg, k, None) for k in
              ('xy_valid', 'z_valid', 'heading_good_for_control', 'dead_reckoning',
               'xy_reset_counter', 'z_reset_counter', 'heading_reset_counter',
               'x', 'y', 'z', 'heading', 'timestamp', 'timestamp_sample')}
    missing = [k for k, v in values.items() if v is None]
    valid = (not missing and all(values[k] is True for k in
                                 ('xy_valid', 'z_valid', 'heading_good_for_control'))
             and values['dead_reckoning'] is False
             and all(finite(values[k]) for k in ('x', 'y', 'z', 'heading')))
    stamp = stamp_us * 1e-6 if isinstance(stamp_us, int) and not isinstance(stamp_us, bool) and stamp_us > 0 else None
    return stamp, values, valid, ('missing PX4 fields: ' + ','.join(missing) if missing else 'PX4 position/heading invalid or dead reckoning')


class ReadinessCollector:
    """Pure collector with caller-supplied ROS and monotonic wall clocks."""
    def __init__(self, *, required=STREAMS, max_age_s=.5, max_gap_s=.5,
                 future_tolerance_s=.005, clock_stall_s=.5, sample_limit=SAMPLE_LIMIT):
        if (any(not finite(v) or v <= 0 for v in (max_age_s, max_gap_s, clock_stall_s))
                or not finite(future_tolerance_s) or future_tolerance_s < 0
                or not isinstance(sample_limit, int) or sample_limit < 2):
            raise ValueError('invalid collector budgets')
        self.required = tuple(required)
        self.max_age, self.max_gap = max_age_s, max_gap_s
        self.future_tolerance, self.clock_stall = future_tolerance_s, clock_stall_s
        self.sample_limit = sample_limit
        self.streams = {}
        self.clock_errors = Counter()
        self.last_ros = self.last_progress_wall = None
        self.first_wall = self.last_clock_wall = None

    def observe_clock(self, wall_s, ros_s):
        if not finite(wall_s) or not finite(ros_s) or ros_s <= 0:
            self.clock_errors['invalid/missing ROS or wall clock'] += 1
            return
        if self.last_clock_wall is not None and wall_s < self.last_clock_wall:
            self.clock_errors['wall clock regressed'] += 1
        self.first_wall = wall_s if self.first_wall is None else self.first_wall
        self.last_clock_wall = wall_s
        if self.last_ros is None or ros_s > self.last_ros:
            self.last_progress_wall = wall_s
        elif ros_s < self.last_ros:
            self.clock_errors['ROS clock regressed/reset'] += 1
        if self.last_progress_wall is not None and wall_s - self.last_progress_wall > self.clock_stall:
            self.clock_errors['ROS clock stopped'] += 1
        self.last_ros = ros_s

    def observe(self, stream, *, wall_s, ros_s, source_stamp_s, values=None,
                valid=True, reason=None, source_clock='ros'):
        self.observe_clock(wall_s, ros_s)
        if source_clock not in ('ros', 'px4_boot'):
            raise ValueError('unrecognized source clock')
        s = self.streams.setdefault(stream, dict(total=0, accepted=0, errors=Counter(),
                                   ages=[], source_gaps=[], receipt_gaps=[], source_clock=source_clock,
                                   last_stamp=None, last_wall=None, last_values=None, resets=0))
        s['total'] += 1
        problems = []
        age = source_gap = receipt_gap = None
        if s['total'] > self.sample_limit:
            problems.append('sample capacity exceeded')
        if not finite(wall_s) or not finite(ros_s) or ros_s <= 0:
            problems.append('invalid clock')
        if not finite(source_stamp_s) or source_stamp_s <= 0:
            problems.append('missing/invalid source stamp')
        else:
            if source_clock == 'ros' and finite(ros_s):
                age = ros_s - source_stamp_s
                if age < -self.future_tolerance or age > self.max_age:
                    problems.append('source capture future/stale')
            if s['last_stamp'] is not None:
                source_gap = source_stamp_s - s['last_stamp']
                if source_gap <= 0:
                    problems.append('source stamp repeated/regressed')
                elif source_gap > self.max_gap:
                    problems.append('source gap exceeded')
            s['last_stamp'] = source_stamp_s
        if finite(wall_s):
            if s['last_wall'] is not None:
                receipt_gap = wall_s - s['last_wall']
                if receipt_gap < 0 or receipt_gap > self.max_gap:
                    problems.append('wall receipt gap exceeded/regressed')
            s['last_wall'] = wall_s
        values = json_safe(values or {})
        previous = s['last_values'] or {}
        for key in ('xy_reset_counter', 'z_reset_counter', 'heading_reset_counter', 'frame_id', 'correction_epoch'):
            if key in values and previous.get(key) is not None and values[key] != previous[key]:
                problems.append('reset/frame change: ' + key)
                s['resets'] += 1
        if not valid:
            problems.append(reason or 'sample reported invalid')
        for key, value in (('ages', age), ('source_gaps', source_gap), ('receipt_gaps', receipt_gap)):
            if finite(value) and len(s[key]) < self.sample_limit:
                s[key].append(value)
        s['last_values'] = values
        s['errors'].update(problems)
        s['accepted'] += not problems
        return dict(stream=stream, receipt_monotonic_s=wall_s, ros_now_s=ros_s,
                    source_stamp_s=source_stamp_s, source_clock=source_clock,
                    capture_age_ros_s=age, source_gap_s=source_gap,
                    receipt_gap_wall_s=receipt_gap, accepted=not problems,
                    reasons=problems, values=values)

    def summary(self, *, wall_s, ros_s, unavailable=None):
        self.observe_clock(wall_s, ros_s)
        reports = {}
        missing = [name for name in self.required if name not in self.streams]
        for name, s in self.streams.items():
            receipt_age = wall_s - s['last_wall'] if finite(wall_s) and s['last_wall'] is not None else None
            current_age = ros_s - s['last_stamp'] if (s['source_clock'] == 'ros' and finite(ros_s)
                                                     and s['last_stamp'] is not None) else None
            current = (finite(receipt_age) and 0 <= receipt_age <= self.max_gap
                       and (s['source_clock'] == 'px4_boot' or
                            (finite(current_age) and -self.future_tolerance <= current_age <= self.max_age)))
            reports[name] = dict(total=s['total'], accepted=s['accepted'], rejected=s['total']-s['accepted'],
                                 rejection_reasons=dict(s['errors']), source_clock=s['source_clock'],
                                 capture_age_ros_s=distribution(s['ages']), source_gap_s=distribution(s['source_gaps']),
                                 receipt_gap_wall_s=distribution(s['receipt_gaps']),
                                 final_receipt_age_wall_s=receipt_age, final_capture_age_ros_s=current_age,
                                 source_age_note='not comparable to ROS clock' if s['source_clock']=='px4_boot' else None,
                                 current=current, reset_or_frame_changes=s['resets'], latest_values=s['last_values'])
        healthy = (not missing and not unavailable and not self.clock_errors and
                   all(reports[n]['total'] >= 2 and reports[n]['rejected'] == 0 and reports[n]['current']
                       for n in self.required))
        return json_safe(dict(schema=1, kind='bench', observation_healthy=healthy,
                              scope='passive timing/validity observation only; not flight authorization or map accuracy',
                              missing_streams=missing, unavailable_topics=unavailable or {},
                              clock_errors=dict(self.clock_errors), streams=reports))


def _write_json(path, data):
    path.write_text(json.dumps(json_safe(data), indent=2, allow_nan=False) + '\n', encoding='utf-8')


def archive_comparison(run, *, name, variant, baseline=None):
    """Use the project writer; configuration labels are explicitly caller claims."""
    root = Path(__file__).resolve().parents[1]
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    from split_sim.run_comparison import write
    annotation = dict(notes=['Receive-only bench; no flight/control/motion/ground-truth measurement.',
                              'Variant is operator-labelled, not inferred: ' + variant,
                              'Run-local evidence: navigation-events.jsonl and navigation-summary.json.'],
                      confounds=['Scene, vehicle motion, CPU/GPU load, network and other processes are not controlled or measured.',
                                 'PX4 boot-clock capture ages are unknown in ROS time.',
                                 'Missing status stamps/fields remain unknown; no receipt-time source fallback.'])
    result = write(run, name=name, kind='bench', baseline=baseline, annotations=annotation)
    for file in ('navigation-events.jsonl', 'navigation-summary.json'):
        path = run/file
        if path.is_file():
            result['evidence'].append(file)
            result['source_sha256'][file] = hashlib.sha256(path.read_bytes()).hexdigest()
    _write_json(run/'run-comparison.json', result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--name', required=True)
    parser.add_argument('--variant', required=True, help='operator claim, e.g. vio-only / vio+depth / vio+depth+localizer')
    parser.add_argument('--baseline', type=Path)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--max-age', type=float, default=.5)
    parser.add_argument('--max-gap', type=float, default=.5)
    parser.add_argument('--raw-topic', default='/visual_slam/tracking/odometry')
    parser.add_argument('--validated-topic', default='/mapping/validated_odometry')
    parser.add_argument('--ros-args', nargs=argparse.REMAINDER, default=[])
    args = parser.parse_args(argv)
    if not args.name.strip() or not args.variant.strip() or not finite(args.duration) or not 1 <= args.duration <= 300:
        parser.error('name/variant are required; duration must be 1–300 wall seconds')
    if args.baseline is not None:
        args.baseline = args.baseline.resolve(strict=True)
    collector = ReadinessCollector(max_age_s=args.max_age, max_gap_s=args.max_gap)
    run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ') + '-navigation-readiness'
    run = args.output_root.resolve()/run_id
    run.mkdir(parents=True, exist_ok=False)
    _write_json(run/'settings.json', dict(run_id=run_id, kind='bench', name=args.name,
                observer_variant_claim=args.variant, duration_wall_s=args.duration,
                max_capture_age_ros_s=args.max_age, max_gap_s=args.max_gap,
                raw_topic=args.raw_topic, validated_topic=args.validated_topic,
                configuration_inferred=False))
    archive_comparison(run, name=args.name, variant=args.variant, baseline=args.baseline)
    unavailable = {}
    node = None
    rclpy = None
    ros_now = None
    started = time.monotonic()
    code = 1
    with (run/'navigation-events.jsonl').open('x', encoding='utf-8') as events:
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from nav_msgs.msg import Odometry
            from std_msgs.msg import String
            rclpy.init(args=['--ros-args']+args.ros_args if args.ros_args else [])

            class NoPublisher:
                def publish(self, _message):
                    return None

            class PassiveNode(Node):
                def create_publisher(self, msg_type, topic, qos_profile, **kwargs):
                    # rclpy creates parameter-event support in Node.__init__.
                    # Suppress even that output; every other publisher is forbidden.
                    if topic == '/parameter_events':
                        return NoPublisher()
                    raise RuntimeError('passive collector cannot create publishers')

            node = PassiveNode('navigation_readiness', enable_rosout=False, start_parameter_services=False)

            def ros_time():
                return node.get_clock().now().nanoseconds * 1e-9

            def callback(stream, decoder, clock='ros'):
                def receive(msg):
                    received_wall, received_ros = time.monotonic(), ros_time()
                    try:
                        stamp, values, valid, why = decoder(msg)
                    except (AttributeError, TypeError, ValueError, OverflowError):
                        stamp, values, valid, why = None, {}, False, 'malformed message/fields'
                    event = collector.observe(stream, wall_s=received_wall, ros_s=received_ros,
                              source_stamp_s=stamp, values=values, valid=valid, reason=why, source_clock=clock)
                    events.write(json.dumps(json_safe(event), allow_nan=False) + '\n')
                return receive

            node.create_subscription(Odometry, args.raw_topic, callback('raw_odometry', odometry_observation), qos_profile_sensor_data)
            node.create_subscription(Odometry, args.validated_topic, callback('validated_odometry', odometry_observation), qos_profile_sensor_data)
            node.create_subscription(String, '/team/localize/status', callback('localizer_status', localizer_observation), qos_profile_sensor_data)
            try:
                from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
                node.create_subscription(VisualSlamStatus, '/visual_slam/status', callback('vio_status', vio_observation), qos_profile_sensor_data)
            except ImportError:
                unavailable['vio_status'] = 'VisualSlamStatus type unavailable'
            try:
                from px4_msgs.msg import VehicleLocalPosition
                version = int(getattr(VehicleLocalPosition, 'MESSAGE_VERSION', 0))
                topic = '/fmu/out/vehicle_local_position' + (f'_v{version}' if version else '')
                node.create_subscription(VehicleLocalPosition, topic, callback('px4_local_position', px4_observation, 'px4_boot'), qos_profile_sensor_data)
            except ImportError:
                unavailable['px4_local_position'] = 'VehicleLocalPosition type unavailable'
            started = time.monotonic()
            while rclpy.ok() and time.monotonic() - started < args.duration:
                rclpy.spin_once(node, timeout_sec=.05)
                ros_now = ros_time()
                collector.observe_clock(time.monotonic(), ros_now)
            if time.monotonic() - started < args.duration:
                unavailable['collection'] = 'ROS stopped before requested wall duration'
        except KeyboardInterrupt:
            unavailable['collection'] = 'operator interrupted collection'
        except Exception as exc:
            unavailable['collection'] = 'collector unavailable: ' + type(exc).__name__
        finally:
            if node is not None:
                ros_now = node.get_clock().now().nanoseconds * 1e-9
                node.destroy_node()
            if rclpy is not None and rclpy.ok():
                rclpy.shutdown()
            summary = collector.summary(wall_s=time.monotonic(), ros_s=ros_now, unavailable=unavailable)
            summary.update(run_id=run_id, name=args.name, operator_variant_claim=args.variant,
                           requested_duration_wall_s=args.duration, elapsed_wall_s=time.monotonic()-started)
            _write_json(run/'navigation-summary.json', summary)
            events.flush()
            archive_comparison(run, name=args.name, variant=args.variant, baseline=args.baseline)
            code = 0 if summary['observation_healthy'] else 1
    print(json.dumps(dict(run_id=run_id, run_directory=str(run), observation_healthy=summary['observation_healthy'],
                          scope=summary['scope'])))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
