#!/usr/bin/env python3
"""skybeta ROS status sidecar: summarise the drone's local-only DDS into a JSON file.

Runs INSIDE the stock ``skybeta:<tag>`` image (bind-mounted read-only by
run_sidecar.sh), as a separate container on the stack's loopback DDS profile,
and writes ``/out/ros_summary.json`` atomically once a second. The host status
agent reads that file; nothing ever subscribes from off the drone.

It is a passive observer:
  * every subscription is BEST_EFFORT (+ VOLATILE, except /fmu/out/* which uses
    PX4's documented TRANSIENT_LOCAL profile), so it can never raise an
    incompatible-QoS warning in the team's stack;
  * it publishes nothing and offers no services;
  * image/IMU/visual-odometry messages are taken ``raw=True`` (serialized bytes,
    never deserialised) and only time-stamped.

Pure parts (RateWindow, topic-name resolution, SummaryState, build_summary,
atomic_write_json) import nothing from ROS, so tests/test_sidecar_pure.py runs
on any Python. rclpy / px4_msgs are imported only inside ``main()``.
"""
import argparse
import collections
import json
import math
import os
import re
import signal
import sys
import time

SCHEMA_VERSION = 1
NODE_NAME = "skybeta_status_sidecar"
PX4_MSGS_SHA_FILE = "/root/skybeta_ws/px4_msgs.sha"
#: VisualSlamStatus track_execution_time* are in SECONDS per the Isaac ROS 3.2
#: .msg comments; reported in ms. Verify on the bench (docs/bench_testing.md, phase 3):
#: if the dashboard shows ~8000 ms for a healthy tracker, set this to 1.0.
SLAM_TIME_TO_MS = 1000.0

# --------------------------------------------------------------------------
# PX4 topic names: copied from skybeta_flight_interface/topics.py (do NOT
# import it -- the sidecar must not depend on that package being built).
# --------------------------------------------------------------------------
_MESSAGE_VERSION_RE = re.compile(r'^\s*uint32\s+MESSAGE_VERSION\s*=\s*(\d+)\s*(?:#.*)?$')


def px4_msgs_msg_dir():
    """Directory holding px4_msgs' installed .msg files, or None."""
    try:
        from ament_index_python.packages import get_package_share_directory  # noqa: PLC0415
        share = get_package_share_directory('px4_msgs')
    except Exception:  # noqa: BLE001 - only when px4_msgs is absent
        return None
    msg_dir = os.path.join(share, 'msg')
    return msg_dir if os.path.isdir(msg_dir) else None


def message_version(msg_name, msg_dir=None):
    """MESSAGE_VERSION of ``msg_name`` ('VehicleStatus'), or 0 if absent."""
    if not msg_dir:
        return 0
    path = os.path.join(msg_dir, '%s.msg' % msg_name)
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            for line in handle:
                match = _MESSAGE_VERSION_RE.match(line)
                if match:
                    return int(match.group(1))
    except OSError:
        return 0
    return 0


def fmu_topic(direction, base_name, msg_name=None, msg_dir=None):
    """/fmu/<direction>/<base_name>[_v<N>] from the installed .msg's MESSAGE_VERSION."""
    if direction not in ('in', 'out'):
        raise ValueError("direction must be 'in' or 'out', got %r" % (direction,))
    if msg_name is None:
        msg_name = ''.join(part.capitalize() for part in str(base_name).split('_'))
    version = message_version(msg_name, msg_dir)
    suffix = '_v%d' % version if version > 0 else ''
    return '/fmu/%s/%s%s' % (direction, base_name, suffix)


#: flight_interface.py's table, extended with PX4 1.16's names.
NAV_STATE_NAMES = {
    0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 3: 'AUTO_MISSION',
    4: 'AUTO_LOITER', 5: 'AUTO_RTL', 6: 'POSITION_SLOW', 10: 'ACRO', 12: 'DESCEND',
    13: 'TERMINATION', 14: 'OFFBOARD', 15: 'STAB', 17: 'AUTO_TAKEOFF',
    18: 'AUTO_LAND', 19: 'AUTO_FOLLOW_TARGET', 20: 'AUTO_PRECLAND',
    21: 'ORBIT', 22: 'AUTO_VTOL_TAKEOFF',
}
NAV_STATE_NAMES.update({23 + i: 'EXTERNAL%d' % (i + 1) for i in range(8)})
BATTERY_WARNING_NAMES = {0: 'NONE', 1: 'LOW', 2: 'CRITICAL', 3: 'EMERGENCY', 4: 'FAILED'}
VO_STATE_NAMES = {0: 'unknown', 1: 'tracking', 2: 'failed'}
ARMING_STATE_ARMED = 2

# (key, topic-or-fmu-spec, ROS type string, qos profile name, raw)
# fmu spec = (direction, uORB base, message) resolved at startup.
WATCHED = (
    ('vehicle_status', ('out', 'vehicle_status', 'VehicleStatus'), 'px4_msgs/msg/VehicleStatus', 'px4_out', False),
    ('battery_status', ('out', 'battery_status', 'BatteryStatus'), 'px4_msgs/msg/BatteryStatus', 'px4_out', False),
    ('vehicle_local_position', ('out', 'vehicle_local_position', 'VehicleLocalPosition'),
     'px4_msgs/msg/VehicleLocalPosition', 'px4_out', False),
    ('visual_odometry_in', ('in', 'vehicle_visual_odometry', 'VehicleOdometry'),
     'px4_msgs/msg/VehicleOdometry', 'be10', True),
    ('slam_odometry', '/visual_slam/tracking/odometry', 'nav_msgs/msg/Odometry', 'be10', False),
    ('slam_status', '/visual_slam/status', 'isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus',
     'be10', False),
    # camera_info, not image_raw: same 30 Hz cadence at ~1 KB instead of ~2 MB. On the bench a
    # raw image_raw subscription read 11-19 Hz while cuVSLAM ran at 30 (rclpy dropping
    # 60 MB/s), and copying the images is load the flight computer does not need.
    ('cam_left_info', '/oak/left/camera_info', 'sensor_msgs/msg/CameraInfo', 'be10', True),
    ('cam_imu', '/oak/imu/data', 'sensor_msgs/msg/Imu', 'be10', True),
    ('skybeta_state', '/skybeta/state', 'std_msgs/msg/String', 'be1', False),
)


def resolve_topics(msg_dir=None):
    out = {}
    for key, spec, _typ, _qos, _raw in WATCHED:
        out[key] = fmu_topic(spec[0], spec[1], spec[2], msg_dir) if isinstance(spec, tuple) else spec
    return out


# --------------------------------------------------------------------------
# Pure state
# --------------------------------------------------------------------------
class RateWindow:
    """Arrival-rate over the last ``window_s`` seconds, from monotonic stamps.

    rate_hz = arrivals in (now - window, now] / window -- None if nothing ever
    arrived. 2 s makes 30 Hz read 30.0 +- 0.5 and a stall visible within 2 s.
    """

    def __init__(self, window_s=2.0, maxlen=600):
        self.window_s = float(window_s)
        self.stamps = collections.deque(maxlen=maxlen)
        self.total = 0
        self.last = None
        self.last_header_stamp_s = None

    def stamp(self, now, header_stamp_s=None):
        self.stamps.append(now)
        self.total += 1
        self.last = now
        if header_stamp_s is not None:
            self.last_header_stamp_s = header_stamp_s

    def rate_hz(self, now):
        if self.total == 0:
            return None
        cutoff = now - self.window_s
        n = 0
        for t in reversed(self.stamps):
            if t <= cutoff:
                break
            n += 1
        return round(n / self.window_s, 2)

    def age_s(self, now):
        return None if self.last is None else round(now - self.last, 3)


def _finite(v):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def _stamp_s(msg):
    try:
        st = msg.header.stamp
        return st.sec + st.nanosec * 1e-9
    except AttributeError:
        return None


class SummaryState:
    """Everything the callbacks record. Callbacks stay O(1): stamp + copy scalars."""

    def __init__(self, topics, clock=time.monotonic):
        self.topics = dict(topics)
        self.clock = clock
        self.rates = {k: RateWindow() for k in self.topics}
        self.publishers = {k: 0 for k in self.topics}
        self.vehicle_status = None
        self.battery = None
        self.local_position = None
        self.odom_position = None
        self.slam_status = None
        self.skybeta_state = None
        self.graph = {"node_count": 0, "topic_count": 0, "fmu_out_topic_count": 0,
                      "fmu_in_topic_count": 0, "nodes": []}

    # -- callbacks (msg objects are duck-typed; tests pass SimpleNamespace) --
    def on_raw(self, key, _data=None):
        self.rates[key].stamp(self.clock())

    HANDLERS = {'vehicle_status': 'on_vehicle_status', 'battery_status': 'on_battery',
                'vehicle_local_position': 'on_local_position', 'slam_odometry': 'on_slam_odometry',
                'slam_status': 'on_slam_status', 'skybeta_state': 'on_skybeta_state'}

    def on_batch(self, key, n, since, last=None):
        """``n`` messages drained from a queue since monotonic ``since``; ``last`` = the newest.

        The drain does not keep arrival times, so they are spread evenly over (since, now] --
        the 2 s rate window then stays exact to about one message. The newest goes through the
        normal handler (every handler keeps only the latest values anyway)."""
        if n <= 0:
            return
        now = self.clock()
        span = max(now - since, 0.0) if since is not None else 0.0
        rw = self.rates[key]
        for i in range(1, n):
            rw.stamp(now - span * (n - i) / n)
        handler = self.HANDLERS.get(key)
        if handler and last is not None:
            getattr(self, handler)(last)
        else:
            rw.stamp(now)

    def on_vehicle_status(self, msg):
        now = self.clock()
        self.rates['vehicle_status'].stamp(now)
        self.vehicle_status = {
            'timestamp': int(getattr(msg, 'timestamp', 0) or 0) or None,
            'arming_state': int(msg.arming_state),
            'nav_state': int(msg.nav_state),
            'failsafe': bool(msg.failsafe),
            'pre_flight_checks_pass': bool(msg.pre_flight_checks_pass),
            'failure_detector_status': int(getattr(msg, 'failure_detector_status', 0)),
            'gcs_connection_lost': bool(getattr(msg, 'gcs_connection_lost', False)),
            'at': now,
        }

    def on_battery(self, msg):
        now = self.clock()
        self.rates['battery_status'].stamp(now)
        current = _finite(msg.current_a)
        remaining = _finite(msg.remaining)
        trem = _finite(getattr(msg, 'time_remaining_s', None))
        warning = int(getattr(msg, 'warning', 0))
        self.battery = {
            'connected': bool(msg.connected),
            'voltage_v': _finite(msg.voltage_v),
            'current_a': None if current is None or current == -1.0 else round(current, 3),
            'remaining': None if remaining is None or remaining < 0 else round(remaining, 3),
            'cell_count': int(getattr(msg, 'cell_count', 0)),
            'warning': warning,
            'warning_name': BATTERY_WARNING_NAMES.get(warning, str(warning)),
            'temperature_c': _finite(getattr(msg, 'temperature', None)),
            'time_remaining_s': None if trem is None or trem <= 0 else round(trem, 1),
            'at': now,
        }

    def on_local_position(self, msg):
        now = self.clock()
        self.rates['vehicle_local_position'].stamp(now)
        lp = {k: bool(getattr(msg, k, False)) for k in
              ('xy_valid', 'z_valid', 'v_xy_valid', 'heading_good_for_control', 'dist_bottom_valid')}
        for k in ('x', 'y', 'z', 'heading', 'dist_bottom'):
            v = _finite(getattr(msg, k, None))
            lp[k] = None if v is None else round(v, 3)
        lp['at'] = now
        self.local_position = lp

    def on_slam_odometry(self, msg):
        now = self.clock()
        self.rates['slam_odometry'].stamp(now, _stamp_s(msg))
        p = msg.pose.pose.position
        self.odom_position = [round(float(p.x), 3), round(float(p.y), 3), round(float(p.z), 3)]

    def on_slam_status(self, msg):
        now = self.clock()
        self.rates['slam_status'].stamp(now, _stamp_s(msg))

        def ms(name):
            v = _finite(getattr(msg, name, None))
            return None if v is None else round(v * SLAM_TIME_TO_MS, 2)

        self.slam_status = {'vo_state': int(msg.vo_state),
                            'track_execution_time_ms': ms('track_execution_time'),
                            'track_execution_time_mean_ms': ms('track_execution_time_mean'),
                            'track_execution_time_max_ms': ms('track_execution_time_max')}

    def on_skybeta_state(self, msg):
        now = self.clock()
        self.rates['skybeta_state'].stamp(now)
        try:
            parsed = json.loads(msg.data)
            if not isinstance(parsed, dict):
                parsed = {'_parse_error': 'not a JSON object', '_value': parsed}
        except (ValueError, TypeError) as e:
            parsed = {'_parse_error': str(e)}
        self.skybeta_state = (parsed, now)

    def set_graph(self, names_and_types, node_names, publisher_counts, fmu_publishers=None):
        """``fmu_publishers``: {/fmu/... topic: publisher count}. The graph also lists topics
        that only this node subscribes to (3 /fmu/out names with nothing else running, seen on
        the bench), so an /fmu topic counts only if something publishes it."""
        names = [n for n, _ in names_and_types]
        fmu = [n for n in names if fmu_publishers is None or fmu_publishers.get(n, 0) > 0]
        self.graph = {
            'node_count': len(node_names),
            'topic_count': len(names),
            'fmu_out_topic_count': sum(1 for n in fmu if n.startswith('/fmu/out/')),
            'fmu_in_topic_count': sum(1 for n in fmu if n.startswith('/fmu/in/')),
            'nodes': sorted(node_names)[:100],
        }
        for k, c in publisher_counts.items():
            self.publishers[k] = int(c)


FC_LINK_MAX_AGE_S = 3.0

DRAIN_PERIOD_S = 0.25   # main(): queue drain interval
QUEUE_DEPTH = 100       # per subscription: 0.25 s of anything up to 400 Hz
IMU_QUEUE_DEPTH = 400   # 200 Hz IMU: 2 s of slack if a drain is late
DRAIN_CAP = 2000        # max messages taken from one queue per drain


def _aged(d, now):
    if d is None:
        return None
    out = {k: v for k, v in d.items() if k != 'at'}
    out['age_s'] = round(now - d['at'], 3)
    return out


def build_summary(state, env, now_mono, now_wall, seq, node_started_at, period_s=1.0):
    """SummaryState -> the ros_summary.json document (schema_version 1)."""
    topics = {}
    for key, topic in state.topics.items():
        rw = state.rates[key]
        typ = next(t for k, _s, t, _q, _r in WATCHED if k == key)
        topics[key] = {'topic': topic, 'type': typ, 'publishers': state.publishers.get(key, 0),
                       'msgs_total': rw.total, 'rate_hz': rw.rate_hz(now_mono),
                       'age_s': rw.age_s(now_mono), 'last_header_stamp_s': rw.last_header_stamp_s}
    vs_age = topics['vehicle_status']['age_s']
    vs = state.vehicle_status
    fc = {'link': vs_age is not None and vs_age < FC_LINK_MAX_AGE_S,
          'armed': None, 'arming_state': None, 'nav_state': None, 'nav_state_name': None,
          'failsafe': None, 'pre_flight_checks_pass': None, 'failure_detector_status': None,
          'gcs_connection_lost': None, 'fc_timestamp_us': None,
          'battery': _aged(state.battery, now_mono),
          'local_position': _aged(state.local_position, now_mono)}
    if vs is not None:
        fc.update(armed=vs['arming_state'] == ARMING_STATE_ARMED, arming_state=vs['arming_state'],
                  nav_state=vs['nav_state'],
                  nav_state_name=NAV_STATE_NAMES.get(vs['nav_state'], str(vs['nav_state'])),
                  failsafe=vs['failsafe'], pre_flight_checks_pass=vs['pre_flight_checks_pass'],
                  failure_detector_status=vs['failure_detector_status'],
                  gcs_connection_lost=vs['gcs_connection_lost'], fc_timestamp_us=vs['timestamp'])
    ss = state.slam_status or {}
    vo = ss.get('vo_state')
    vio = {'vo_state': vo, 'vo_state_name': VO_STATE_NAMES.get(vo) if vo is not None else None,
           'track_execution_time_ms': ss.get('track_execution_time_ms'),
           'track_execution_time_mean_ms': ss.get('track_execution_time_mean_ms'),
           'track_execution_time_max_ms': ss.get('track_execution_time_max_ms'),
           'odom_position_enu': state.odom_position,
           'odom_rate_hz': topics['slam_odometry']['rate_hz'],
           'bridge_rate_hz': topics['visual_odometry_in']['rate_hz'],
           'cam_left_rate_hz': topics['cam_left_info']['rate_hz'],
           'imu_rate_hz': topics['cam_imu']['rate_hz']}
    sky, sky_at = state.skybeta_state if state.skybeta_state else (None, None)
    return {
        'schema_version': SCHEMA_VERSION,
        'written_at': round(now_wall, 3),
        'seq': seq,
        'node_started_at': round(node_started_at, 3),
        'period_s': period_s,
        'env': dict(env),
        'resolved_topics': dict(state.topics),
        'graph': dict(state.graph),
        'topics': topics,
        'fc': fc,
        'vio': vio,
        'skybeta_state': sky,
        'skybeta_state_age_s': None if sky_at is None else round(now_mono - sky_at, 3),
    }


def atomic_write_json(path, doc):
    """Write ``doc`` so readers only ever see a complete file (same-dir tmp + fsync + replace)."""
    d = os.path.dirname(os.path.abspath(path))
    tmp = os.path.join(d, '.' + os.path.basename(path) + '.tmp')
    data = json.dumps(doc, allow_nan=False, default=str).encode('utf-8')
    with open(tmp, 'wb') as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
    try:
        os.chmod(path, 0o644)
    except OSError:
        pass


def read_env(version):
    sha = None
    try:
        with open(PX4_MSGS_SHA_FILE, 'r', encoding='utf-8') as f:
            sha = f.read().strip() or None
    except OSError:
        pass
    dom = os.environ.get('ROS_DOMAIN_ID', '0')
    return {'ros_domain_id': int(dom) if dom.isdigit() else None,
            'dds_scope': os.environ.get('SKYBETA_DDS_SCOPE'),
            'profiles_file': os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE'),
            'rmw': os.environ.get('RMW_IMPLEMENTATION'),
            'px4_msgs_sha': sha, 'sidecar_version': version}


# --------------------------------------------------------------------------
# ROS part (only runs inside the image)
# --------------------------------------------------------------------------
def main(argv=None):  # pragma: no cover - needs rclpy; exercised on the bench (phase 3)
    p = argparse.ArgumentParser(description='skybeta ROS status sidecar')
    p.add_argument('--out', default='/out/ros_summary.json')
    p.add_argument('--period', type=float, default=1.0)
    p.add_argument('--drain', type=float, default=DRAIN_PERIOD_S,
                   help='seconds between queue drains (default %(default)s)')
    p.add_argument('--version', default=os.environ.get('SKYBETA_STATUS_VERSION', 'unknown'))
    args = p.parse_args(argv)
    os.umask(0o022)

    import rclpy  # noqa: PLC0415
    from rclpy.node import Node  # noqa: PLC0415
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,  # noqa: PLC0415
                           QoSReliabilityPolicy)
    from rclpy.serialization import deserialize_message  # noqa: PLC0415

    def qos(depth, durability):
        return QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=durability,
                          history=QoSHistoryPolicy.KEEP_LAST, depth=depth)

    # Depths hold several drain periods: a full queue drops the oldest and the rate reads low.
    profiles = {'px4_out': qos(QUEUE_DEPTH, QoSDurabilityPolicy.TRANSIENT_LOCAL),
                'be10': qos(QUEUE_DEPTH, QoSDurabilityPolicy.VOLATILE),
                'be1': qos(QUEUE_DEPTH, QoSDurabilityPolicy.VOLATILE),
                'imu': qos(IMU_QUEUE_DEPTH, QoSDurabilityPolicy.VOLATILE)}

    def load_type(type_str):
        pkg, _, name = type_str.split('/')
        try:
            mod = __import__('%s.msg' % pkg, fromlist=[name])
            return getattr(mod, name)
        except (ImportError, AttributeError):
            return None

    rclpy.init(args=None)
    node = Node(NODE_NAME)
    topics = resolve_topics(px4_msgs_msg_dir())
    node.get_logger().info('resolved topics: %s' % json.dumps(topics, sort_keys=True))
    state = SummaryState(topics)
    # No executor. Humble's rclpy executor costs ~1.2 ms of Python per delivered message on the
    # Orin (bench 2026-10-16: 300 msg/s = 40 % of a core, the same with spin_once or a
    # SingleThreadedExecutor; nothing subscribed = 0.8 %). The node is never spun: Fast DDS
    # fills each reader's queue on its own threads, and drain() empties the queues every
    # --drain seconds in C (take_message, raw bytes), deserialising only the newest message.
    subs = []  # (key, subscription, msg_type, deserialise?)
    for key, _spec, type_str, qname, raw in WATCHED:
        msg_type = load_type(type_str)
        if msg_type is None:
            node.get_logger().warning('%s: type %s not importable; not subscribed' % (key, type_str))
            continue
        sub = node.create_subscription(msg_type, topics[key], lambda _m: None,
                                       profiles['imu' if key == 'cam_imu' else qname], raw=True)
        subs.append((key, sub, msg_type, not raw))

    last_drain = [time.monotonic()]

    def drain():
        since = last_drain[0]
        for key, sub, msg_type, deser in subs:
            n, newest = 0, None
            with sub.handle:
                while n < DRAIN_CAP:
                    got = sub.handle.take_message(msg_type, True)
                    if got is None:
                        break
                    newest, n = got[0], n + 1
            if n:
                last = None
                if deser:
                    try:
                        last = deserialize_message(newest, msg_type)
                    except Exception as e:  # noqa: BLE001 - one bad message must not stop the loop
                        node.get_logger().warning('%s: cannot deserialise: %s' % (key, e))
                state.on_batch(key, n, since, last)
        last_drain[0] = time.monotonic()

    env = read_env(args.version)
    started_wall = time.time()
    seq = [0]

    def tick():
        try:
            names_and_types = node.get_topic_names_and_types()
            node_names = ['%s%s%s' % (ns.rstrip('/'), '/', n) if ns else '/' + n
                          for n, ns in node.get_node_names_and_namespaces()]
            counts = {k: node.count_publishers(t) for k, t in topics.items()}
            fmu_pubs = {n: node.count_publishers(n) for n, _ in names_and_types
                        if n.startswith('/fmu/')}
            state.set_graph(names_and_types, node_names, counts, fmu_pubs)
        except Exception as e:  # noqa: BLE001 - keep writing even if a graph query fails
            node.get_logger().warning('graph probe failed: %s' % e)
        seq[0] += 1
        doc = build_summary(state, env, time.monotonic(), time.time(), seq[0], started_wall,
                            args.period)
        try:
            atomic_write_json(args.out, doc)
        except OSError as e:
            node.get_logger().error('cannot write %s: %s' % (args.out, e))

    running = [True]

    def _stop(signum, _frame):
        running[0] = False

    # PID 1 in the container after `exec`: without handlers SIGTERM would be ignored.
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    node.get_logger().info('writing %s every %.1fs (domain %s, scope %s)'
                           % (args.out, args.period, env['ros_domain_id'], env['dds_scope']))
    try:
        next_tick = time.monotonic()
        while running[0] and rclpy.ok():
            drain()
            if time.monotonic() >= next_tick:
                tick()
                next_tick += args.period
                if next_tick < time.monotonic():  # fell behind (e.g. suspended): don't burst
                    next_tick = time.monotonic() + args.period
            time.sleep(args.drain)
    finally:
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except AttributeError:
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
