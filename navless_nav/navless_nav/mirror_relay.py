"""Stream the drone's live pose to a laptop for the Gazebo mirror (gazebo_sim/mirror/).

Read-only: subscribes, never publishes, never touches /fmu/*. Run it inside the stack
container with the team overlay sourced, with the drone on the bench or carried, props off:

    ros2 run navless_nav mirror_relay                          # TCP 8775, 30 Hz
    ros2 run navless_nav mirror_relay --ros-args -p rate_hz:=20
    ros2 run navless_nav mirror_relay --ros-args -p bind:=192.168.55.1   # USB tether only

The laptop connects to <drone-ip>:8775. The drone never connects out, so this works
from WSL2 behind NAT, over Wi-Fi or USB (192.168.55.1). By default it listens on every
interface, so anyone on the shared Wi-Fi can read it (read-only); bind it to the USB
address when tethered. It gets one JSON object per line:

    {"t": 12.34, "age": 0.02,                   receive time, odometry age (s)
     "p": [x, y, z], "q": [x, y, z, w],         cuVSLAM odom pose, ENU (start frame)
     "vo_state": 1,                             0 unknown, 1 tracking, 2 lost
     "armed": false, "offboard": false, "nav": "POSCTL",
     "px4_link": true, "landed": true, "local_enu": [x, y, z] | null,   flight_interface's PX4 view
     "setpoint": [x, y, z] | null,              last pose this team's node commanded
     "cam": {"p": [...], "q": [...]} | null,    base_link -> camera mount (static TF)
     "tof": {"left": m|null, "right": m|null,   side ToF (/tube/*/range); null = no return
             "age_left": s|null, "age_right": s|null,   per side, so one dead sensor shows
             "age": s} | null,                          as stale; age = the older side
     "vio": {"n": count, "still": s,            /visual_slam/tracking/odometry messages so far;
             "jump": [t, m] | null,             time the pose has stood exactly still (a frozen
             "gate": {...} | null},             pose repeats bit for bit); the last step over 8 cm;
                                                Koji's /mapping/gate_status (accepted, rejected, age)
     "node": "ROUTE" | null}                    mission state, when the sender knows it

and, at img_hz (default 5; 0 = off), a separate line with the real left camera:

    {"img": "<base64 JPEG>", "w": 320, "h": 200, "t": 12.34}

The camera thumbnail subscribes to /oak/left/image_mono, which makes the converter also
send every frame to this process. Fine on the bench; use img_hz:=0 when flying.
"""
import base64
import json
import math
import socket
import threading

CAMERA_PARENT = 'base_link'
CAMERA_CHILD = 'oak_parent_frame'      # the stack's camera mount (left lens)


def _r(v, n=4):
    return [round(float(x), n) for x in v]


def _range(m):
    return None if m is None or not math.isfinite(m) else round(float(m), 3)


def _age(a):
    return None if a is None else round(float(a), 3)


def _tof(tof):
    """tof: (left_m, right_m, age_s) or (left_m, right_m, left_age_s, right_age_s);
    a side that never reported has age None."""
    if tof is None:
        return None
    left, right, *ages = tof
    age_l, age_r = (ages[0], ages[0]) if len(ages) == 1 else ages
    known = [a for a in (age_l, age_r) if a is not None]
    return {'left': _range(left), 'right': _range(right),
            'age_left': _age(age_l), 'age_right': _age(age_r),
            'age': _age(max(known)) if known else None}


JUMP_M = 0.08   # one odometry step longer than this is a jump (carried at walking pace: ~3 cm a step at 30 Hz)


class OdomWatch:
    """What the odometry topic itself does, for the tube hand-carry test: messages so far, how long the
    pose has stood exactly still (a frozen pose repeats bit for bit; a carried drone's VIO never does),
    and the last jump, one step longer than JUMP_M (cuVSLAM re-initialising on the way out)."""

    def __init__(self, jump_m=JUMP_M):
        self.jump_m, self.n, self.last, self.changed_at, self.jump = jump_m, 0, None, None, None

    def update(self, t, p):
        p = tuple(float(v) for v in p)
        if self.last is None or p != self.last:
            self.changed_at = t
        if self.last is not None and math.dist(p, self.last) > self.jump_m:
            self.jump = (t, math.dist(p, self.last))
        self.last = p
        self.n += 1

    def block(self, now, gate=None):
        """The frame's "vio" block (see the module docstring)."""
        return {'n': self.n,
                'still': None if self.changed_at is None else round(now - self.changed_at, 3),
                'jump': None if self.jump is None else [round(self.jump[0], 3), round(self.jump[1], 3)],
                'gate': gate}


def make_image_frame(t, jpeg, width, height):
    """A camera thumbnail line (see the module docstring)."""
    return json.dumps({'img': base64.b64encode(jpeg).decode('ascii'), 'w': int(width),
                       'h': int(height), 't': round(float(t), 3)}, separators=(',', ':')) + '\n'


def make_frame(t, age, pose, vo_state=None, state=None, setpoint=None, cam=None,
               tof=None, node=None, vio=None):
    """One newline-terminated JSON frame (see the module docstring).
    tof: (left_m, right_m, left_age_s, right_age_s), inf/None for no return (see _tof)."""
    p, q = pose
    s = state or {}
    return json.dumps({
        'tof': _tof(tof),
        'node': node,
        't': round(float(t), 3),
        'age': None if age is None else round(float(age), 3),
        'p': _r(p), 'q': _r(q, 5),
        'vo_state': vo_state,
        'armed': bool(s.get('armed', False)),
        'offboard': bool(s.get('offboard_active', False)),
        'nav': s.get('nav_state_name'),
        # flight_interface's view of PX4: its EKF position back in ENU, for the pre-flight
        # "does PX4's height agree with VIO's" check (a PX4 height boot offset flies every
        # setpoint that far low).
        'px4_link': s.get('px4_link'),
        'landed': s.get('landed'),
        'local_enu': None if s.get('local_enu') is None else _r(s['local_enu']),
        'setpoint': None if setpoint is None else _r(setpoint),
        'cam': None if cam is None else {'p': _r(cam[0]), 'q': _r(cam[1], 5)},
        'vio': vio,
    }, separators=(',', ':')) + '\n'


class FrameServer:
    """Newline JSON over TCP to every connected client. A client that stalls or
    goes away is dropped, so it can never hold up the node."""

    def __init__(self, port, host='0.0.0.0', on_connect=None):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.listen(4)
        self.port = self.sock.getsockname()[1]
        self.on_connect = on_connect
        self.clients = []
        self.lock = threading.Lock()
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            try:
                conn, addr = self.sock.accept()
            except OSError:
                return
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            conn.settimeout(0.2)
            with self.lock:
                self.clients.append(conn)
            if self.on_connect:
                self.on_connect(addr)

    def send(self, line):
        data = line.encode()
        with self.lock:
            alive = []
            for conn in self.clients:
                try:
                    conn.sendall(data)
                    alive.append(conn)
                except OSError:
                    conn.close()
            self.clients = alive

    @property
    def n_clients(self):
        with self.lock:
            return len(self.clients)

    def close(self):
        self.sock.close()
        with self.lock:
            for conn in self.clients:
                conn.close()
            self.clients = []


def main(args=None):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from sensor_msgs.msg import Image, Range
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    from navless_nav.course_mission import (ODOM_TOPIC, POSE_TOPIC,
                                            STATE_TOPIC, VisualSlamStatus)

    class MirrorRelay(Node):
        def __init__(self):
            super().__init__('navless_mirror_relay')
            # dynamic_typing, as in course_mission: `img_hz:=2` or `rate_hz:=20` arrive as
            # ints, and a typed double parameter would kill the relay at startup.
            any_type = ParameterDescriptor(dynamic_typing=True)
            self.declare_parameter('port', 8775, any_type)   # 8765 is the organisers' status agent
            self.declare_parameter('rate_hz', 30.0, any_type)
            self.declare_parameter('img_hz', 5.0, any_type)
            self.declare_parameter('img_topic', '/oak/left/image_mono')
            self.declare_parameter('bind', '0.0.0.0')
            port = int(self.get_parameter('port').value)
            rate = float(self.get_parameter('rate_hz').value)
            self.img_period = 1.0 / max(float(self.get_parameter('img_hz').value), 1e-9)
            self.pose = self.pose_at = self.vo_state = self.setpoint = self.cam = None
            self.state = {}
            self.tof = {'left': None, 'right': None}
            self.tof_at = {'left': None, 'right': None}
            self.odom = OdomWatch()
            self.gate = self.gate_at = None
            self.img_at = -1e9
            self.create_subscription(Range, '/tube/left/range',
                                     lambda m: self.tof_cb('left', m), qos_profile_sensor_data)
            self.create_subscription(Range, '/tube/right/range',
                                     lambda m: self.tof_cb('right', m), qos_profile_sensor_data)
            if float(self.get_parameter('img_hz').value) > 0:
                self.create_subscription(Image, str(self.get_parameter('img_topic').value),
                                         self.image_cb, qos_profile_sensor_data)
            self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 10)
            self.create_subscription(String, STATE_TOPIC, self.state_cb, 10)
            self.create_subscription(PoseStamped, POSE_TOPIC, self.setpoint_cb, 10)
            # Koji's odometry gate (hardware_slam/odom_gate.py), when it runs: JSON counts
            self.create_subscription(String, '/mapping/gate_status', self.gate_cb, 10)
            if VisualSlamStatus is not None:
                self.create_subscription(VisualSlamStatus, '/visual_slam/status',
                                         self.status_cb, 10)
            static = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(TFMessage, '/tf_static', self.tf_cb, static)
            bind = str(self.get_parameter('bind').value)
            self.server = FrameServer(port, host=bind, on_connect=lambda a: self.get_logger().info(
                f'mirror client connected from {a[0]}:{a[1]}'))
            self.create_timer(1.0 / rate, self.tick)
            self.get_logger().info(
                f'mirror relay on TCP {bind}:{self.server.port} at {rate:.0f} Hz '
                '(read-only: publishes nothing)')

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def odom_cb(self, msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            self.pose = ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
            self.pose_at = self.now()
            self.odom.update(self.pose_at, (p.x, p.y, p.z))

        def status_cb(self, msg):
            self.vo_state = int(msg.vo_state)

        def state_cb(self, msg):
            try:
                self.state = json.loads(msg.data)
            except ValueError:
                pass

        def setpoint_cb(self, msg):
            p = msg.pose.position
            self.setpoint = (p.x, p.y, p.z)

        def gate_cb(self, msg):
            try:
                d = json.loads(msg.data)
            except ValueError:
                return
            if isinstance(d, dict):
                self.gate = {'accepted': d.get('accepted'), 'rejected': d.get('rejected')}
                self.gate_at = self.now()

        def tof_cb(self, side, msg):
            self.tof[side] = float(msg.range)
            self.tof_at[side] = self.now()

        def image_cb(self, msg):
            now = self.now()
            if now - self.img_at < self.img_period or not self.server.n_clients:
                return
            self.img_at = now
            try:
                import cv2
                import numpy as np
                if msg.encoding in ('mono8', '8UC1'):
                    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
                elif msg.encoding in ('bgr8', 'rgb8'):
                    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY if msg.encoding == 'bgr8' else cv2.COLOR_RGB2GRAY)
                else:
                    return
                small = cv2.resize(img, (320, int(320 * msg.height / msg.width)), interpolation=cv2.INTER_AREA)
                ok, jpg = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    self.server.send(make_image_frame(now, jpg.tobytes(), small.shape[1], small.shape[0]))
            except Exception as exc:     # never let a thumbnail take the relay down
                self.get_logger().warn(f'camera thumbnail failed: {exc}', throttle_duration_sec=10.0)

        def tf_cb(self, msg):
            for tf in msg.transforms:
                if (tf.header.frame_id == CAMERA_PARENT
                        and tf.child_frame_id == CAMERA_CHILD):
                    t, r = tf.transform.translation, tf.transform.rotation
                    self.cam = ((t.x, t.y, t.z), (r.x, r.y, r.z, r.w))

        def tick(self):
            if self.pose is None or not self.server.n_clients:
                return
            now = self.now()
            tof = None
            if any(t is not None for t in self.tof_at.values()):
                tof = (self.tof['left'], self.tof['right'],
                       *(None if self.tof_at[s] is None else now - self.tof_at[s]
                         for s in ('left', 'right')))
            gate = None if self.gate is None else dict(self.gate, age=round(now - self.gate_at, 3))
            self.server.send(make_frame(now, now - self.pose_at, self.pose,
                                        self.vo_state, self.state,
                                        self.setpoint, self.cam, tof,
                                        vio=self.odom.block(now, gate)))

    rclpy.init(args=args)
    node = MirrorRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
