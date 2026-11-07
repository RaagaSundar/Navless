"""Stream the drone's live pose to a laptop for the Gazebo mirror (gazebo_sim/mirror/).

Read-only: subscribes, never publishes, never touches /fmu/*. Run it inside the stack
container with the team overlay sourced, with the drone on the bench or carried, props off:

    ros2 run navless_nav mirror_relay                          # TCP 8775, 30 Hz
    ros2 run navless_nav mirror_relay --ros-args -p rate_hz:=20

The laptop connects to <drone-ip>:8775. The drone never connects out, so this works
from WSL2 behind NAT, over Wi-Fi or USB (192.168.55.1). It gets one JSON object per line:

    {"t": 12.34, "age": 0.02,                   receive time, odometry age (s)
     "p": [x, y, z], "q": [x, y, z, w],         cuVSLAM odom pose, ENU (start frame)
     "vo_state": 1,                             0 unknown, 1 tracking, 2 lost
     "armed": false, "offboard": false, "nav": "POSCTL",
     "setpoint": [x, y, z] | null,              last pose this team's node commanded
     "cam": {"p": [...], "q": [...]} | null,    base_link -> camera mount (static TF)
     "tof": {"left": m|null, "right": m|null, "age": s} | null,   side ToF (/tube/*/range)
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


def make_image_frame(t, jpeg, width, height):
    """A camera thumbnail line (see the module docstring)."""
    return json.dumps({'img': base64.b64encode(jpeg).decode('ascii'), 'w': int(width),
                       'h': int(height), 't': round(float(t), 3)}, separators=(',', ':')) + '\n'


def make_frame(t, age, pose, vo_state=None, state=None, setpoint=None, cam=None,
               tof=None, node=None):
    """One newline-terminated JSON frame (see the module docstring).
    tof: (left_m, right_m, age_s) with inf/None for no return."""
    p, q = pose
    s = state or {}
    return json.dumps({
        'tof': None if tof is None else {'left': _range(tof[0]), 'right': _range(tof[1]),
                                         'age': None if tof[2] is None else round(float(tof[2]), 3)},
        'node': node,
        't': round(float(t), 3),
        'age': None if age is None else round(float(age), 3),
        'p': _r(p), 'q': _r(q, 5),
        'vo_state': vo_state,
        'armed': bool(s.get('armed', False)),
        'offboard': bool(s.get('offboard_active', False)),
        'nav': s.get('nav_state_name'),
        'setpoint': None if setpoint is None else _r(setpoint),
        'cam': None if cam is None else {'p': _r(cam[0]), 'q': _r(cam[1], 5)},
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
            self.declare_parameter('port', 8775)   # 8765 is the organisers' status agent
            self.declare_parameter('rate_hz', 30.0)
            self.declare_parameter('img_hz', 5.0)
            self.declare_parameter('img_topic', '/oak/left/image_mono')
            port = int(self.get_parameter('port').value)
            rate = float(self.get_parameter('rate_hz').value)
            self.img_period = 1.0 / max(float(self.get_parameter('img_hz').value), 1e-9)
            self.pose = self.pose_at = self.vo_state = self.setpoint = self.cam = None
            self.state = {}
            self.tof = {'left': None, 'right': None}
            self.tof_at = None
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
            if VisualSlamStatus is not None:
                self.create_subscription(VisualSlamStatus, '/visual_slam/status',
                                         self.status_cb, 10)
            static = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(TFMessage, '/tf_static', self.tf_cb, static)
            self.server = FrameServer(port, on_connect=lambda a: self.get_logger().info(
                f'mirror client connected from {a[0]}:{a[1]}'))
            self.create_timer(1.0 / rate, self.tick)
            self.get_logger().info(
                f'mirror relay on TCP {self.server.port} at {rate:.0f} Hz '
                '(read-only: publishes nothing)')

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def odom_cb(self, msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            self.pose = ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
            self.pose_at = self.now()

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

        def tof_cb(self, side, msg):
            self.tof[side] = float(msg.range)
            self.tof_at = self.now()

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
            if self.tof_at is not None:
                tof = (self.tof['left'], self.tof['right'], now - self.tof_at)
            self.server.send(make_frame(now, now - self.pose_at, self.pose,
                                        self.vo_state, self.state,
                                        self.setpoint, self.cam, tof))

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
