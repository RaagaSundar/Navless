"""Stream the drone's live pose to a laptop for the Gazebo mirror (gazebo_sim/mirror/).

Read-only: subscribes, never publishes, never touches /fmu/*. Run it inside the stack
container with the team overlay sourced, with the drone on the bench or carried, props off:

    ros2 run navless_nav mirror_relay                          # TCP 8765, 30 Hz
    ros2 run navless_nav mirror_relay --ros-args -p rate_hz:=20

The laptop connects to <drone-ip>:8765. The drone never connects out, so this works
from WSL2 behind NAT, over Wi-Fi or USB (192.168.55.1). It gets one JSON object per line:

    {"t": 12.34, "age": 0.02,                   receive time, odometry age (s)
     "p": [x, y, z], "q": [x, y, z, w],         cuVSLAM odom pose, ENU (start frame)
     "vo_state": 1,                             0 unknown, 1 tracking, 2 lost
     "armed": false, "offboard": false, "nav": "POSCTL",
     "setpoint": [x, y, z] | null,              last pose this team's node commanded
     "cam": {"p": [...], "q": [...]} | null}    base_link -> camera mount (static TF)
"""
import json
import socket
import threading

CAMERA_PARENT = 'base_link'
CAMERA_CHILD = 'oak_parent_frame'      # the stack's camera mount (left lens)


def _r(v, n=4):
    return [round(float(x), n) for x in v]


def make_frame(t, age, pose, vo_state=None, state=None, setpoint=None, cam=None):
    """One newline-terminated JSON frame (see the module docstring)."""
    p, q = pose
    s = state or {}
    return json.dumps({
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
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    from navless_nav.course_mission import (ODOM_TOPIC, POSE_TOPIC,
                                            STATE_TOPIC, VisualSlamStatus)

    class MirrorRelay(Node):
        def __init__(self):
            super().__init__('navless_mirror_relay')
            self.declare_parameter('port', 8765)
            self.declare_parameter('rate_hz', 30.0)
            port = int(self.get_parameter('port').value)
            rate = float(self.get_parameter('rate_hz').value)
            self.pose = self.pose_at = self.vo_state = self.setpoint = self.cam = None
            self.state = {}
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
            self.server.send(make_frame(now, now - self.pose_at, self.pose,
                                        self.vo_state, self.state,
                                        self.setpoint, self.cam))

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
