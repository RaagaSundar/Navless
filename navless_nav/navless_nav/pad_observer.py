"""Landing-pad centre from the forward OAK camera -> /team/pad/center.

Per frame (rate limited): Canny edges of the left mono image -> undistorted
rays -> floor points at the odometry height (line_reference's
floor_points) -> pad_reference.fit_pad. A fix is published as PointStamped,
frame 'odom' (the raw odometry frame), stamped with the image time.

Inputs: /oak/left/image_raw + camera_info, the static TF base_link -> the
camera's optical frame, and the odometry course_mission flies on (pose at
the image time, within 50 ms). Publishes nothing else: no setpoints,
nothing on /fmu/in/*. course_mission only uses it with pad_vision:=true.
"""
import collections
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

from navless_nav.route import REPO_ROOT  # noqa: F401  (puts the repo on sys.path)
from line_reference.reference import floor_points
from navless_nav import pad_reference
from navless_nav.pad_reference import fit_pad
from rclpy.executors import ExternalShutdownException

DISTORTION_MODELS = ('plumb_bob', 'rational_polynomial', '')


def quat_to_rot(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def gray(msg):
    buf = np.frombuffer(msg.data, np.uint8)
    if msg.encoding.lower() == 'mono8':
        return buf.reshape(msg.height, msg.step)[:, :msg.width]
    if msg.encoding.lower() in ('bgr8', 'rgb8'):
        a = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(a, cv2.COLOR_BGR2GRAY if msg.encoding.lower() == 'bgr8'
                            else cv2.COLOR_RGB2GRAY)
    raise ValueError(f'unsupported encoding {msg.encoding}')


class PadObserver(Node):

    def __init__(self):
        super().__init__('pad_observer')
        p = lambda n, v: self.declare_parameter(n, v).value   # noqa: E731
        self.rate_hz = float(p('rate_hz', 5.0))
        self.base_frame = p('base_frame', 'base_link')
        # Odometry z is measured from where it started; this is that point's
        # height above the floor (course_mission's start_height_m). An error
        # here only scales the view, and fit_pad's known ring sizes undo that.
        self.start_height = float(p('start_height_m', 0.07))
        self.canny = (float(p('canny_low', 50.0)), float(p('canny_high', 150.0)))
        self.max_points = int(p('max_points', 6000))
        self.max_range = float(p('max_range_m', 3.0))
        # Image age allowed when the fit finishes; the OAK stream alone has run ~0.12 s old.
        self.max_age = float(p('max_age_s', 0.3))
        # Diagnostics: save each processed frame's floor points here (npz), for offline tuning.
        self.debug_dir = str(p('debug_dir', ''))
        self.odom = collections.deque(maxlen=100)      # (stamp, position, R odom<-base)
        self.info = None
        self.last = -math.inf
        self.warned = set()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PointStamped, '/team/pad/center', 10)
        self.create_subscription(Odometry, p('odometry_topic', '/visual_slam/tracking/odometry'),
                                 self.odom_cb, 50)
        self.create_subscription(CameraInfo, p('camera_info_topic', '/oak/left/camera_info'),
                                 self.info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, p('image_topic', '/oak/left/image_raw'),
                                 self.image_cb, qos_profile_sensor_data)
        self.get_logger().info('pad_observer: watching for the landing pad on /team/pad/center.')

    def warn_once(self, key, text):
        if key not in self.warned:
            self.warned.add(key)
            self.get_logger().warn(text)

    def odom_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.odom.append((stamp_s(msg.header), np.array([p.x, p.y, p.z], float),
                          quat_to_rot(q.x, q.y, q.z, q.w)))

    def info_cb(self, msg):
        self.info = msg

    def pose_at(self, t):
        if not self.odom:
            return None
        best = min(self.odom, key=lambda o: abs(o[0] - t))
        return best if abs(best[0] - t) <= 0.05 else None

    def image_cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last < 1.0 / self.rate_hz:
            return
        self.last = now
        t = stamp_s(msg.header)
        info = self.info
        if info is None or info.distortion_model not in DISTORTION_MODELS:
            self.warn_once('info', f'no usable CameraInfo (model '
                           f'{getattr(info, "distortion_model", None)}): no pad fixes.')
            return
        pose = self.pose_at(t)
        if pose is None:
            self.warn_once('odom', 'no odometry within 50 ms of the image: no pad fixes.')
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time())
        except tf2_ros.TransformException as exc:
            self.warn_once('tf', f'no TF {self.base_frame} <- {msg.header.frame_id} ({exc}).')
            return
        tr, rq = tf.transform.translation, tf.transform.rotation
        _, pos, r_ob = pose
        height = pos[2] + self.start_height
        if height <= 0.1:
            return

        edges = cv2.Canny(cv2.GaussianBlur(gray(msg), (5, 5), 0), *self.canny)
        v, u = np.nonzero(edges)
        if len(u) > self.max_points:
            keep = np.random.default_rng().choice(len(u), self.max_points, replace=False)
            u, v = u[keep], v[keep]
        if len(u) == 0:
            return
        k = np.array(info.k, float).reshape(3, 3)
        d = np.array(info.d, float) if len(info.d) else np.zeros(5)
        rays = cv2.undistortPoints(np.column_stack([u, v]).astype(np.float64)[:, None, :], k, d)
        r_oo = r_ob @ quat_to_rot(rq.x, rq.y, rq.z, rq.w)          # odom <- optical
        cam = r_ob @ np.array([tr.x, tr.y, tr.z])                   # camera from base, odom axes
        pts = floor_points([tuple(x) for x in rays[:, 0, :]], (1.0, 1.0, 0.0, 0.0),
                           r_oo.tolist(), tuple(cam), height, self.max_range)
        if self.debug_dir:
            np.savez(f'{self.debug_dir}/{t:.3f}.npz', points=np.asarray(pts), cam=cam, pos=pos,
                     height=height)
        fix = fit_pad(pts, t, self.get_clock().now().nanoseconds * 1e-9, origin_xy=cam[:2],
                      max_age_s=self.max_age)
        if fix is None:
            self.get_logger().info(f'no pad: {pad_reference.last_reject}', throttle_duration_sec=2.0)
            return
        out = PointStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'odom'
        out.point.x = float(pos[0] + fix.center_xy[0])
        out.point.y = float(pos[1] + fix.center_xy[1])
        out.point.z = float(fix.scale)       # diagnostic: true/measured size
        self.pub.publish(out)
        self.get_logger().info(
            f'pad at odom ({out.point.x:.2f}, {out.point.y:.2f}): {fix.rings} rings, '
            f'{fix.edges} edges, rms {fix.rms_m * 100:.1f} cm, scale {fix.scale:.3f}',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = PadObserver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
