"""Stereo tube side fitting + timestamped SLAM odometry adapter.

Portable code consumes PointCloud2, Odometry, tracking status and camera TF.
No simulator imports, truth, arming, PX4 writes or synthetic position fixes.
"""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np


def rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 0.01:
        raise ValueError('invalid xyzw quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class OdomHistory:
    """Interpolate continuous odom; invalidate across gaps, jumps and clock resets."""
    def __init__(self):
        self.samples = deque(maxlen=100)
        self.epoch = 0

    def clear(self):
        self.samples.clear()
        self.epoch += 1

    def add(self, stamp, position, quaternion):
        p, q = np.asarray(position, float), np.asarray(quaternion, float)
        try:
            rotation(q)
        except ValueError:
            self.clear()
            return False
        if p.shape != (3,) or not np.isfinite(p).all() or not math.isfinite(stamp):
            self.clear()
            return False
        if self.samples:
            t0, p0, q0 = self.samples[-1]
            dt = stamp - t0
            angle = 2 * math.acos(min(1.0, abs(float(np.dot(q0, q)))))
            if (dt <= 0 or dt > 0.25 or np.linalg.norm(p-p0) > 3*dt + 0.03
                    or angle > 3*dt + 0.03):
                self.clear()
                # Do not use the discontinuity sample as a valid transform.
                return False
        self.samples.append((stamp, p, q / np.linalg.norm(q)))
        return True

    def at(self, stamp):
        if len(self.samples) < 3 or self.samples[-1][0] - self.samples[0][0] < 0.10:
            return None
        for (ta, pa, qa), (tb, pb, qb) in zip(self.samples, list(self.samples)[1:]):
            if ta <= stamp <= tb:
                f = (stamp-ta)/(tb-ta)
                if np.dot(qa, qb) < 0:
                    qb = -qb
                q = (1-f)*qa + f*qb
                return (1-f)*pa + f*pb, rotation(q / np.linalg.norm(q))
        return None  # never extrapolate a cloud beyond measured odometry


@dataclass
class TubeFit:
    center: np.ndarray
    yaw: float
    width: float
    residual: float
    support: tuple


def _side_line(s, q, rng):
    if len(s) < 30 or np.ptp(s) < 0.30:
        return None
    best = None
    for _ in range(90):
        i, j = rng.choice(len(s), 2, replace=False)
        if abs(s[j]-s[i]) < 0.20:
            continue
        m = (q[j]-q[i])/(s[j]-s[i])
        if abs(m) > math.tan(math.radians(15)):
            continue
        b = q[i]-m*s[i]
        mask = np.abs(q - m*s - b) < 0.025
        if best is None or mask.sum() > best.sum():
            best = mask
    if best is None or best.sum() < max(30, 0.60*len(s)):
        return None
    x, y = s[best], q[best]
    if np.ptp(x) < 0.30:
        return None
    m, b = np.linalg.lstsq(np.column_stack((x, np.ones(len(x)))), y, rcond=None)[0]
    residual = float(np.sqrt(np.mean((y-m*x-b)**2)))
    return m, b, residual, int(best.sum()), float(x.min()), float(x.max())


def fit_tube(points_odom, body_position, axis_yaw=0.0, expected_width=1.45):
    """Fit both sides in a thin horizontal slice through the body.

    A straight, constant-section tube has parallel side lines at a fixed
    height, including an oval tube. Both sides must be observed over a common
    axial span. Floor, roof, one wall, a mouth rim and unstructured clouds
    cannot alone establish a centerline.
    """
    pts, body = np.asarray(points_odom, float), np.asarray(body_position, float)
    if (pts.ndim != 2 or pts.shape[1] != 3 or body.shape != (3,)
            or not np.isfinite(body).all() or not math.isfinite(axis_yaw)
            or not math.isfinite(expected_width) or expected_width <= 0):
        return None
    pts = pts[np.isfinite(pts).all(axis=1)] - body
    a = np.array([math.cos(axis_yaw), math.sin(axis_yaw), 0.0])
    n = np.array([-a[1], a[0], 0.0])
    pts = pts[(np.abs(pts[:, 2]) < 0.10) & (np.linalg.norm(pts, axis=1) < 3.0)]
    # Bound callback cost deterministically; avoid an unbounded RANSAC cloud.
    if len(pts) > 6000:
        pts = pts[np.linspace(0, len(pts)-1, 6000, dtype=int)]
    s, q = pts @ a, pts @ n
    fits = []
    for sign in (-1, 1):
        mask = (sign*q > 0.20) & (sign*q < 1.30)
        line = _side_line(s[mask], q[mask], np.random.default_rng(7))
        if line is None:
            return None
        fits.append(line)
    lo, hi = fits
    if (abs(math.atan(lo[0])-math.atan(hi[0])) > math.radians(4)
            or min(lo[5], hi[5])-max(lo[4], hi[4]) < 0.25):
        return None
    m, b = (lo[0]+hi[0])/2, (lo[1]+hi[1])/2
    width = (hi[1]-lo[1])/math.sqrt(1+m*m)
    if abs(width-expected_width) > 0.18 or max(lo[2], hi[2]) > 0.018:
        return None
    # Avoid extrapolating a measured line far back from distant wall patches.
    if max(lo[4], hi[4]) > 1.5 or min(lo[5], hi[5]) < -1.5:
        return None
    return TubeFit(body+b*n, axis_yaw+math.atan(m), width,
                   max(lo[2], hi[2]), (lo[3], hi[3]))


def main(args=None):
    import json
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import String
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
    import tf2_ros

    def seconds(stamp):
        return stamp.sec + stamp.nanosec*1e-9

    class StereoTube(Node):
        def __init__(self):
            super().__init__('stereo_tube_centering')
            def p(name, default):
                return self.declare_parameter(name, default).value
            self.odom_frame = p('odom_frame', 'odom')
            self.base_frame = p('base_frame', 'base_link')
            self.axis = math.radians(float(p('start_bearing_deg', 0.0)))
            self.width = float(p('expected_width_m', 1.45))
            self.hold_s = min(3.0, max(0.0, float(p('hold_s', 3.0))))
            self.anchor = None
            self.anchor_stamp = None
            self.anchor_epoch = 0
            self.history = OdomHistory()
            self.tracking_stamp = None
            self.tracking = False
            self.last_cloud = -math.inf
            self.pending = None
            self.cloud_received_at = None
            self.tf = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.tf, self)
            self.pub = self.create_publisher(PoseStamped, '/tube/stereo/centerline', 2)
            self.diag = self.create_publisher(String, '/tube/stereo/status', 2)
            self.create_subscription(Odometry, p('odometry_topic', '/mapping/validated_odometry'),
                                     self.odom, qos_profile_sensor_data)
            self.create_subscription(VisualSlamStatus, '/visual_slam/status', self.status,
                                     qos_profile_sensor_data)
            self.create_subscription(PointCloud2, p('cloud_topic', '/team/stereo/points'),
                                     self.cloud, qos_profile_sensor_data)
            self.create_timer(0.02, self.process)

        def now(self):
            return self.get_clock().now().nanoseconds*1e-9

        def reject(self, reason):
            self.anchor = None
            msg = String()
            msg.data = json.dumps({'valid': False, 'reason': reason, 'epoch': self.history.epoch})
            self.diag.publish(msg)

        def status(self, msg):
            # VisualSlamStatus has no header in Isaac ROS 3.2.
            self.tracking_stamp = self.now()
            self.tracking = msg.vo_state == 1
            if not self.tracking:
                self.history.clear()
                self.pending = None
                self.reject('tracking lost')

        def healthy(self):
            return (self.tracking and self.tracking_stamp is not None
                    and 0 <= self.now()-self.tracking_stamp <= 0.25)

        def odom(self, msg):
            stamp = seconds(msg.header.stamp)
            if (not self.healthy() or msg.header.frame_id != self.odom_frame
                    or msg.child_frame_id != self.base_frame or not 0 <= self.now()-stamp <= 0.25):
                self.history.clear()
                self.pending = None
                self.reject('invalid or stale odometry')
                return
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            if not self.history.add(stamp, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)):
                self.pending = None
                self.last_cloud = -math.inf  # allow a new simulation-clock epoch
                self.reject('odometry discontinuity')

        def cloud(self, msg):
            stamp = seconds(msg.header.stamp)
            if stamp <= self.last_cloud or not 0 <= self.now()-stamp <= 0.25:
                self.pending = None
                self.reject('stale or out-of-order cloud')
                return
            self.last_cloud = stamp
            self.cloud_received_at = self.now()
            self.pending = msg  # process once odometry brackets the sensor time

        def process(self):
            msg = self.pending
            if msg is None:
                if self.anchor is not None and (not self.healthy()
                        or self.cloud_received_at is None or self.now()-self.cloud_received_at > 0.30):
                    self.reject('cloud stream or tracking stale')
                return
            stamp = seconds(msg.header.stamp)
            if not self.healthy() or not 0 <= self.now()-stamp <= 0.25:
                self.pending = None
                self.reject('stale cloud or tracking')
                return
            pose = self.history.at(stamp)
            if pose is None:
                return
            self.pending = None
            try:
                tf = self.tf.lookup_transform(self.base_frame, msg.header.frame_id,
                                              Time.from_msg(msg.header.stamp))
                t, q = tf.transform.translation, tf.transform.rotation
                r = rotation((q.x, q.y, q.z, q.w))
                raw = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
                pts = np.asarray(raw).reshape(-1, 3) @ r.T + (t.x, t.y, t.z)
                position, rot = pose
                fit = fit_tube(pts @ rot.T + position, position, self.axis, self.width)
            except (tf2_ros.TransformException, ValueError, TypeError) as exc:
                self.reject(f'invalid cloud/TF: {type(exc).__name__}')
                return
            if fit is None:
                anchor_ok = (self.anchor is not None and self.anchor_epoch == self.history.epoch
                             and 0 <= self.now()-self.anchor_stamp <= self.hold_s
                             and np.linalg.norm(position[:2]-self.anchor.center[:2]) <= 1.5)
                if not anchor_ok:
                    self.reject('insufficient two-wall geometry / expired anchor')
                    return
                diag = String()
                diag.data = json.dumps({'valid': True, 'observed': False,
                                        'anchor_age_s': self.now()-self.anchor_stamp,
                                        'epoch': self.history.epoch})
                self.diag.publish(diag)
                return
            self.anchor, self.anchor_stamp, self.anchor_epoch = fit, stamp, self.history.epoch
            out = PoseStamped()
            out.header.stamp, out.header.frame_id = msg.header.stamp, self.odom_frame
            out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, fit.center)
            out.pose.orientation.z, out.pose.orientation.w = math.sin(fit.yaw/2), math.cos(fit.yaw/2)
            self.pub.publish(out)
            diag = String()
            diag.data = json.dumps({'valid': True, 'observed': True, 'width_m': fit.width, 'residual_m': fit.residual,
                                    'support': fit.support, 'epoch': self.history.epoch})
            self.diag.publish(diag)

    rclpy.init(args=args)
    node = StereoTube()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
