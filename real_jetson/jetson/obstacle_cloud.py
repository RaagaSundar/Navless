"""Time-align stereo observations with measured odometry and the calibrated camera mount."""
import numpy as np
from cloud_projection import PoseHistory, stamp_s, decode_xyz


class ObstacleCloud:
    def __init__(self, node, receive):
        from sensor_msgs.msg import PointCloud2
        from rclpy.qos import qos_profile_sensor_data
        from tf2_ros import Buffer, TransformListener
        self.node, self.receive = node, receive
        self.poses = PoseHistory()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, node)
        self.stamp = None
        self.reason = 'waiting for calibrated stereo points'
        self.subscription = node.create_subscription(
            PointCloud2, '/team/stereo/points', self.callback, qos_profile_sensor_data)

    def record(self, msg):
        try:
            self.poses.add(msg)
        except (AttributeError, ValueError, TypeError) as exc:
            self.reason = str(exc)

    def callback(self, msg):
        from rclpy.time import Time
        from tf2_ros import TransformException
        try:
            now = self.node.get_clock().now().nanoseconds * 1e-9
            stamp = stamp_s(msg.header)
            if self.stamp is not None and stamp <= self.stamp:
                raise ValueError('non-advancing cloud stamp')
            pose = self.poses.at(stamp, now)
            if pose.child != 'base_link' or not msg.header.frame_id:
                raise ValueError('missing camera or body frame')
            tf = self.tf.lookup_transform(pose.child, msg.header.frame_id, Time.from_msg(msg.header.stamp))
            if msg.width * msg.height > 500000:
                raise ValueError('oversize stereo cloud')
            coords = decode_xyz(msg)
            if not (np.isfinite(coords).all(axis=1) & (coords[:,2] >= .30) & (coords[:,2] <= 2.8)).any():
                raise ValueError('no usable stereo returns')
            self.receive(msg, pose, tf, now)
            self.stamp, self.reason = stamp, ''
        except (TransformException, ValueError, AttributeError, TypeError, OverflowError) as exc:
            self.reason = str(exc)

    def problem(self, now):
        if self.node.count_publishers('/team/stereo/points') != 1:
            return 'expected one stereo obstacle cloud publisher'
        if self.reason:
            return self.reason
        if self.stamp is None or not 0 <= now-self.stamp <= .75:
            return 'stereo obstacle stream stale'
        return None
