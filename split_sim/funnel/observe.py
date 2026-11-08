"""Passive, fixed-duration funnel observation. All angles are in the optical frame."""
import json
import time
from pathlib import Path
import cv2
import numpy as np
import rclpy
from message_filters import Subscriber, TimeSynchronizer
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as qos
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import Odometry
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from opening import opening


class Observer(Node):
    def __init__(self, **kwargs):
        super().__init__('funnel_observer', **kwargs)
        self.measurement = None
        self.status = (False, 0)
        self.create_subscription(VisualSlamStatus, '/visual_slam/status', self.tracking, qos)
        self.inputs = [Subscriber(self, cls, topic, qos_profile=qos) for cls, topic in (
            (Image, '/oak/left/image_mono'), (CameraInfo, '/oak/left/camera_info'),
            (Odometry, '/visual_slam/tracking/odometry'))]
        self.sync = TimeSynchronizer(self.inputs, 12)
        self.sync.registerCallback(self.sample)
        self.last = 0

    def tracking(self, msg):
        self.status = (msg.vo_state == 1, time.monotonic())

    def sample(self, msg, info, odom):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if stamp <= self.last:
            return
        self.last = stamp
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if (msg.header.frame_id != info.header.frame_id or msg.encoding != 'mono8'
                or (info.width, info.height) != (msg.width, msg.height)):
            return
        image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
        if not Path('left.png').exists():
            cv2.imwrite('left.png', image)
        tracking = self.status[0] and time.monotonic()-self.status[1] < 0.5
        aim = opening(image, info.p[0], info.p[2], info.p[6]) if tracking and 0 <= age < 0.25 else None
        self.measurement = (stamp * 1e-9, aim)
        print(json.dumps(dict(stamp_ns=stamp, sim_age_s=age, tracking=tracking, aim=aim)))
        if aim and not Path('aim.png').exists():
            preview = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            for key in ('left_px', 'right_px'):
                cv2.line(preview, (aim[key], int(info.p[6])-25),
                         (aim[key], int(info.p[6])+25), (0, 170, 255), 2)
            cv2.drawMarker(preview, (round(aim['center_px']), int(info.p[6])), (0, 255, 0))
            cv2.imwrite('aim.png', preview)


if __name__ == '__main__':
    rclpy.init()
    node = Observer()
    deadline = time.monotonic() + 55
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)
    node.destroy_node()
    rclpy.shutdown()
