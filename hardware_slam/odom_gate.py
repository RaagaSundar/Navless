#!/usr/bin/env python3
"""Independent, fail-closed odometry relay. No HTTP, point clouds or commands.

On top of map_server.valid_odometry (tracking, age, frames, finite, covariance), every sample
must continue the last one it relays (navigation_integrity.PoseIntegrity, source time): a jump
or a backwards source clock latches a fault, and nothing more is relayed until this process is
restarted (on the ground: a SLAM reset or mode switch needs that too). /mapping/gate_status
reports the fault. This only protects PX4 if the approved bridge consumes
/mapping/validated_odometry (HANDHELD_SLAM_PX4_GATE=1); it never reroutes anything itself.
"""
import json
import os
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from map_server import valid_odometry

try:        # installed next to this file on the drone (see README)
    from navigation_integrity import PoseIntegrity
except ImportError:     # the repo checkout; without it the gate does not start
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    '..', 'navless_nav', 'navless_nav'))
    from navigation_integrity import PoseIntegrity


class OdomGate(Node):
    def __init__(self):
        super().__init__('handheld_odometry_gate')
        self.tracking = False
        self.status_time = 0
        self.accepted = self.rejected = 0
        self.last_stamp = 0
        self.sample_age_ms = None
        # valid_odometry's own limits: 250 ms old, 50 ms ahead (camera vs node clock)
        self.integrity = PoseIntegrity(max_age_s=0.25, future_tolerance_s=0.05)
        self.pub = self.create_publisher(Odometry, '/mapping/validated_odometry', 10)
        self.diag = self.create_publisher(String, '/mapping/gate_status', 1)
        self.status_sub = self.create_subscription(VisualSlamStatus, '/visual_slam/status', self.status, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.odom, qos_profile_sensor_data)
        self.create_timer(1.0, self.report)

    def status(self, msg):
        self.tracking = msg.vo_state == 1
        self.status_time = time.monotonic()

    def odom(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        self.sample_age_ms = round((now-stamp*1e-9)*1000, 2)
        # PoseIntegrity drops a repeated stamp and latches a backwards one (after the first relay).
        if valid_odometry(msg, now, self.tracking, time.monotonic()-self.status_time):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            if self.integrity.observe(stamp_s=stamp * 1e-9, now_s=now, frame_id=msg.header.frame_id,
                                      child_frame_id=msg.child_frame_id, position=(p.x, p.y, p.z),
                                      quaternion=(q.x, q.y, q.z, q.w)):
                self.last_stamp = stamp
                self.pub.publish(msg)
                self.accepted += 1
                return
            if self.integrity.fault:
                self.get_logger().error(f'odometry discontinuity ({self.integrity.fault}): relaying '
                                        'nothing more until this gate is restarted.', once=True)
        self.rejected += 1

    def report(self):
        msg = String()
        msg.data = json.dumps({'accepted':self.accepted, 'rejected':self.rejected,
                               'sample_age_ms':self.sample_age_ms,
                               'tracking':self.tracking and time.monotonic()-self.status_time<0.25,
                               'fault':self.integrity.fault, 'integrity':self.integrity.reason})
        self.diag.publish(msg)


if __name__ == '__main__':
    rclpy.init()
    node = OdomGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
