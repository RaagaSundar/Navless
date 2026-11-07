#!/usr/bin/env python3
"""Independent, fail-closed odometry relay. No HTTP, point clouds or commands."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from map_server import valid_odometry
from source_admission import SourceAdmission, LandedBootstrap
from px4_msgs.msg import VehicleStatus, VehicleLandDetected


class OdomGate(Node):
    def __init__(self):
        super().__init__('handheld_odometry_gate')
        self.tracking = False
        self.status_time = 0
        self.admission = SourceAdmission()
        self.bootstrap = LandedBootstrap()
        self.px4_status = self.px4_land = None
        qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 lambda m:setattr(self,'px4_status',m), qos)
        self.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected',
                                 lambda m:setattr(self,'px4_land',m), qos)
        self.accepted = self.rejected = 0
        self.last_stamp = 0
        self.sample_age_ms = None
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
        if stamp > self.last_stamp and valid_odometry(msg, now, self.tracking, time.monotonic()-self.status_time):
            self.last_stamp = stamp
            p = msg.pose.pose.position
            xyz = (p.x, p.y, p.z)
            if (not self.bootstrap.observe(now, stamp*1e-9, xyz, self.px4_status, self.px4_land)
                    or not self.admission.accept(stamp * 1e-9, xyz)):
                self.rejected += 1
                return
            self.pub.publish(msg)
            self.accepted += 1
        else:
            self.rejected += 1

    def report(self):
        msg = String()
        msg.data = json.dumps({'accepted':self.accepted, 'rejected':self.rejected,
                               'sample_age_ms':self.sample_age_ms, 'quarantined':self.admission.quarantined,
                               'admission_reason':self.admission.reason,'landed_bootstrap':self.bootstrap.ready,
                               'tracking':self.tracking and time.monotonic()-self.status_time<0.25})
        self.diag.publish(msg)


if __name__ == '__main__':
    rclpy.init()
    node = OdomGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
