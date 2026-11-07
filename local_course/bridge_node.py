#!/usr/bin/env python3
"""Local SITL RTAB-Map gate/EV adapter. Never issues vehicle commands.

Run with use_sim_time:=true on isolated ROS_DOMAIN_ID=220. Gravity alignment
must be established by camera mounting/startup, since IMU fusion is disabled.
Restart this node after world reset (backward clock). No velocity fusion.
NED requires calibrated_start=true and a reviewed level launch bearing.
"""
import json
import time
from collections import OrderedDict

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import OdomInfo
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition
from std_msgs.msg import String
try:
    from .bridge_core import Gate, convert, stamp_ns
except ImportError:
    from bridge_core import Gate, convert, stamp_ns


class Bridge(Node):
    def __init__(self):
        super().__init__('local_course_ev_bridge')
        if not self.get_parameter('use_sim_time').value:
            raise RuntimeError('Local SITL bridge requires use_sim_time:=true')
        self.declare_parameter('calibrated_start', False)
        self.declare_parameter('start_bearing_deg', 0.0)
        self.calibrated_bearing = (float(self.get_parameter('start_bearing_deg').value)
                                   if self.get_parameter('calibrated_start').value else None)
        if self.calibrated_bearing is not None:
            import math
            if not math.isfinite(self.calibrated_bearing):
                raise ValueError('Calibrated launch bearing must be finite')
            self.get_logger().warn('NED fixture calibration assumes level stationary launch with known north bearing; no compass/truth correction')
        self.gate = Gate()
        self.odoms, self.infos = OrderedDict(), OrderedDict()
        self.last_receipt = 0.0
        self.px4_clock = None
        self.last_lost_stamp = 0
        self.counts = {'accepted':0, 'rejected':0}
        self.ev = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', qos_profile_sensor_data)
        self.validated = self.create_publisher(Odometry, '/local_course/validated_odometry', 10)
        self.diag = self.create_publisher(String, '/local_course/bridge_status', 10)
        self.create_subscription(Odometry, '/local_course/vo/odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(OdomInfo, '/local_course/vo/odom_info', self.info, qos_profile_sensor_data)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.px4_time, qos_profile_sensor_data)
        self.create_timer(0.1, self.report, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def px4_time(self, msg):
        self.px4_clock = (int(msg.timestamp)*1000, time.monotonic())

    def remember(self, cache, key, value):
        cache[key] = value
        while len(cache) > 10:
            cache.popitem(last=False)

    def odom(self, msg):
        key = stamp_ns(msg)
        self.remember(self.odoms, key, (msg, time.monotonic()))
        self.process(key)

    def info(self, msg):
        key = stamp_ns(msg)
        self.remember(self.infos, key, (msg, time.monotonic()))
        if msg.lost:
            self.last_lost_stamp = max(key, self.last_lost_stamp)
            self.gate.lost('rtabmap_lost')
        self.process(key)

    def process(self, key):
        if key not in self.odoms or key not in self.infos:
            return
        msg, received = self.odoms.pop(key)
        info, health_received = self.infos.pop(key)
        wall = time.monotonic()
        # Require same camera sample health; no stale latched tracking flag.
        now = self.get_clock().now().nanoseconds
        # Reject incompatible XRCE epoch clocks or absent/stale autopilot.
        if (self.px4_clock is None or wall-self.px4_clock[1] > 1.0
                or abs(now-self.px4_clock[0]) > 500_000_000):
            self.gate.lost('px4_clock_unverified')
            self.counts['rejected'] += 1
            return
        self.gate.watchdog(now, wall-self.last_receipt)
        good = self.gate.accept(msg, now, not info.lost and key > self.last_lost_stamp,
                                max(0.0, (now-key)*1e-9),
                                wall-min(received, health_received) <= 2.0)
        self.counts['accepted' if good else 'rejected'] += 1
        if not good:
            return
        self.last_receipt = wall
        out = VehicleOdometry()
        for field, value in convert(msg, now, self.gate.reset, self.calibrated_bearing).items():
            setattr(out, field, value)
        self.validated.publish(msg)
        self.ev.publish(out)

    def report(self):
        self.gate.watchdog(self.get_clock().now().nanoseconds,
                           time.monotonic()-self.last_receipt)
        msg = String()
        msg.data = json.dumps(dict(self.counts, tracking=self.gate.tracking,
                                   reason=self.gate.reason, reset_counter=self.gate.reset))
        self.diag.publish(msg)


def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
