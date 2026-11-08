#!/usr/bin/env python3
"""Health-only adapter for the existing Jetson odometry gate. Never sends poses or commands."""
import json
import math
import time


class GateHealth:
    """Require new validated samples AND fresh, healthy gate reports."""
    def __init__(self):
        self.sample_stamp = self.sample_received = None
        self.sample_valid = self.report_valid = False
        self.report_received = self.accepted_advanced = None
        self.accepted = None
        self.last_ros = None

    def observe_clock(self, now):
        if self.last_ros is not None and now < self.last_ros:
            self.__init__()
        self.last_ros = now

    def odometry(self, msg, ros_now, wall_now):
        self.observe_clock(ros_now)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        covariance = list(msg.pose.covariance)
        diagonal = [covariance[i] for i in (0, 7, 14, 21, 28, 35)]
        valid = (msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_link'
                 and all(math.isfinite(v) for v in (*values, *covariance))
                 and abs(sum(v*v for v in values[3:])-1.0) <= .02
                 and all(v >= 0 for v in diagonal) and diagonal != [1.0]*6
                 and stamp > 0 and 0 <= ros_now-stamp <= .25)
        self.sample_valid = valid
        if valid and (self.sample_stamp is None or stamp > self.sample_stamp):
            self.sample_stamp, self.sample_received = stamp, wall_now

    def report(self, text, wall_now):
        try:
            data = json.loads(text)
            accepted, age = data['accepted'], data['sample_age_ms']
            valid = (type(accepted) is int and accepted > 0
                     and type(age) in (int, float) and math.isfinite(age) and 0 <= age <= 250
                     and data['tracking'] is True and data.get('quarantined', False) is False
                     and data.get('landed_bootstrap', True) is True)
        except (ValueError, KeyError, TypeError):
            self.report_valid = False
            return
        self.report_valid = valid
        self.report_received = wall_now
        if valid:
            if self.accepted is not None and accepted < self.accepted:
                self.sample_valid = False  # Gate restarted: earn a new validated sample.
            if self.accepted is None or accepted != self.accepted:
                self.accepted_advanced = wall_now
            self.accepted = accepted

    def healthy(self, ros_now, wall_now):
        self.observe_clock(ros_now)
        return bool(self.sample_valid and self.report_valid
                    and self.sample_stamp is not None and 0 <= ros_now-self.sample_stamp <= .25
                    and self.sample_received is not None and 0 <= wall_now-self.sample_received <= .25
                    and self.report_received is not None and 0 <= wall_now-self.report_received <= 1.5
                    and self.accepted_advanced is not None and 0 <= wall_now-self.accepted_advanced <= 1.5)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String

    class HealthNode(Node):
        def __init__(self):
            super().__init__('test_flight_slam_health')
            self.gate = GateHealth()
            odom = self.declare_parameter('validated_odom_topic', '/mapping/validated_odometry').value
            status = self.declare_parameter('gate_status_topic', '/mapping/gate_status').value
            self.output = self.declare_parameter('vision_health_topic', '/onboard/vision_health').value
            self.pub = self.create_publisher(Bool, self.output, 1)
            self.create_subscription(Odometry, odom, self.odom, qos_profile_sensor_data)
            self.create_subscription(String, status, lambda m: self.gate.report(m.data, time.monotonic()), qos_profile_sensor_data)
            self.create_timer(.1, self.publish)

        def now_seconds(self):
            return self.get_clock().now().nanoseconds*1e-9

        def odom(self, msg):
            self.gate.odometry(msg, self.now_seconds(), time.monotonic())

        def publish(self):
            # Refuse ambiguous health ownership. This node never changes the odometry gate.
            okay = self.count_publishers(self.output) == 1 and self.gate.healthy(self.now_seconds(), time.monotonic())
            self.pub.publish(Bool(data=okay))

    rclpy.init()
    node = HealthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
