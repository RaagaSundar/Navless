#!/usr/bin/env python3
"""A pretend vehicle + flight_interface, for testing basic_flight.py with no drone and no PX4.

Run it on an ISOLATED ROS domain (e.g. ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1), never on the drone's.
It plays the pilot and PX4: arms after arm_after_s, switches to Position (Offboard starts) a moment
later, flies toward /skybeta/setpoint/pose (max speed 0.8 m/s, yaw 60 deg/s, floored at 0.3 m like
the interface) or along /skybeta/setpoint/twist (expires after 1 s), detects a landing when it's on
the floor and still being pushed down, and disarms 2 s later. Publishes /visual_slam/tracking/odometry
and /skybeta/state. Logs a summary line per second and a final JSON line "FAKE_RESULT ...".
"""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class Fake(Node):
    def __init__(self, start_yaw_deg, arm_after_s, fail):
        super().__init__('fake_vehicle')
        self.x, self.y, self.z, self.yaw = 0.0, 0.0, 0.07, math.radians(start_yaw_deg)
        self.vz = 0.0
        self.pose = None
        self.twist, self.twist_at = None, -9.0
        self.t0 = time.monotonic()
        self.arm_after = arm_after_s
        self.armed = self.offboard = False
        self.landed, self.floor_since, self.disarm_at = True, None, None
        self.done, self.fail = False, fail
        self.maxz = 0.0
        self.track = []
        self.odom_pub = self.create_publisher(Odometry, '/visual_slam/tracking/odometry', 10)
        self.state_pub = self.create_publisher(String, '/skybeta/state', 10)
        self.create_subscription(PoseStamped, '/skybeta/setpoint/pose', self.on_pose, 10)
        self.create_subscription(TwistStamped, '/skybeta/setpoint/twist', self.on_twist, 10)
        self.dt = 0.02
        self.create_timer(self.dt, self.step)
        self.last_print = 0.0

    def on_pose(self, m):
        q = m.pose.orientation
        yaw = None if (q.x == q.y == q.z == q.w == 0.0) else math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        self.pose = (m.pose.position.x, m.pose.position.y, max(0.3, m.pose.position.z), yaw)
        self.twist = None

    def on_twist(self, m):
        self.twist, self.twist_at = (m.twist.linear.x, m.twist.linear.y, m.twist.linear.z), time.monotonic()

    def step(self):
        t = time.monotonic() - self.t0
        if not self.armed and not self.done and t > self.arm_after:
            self.armed = True                              # pilot arms (Altitude)
        if self.armed and not self.offboard and not self.done and t > self.arm_after + 1.0 and self.landed:
            self.offboard = True                           # pilot switches to Position
        if self.fail == 'pilot' and self.offboard and t > self.arm_after + 6.0:
            self.offboard = False                          # pilot takes over mid-test
        vx = vy = vz = 0.0
        if self.offboard:
            now = time.monotonic()
            if self.twist is not None and now - self.twist_at <= 1.0:
                vx, vy, vz = self.twist
            elif self.pose is not None:
                px, py, pz, pyaw = self.pose
                d = [px - self.x, py - self.y, pz - self.z]
                n = math.sqrt(sum(c * c for c in d))
                s = min(0.8, n / 0.5)                      # ~P controller, capped
                if n > 1e-6:
                    vx, vy, vz = (c / n * s for c in d)
                if pyaw is not None:
                    e = (pyaw - self.yaw + math.pi) % (2 * math.pi) - math.pi
                    self.yaw += max(-math.radians(60) * self.dt, min(math.radians(60) * self.dt, e))
        self.x += vx * self.dt
        self.y += vy * self.dt
        self.z = max(0.07, self.z + vz * self.dt)
        self.maxz = max(self.maxz, self.z)
        on_floor = self.z <= 0.075
        if self.offboard:
            if on_floor and vz < 0:
                self.floor_since = self.floor_since or t
                if t - self.floor_since > 1.0:
                    self.landed = True
                    self.disarm_at = self.disarm_at or t + 2.0
            else:
                self.floor_since = None
                if self.z > 0.1:
                    self.landed = False
        if self.disarm_at is not None and t >= self.disarm_at and self.armed:
            self.armed = self.offboard = False
            self.done = True
        if not self.offboard and not on_floor and self.done is False and self.fail == 'pilot' and t > self.arm_after + 6.0:
            self.z = max(0.07, self.z - 0.3 * self.dt)     # pilot lands it
            if self.z <= 0.075:
                self.armed, self.done = False, True
        o = Odometry()
        o.header.frame_id, o.child_frame_id = 'odom', 'base_link'
        o.header.stamp = self.get_clock().now().to_msg()
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = self.x, self.y, self.z
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.odom_pub.publish(o)
        self.state_pub.publish(String(data=json.dumps({"armed": self.armed, "offboard_active": self.offboard,
                                                       "landed": self.landed, "clamped": []})))
        self.track.append((round(t, 2), round(self.x, 3), round(self.y, 3), round(self.z, 3), round(math.degrees(self.yaw), 1)))
        if t - self.last_print >= 1.0:
            self.last_print = t
            print(f"t {t:5.1f}  armed {int(self.armed)} offboard {int(self.offboard)} landed {int(self.landed)}  "
                  f"pos ({self.x:+.2f}, {self.y:+.2f}, {self.z:.2f})  yaw {math.degrees(self.yaw):6.1f}", flush=True)


def main():
    start_yaw = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
    limit = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0
    fail = sys.argv[3] if len(sys.argv) > 3 else ''
    rclpy.init()
    f = Fake(start_yaw, 3.0, fail)
    t_end = time.monotonic() + limit
    while rclpy.ok() and time.monotonic() < t_end and not (f.done and time.monotonic() - f.t0 > 2):
        rclpy.spin_once(f, timeout_sec=0.05)
    tr = f.track
    xs, ys = [p[1] for p in tr], [p[2] for p in tr]
    print("FAKE_RESULT " + json.dumps({"done": f.done, "max_z": round(f.maxz, 3), "end": tr[-1] if tr else None,
                                       "x_range": [min(xs), max(xs)], "y_range": [min(ys), max(ys)],
                                       "yaw_range": [min(p[4] for p in tr), max(p[4] for p in tr)]}), flush=True)
    f.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
