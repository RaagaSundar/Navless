"""Navless course node: take over on the pilot's arm, take off, hover.

Same contract as skybeta's competitor_example, and nothing else:

    subscribe  /visual_slam/tracking/odometry   where we are (ENU, odom)
    subscribe  /skybeta/state                   what PX4 is doing (JSON)
    publish    /skybeta/setpoint/pose           where we want to be (ENU, odom)

Never publishes to /fmu/*, never arms, never changes mode: flight_interface
owns that, and enters Offboard once when the pilot arms, then switches to
Position on the ground.

States (10 Hz timer):

    WAIT_FOR_OFFBOARD  stream the current position. A setpoint from us in the
                       last second is one of flight_interface's conditions
                       for taking over.
    TAKEOFF            climb straight up. PX4 disarms if we are not off the
                       ground 10 s after arming, so this starts on the first
                       tick of Offboard.
    HOLD               hover at the last target.
    PAUSED             the pilot took over. Nothing advances; once the
                       vehicle is disarmed we go back to waiting.
"""
import json
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

IDLE_STATES = ('WAIT_FOR_OFFBOARD', 'PAUSED')


class CourseMission(Node):

    def __init__(self, **kwargs):
        super().__init__('navless_nav', **kwargs)

        self.declare_parameter('takeoff_altitude_m', 0.85)
        self.declare_parameter('arrival_radius_m', 0.25)
        self.declare_parameter('takeoff_timeout_s', 15.0)
        self.declare_parameter('publish_rate_hz', 10.0)

        def param(name):
            return float(self.get_parameter(name).value)

        self.takeoff_altitude = param('takeoff_altitude_m')
        self.arrival_radius = param('arrival_radius_m')
        self.takeoff_timeout = param('takeoff_timeout_s')
        rate_hz = param('publish_rate_hz')

        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/skybeta/setpoint/pose', 10)
        self.create_subscription(
            Odometry, '/visual_slam/tracking/odometry', self.odometry_cb, 10)
        self.create_subscription(String, '/skybeta/state', self.state_cb, 10)

        self.state = 'WAIT_FOR_OFFBOARD'
        self.state_since = self.now_s()
        self.target = None
        self.position = None
        self.offboard_active = False
        self.armed = False
        self.log_once = set()

        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            f'navless_nav ready: take off to {self.takeoff_altitude:.2f} m. '
            'Waiting for the pilot to arm, then switch to Position...')

    # -- inputs ----------------------------------------------------------
    def odometry_cb(self, msg):
        p = msg.pose.pose.position
        self.position = (float(p.x), float(p.y), float(p.z))

    def state_cb(self, msg):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        self.offboard_active = bool(state.get('offboard_active', False))
        self.armed = bool(state.get('armed', False))
        if state.get('clamped'):
            self.info_once(
                'clamped:' + ','.join(state['clamped']),
                f'flight_interface is CLAMPING our setpoints '
                f'({", ".join(state["clamped"])}); envelope '
                f'{state.get("envelope")}')

    # -- outputs ---------------------------------------------------------
    def send(self, x, y, z):
        """One ENU pose setpoint in odom. Zero quaternion = hold heading."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 0.0
        self.setpoint_pub.publish(msg)

    # -- helpers ---------------------------------------------------------
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def info_once(self, key, message):
        if key not in self.log_once:
            self.log_once.add(key)
            self.get_logger().info(message)

    def arrived(self, target):
        return math.dist(self.position, target) <= self.arrival_radius

    def goto(self, state, target=None):
        self.get_logger().info(f'{self.state} -> {state}')
        self.state = state
        self.state_since = self.now_s()
        self.target = target

    # -- state machine ---------------------------------------------------
    def tick(self):
        if self.position is None:
            self.info_once(
                'no_odom', 'no /visual_slam/tracking/odometry yet; nothing '
                'is commanded until cuVSLAM is tracking.')
            return
        if self.state not in IDLE_STATES and not self.offboard_active:
            self.get_logger().warn(
                f'Offboard lost in {self.state}: the pilot has control. '
                'Paused until the vehicle is disarmed.')
            self.goto('PAUSED')
        getattr(self, '_state_' + self.state.lower())()

    def _state_wait_for_offboard(self):
        if not self.offboard_active:
            self.send(*self.position)
            return
        x, y, _ = self.position
        self.goto('TAKEOFF', (x, y, self.takeoff_altitude))
        self._state_takeoff()       # climb on this tick, not the next one

    def _state_takeoff(self):
        self.send(*self.target)
        if self.arrived(self.target):
            self.goto('HOLD', self.target)
        elif self.now_s() - self.state_since > self.takeoff_timeout:
            self.get_logger().error(
                f'takeoff target not reached in {self.takeoff_timeout:.0f} s '
                f'({math.dist(self.position, self.target):.2f} m off); '
                'holding here. Check `clamped` in /skybeta/state and VIO.')
            self.goto('HOLD', self.position)

    def _state_hold(self):
        self.send(*self.target)

    def _state_paused(self):
        # PX4 ignores setpoints outside Offboard; keep the stream up so a
        # later arming can take over again, and re-arm the mission only
        # once the vehicle has been disarmed.
        self.send(*self.position)
        if not self.armed:
            self.goto('WAIT_FOR_OFFBOARD')


def main(args=None):
    rclpy.init(args=args)
    node = CourseMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
