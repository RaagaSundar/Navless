"""Navless course node: take over on the pilot's arm, fly a planner route.

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
    ROUTE              walk route_file's waypoints with a speed-limited
                       carrot (navless_nav/route.py). The carrot freezes
                       while cuVSLAM reports lost tracking or odometry is
                       stale, and resumes when it is back.
    LAND               route done: descend onto its end point (the landing
                       pad) with twist setpoints until PX4 detects the
                       touchdown and disarms (COM_DISARM_LAND). The twist
                       never stops before that: an expired twist makes
                       flight_interface fly back to the last pose, 0.85 m
                       up.
    DONE               landed. Publishes nothing, so a later arming cannot
                       hand the vehicle to us again.
    HOLD               hover at the last target.
    PAUSED             the pilot took over. Nothing advances; once the
                       vehicle is disarmed we go back to waiting.

Every setpoint outside the idle states is pulled inside the net by
net_margin_m: flight_interface clamps altitude only, never x/y.
"""
import json
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String

from navless_nav.route import (Carrot, clamp_to_net, course_to_odom,
                               load_route, resolve_route_path)

try:
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
except ImportError:   # off the drone: fall back to odometry staleness only
    VisualSlamStatus = None

VO_LOST = 2           # VisualSlamStatus.vo_state: 0 unknown, 1 tracking, 2 lost

IDLE_STATES = ('WAIT_FOR_OFFBOARD', 'PAUSED', 'DONE')

PARAMETERS = (
    ('takeoff_altitude_m', 0.85),
    ('arrival_radius_m', 0.25),
    ('takeoff_timeout_s', 15.0),
    ('publish_rate_hz', 10.0),
    # Route: a planner file, and how the drone sat on the launch mark at
    # power-up (nose bearing clockwise from course north; height of the odom
    # origin above the floor).
    ('route_file', 'route_gate_b.json'),
    ('start_bearing_deg', 0.0),
    ('start_height_m', 0.0),
    ('cruise_speed_mps', 0.5),
    ('max_lead_m', 0.6),
    # Refuse to start the route from somewhere else (e.g. a re-arm after the
    # pilot moved the drone): its first point is the launch mark.
    ('max_start_offset_m', 0.6),
    ('stall_timeout_s', 20.0),
    # Keep-out from the net (drone is 0.475 m wide with props).
    ('net_margin_m', 0.4),
    ('odom_timeout_s', 0.5),
    # Landing on the route's last point (scored: 7 points for the pad).
    ('land_at_end', True),
    ('land_speed_mps', 0.3),
    ('land_xy_max_mps', 0.3),     # centring on the pad while descending
)


class CourseMission(Node):

    def __init__(self, **kwargs):
        super().__init__('navless_nav', **kwargs)

        # dynamic_typing: `start_bearing_deg:=90` arrives as an int, and a
        # typed double parameter would kill the node at startup -- which on
        # the day means no takeover at all.
        any_type = ParameterDescriptor(dynamic_typing=True)
        for name, default in PARAMETERS:
            self.declare_parameter(name, default, any_type)

        def param(name):
            return float(self.get_parameter(name).value)

        self.takeoff_altitude = param('takeoff_altitude_m')
        self.arrival_radius = param('arrival_radius_m')
        self.takeoff_timeout = param('takeoff_timeout_s')
        rate_hz = param('publish_rate_hz')
        self.cruise_speed = param('cruise_speed_mps')
        self.max_lead = param('max_lead_m')
        self.max_start_offset = param('max_start_offset_m')
        self.stall_timeout = param('stall_timeout_s')
        self.net_margin = param('net_margin_m')
        self.odom_timeout = param('odom_timeout_s')
        self.land_at_end = str(self.get_parameter('land_at_end').value
                               ).lower() in ('true', '1')
        self.land_speed = param('land_speed_mps')
        self.land_xy_max = param('land_xy_max_mps')
        self.frame = None           # (launch mark xy, start bearing) once known
        self.route = self.load_route(
            str(self.get_parameter('route_file').value),
            param('start_bearing_deg'), param('start_height_m'))
        self.carrot = None
        self.last_tick = None
        self.progress_at = None

        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/skybeta/setpoint/pose', 10)
        self.twist_pub = self.create_publisher(
            TwistStamped, '/skybeta/setpoint/twist', 10)
        self.create_subscription(
            Odometry, '/visual_slam/tracking/odometry', self.odometry_cb, 10)
        self.create_subscription(String, '/skybeta/state', self.state_cb, 10)
        if VisualSlamStatus is not None:
            self.create_subscription(
                VisualSlamStatus, '/visual_slam/status', self.status_cb, 10)

        self.state = 'WAIT_FOR_OFFBOARD'
        self.state_since = self.now_s()
        self.target = None
        self.position = None
        self.odom_at = None
        self.vo_state = None
        self.vio_was_ok = True
        self.offboard_active = False
        self.armed = False
        self.landed = False
        self.log_once = set()

        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            f'navless_nav ready: take off to {self.takeoff_altitude:.2f} m, '
            f'then fly {len(self.route or [])} waypoints at '
            f'{self.cruise_speed:.2f} m/s. Waiting for the pilot to arm, '
            'then switch to Position...')

    def load_route(self, name, bearing_deg, height_m):
        """Odom-frame waypoints, or None (we then only take off and hover)."""
        try:
            path = resolve_route_path(name)
            waypoints, launch, kind = load_route(path)
        except (OSError, ValueError, KeyError) as exc:
            self.get_logger().error(
                f'route not loaded ({exc}); will take off and hover only.')
            return None
        odom = [course_to_odom(p, launch, bearing_deg, height_m)
                for p in waypoints]
        self.frame = (launch, bearing_deg)
        self.get_logger().info(
            f'route {kind} from {path}: {len(odom)} waypoints, launch mark '
            f'{launch}, start bearing {bearing_deg:.1f} deg, ends at odom '
            f'({odom[-1][0]:.2f}, {odom[-1][1]:.2f}, {odom[-1][2]:.2f})')
        return odom

    # -- inputs ----------------------------------------------------------
    def odometry_cb(self, msg):
        p = msg.pose.pose.position
        self.position = (float(p.x), float(p.y), float(p.z))
        self.odom_at = self.now_s()

    def status_cb(self, msg):
        self.vo_state = int(msg.vo_state)

    def vio_ok(self):
        fresh = (self.odom_at is not None
                 and self.now_s() - self.odom_at <= self.odom_timeout)
        return fresh and self.vo_state != VO_LOST

    def state_cb(self, msg):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        self.offboard_active = bool(state.get('offboard_active', False))
        self.armed = bool(state.get('armed', False))
        self.landed = bool(state.get('landed', False))
        if state.get('clamped'):
            self.info_once(
                'clamped:' + ','.join(state['clamped']),
                f'flight_interface is CLAMPING our setpoints '
                f'({", ".join(state["clamped"])}); envelope '
                f'{state.get("envelope")}')

    # -- outputs ---------------------------------------------------------
    def send(self, x, y, z, fence=True):
        """One ENU pose setpoint in odom. Zero quaternion = hold heading."""
        if fence and self.frame is not None:
            (x, y, z), moved = clamp_to_net(
                (x, y, z), self.frame[0], self.frame[1], self.net_margin)
            if moved:
                self.info_once(
                    'fenced', f'setpoint pulled inside the net (margin '
                    f'{self.net_margin:.2f} m); route or start bearing wrong?')
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 0.0
        self.setpoint_pub.publish(msg)

    def send_twist(self, vx, vy, vz):
        """ENU world velocity in odom; expires after 1 s unless refreshed."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.z = 0.0
        self.twist_pub.publish(msg)

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
            if self.state == 'LAND' and self.landed:
                self.get_logger().info(
                    'touched down and disarmed: mission done. This node '
                    'will not take the vehicle again.')
                self.goto('DONE')
                return
            self.get_logger().warn(
                f'Offboard lost in {self.state}: the pilot has control. '
                'Paused until the vehicle is disarmed.')
            self.goto('PAUSED')
        getattr(self, '_state_' + self.state.lower())()

    def _state_wait_for_offboard(self):
        if not self.offboard_active:
            self.send(*self.position, fence=False)
            return
        x, y, _ = self.position
        self.goto('TAKEOFF', (x, y, self.takeoff_altitude))
        self._state_takeoff()       # climb on this tick, not the next one

    def _state_takeoff(self):
        self.send(*self.target)
        if self.arrived(self.target):
            self.start_route()
        elif self.now_s() - self.state_since > self.takeoff_timeout:
            self.get_logger().error(
                f'takeoff target not reached in {self.takeoff_timeout:.0f} s '
                f'({math.dist(self.position, self.target):.2f} m off); '
                'holding here. Check `clamped` in /skybeta/state and VIO.')
            self.goto('HOLD', self.position)

    def start_route(self):
        if not self.route:
            self.goto('HOLD', self.target)
            return
        offset = math.dist(self.position[:2], self.route[0][:2])
        if offset > self.max_start_offset:
            self.get_logger().error(
                f'{offset:.2f} m from the route start (limit '
                f'{self.max_start_offset:.2f} m): not flying it from here. '
                'Holding. Power up on the launch mark.')
            self.goto('HOLD', self.target)
            return
        self.carrot = Carrot(self.route, self.cruise_speed, self.max_lead)
        self.last_tick = self.now_s()
        self.progress_at = (self.last_tick, 0, self.carrot.point)
        self.goto('ROUTE')

    def _state_route(self):
        now = self.now_s()
        dt = min(now - self.last_tick, 0.2)
        self.last_tick = now
        vio_ok = self.vio_ok()
        if vio_ok != self.vio_was_ok:
            self.vio_was_ok = vio_ok
            if vio_ok:
                self.get_logger().info('VIO back: resuming the route.')
            else:
                self.get_logger().warn(
                    f'VIO lost (vo_state {self.vo_state}, odometry '
                    f'{now - (self.odom_at or now):.2f} s old): carrot '
                    'frozen at its current point.')
        # With no trustworthy position, do not move the target: PX4 falls
        # back on its own estimate and holds the last point.
        point = (self.carrot.step(self.position, dt) if vio_ok
                 else self.carrot.point)
        self.send(*point)
        if vio_ok and self.carrot.done(self.position, self.arrival_radius):
            if self.land_at_end:
                self.get_logger().info('route complete; landing on its end.')
                self.goto('LAND', point)
                self._state_land()      # no tick without a setpoint
            else:
                self.get_logger().info('route complete; holding at its end.')
                self.goto('HOLD', point)
            return
        if (self.carrot.index, point) != self.progress_at[1:]:
            self.progress_at = (now, self.carrot.index, point)
        elif now - self.progress_at[0] > self.stall_timeout:
            self.get_logger().error(
                f'no progress for {self.stall_timeout:.0f} s at waypoint '
                f'{self.carrot.index + 1}/{len(self.route)} '
                f'({math.dist(self.position, point):.2f} m off); holding. '
                'Check `clamped` in /skybeta/state and VIO.')
            self.goto('HOLD', self.position)

    def _state_land(self):
        # Straight down at land_speed, nudged back over the pad. Keep pushing
        # down after touchdown: PX4's land detector needs it, and it disarms
        # COM_DISARM_LAND (2 s) later, which ends Offboard -> DONE in tick().
        ex = self.target[0] - self.position[0]
        ey = self.target[1] - self.position[1]
        norm = math.hypot(ex, ey)
        scale = min(1.0, self.land_xy_max / norm) if norm > 0.0 else 0.0
        self.send_twist(ex * scale, ey * scale, -self.land_speed)

    def _state_done(self):
        pass

    def _state_hold(self):
        self.send(*self.target)

    def _state_paused(self):
        # PX4 ignores setpoints outside Offboard; keep the stream up so a
        # later arming can take over again, and re-arm the mission only
        # once the vehicle has been disarmed.
        self.send(*self.position, fence=False)
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
