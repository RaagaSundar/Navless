"""Navless course node: take over on the pilot's arm, fly a planner route.

It uses the stack's competitor topics, and nothing else (FI_NS below is the
flight interface's topic namespace):

    subscribe  /visual_slam/tracking/odometry   where we are (ENU, odom)
    subscribe  FI_NS/state                      what PX4 is doing (JSON)
    publish    FI_NS/setpoint/{pose,twist}      where we want to be (ENU, odom)

Never publishes to /fmu/*, never arms, never changes mode: flight_interface
owns that, and enters Offboard once when the pilot arms, then switches to
Position on the ground.

States (10 Hz timer):

    WAIT_FOR_OFFBOARD  stream the current position. A setpoint from us in the
                       last second is one of flight_interface's conditions
                       for taking over. Pre-arm height check: we stream only
                       once PX4's height (FI_NS/state `local_enu` z) has
                       matched odom z for prearm_settle_s. PX4 can boot with
                       its height ~0.6 m off (EKF2 parks the error in its
                       terrain estimate, and with EV_CTRL 9 nothing pulls it
                       back); flight_interface passes odom z to PX4 unchanged,
                       so every setpoint would then be flown that far off.
                       With no stream, flight_interface refuses the takeover
                       and the pilot keeps Position.
    REFUSED            Offboard came anyway while the heights disagreed (the
                       stream had stopped less than flight_interface's
                       setpoint timeout before the switch): descend in place
                       with twist setpoints until PX4 disarms, never climb.
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
                       hand the vehicle to us again. With fly_home, the
                       pad touchdown goes back to WAIT_FOR_OFFBOARD instead,
                       with the route swapped for the home leg (climb to
                       home_altitude_m, straight to the launch mark); the
                       pilot re-arms, and the touchdown there is DONE.
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

# The flight interface's topics. Their names are fixed by the image.
FI_NS = '/skybeta'
STATE_TOPIC = FI_NS + '/state'
POSE_TOPIC = FI_NS + '/setpoint/pose'
TWIST_TOPIC = FI_NS + '/setpoint/twist'
ODOM_TOPIC = '/visual_slam/tracking/odometry'

IDLE_STATES = ('WAIT_FOR_OFFBOARD', 'PAUSED', 'DONE')

# Pre-arm height check (see heights_agree): once passed, it holds until the
# disagreement is this much past prearm_height_tol_m, or PX4 has sent no
# height for PREARM_PX4_TIMEOUT_S.
PREARM_HYSTERESIS_M = 0.05
PREARM_PX4_TIMEOUT_S = 1.0

PARAMETERS = (
    ('takeoff_altitude_m', 0.85),
    # Climb target this far along the route's first leg instead of straight
    # above the drone, m (see takeoff_target). 0 = straight up. Gazebo, drone
    # profile, centreline route: Barrier 1 clearance during the climb 3.1-4.7
    # cm at 0, 16.1 cm at 0.15, 18.3 cm at 0.25. The first corridor is open
    # for 1.6 m ahead, so the larger lean is free if the real drone does not
    # drift at all, and it covers more backward drift if it does. Never on
    # the fly_home take-off from the pad (see lead_m).
    ('takeoff_lead_m', 0.25),
    ('arrival_radius_m', 0.25),
    ('takeoff_timeout_s', 15.0),
    ('publish_rate_hz', 10.0),
    # Route: a planner file, and how the drone sat on the launch mark at
    # power-up (nose bearing clockwise from course north; height of the odom
    # origin above the floor).
    ('route_file', 'route_gate_b.json'),
    ('start_bearing_deg', 0.0),
    # Pixhawk IMU on its feet: 0.07 m on the Gazebo stand-in, built from the
    # airframe's dimensions. Measure it on the real drone.
    ('start_height_m', 0.07),
    ('cruise_speed_mps', 0.5),
    ('max_lead_m', 0.6),
    # Slow through bends: speed <= sqrt(this * bend radius). 0 = off.
    ('max_lat_accel_mps2', 0.25),
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
    # Bonus return leg after the pad landing. Off by default. When on, the
    # pilot's next arming climbs to home_altitude_m (above the floor; the
    # walls are 1.727 m), flies straight over the wall to the launch mark and
    # lands there. At 2.2 m the feet clear the wall by ~0.38 m (~0.23 m if
    # PX4 flies prearm_height_tol_m, 0.15 m, low). The ceiling is
    # navless.launch.py's max_altitude_m 2.5 m above the odom origin (2.57 m
    # above the floor; flight_interface's own default is 3.0): the 2.13 m
    # odom setpoint stays 0.37 m under it. Lower that clamp and this too.
    ('fly_home', False),
    ('home_altitude_m', 2.2),
    # The hop's horizontal leg starts only once odom z, or PX4's own height,
    # is this close to the hop height (not at arrival_radius_m): starting
    # early crossed Barrier 1 at 1.91-1.99 m in SITL with home_altitude_m
    # 2.03. See climbed().
    ('home_climb_tol_m', 0.05),
    # The home leg lands only within this of the launch mark (not
    # arrival_radius_m): Barrier 1 is right behind the mark, and descending
    # while still closing the last 0.25 m left 14.2 cm to it in SITL.
    ('home_arrival_radius_m', 0.10),
    # Pre-arm height check (see WAIT_FOR_OFFBOARD). Largest |PX4 z - odom z|
    # we will take off with, m; 0 = off. Assumes flight_interface's
    # anchor_setpoints is false (its default, and navless.launch.py's). The
    # Gazebo stand-in reports PX4's height in world z, start_height_m above
    # odom, hence the margin over the 0.07 m stand-in offset.
    ('prearm_height_tol_m', 0.15),
    ('prearm_settle_s', 2.0),     # ...and for this long without a break
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
        self.takeoff_lead = max(0.0, param('takeoff_lead_m'))
        self.arrival_radius = param('arrival_radius_m')
        self.takeoff_timeout = param('takeoff_timeout_s')
        rate_hz = param('publish_rate_hz')
        self.cruise_speed = param('cruise_speed_mps')
        self.max_lead = param('max_lead_m')
        self.max_lat_accel = param('max_lat_accel_mps2')
        self.max_start_offset = param('max_start_offset_m')
        self.stall_timeout = param('stall_timeout_s')
        self.net_margin = param('net_margin_m')
        self.odom_timeout = param('odom_timeout_s')
        self.land_at_end = str(self.get_parameter('land_at_end').value
                               ).lower() in ('true', '1')
        self.land_speed = param('land_speed_mps')
        self.land_xy_max = param('land_xy_max_mps')
        self.fly_home = str(self.get_parameter('fly_home').value
                            ).lower() in ('true', '1')
        # Odom z of the hop: odom starts start_height_m above the floor.
        self.home_z = param('home_altitude_m') - param('start_height_m')
        self.home_climb_tol = param('home_climb_tol_m')
        self.home_arrival_radius = param('home_arrival_radius_m')
        self.going_home = False
        self.prearm_tol = param('prearm_height_tol_m')
        self.prearm_settle = param('prearm_settle_s')
        self.px4_z = None           # PX4's last valid height (local_enu z)
        self.px4_z_at = None
        self.heights_agree_since = None
        self.prearm_passed = False
        self.prearm_warned_at = None
        self.frame = None          # (launch mark xy, start bearing) once known
        self.route = self.load_route(
            str(self.get_parameter('route_file').value),
            param('start_bearing_deg'), param('start_height_m'))
        self.carrot = None
        self.last_tick = None
        self.progress_at = None

        self.setpoint_pub = self.create_publisher(
            PoseStamped, POSE_TOPIC, 10)
        self.twist_pub = self.create_publisher(
            TwistStamped, TWIST_TOPIC, 10)
        self.create_subscription(
            Odometry, ODOM_TOPIC, self.odometry_cb, 10)
        self.create_subscription(String, STATE_TOPIC, self.state_cb, 10)
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
        local = state.get('local_enu')
        try:
            z = float(local[2])
        except (TypeError, IndexError, ValueError):
            z = None
        if z is not None and math.isfinite(z):
            # Keep the last valid height: one message without a position
            # must not refuse a takeover (heights_agree checks its age).
            self.px4_z, self.px4_z_at = z, self.now_s()
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
        if state == 'WAIT_FOR_OFFBOARD':
            # Every arming (the fly_home re-arm on the pad too) re-checks.
            self.heights_agree_since = None
            self.prearm_passed = False

    def heights_agree(self):
        """Pre-arm check: PX4's height has matched odom z for prearm_settle_s."""
        if self.prearm_tol <= 0.0:
            self.info_once('prearm_off', 'pre-arm height check is OFF '
                           '(prearm_height_tol_m 0).')
            return True
        now = self.now_s()
        fresh = (self.px4_z is not None
                 and now - self.px4_z_at <= PREARM_PX4_TIMEOUT_S)
        dz = abs(self.px4_z - self.position[2]) if fresh else None
        # Hysteresis: pass at prearm_height_tol_m held for prearm_settle_s;
        # once passed, only a clear disagreement (PREARM_HYSTERESIS_M more)
        # or no PX4 height for PREARM_PX4_TIMEOUT_S revokes it. Otherwise
        # noise at the threshold or an arming transient can drop the stream
        # just as the pilot switches, and the takeover becomes a REFUSED
        # descent (Gazebo, simulated pilot arming at a 0.15 m boot offset).
        limit = self.prearm_tol + (PREARM_HYSTERESIS_M if self.prearm_passed
                                   else 0.0)
        if dz is None or dz > limit:
            self.heights_agree_since = None
            self.prearm_passed = False
            if (self.prearm_warned_at is None
                    or now - self.prearm_warned_at >= 5.0):
                self.prearm_warned_at = now
                seen = ('no valid local position from PX4' if dz is None else
                        f'PX4 height {self.px4_z:.2f} m vs odom '
                        f'{self.position[2]:.2f} m: {dz:.2f} m apart')
                self.get_logger().warn(
                    f'pre-arm height check: {seen} (limit '
                    f'{limit:.2f} m). Not streaming setpoints, so '
                    'flight_interface will not take over. Keep the drone '
                    'still on the floor; if it stays off, power-cycle the '
                    'Pixhawk.')
            return False
        if self.heights_agree_since is None:
            self.heights_agree_since = now
        if now - self.heights_agree_since < self.prearm_settle:
            return False
        if not self.prearm_passed:
            self.prearm_passed = True
            self.prearm_warned_at = None
            self.get_logger().info(
                f'pre-arm height check passed: PX4 and odom agree to '
                f'{dz:.2f} m. Streaming; ready for the arming.')
        return True

    # -- state machine ---------------------------------------------------
    def tick(self):
        if self.position is None:
            self.info_once(
                'no_odom', 'no /visual_slam/tracking/odometry yet; nothing '
                'is commanded until cuVSLAM is tracking.')
            return
        if self.state not in IDLE_STATES and not self.offboard_active:
            if self.state == 'LAND' and self.landed and (
                    self.fly_home and not self.going_home):
                # Home leg: straight up, then over the wall to the launch
                # mark, which is the odom origin. The route start check
                # (max_start_offset_m) keeps it from flying from anywhere
                # but this touchdown.
                x, y, _ = self.position
                self.route = [(x, y, self.home_z), (0.0, 0.0, self.home_z)]
                self.going_home = True
                self.get_logger().info(
                    'touched down on the pad and disarmed. fly_home: the '
                    'next arming flies back to the launch mark at '
                    f'{self.home_z:.2f} m (odom). Leave it disarmed to stop.')
                self.goto('WAIT_FOR_OFFBOARD')
                return
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
        ready = self.heights_agree()
        if not self.offboard_active:
            if ready:
                self.send(*self.position, fence=False)
            return
        if not ready:
            self.get_logger().error(
                'Offboard came while PX4 height and odom disagree: not taking '
                'off. Descending in place until PX4 disarms.')
            self.goto('REFUSED', self.position)
            self._state_refused()       # no tick without a setpoint
            return
        self.goto('TAKEOFF', self.takeoff_target())
        self._state_takeoff()       # climb on this tick, not the next one

    def takeoff_target(self):
        """Straight up to the route's first height (no height step when the
        route opens; Gazebo: 7 cm once start_height_m is set), leaning
        takeoff_lead_m into the route's first leg. Barrier 1 is behind the
        launch mark and the first corridor ahead of it; in Gazebo the liftoff
        drifts ~15 cm backwards (thrust-model mismatch), to 3 cm from the
        barrier, and the lean takes that margin back."""
        x, y, _ = self.position
        if not self.route:
            return (x, y, self.takeoff_altitude)
        r0 = self.route[0]
        p = Carrot(self.route, 1.0, math.inf).advance(self.lead_m())
        return (x + p[0] - r0[0], y + p[1] - r0[1], r0[2])

    def lead_m(self):
        """takeoff_lead_m, but 0 on the fly_home take-off: the pad has room
        all round (over 0.7 m to anything in Gazebo's model), and a lean
        there only starts the hop over Barrier 1 closer to the wall, with
        less run-up to finish the climb."""
        return 0.0 if self.going_home else self.takeoff_lead

    def _state_takeoff(self):
        self.send(*self.target)
        if self.arrived(self.target) and self.climbed():
            self.start_route()
        elif self.now_s() - self.state_since > self.takeoff_timeout:
            self.get_logger().error(
                f'takeoff target not reached in {self.takeoff_timeout:.0f} s '
                f'({math.dist(self.position, self.target):.2f} m off); '
                f'holding here. Check `clamped` in {STATE_TOPIC} and VIO.')
            self.goto('HOLD', self.position)

    def climbed(self):
        """On the fly_home hop, the climb must be (nearly) complete before
        the leg over Barrier 1 starts; elsewhere arrival_radius_m is enough.
        Complete by odom, or by PX4's own height: PX4 holds where its
        estimate says the setpoint is, which can sit a few cm off odom (7 cm
        short at 2.2 m in SITL), so waiting longer would never get there."""
        if not self.going_home:
            return True
        need = self.target[2] - self.home_climb_tol
        return (self.position[2] >= need
                or (self.px4_z is not None and self.px4_z >= need))

    def start_route(self):
        if not self.route:
            self.goto('HOLD', self.target)
            return
        carrot = Carrot(self.route, self.cruise_speed, self.max_lead,
                        self.max_lat_accel)
        carrot.advance(self.lead_m())       # never pull back behind the lean
        # Measured from where the route starts after the lean, so the lean
        # does not use up max_start_offset_m.
        offset = math.dist(self.position[:2], carrot.point[:2])
        if offset > self.max_start_offset:
            self.get_logger().error(
                f'{offset:.2f} m from the route start (limit '
                f'{self.max_start_offset:.2f} m): not flying it from here. '
                'Holding. Power up on the launch mark.')
            self.goto('HOLD', self.target)
            return
        self.carrot = carrot
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
        radius = (self.home_arrival_radius if self.going_home
                  else self.arrival_radius)
        if vio_ok and self.carrot.done(self.position, radius):
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
                f'Check `clamped` in {STATE_TOPIC} and VIO.')
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

    def _state_refused(self):
        # Push down, never up: on the floor this is PX4's land detector, then
        # its disarm. Ends like any lost Offboard: PAUSED, then waiting again.
        self.send_twist(0.0, 0.0, -self.land_speed)

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
