"""Navless course node: take over on the pilot's arm, fly a planner route.

It uses the stack's competitor topics, and nothing else (FI_NS below is the
flight interface's topic namespace):

    subscribe  /visual_slam/tracking/odometry   where we are (ENU, odom)
    subscribe  FI_NS/state                      what PX4 is doing (JSON)
    subscribe  /tube/offset                     tube centre from the side ToF
                                                sensors (tof_tube_centering only)
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
                       stale, and resumes when it is back. With blind_carry,
                       a loss inside the dark tube instead keeps the carrot
                       pacing PX4's own estimate for a few seconds, so the
                       drone does not stop where tracking cannot return.
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
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String

from navless_nav.route import (Carrot, clamp_to_net, course_to_odom,
                               create_live_course_planner, load_route,
                               odom_to_course, plan_dynamic_course_route,
                               resolve_route_path)
from navless_nav.tube_guidance import TubeGuidance

try:
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
except ImportError:   # off the drone: fall back to odometry staleness only
    VisualSlamStatus = None

try:
    from sensor_msgs.msg import PointCloud2
except ImportError:
    PointCloud2 = None

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

# The tube (course_v3.json `tunnel`: footprint x 182-240 in, y 108-168 in,
# axis north-south, "Dark inside"), in course metres (in * 0.0254 - 3.048).
TUBE_X = (1.5748, 3.048)
TUBE_Y = (-0.3048, 1.2192)

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
    # Where we read our own position. Use the same odometry that feeds PX4's
    # vision fusion: cuVSLAM's raw topic through the stock px4_odom_bridge,
    # or Koji's gated /mapping/validated_odometry (which drops samples while
    # cuVSLAM is not tracking) if PX4 is fed that. Then a VIO loss looks the
    # same to this node as to PX4 (see blind_carry and odom_arriving).
    ('odometry_topic', ODOM_TOPIC),
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
    # Tight turns (the wall-end arcs round C1, C2b and A1 on the Gate B
    # route): chasing a carrot `lead` ahead round an arc of radius R, the
    # drone cuts inside by ~lead^2 / (2 R). With this > 0 the lead shrinks
    # there so the cut stays under it, m (Carrot.lead_limit). 0 = off.
    ('max_turn_sagitta_m', 0.0),
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
    # lands there. At 2.2 m the feet clear the wall by ~0.38 m (~0.18 m if
    # PX4 flies 0.20 m low, the most the pre-arm check holds a pass for:
    # prearm_height_tol_m 0.15 + PREARM_HYSTERESIS_M 0.05). The ceiling is
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
    # The tube is dark inside, so cuVSLAM may lose tracking there. Off: a VIO
    # loss freezes the carrot, which in the tube means hovering blind until
    # PX4's dead reckoning times out (EKF2_NOAID_TOUT, 5 s on the fleet) and
    # it fails safe. On: while the carrot is in the tube (plus
    # blind_zone_margin_m at each mouth), a VIO loss keeps it pacing PX4's
    # own estimate (FI_NS/state local_enu) along the route for at most
    # blind_carry_max_s per loss. A loss anywhere else still freezes it.
    ('blind_carry', False),
    ('blind_carry_max_s', 4.0),
    ('blind_zone_margin_m', 0.4),
    # Experimental stereo wall correction. Never enabled by a hardware default.
    ('stereo_tube_centering', False),
    ('stereo_tube_max_shift_m', 0.20),
    ('stereo_tube_rate_mps', 0.10),
    ('stereo_tube_hold_s', 3.0),
    # Side ToF tube centring, off by default. tube/tube_node.py publishes
    # /tube/offset (how far the tube centreline is to the drone's left) only
    # while both side readings span the tube's width. While the drone and
    # the carrot are in the tube, the target is pulled toward that centre:
    # never more than tof_tube_max_shift_m from the route, so a misreading
    # can do no worse, and at tof_tube_rate_mps. It drops back to the route
    # at once when the offset is older than tof_tube_timeout_s, outside the
    # tube, or with nothing to measure from (VIO lost, no blind carry).
    # Needs the bus-7 sensor on the drone's LEFT: swapped sensors pull
    # toward a wall, up to the shift limit.
    ('tof_tube_centering', False),
    ('tof_tube_max_shift_m', 0.15),
    ('tof_tube_rate_mps', 0.10),
    ('tof_tube_timeout_s', 0.3),
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
        self.max_sagitta = param('max_turn_sagitta_m')
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
        self.px4_pos = None         # PX4's position (local_enu), odom-comparable
        self.px4_at = None
        self.blind_carry = str(self.get_parameter('blind_carry').value
                               ).lower() in ('true', '1')
        self.blind_max_s = param('blind_carry_max_s')
        self.blind_margin = param('blind_zone_margin_m')
        self.blind_since = None     # start of the current VIO loss in ROUTE
        self.blind_carrying = False
        self.stereo_tube = str(self.get_parameter('stereo_tube_centering').value).lower() in ('true', '1')
        self.tube_guidance = TubeGuidance(param('stereo_tube_max_shift_m'), param('stereo_tube_rate_mps'),
                                          param('stereo_tube_hold_s'))
        self.tube_invalid_at = -math.inf
        self.tube_status_at = None
        self.tof_tube = str(self.get_parameter('tof_tube_centering').value).lower() in ('true', '1')
        if self.tof_tube and self.stereo_tube:
            self.get_logger().error(
                'stereo_tube_centering and tof_tube_centering are both on: '
                'steering on stereo only.')
            self.tof_tube = False
        self.tof_guidance = TubeGuidance(
            param('tof_tube_max_shift_m'), param('tof_tube_rate_mps'),
            param('tof_tube_timeout_s'), clamp=True)
        self.tof_offset = None      # (stamp, metres to the left) from /tube/offset
        self.tof_used_stamp = None
        self.yaw = None             # odometry heading: the offset is along body y
        self.vo_at = None
        self.state_at = None
        self.heights_agree_since = None
        self.prearm_passed = False
        self.prearm_warned_at = None
        self.prearm_anchor_pos = None
        self.prearm_stability_tol = 0.05
        self.raw_position = None
        self.odom_zero_offset = (0.0, 0.0, 0.0)
        self.odom_zeroed = False
        self.start_bearing = param('start_bearing_deg')
        self.start_height = param('start_height_m')
        self.preferred_branch = (
            'gate_b' if 'gate_b' in str(self.get_parameter('route_file').value).lower()
            else 'window'
        )
        self.planner_cfg, self.planner_grid, self.planner_router = create_live_course_planner()
        self.live_grid_updated = False
        self.last_replan_at = 0.0
        self.frame = None          # (launch mark xy, start bearing) once known
        self.route = self.load_route(
            str(self.get_parameter('route_file').value),
            self.start_bearing, self.start_height)
        self.carrot = None
        self.last_tick = None
        self.progress_at = None

        self.setpoint_pub = self.create_publisher(
            PoseStamped, POSE_TOPIC, 10)
        self.twist_pub = self.create_publisher(
            TwistStamped, TWIST_TOPIC, 10)
        self.create_subscription(
            Odometry, str(self.get_parameter('odometry_topic').value),
            self.odometry_cb, 10)
        self.create_subscription(String, STATE_TOPIC, self.state_cb, 10)
        if self.stereo_tube:
            self.create_subscription(PoseStamped, '/tube/stereo/centerline', self.tube_center_cb, 2)
            self.create_subscription(String, '/tube/stereo/status', self.tube_status_cb, 2)
        if self.tof_tube:
            # tube_node publishes best effort; a reliable subscriber would
            # never match it.
            self.create_subscription(PointStamped, '/tube/offset', self.tof_offset_cb,
                                     qos_profile_sensor_data)
        if VisualSlamStatus is not None:
            self.create_subscription(
                VisualSlamStatus, '/visual_slam/status', self.status_cb, 10)
        if PointCloud2 is not None and not hasattr(PointCloud2, '__getattr__'):
            self.create_subscription(
                PointCloud2, '/team/stereo/points', self.pointcloud_cb, 10)

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
        """Plan 3D A* + EGO B-spline route dynamically in odom-frame (never relying on static route.json)."""
        try:
            low = str(name).lower()
            if not any(k in low for k in ('window', 'gate_b', 'dynamic', 'route.json')):
                resolve_route_path(name)
            branch = 'gate_b' if 'gate_b' in low else 'window'
            waypoints, launch, kind = plan_dynamic_course_route(
                branch=branch, router=self.planner_router)
        except (OSError, ValueError, KeyError, RuntimeError) as exc:
            self.get_logger().error(
                f'route not loaded ({exc}); will take off and hover only.')
            return None
        odom = [course_to_odom(p, launch, bearing_deg, height_m)
                for p in waypoints]
        self.frame = (launch, bearing_deg)
        self.get_logger().info(
            f'planned dynamic {kind}: {len(odom)} waypoints, launch mark '
            f'{launch}, start bearing {bearing_deg:.1f} deg, ends at odom '
            f'({odom[-1][0]:.2f}, {odom[-1][1]:.2f}, {odom[-1][2]:.2f})')
        return odom

    def pointcloud_cb(self, msg):
        """Insert live stereo pointcloud obstacles into the 3D occupancy grid for online replanning."""
        if self.position is None or self.frame is None:
            return
        from planner.px4_bridge_node import decode_ros_pointcloud2_optical
        cur_course = odom_to_course(
            self.position, self.frame[0], self.frame[1], self.start_height)
        yaw_enu = 0.5 * math.pi - math.radians(self.frame[1])
        pts_world = decode_ros_pointcloud2_optical(
            msg, cur_course, yaw_enu, max_depth_m=2.80)
        if pts_world.size > 0:
            n_added = self.planner_grid.insert_point_cloud(
                pts_world, sensor_origin_m=cur_course, min_live_hits=2)
            if n_added > 0:
                self.live_grid_updated = True

    def maybe_replan_for_discrepancies(self, now):
        """Replan remaining route via 3D A* + B-spline if live stereo depth reveals a map discrepancy."""
        if (not self.live_grid_updated or self.going_home
                or self.frame is None or self.carrot is None
                or (now - self.last_replan_at) < 0.5):
            return
        self.live_grid_updated = False
        rem_odom = self.route[self.carrot.index:]
        blocked = False
        for q in rem_odom:
            c_pt = odom_to_course(q, self.frame[0], self.frame[1], self.start_height)
            if self.planner_grid.is_occupied(c_pt, inflated=True):
                blocked = True
                break
        if not blocked:
            return
        cur_course = odom_to_course(
            self.position, self.frame[0], self.frame[1], self.start_height)
        try:
            wps_enu, launch, kind = plan_dynamic_course_route(
                branch=self.preferred_branch,
                start_enu=cur_course,
                router=self.planner_router,
            )
            new_odom = [course_to_odom(p, launch, self.frame[1], self.start_height)
                        for p in wps_enu]
            self.route = new_odom
            self.carrot = Carrot(
                self.route, self.cruise_speed, self.max_lead,
                self.max_lat_accel, self.max_sagitta)
            self.last_replan_at = now
            self.get_logger().info(
                f'Online 3D A* + B-spline replan ({kind}): {len(new_odom)} waypoints around obstacle.')
        except (ValueError, RuntimeError) as exc:
            self.get_logger().warn(f'Online replan skipped ({exc}).')

    # -- inputs ----------------------------------------------------------
    def odometry_cb(self, msg):
        p = msg.pose.pose.position
        raw = (float(p.x), float(p.y), float(p.z))
        self.raw_position = raw
        self.position = (
            raw[0] - self.odom_zero_offset[0],
            raw[1] - self.odom_zero_offset[1],
            raw[2] - self.odom_zero_offset[2],
        )
        self.odom_at = self.now_s()
        if self.tof_tube:
            q = msg.pose.pose.orientation
            self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def status_cb(self, msg):
        self.vo_state = int(msg.vo_state)
        self.vo_at = self.now_s()
        if self.vo_state != 1:
            self.tube_guidance.clear()
            self.tube_invalid_at = self.now_s()

    def tube_status_cb(self, msg):
        self.tube_status_at = self.now_s()
        try:
            valid = json.loads(msg.data).get('valid') is True
        except (ValueError, AttributeError):
            valid = False
        if not valid:
            self.tube_guidance.clear()
            self.tube_invalid_at = self.now_s()

    def tube_center_cb(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        values = (q.x, q.y, q.z, q.w)
        if (msg.header.frame_id != 'odom' or stamp <= self.tube_invalid_at
                or not all(math.isfinite(v) for v in values)
                or abs(sum(v*v for v in values)-1) > 0.01
                or abs(q.x) > 1e-6 or abs(q.y) > 1e-6):
            self.tube_guidance.clear()
            return
        self.tube_guidance.update(stamp, (p.x, p.y, p.z),
                                  2*math.atan2(q.z, q.w), self.now_s())

    def tof_offset_cb(self, msg):
        """tube_node's /tube/offset: point.y = how far the tube centreline is
        to the drone's left (body frame), point.z = measured width."""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        offset = float(msg.point.y)
        self.tof_offset = ((stamp, offset) if msg.header.frame_id == 'base_link'
                           and math.isfinite(offset) else None)

    def vio_ok(self):
        fresh = (self.odom_at is not None
                 and self.now_s() - self.odom_at <= self.odom_timeout)
        return fresh and self.vo_state != VO_LOST

    def state_cb(self, msg):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        self.state_at = self.now_s()
        self.offboard_active = bool(state.get('offboard_active', False))
        self.armed = bool(state.get('armed', False))
        self.landed = bool(state.get('landed', False))
        local = state.get('local_enu')
        try:
            xyz = tuple(float(v) for v in local[:3])
        except (TypeError, ValueError):
            xyz = ()
        ok = len(xyz) == 3 and all(math.isfinite(v) for v in xyz)
        self.px4_pos = xyz if ok else None
        self.px4_at = self.now_s()
        if ok:
            # Keep the last valid height: one message without a position
            # must not refuse a takeover (heights_agree checks its age).
            self.px4_z, self.px4_z_at = xyz[2], self.px4_at
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
            self.prearm_anchor_pos = None

    def heights_agree(self):
        """Pre-arm check: PX4's height matches odom z, VIO is stationary, and launch offset is zeroed."""
        if self.prearm_tol <= 0.0:
            self.info_once('prearm_off', 'pre-arm height check is OFF '
                           '(prearm_height_tol_m 0).')
            return True
        now = self.now_s()
        if not self.vio_ok():
            self.heights_agree_since = None
            self.prearm_passed = False
            self.prearm_anchor_pos = None
            return False
        raw_pos = self.raw_position if self.raw_position is not None else self.position
        if self.prearm_anchor_pos is None:
            self.prearm_anchor_pos = raw_pos
        elif math.dist(raw_pos, self.prearm_anchor_pos) > self.prearm_stability_tol:
            self.prearm_anchor_pos = raw_pos
            self.heights_agree_since = None
            self.prearm_passed = False
            return False
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
        if not self.odom_zeroed and not self.going_home and raw_pos is not None:
            if 0.0 < math.hypot(raw_pos[0], raw_pos[1]) <= 0.20 and abs(raw_pos[2]) <= 0.20:
                self.odom_zero_offset = raw_pos
                self.position = (0.0, 0.0, 0.0)
            self.odom_zeroed = True
        if not self.prearm_passed:
            self.prearm_passed = True
            self.prearm_warned_at = None
            self.get_logger().info(
                f'pre-arm height & zero check passed: PX4 and odom agree to '
                f'{dz:.2f} m. Streaming; ready for the arming.')
        return True

    # -- state machine ---------------------------------------------------
    def tick(self):
        if self.position is None:
            self.info_once(
                'no_odom', 'no /visual_slam/tracking/odometry yet; nothing '
                'is commanded until cuVSLAM is tracking.')
            return
        if self.stereo_tube and self.state not in ('WAIT_FOR_OFFBOARD', 'DONE'):
            fresh = self.state_at is not None and 0 <= self.now_s()-self.state_at <= 0.5
            if not fresh or (not self.offboard_active and not self.landed):
                self.tube_guidance.clear()
                if not self.offboard_active and self.state != 'PAUSED':
                    self.goto('PAUSED')
                return  # let the existing flight interface own loss-of-stream behavior
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
                or (self.px4_z is not None
                    and self.now_s() - self.px4_z_at <= PREARM_PX4_TIMEOUT_S
                    and self.px4_z >= need))

    def start_route(self):
        if not self.route:
            self.goto('HOLD', self.target)
            return
        effective_sagitta = self.max_sagitta if self.max_sagitta > 0.0 else 0.04
        carrot = Carrot(self.route, self.cruise_speed, self.max_lead,
                        self.max_lat_accel, effective_sagitta)
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
        self.blind_since = None     # a fly_home leg starts with a clean slate
        self.blind_carrying = False
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
                # If the drone got ahead of the carrot while blind, move the
                # carrot up to it rather than flying back (in the tube, back
                # into the dark).
                moved = self.carrot.catch_up(self.position)
                self.get_logger().info(
                    'VIO back: resuming the route'
                    + (f' ({moved:.2f} m further on, where the drone is).'
                       if moved > 0.0 else '.'))
            else:
                self.get_logger().warn(
                    f'VIO lost (vo_state {self.vo_state}, odometry '
                    f'{now - (self.odom_at or now):.2f} s old).')
        if vio_ok:
            self.blind_since = None
            self.set_carrying(False, 'VIO back')
            self.maybe_replan_for_discrepancies(now)
            point = self.carrot.step(self.position, dt)
            ref = self.position     # where the drone is, in the target's frame
        else:
            # With no trustworthy position, do not move the target: PX4
            # falls back on its own estimate and holds the last point. In
            # the dark tube (blind_carry) keep pacing that estimate instead,
            # so the drone does not stop where tracking cannot come back.
            if self.blind_since is None:
                self.blind_since = now
            px4 = self.blind_carry_position(now)
            ref = px4               # None unless carrying on PX4's estimate
            if px4 is not None:
                # at the route's own speed: a faster carry (1.0 m/s, 1.2 m
                # lead) drifted 49 cm and touched the tube in Gazebo
                point = self.carrot.step(px4, dt)
            elif self.odom_arriving():
                # cuVSLAM reports lost but keeps publishing (a frozen pose,
                # or worse), and the stack's px4_odom_bridge forwards every
                # message, so PX4 fuses it. Against that estimate a carrot
                # ahead makes PX4 push on with no feedback (Gazebo 'freeze':
                # through the tube into the south net). Hold at the latest
                # pose instead: zero error, whatever the stream does.
                self.info_once(f'odom-during-loss:{self.blind_since}',
                               'VIO lost but odometry still arriving (frozen '
                               'or unreliable): holding at its latest pose.')
                point = self.position
            else:
                point = self.carrot.point
        if self.stereo_tube:
            inside = self.in_tube(self.position, point)
            healthy = (vio_ok and self.vo_state == 1 and self.vo_at is not None
                       and 0 <= now-self.vo_at <= 0.25 and self.armed and self.offboard_active
                       and self.tube_status_at is not None and 0 <= now-self.tube_status_at <= 0.30)
            if healthy and not inside:
                # Observe the tube on approach, without steering outside it.
                self.tube_guidance.shift = 0.0
                self.tube_guidance.last_time = None
            else:
                point = self.tube_guidance.apply(
                    point, self.position, math.radians(self.frame[1]) if self.frame else 0,
                    now, enabled=inside and healthy, max_lead=self.max_lead)
        if self.tof_tube:
            point = self.tof_tube_point(point, ref, now)
        self.send(*point)
        if self.going_home:
            # Over the launch mark by x/y only: PX4 can hold the hop height a
            # few cm off odom (7 cm in SITL), which a 3D 0.10 m test could
            # never meet.
            arrived = (self.carrot.at_end and math.dist(
                self.position[:2], self.carrot.point[:2])
                <= self.home_arrival_radius)
        else:
            arrived = self.carrot.done(self.position, self.arrival_radius)
        if vio_ok and arrived:
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

    def in_tube(self, position, point):
        """Drone and target both inside the tube's footprint, at flying
        height, and not on the fly_home hop."""
        if self.frame is None or self.going_home:
            return False
        x, y, _ = odom_to_course(position, self.frame[0], self.frame[1])
        gx, gy, _ = odom_to_course(point, self.frame[0], self.frame[1])
        height = position[2] + self.start_height
        return (TUBE_X[0] < x < TUBE_X[1] and TUBE_Y[0] < y < TUBE_Y[1]
                and TUBE_X[0] < gx < TUBE_X[1] and TUBE_Y[0] < gy < TUBE_Y[1]
                and 0.3 < height < 1.5)

    def tof_tube_point(self, point, ref, now):
        """The route target pulled toward the tube centre the side ToF
        sensors measure (see tof_tube_centering). ref is where the drone is,
        in the frame the target is flown in: odometry, or PX4's own estimate
        during a blind carry. None means nothing to measure from."""
        m = self.tof_offset
        if not (ref is not None and m is not None and self.yaw is not None
                and self.armed and self.offboard_active and self.in_tube(ref, point)):
            self.tof_guidance.clear()       # no correction outlives its data
            return point
        axis = math.radians(self.frame[1])  # the tube runs course north-south
        if m[0] != self.tof_used_stamp:
            self.tof_used_stamp = m[0]
            # The offset is along the body's y axis: the centre in odom.
            left = (-math.sin(self.yaw), math.cos(self.yaw))
            centre = (ref[0] + m[1] * left[0], ref[1] + m[1] * left[1], ref[2])
            self.tof_guidance.update(m[0], centre, axis, now)
        return self.tof_guidance.apply(point, ref, axis, now, enabled=True,
                                       max_lead=self.max_lead)

    def in_blind_zone(self, point):
        """The odom point is in the tube, or within blind_zone_margin_m of a mouth."""
        if self.frame is None:
            return False
        x, y, _ = odom_to_course(point, self.frame[0], self.frame[1])
        return (TUBE_X[0] <= x <= TUBE_X[1]
                and TUBE_Y[0] - self.blind_margin <= y
                <= TUBE_Y[1] + self.blind_margin)

    def blind_carry_position(self, now):
        """PX4's own position to pace the carrot with during a VIO loss, or
        None to freeze it (blind_carry off, outside the tube zone, past
        blind_carry_max_s, or no fresh PX4 position)."""
        why = None
        if not self.blind_carry:
            why = 'blind_carry off'
        elif self.odom_arriving():
            why = 'odometry still arriving during the loss'
        elif now - self.blind_since > self.blind_max_s:
            why = f'no VIO for {self.blind_max_s:.1f} s'
        elif not self.in_blind_zone(self.carrot.point):
            why = 'outside the tube'
        elif (self.px4_pos is None or self.px4_at is None
              or now - self.px4_at > self.odom_timeout):
            why = 'no fresh PX4 position'
        self.set_carrying(why is None, why)
        return self.px4_pos if why is None else None

    def odom_arriving(self):
        """Odometry kept coming after VIO was flagged lost (a message at
        least 0.1 s after the loss, so the last one before a real stop does
        not count). cuVSLAM runs with IMU fusion off on skybeta, so that is a
        frozen or unreliable pose, not dead reckoning: never carry on it."""
        return (self.blind_since is not None and self.odom_at is not None
                and self.odom_at > self.blind_since + 0.1)

    def set_carrying(self, carrying, why):
        if carrying == self.blind_carrying:
            if not carrying and self.blind_since is not None:
                self.info_once(f'frozen:{self.blind_since}',
                               f'carrot frozen ({why}).')
            return
        self.blind_carrying = carrying
        if carrying:
            self.get_logger().warn(
                'VIO lost in the tube: carrying the carrot on PX4\'s own '
                f'estimate for up to {self.blind_max_s:.1f} s.')
        else:
            self.get_logger().warn(f'blind carry ended ({why}).')

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
