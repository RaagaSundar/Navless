"""First-flight tests for skybeta-19, the organisers' way: setpoints only, the pilot arms.

Three tests, picked with the `test` parameter (one launch file each):

    hover    climb 1 m, hold 10 s, land
    yaw      climb 1 m, hold 2 s, yaw 180 deg clockwise at 30 deg/s, hold 2 s, land
    forward  climb 1 m, hold 2 s, fly 2 m along the starting heading at 0.4 m/s, hold 10 s,
             fly back to the start at 0.4 m/s (heading unchanged), hold 2 s, land

The contract (skybeta README, competitor_guide.md): this node publishes ONLY
/skybeta/setpoint/pose and /skybeta/setpoint/twist. It reads /skybeta/state,
/visual_slam/tracking/odometry and (read-only) /fmu/out/vehicle_local_position. It never
publishes to /fmu/*, never arms, never changes mode. skybeta_flight_interface owns Offboard, the
heartbeat and the safety envelope (altitude band, speed cap) and clamps whatever we ask for.

PX4's frame, not cuVSLAM's. flight_interface passes our ENU setpoint to PX4 with the plain
ENU->NED map, but PX4 holds position in EKF2's frame, which differs from cuVSLAM's `odom` by "a
constant yaw offset and a constant translation" (competitor_guide.md). Commanding "stay at the odom
position" then sends PX4 to a different spot the moment Offboard starts: the drone lifts off and
slides sideways. So this node takes its position AND heading from PX4's own estimate
(/fmu/out/vehicle_local_position, NED -> ENU) for everything: the "stay here" stream before
takeoff, the hold point, distances, the forward direction, the yaw target and arrival checks.
Whatever the offset is, PX4 is only ever told to stay where it already believes it is, or to move
relative to that. The offset (PX4 minus cuVSLAM, position and heading) is logged at takeoff.
cuVSLAM's odometry is still watched: if it goes stale, the carrot freezes.

Sequence on the drone (props ON only after a props-off rehearsal):
  1. drone on the floor, stack up, cuVSLAM tracking, this launch running in the competitor window;
     the node streams "stay where you are" so flight_interface can take over.
  2. safety pilot arms with the mode switch on Altitude, throttle low;
  3. pilot switches to Position within a few seconds: Offboard starts and the test flies itself.
     It must leave the ground within 10 s of arming or PX4 disarms (COM_DISARM_PRFLT).
  4. the pilot can take over at any time (throttle towards centre, or the mode switch). The node
     notices Offboard ending and stops; it never tries to get Offboard back.
  5. landing: straight down with twist setpoints until PX4 detects the touchdown and disarms
     (COM_DISARM_LAND). Then the node stops publishing for good, so a later arming can't hand the
     vehicle to it again. Restart the launch to fly another test.

Motion is commanded as a pose "carrot" that moves at the test's speed (pose setpoints are not
speed-limited by the interface), so nothing jumps. If odometry goes stale (> odom_timeout_s) the
carrot freezes where it is; the pilot decides what happens next.

Frames: ENU (x east, y north, z up) in PX4's local frame, as flight_interface maps it.
Distances and headings are relative to where Offboard started. Heading: ENU yaw, CCW positive;
"clockwise" in the yaw test means decreasing ENU yaw, as seen from above.
"""
import json
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition
try:                                       # EKF2's sensor-fusion flags (is vision fused? magnetometer?)
    from px4_msgs.msg import EstimatorStatusFlags
except ImportError:
    EstimatorStatusFlags = None
try:                                       # cuVSLAM's tracking state (Isaac ROS)
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
except ImportError:
    VisualSlamStatus = None

POSE_TOPIC = '/skybeta/setpoint/pose'
TWIST_TOPIC = '/skybeta/setpoint/twist'
STATE_TOPIC = '/skybeta/state'
ODOM_TOPIC = '/visual_slam/tracking/odometry'
PX4_POS_TOPIC = '/fmu/out/vehicle_local_position'     # read only
TESTS = ('hover', 'yaw', 'forward')
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


LOG_COLUMNS = [
    'wall_time', 't', 'test', 'mode', 'step', 'note',
    'offboard', 'armed', 'landed', 'clamped',
    'px4_x', 'px4_y', 'px4_z', 'px4_yaw_deg', 'px4_vx', 'px4_vy', 'px4_vz', 'px4_age_s',
    'px4_xy_resets', 'px4_z_resets', 'px4_heading_resets', 'px4_delta_xy',
    'odom_x', 'odom_y', 'odom_z', 'odom_yaw_deg', 'odom_age_s', 'vo_state',
    'off_x', 'off_y', 'off_z', 'off_yaw_deg',
    'sp_kind', 'sp_x', 'sp_y', 'sp_z', 'sp_yaw_deg', 'sp_vx', 'sp_vy', 'sp_vz',
    'carrot_err_xy', 'carrot_err_z', 'corr_x', 'corr_y',
    'ekf_ev_pos', 'ekf_ev_yaw', 'ekf_ev_hgt', 'ekf_rng_hgt', 'ekf_mag_hdg', 'ekf_mag_3d', 'ekf_yaw_align',
]


class FlightLog:
    """One CSV row per control tick, plus a row for every event: the flight recorder.

    What PX4 believed (EKF2 position, velocity, heading, reset counters, fusion flags), what cuVSLAM
    saw (odometry, tracking state), the frame offset between them, and what we commanded. Written to
    <log_dir>/<YYYYmmdd-HHMMSS>_<test>.csv (~/team_ws/flight_logs on the drone), flushed every second,
    so a crash keeps everything up to the last second."""

    def __init__(self, log_dir, test):
        os.makedirs(log_dir, exist_ok=True)
        self.path = os.path.join(log_dir, f"{time.strftime('%Y%m%d-%H%M%S')}_{test}.csv")
        self.f = open(self.path, 'w', newline='')
        self.w = csv.DictWriter(self.f, fieldnames=LOG_COLUMNS, extrasaction='ignore')
        self.w.writeheader()
        self.last_flush = time.monotonic()

    def row(self, values):
        self.w.writerow({k: ('' if v is None else (round(v, 4) if isinstance(v, float) else v))
                         for k, v in values.items()})
        if time.monotonic() - self.last_flush >= 1.0:
            self.f.flush()
            self.last_flush = time.monotonic()

    def close(self):
        try:
            self.f.flush()
            self.f.close()
        except ValueError:
            pass


def plan(test, p):
    """The test as a list of steps: (kind, args). Kinds: climb, hold, yaw, move, land."""
    steps = [('climb', p['climb_m']), ('hold', p['settle_s'] if test != 'hover' else p['hover_s'])]
    if test == 'yaw':
        steps += [('yaw', -math.radians(p['yaw_deg'])), ('hold', p['settle_s'])]
    elif test == 'forward':
        steps += [('move', p['forward_m']), ('hold', p['hover_s']), ('move', 0.0), ('hold', p['settle_s'])]
    return steps + [('land', None)]


class BasicFlight(Node):
    def __init__(self, **kwargs):
        super().__init__('navless_basic_flight', **kwargs)
        d = {'test': 'hover', 'climb_m': 1.0, 'hover_s': 10.0, 'settle_s': 2.0, 'yaw_deg': 180.0,
             'yaw_rate_dps': 30.0, 'forward_m': 2.0, 'speed_mps': 0.4, 'climb_speed_mps': 0.4,
             'land_speed_mps': 0.3, 'land_xy_max_mps': 0.2, 'arrive_xy_m': 0.15, 'arrive_z_m': 0.10,
             'arrive_yaw_deg': 5.0, 'step_timeout_s': 20.0, 'odom_timeout_s': 0.5, 'rate_hz': 20.0,
             'log_dir': '/root/team_ws/flight_logs',
             # Drift correction: PX4 leaves a steady horizontal offset when something keeps pushing the
             # drone (13 cm in the 2026-11-07 yaw flight). Slowly move the commanded point the other way
             # so the drone itself sits on the target. Integral action on our setpoint only: PX4's
             # parameters are untouched.
             'drift_correction': True, 'drift_ki': 0.3, 'drift_max_m': 0.20, 'drift_deadband_m': 0.02,
             'drift_max_error_m': 0.40, 'drift_min_height_m': 0.40}
        for k, v in d.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in d}
        self.test = str(self.p['test']).lower()
        if self.test not in TESTS:
            raise SystemExit(f"test must be one of {TESTS}, not {self.test!r}")
        self.steps = plan(self.test, self.p)

        self.pose_pub = self.create_publisher(PoseStamped, POSE_TOPIC, 10)
        self.twist_pub = self.create_publisher(TwistStamped, TWIST_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 10)
        self.create_subscription(String, STATE_TOPIC, self.state_cb, 10)
        self.create_subscription(VehicleLocalPosition, PX4_POS_TOPIC, self.px4_cb, PX4_QOS)
        self.ekf = {}                      # EKF2 fusion flags, for the log
        if EstimatorStatusFlags is not None:
            self.create_subscription(EstimatorStatusFlags, '/fmu/out/estimator_status_flags', self.ekf_cb, PX4_QOS)
        self.vo_state = None
        if VisualSlamStatus is not None:
            self.create_subscription(VisualSlamStatus, '/visual_slam/status', self.vo_cb, 10)

        self.pos = self.yaw = None         # PX4's own position / heading (ENU): what we fly on
        self.px4_vel = None
        self.px4_resets = (None, None, None, None)
        self.px4_at = None
        self.sp = {}                       # last setpoint sent, for the log
        self.corr = [0.0, 0.0]             # drift correction added to the carrot (x, y), metres
        self.note = ''
        self.log = FlightLog(str(self.p['log_dir']), self.test)
        self.odom_pos = self.odom_yaw = None   # cuVSLAM's, for the vision-health check and offset log
        self.odom_at = None
        self.offboard = self.armed = self.landed = False
        self.state_at = None
        self.mode = 'WAIT'                 # WAIT -> RUN -> (PAUSED) -> DONE
        self.origin = self.yaw0 = None     # pose when Offboard started
        self.carrot = None                 # (x, y, z, yaw) commanded
        self.i = 0                         # step index
        self.step_t0 = self.hold_t0 = None
        self.logged = set()
        self.create_timer(1.0 / float(self.p['rate_hz']), self.tick)
        names = ' -> '.join(k if a is None else f'{k} {a:.2f}' for k, a in self.steps)
        self.get_logger().info(f'test "{self.test}": {names}. Publishes only {POSE_TOPIC} / {TWIST_TOPIC}; '
                               'never arms. Waiting for the pilot to arm, then switch to Position. '
                               f'Flight log: {self.log.path}')

    # -- inputs --------------------------------------------------------------------------------
    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.odom_pos = (float(p.x), float(p.y), float(p.z))
        self.odom_yaw = yaw_of(msg.pose.pose.orientation)
        self.odom_at = self.now()

    def px4_cb(self, msg):
        """PX4's EKF2 estimate, NED -> ENU (the inverse of flight_interface's plain map)."""
        if not (msg.xy_valid and msg.z_valid) or not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
            self.once('px4invalid', 'PX4 local position not valid: nothing is commanded until it is.', warn=True)
            return
        self.pos = (float(msg.y), float(msg.x), -float(msg.z))
        if math.isfinite(msg.heading):
            self.yaw = wrap(math.pi / 2.0 - float(msg.heading))   # NED heading (CW from x) -> ENU yaw
        self.px4_vel = (float(msg.vy), float(msg.vx), -float(msg.vz))
        g = lambda name: getattr(msg, name, None)
        dxy = g('delta_xy')
        self.px4_resets = (g('xy_reset_counter'), g('z_reset_counter'), g('heading_reset_counter'),
                           None if dxy is None else round(math.hypot(float(dxy[0]), float(dxy[1])), 4))
        self.px4_at = self.now()

    def ekf_cb(self, msg):
        self.ekf = {k: int(bool(getattr(msg, f'cs_{k}', False)))
                    for k in ('ev_pos', 'ev_yaw', 'ev_hgt', 'rng_hgt', 'mag_hdg', 'mag_3d', 'yaw_align')}

    def vo_cb(self, msg):
        self.vo_state = int(msg.vo_state)

    def state_cb(self, msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        self.offboard = bool(s.get('offboard_active', False))
        self.armed = bool(s.get('armed', False))
        self.landed = bool(s.get('landed', False))
        self.state_at = self.now()
        self.clamped = ','.join(s.get('clamped') or [])
        if s.get('clamped'):
            self.once('clamp:' + ','.join(s['clamped']),
                      f'flight_interface is CLAMPING our setpoint ({", ".join(s["clamped"])}); '
                      f'envelope {s.get("envelope")}', warn=True)

    # -- outputs -------------------------------------------------------------------------------
    def send_pose(self, x, y, z, yaw=None):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
        if yaw is None:
            m.pose.orientation.w = 0.0          # all-zero quaternion = don't command yaw
        else:
            m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        self.pose_pub.publish(m)
        self.sp = {'sp_kind': 'pose', 'sp_x': float(x), 'sp_y': float(y), 'sp_z': float(z),
                   'sp_yaw_deg': None if yaw is None else math.degrees(yaw)}

    def send_twist(self, vx, vy, vz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = float(vx), float(vy), float(vz)
        self.twist_pub.publish(m)
        self.sp = {'sp_kind': 'twist', 'sp_vx': float(vx), 'sp_vy': float(vy), 'sp_vz': float(vz)}

    def write_log(self):
        """One recorder row: PX4, cuVSLAM, the offset between them, our command, EKF2 flags."""
        now = self.now()
        v = {'wall_time': round(time.time(), 3), 't': now, 'test': self.test, 'mode': self.mode,
             'step': self.steps[self.i][0] if self.mode == 'RUN' else '', 'note': self.note,
             'offboard': int(self.offboard), 'armed': int(self.armed), 'landed': int(self.landed),
             'clamped': getattr(self, 'clamped', ''), 'vo_state': self.vo_state}
        self.note = ''
        if self.pos is not None:
            v.update(px4_x=self.pos[0], px4_y=self.pos[1], px4_z=self.pos[2],
                     px4_age_s=None if self.px4_at is None else now - self.px4_at)
            if self.yaw is not None:
                v['px4_yaw_deg'] = math.degrees(self.yaw)
        if self.px4_vel is not None:
            v.update(px4_vx=self.px4_vel[0], px4_vy=self.px4_vel[1], px4_vz=self.px4_vel[2])
        v.update(dict(zip(('px4_xy_resets', 'px4_z_resets', 'px4_heading_resets', 'px4_delta_xy'), self.px4_resets)))
        if self.odom_pos is not None:
            v.update(odom_x=self.odom_pos[0], odom_y=self.odom_pos[1], odom_z=self.odom_pos[2],
                     odom_yaw_deg=math.degrees(self.odom_yaw), odom_age_s=now - self.odom_at)
            if self.pos is not None:
                v.update(off_x=self.pos[0] - self.odom_pos[0], off_y=self.pos[1] - self.odom_pos[1],
                         off_z=self.pos[2] - self.odom_pos[2])
                if self.yaw is not None:
                    v['off_yaw_deg'] = math.degrees(wrap(self.yaw - self.odom_yaw))
        v.update(self.sp)
        if self.carrot is not None and self.pos is not None:
            v.update(carrot_err_xy=math.hypot(self.pos[0] - self.carrot[0], self.pos[1] - self.carrot[1]),
                     carrot_err_z=self.pos[2] - self.carrot[2], corr_x=self.corr[0], corr_y=self.corr[1])
        v.update({f'ekf_{k}': x for k, x in self.ekf.items()})
        self.log.row(v)

    def event(self, text, level='info'):
        """Log to the console AND into the recorder's `note` column."""
        lg = self.get_logger()          # rclpy: one call site may only ever use one severity
        if level == 'warn':
            lg.warning(text)
        elif level == 'error':
            lg.error(text)
        else:
            lg.info(text)
        self.note = (self.note + ' | ' if self.note else '') + text[:160]

    def destroy_node(self):
        self.log.close()
        return super().destroy_node()

    # -- helpers -------------------------------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def once(self, key, text, warn=False):
        if key not in self.logged:
            self.logged.add(key)
            self.event(text, "warn" if warn else "info")

    def odom_fresh(self):
        """Both PX4's estimate and cuVSLAM's odometry (vision health) are recent."""
        lim, now = float(self.p['odom_timeout_s']), self.now()
        return (self.odom_at is not None and now - self.odom_at <= lim
                and self.px4_at is not None and now - self.px4_at <= lim)

    def offset_text(self):
        """PX4's estimate minus cuVSLAM's, position and heading: the frame offset this node ignores."""
        if self.odom_pos is None or self.pos is None:
            return 'offset unknown (no cuVSLAM odometry yet)'
        d = [a - b for a, b in zip(self.pos, self.odom_pos)]
        dy = math.degrees(wrap(self.yaw - self.odom_yaw)) if None not in (self.yaw, self.odom_yaw) else float('nan')
        return (f'PX4 minus cuVSLAM: x {d[0]:+.3f} y {d[1]:+.3f} z {d[2]:+.3f} m '
                f'(horizontal {math.hypot(d[0], d[1]):.3f} m), heading {dy:+.1f} deg')

    def toward(self, cur, goal, max_step):
        d = goal - cur
        return goal if abs(d) <= max_step else cur + math.copysign(max_step, d)

    # -- the state machine (rate_hz) -----------------------------------------------------------
    def tick(self):
        try:
            self._tick()
        finally:
            self.write_log()                       # every tick, whatever happened

    def _tick(self):
        if self.mode == 'DONE':
            return                                   # publish nothing: no second takeover
        if self.pos is None or self.yaw is None:
            self.once('nopx4', f'no valid PX4 position/heading ({PX4_POS_TOPIC}) yet: nothing is commanded.', warn=True)
            return
        if self.mode == 'WAIT':
            if not self.odom_fresh():
                # No setpoint stream without healthy vision: flight_interface then can't take over and
                # the drone stays under the pilot.
                self.once('waitvision', f'waiting for fresh {ODOM_TOPIC} and PX4 position before streaming.', warn=True)
                return
            if not self.offboard:
                self.send_pose(*self.pos)            # "stay here", in PX4's frame: no jump at handover
                if 'offset0' not in self.logged or self.now() - getattr(self, 'offset_log_at', 0.0) > 10.0:
                    self.logged.add('offset0')
                    self.offset_log_at = self.now()
                    self.event('waiting; frame offset ' + self.offset_text())
                return
            self.origin, self.yaw0 = self.pos, self.yaw
            self.carrot = [self.pos[0], self.pos[1], self.pos[2], self.yaw]
            self.mode, self.i, self.step_t0, self.hold_t0 = 'RUN', 0, self.now(), None
            self.event(f'Offboard active at PX4 ({self.pos[0]:.2f}, {self.pos[1]:.2f}, {self.pos[2]:.2f}) '
                                   f'heading {math.degrees(self.yaw):.0f} deg: starting "{self.test}". '
                                   f'Frame offset ignored: {self.offset_text()}')
        if self.mode == 'RUN' and not self.offboard:
            kind = self.steps[self.i][0]
            if kind == 'land' and (self.landed or not self.armed):
                self.event('touched down and disarmed: test complete. This node is done.')
                self.mode = 'DONE'
                return
            self.event(f'Offboard ended during "{kind}": the pilot has control. Stopping.', "warn")
            self.mode = 'PAUSED'
        if self.mode == 'PAUSED':
            # PX4 ignores setpoints outside Offboard. Never retake: once disarmed, stop for good.
            if not self.armed:
                self.event('disarmed: this node is done. Restart the launch to fly again.')
                self.mode = 'DONE'
            return
        self.run_step()

    def advance(self, why):
        self.i += 1
        self.step_t0, self.hold_t0 = self.now(), None
        kind, arg = self.steps[self.i]
        self.event(f'{why} -> {kind}' + ('' if arg is None else f' {arg:.2f}'))

    def update_drift_correction(self, c, z0, dt, fresh):
        """Integral action on the commanded point: if the drone keeps sitting off the target (the carrot) in
        one direction, move the command the other way, slowly, so the drone itself ends up on the target.
        Only once airborne (drift_min_height_m above the start), only while the error is believable
        (< drift_max_error_m: a bigger one is a tracking problem or a pilot, not a steady push), only from
        fresh data, never inside the deadband, and capped at drift_max_m. Arrival checks still compare the
        drone with the TRUE target, so the correction can't fake an arrival."""
        if not bool(self.p['drift_correction']) or not fresh:
            return
        if self.steps[self.i][0] not in ('hold', 'yaw'):
            # Only learn while the target stands still. During climbs and moves the drone lags the moving
            # target by design; integrating that lag winds the correction up and overshoots the move
            # (13 cm at each end of `forward` in the fake-vehicle test). Keep the learned value meanwhile.
            return
        if self.pos[2] < z0 + float(self.p['drift_min_height_m']):
            return
        ex, ey = c[0] - self.pos[0], c[1] - self.pos[1]          # target minus drone
        e = math.hypot(ex, ey)
        if e > float(self.p['drift_max_error_m']):
            self.once(f'driftbig{self.i}', f'drift correction paused: {e:.2f} m from target is too far to be a '
                      'steady push', warn=True)
            return
        if e <= float(self.p['drift_deadband_m']):
            return
        k = float(self.p['drift_ki']) * dt
        cx, cy = self.corr[0] + k * ex, self.corr[1] + k * ey
        n, lim = math.hypot(cx, cy), float(self.p['drift_max_m'])
        if n > lim:
            cx, cy = cx * lim / n, cy * lim / n
            self.once('driftcap', f'drift correction at its {lim:.2f} m cap: something keeps pushing the drone '
                      '(check the battery / centre of gravity, props).', warn=True)
        self.corr = [cx, cy]

    def run_step(self):
        kind, arg = self.steps[self.i]
        dt = 1.0 / float(self.p['rate_hz'])
        x0, y0, z0 = self.origin
        c = self.carrot
        fresh = self.odom_fresh()
        if not fresh:
            self.once(f'stale{self.i}', f'odometry stale during "{kind}": carrot frozen, holding.', warn=True)
        if kind == 'land':
            ex, ey = c[0] - self.pos[0], c[1] - self.pos[1]
            n = math.hypot(ex, ey)
            k = min(1.0, float(self.p['land_xy_max_mps']) / n) if n > 1e-6 else 0.0
            self.send_twist(ex * k, ey * k, -float(self.p['land_speed_mps']))   # until PX4 disarms
            return
        if fresh:
            if kind == 'climb':
                c[2] = self.toward(c[2], z0 + arg, float(self.p['climb_speed_mps']) * dt)
            elif kind == 'yaw':
                goal = self.yaw0 + arg
                c[3] = self.toward(c[3], goal, math.radians(float(self.p['yaw_rate_dps'])) * dt)
            elif kind == 'move':
                gx, gy = x0 + arg * math.cos(self.yaw0), y0 + arg * math.sin(self.yaw0)
                ex, ey = gx - c[0], gy - c[1]
                n = math.hypot(ex, ey)
                s = float(self.p['speed_mps']) * dt
                if n <= s:
                    c[0], c[1] = gx, gy
                else:
                    c[0], c[1] = c[0] + ex / n * s, c[1] + ey / n * s
        self.update_drift_correction(c, z0, dt, fresh)
        self.send_pose(c[0] + self.corr[0], c[1] + self.corr[1], c[2], c[3])
        # arrival: the carrot has reached its goal AND the drone is at the carrot
        at_xy = math.hypot(self.pos[0] - c[0], self.pos[1] - c[1]) <= float(self.p['arrive_xy_m'])
        at_z = abs(self.pos[2] - c[2]) <= float(self.p['arrive_z_m'])
        at_yaw = abs(wrap(self.yaw - c[3])) <= math.radians(float(self.p['arrive_yaw_deg']))
        if kind == 'climb':
            done = abs(c[2] - (z0 + arg)) < 1e-6 and at_z and at_xy
        elif kind == 'yaw':
            done = abs(wrap(c[3] - (self.yaw0 + arg))) < 1e-6 and at_yaw and at_xy and at_z
        elif kind == 'move':
            gx, gy = x0 + arg * math.cos(self.yaw0), y0 + arg * math.sin(self.yaw0)
            done = math.hypot(c[0] - gx, c[1] - gy) < 1e-6 and at_xy and at_z
        else:                                          # hold: the clock runs only while at the carrot
            if at_xy and at_z:
                self.hold_t0 = self.hold_t0 or self.now()
                done = self.now() - self.hold_t0 >= arg
            else:
                self.hold_t0, done = None, False
        limit = float(self.p['step_timeout_s']) + (arg if kind == 'hold' else 0.0)
        if done:
            self.advance(f'{kind} done')
        elif self.now() - self.step_t0 > limit:
            self.event(level="error", text=f'"{kind}" not finished in {limit:.0f} s: landing here. '
                                    'Check clamps (/skybeta/state) and tracking.')
            self.i = len(self.steps) - 1
            self.step_t0 = self.now()


def main(args=None):
    rclpy.init(args=args)
    node = BasicFlight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
