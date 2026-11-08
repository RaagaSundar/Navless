"""First-flight tests for skybeta-19, the organisers' way: setpoints only, the pilot arms.

Three tests, picked with the `test` parameter (one launch file each):

    hover    climb 1 m, hold 10 s, land
    yaw      climb 1 m, hold 2 s, yaw 180 deg clockwise at 30 deg/s, hold 2 s, land
    forward  climb 1 m, hold 2 s, fly 2 m along the starting heading at 0.4 m/s, hold 10 s,
             fly back to the start at 0.4 m/s (heading unchanged), hold 2 s, land

The contract (skybeta README, competitor_guide.md): this node publishes ONLY
/skybeta/setpoint/pose and /skybeta/setpoint/twist, and reads /skybeta/state and
/visual_slam/tracking/odometry. It never talks to /fmu/*, never arms, never changes mode.
skybeta_flight_interface owns Offboard, the heartbeat and the safety envelope (altitude band,
speed cap) and clamps whatever we ask for.

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

Frames: ENU in cuVSLAM's `odom` frame (x/y = the camera's axes when the stack started, z up).
Distances and headings are relative to where Offboard started. Heading: ENU yaw, CCW positive;
"clockwise" in the yaw test means decreasing ENU yaw, as seen from above.
"""
import json
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

POSE_TOPIC = '/skybeta/setpoint/pose'
TWIST_TOPIC = '/skybeta/setpoint/twist'
STATE_TOPIC = '/skybeta/state'
ODOM_TOPIC = '/visual_slam/tracking/odometry'
TESTS = ('hover', 'yaw', 'forward')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


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
             'arrive_yaw_deg': 5.0, 'step_timeout_s': 20.0, 'odom_timeout_s': 0.5, 'rate_hz': 20.0}
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

        self.pos = self.yaw = None
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
                               'never arms. Waiting for the pilot to arm, then switch to Position.')

    # -- inputs --------------------------------------------------------------------------------
    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.pos = (float(p.x), float(p.y), float(p.z))
        self.yaw = yaw_of(msg.pose.pose.orientation)
        self.odom_at = self.now()

    def state_cb(self, msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        self.offboard = bool(s.get('offboard_active', False))
        self.armed = bool(s.get('armed', False))
        self.landed = bool(s.get('landed', False))
        self.state_at = self.now()
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

    def send_twist(self, vx, vy, vz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = float(vx), float(vy), float(vz)
        self.twist_pub.publish(m)

    # -- helpers -------------------------------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def once(self, key, text, warn=False):
        if key not in self.logged:
            self.logged.add(key)
            (self.get_logger().warn if warn else self.get_logger().info)(text)

    def odom_fresh(self):
        return self.odom_at is not None and self.now() - self.odom_at <= float(self.p['odom_timeout_s'])

    def toward(self, cur, goal, max_step):
        d = goal - cur
        return goal if abs(d) <= max_step else cur + math.copysign(max_step, d)

    # -- the state machine (rate_hz) -----------------------------------------------------------
    def tick(self):
        if self.mode == 'DONE':
            return                                   # publish nothing: no second takeover
        if self.pos is None:
            self.once('noodom', f'no {ODOM_TOPIC} yet: nothing is commanded until cuVSLAM tracks.', warn=True)
            return
        if self.mode == 'WAIT':
            if not self.offboard:
                self.send_pose(*self.pos)            # "stay here": lets flight_interface take over
                return
            self.origin, self.yaw0 = self.pos, self.yaw
            self.carrot = [self.pos[0], self.pos[1], self.pos[2], self.yaw]
            self.mode, self.i, self.step_t0, self.hold_t0 = 'RUN', 0, self.now(), None
            self.get_logger().info(f'Offboard active at ({self.pos[0]:.2f}, {self.pos[1]:.2f}, {self.pos[2]:.2f}) '
                                   f'heading {math.degrees(self.yaw):.0f} deg: starting "{self.test}".')
        if self.mode == 'RUN' and not self.offboard:
            kind = self.steps[self.i][0]
            if kind == 'land' and (self.landed or not self.armed):
                self.get_logger().info('touched down and disarmed: test complete. This node is done.')
                self.mode = 'DONE'
                return
            self.get_logger().warn(f'Offboard ended during "{kind}": the pilot has control. Stopping.')
            self.mode = 'PAUSED'
        if self.mode == 'PAUSED':
            # PX4 ignores setpoints outside Offboard. Never retake: once disarmed, stop for good.
            if not self.armed:
                self.get_logger().info('disarmed: this node is done. Restart the launch to fly again.')
                self.mode = 'DONE'
            return
        self.run_step()

    def advance(self, why):
        self.i += 1
        self.step_t0, self.hold_t0 = self.now(), None
        kind, arg = self.steps[self.i]
        self.get_logger().info(f'{why} -> {kind}' + ('' if arg is None else f' {arg:.2f}'))

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
        self.send_pose(c[0], c[1], c[2], c[3])
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
            self.get_logger().error(f'"{kind}" not finished in {limit:.0f} s: landing here. '
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
