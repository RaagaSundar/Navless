"""Skybeta competitor node. All coordinates and yaw are in the continuous ENU odom frame."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String
from px4_msgs.msg import VehicleStatus
from telemetry import Telemetry

POSE = '/skybeta/setpoint/pose'
TWIST = '/skybeta/setpoint/twist'
STATE = '/skybeta/state'


def xyz(v):
    return isinstance(v, list) and len(v) == 3 and all(type(x) in (int, float) and math.isfinite(x) for x in v)


def angle(v):
    return math.atan2(math.sin(v), math.cos(v))


class Mission(Telemetry):
    def __init__(self, plan, passive=False):
        if plan not in ('hover', 'yaw', 'forward'):
            raise ValueError('unknown plan')
        super().__init__('skybeta_test_check' if passive else 'skybeta_test_' + plan)
        self.passive = passive
        self.plan = plan
        self.fi = self.home = self.since = self.prime = None
        self.saw_armed = False
        self.seen_off = set()
        self.last_problem = None
        self.fi_stamp = 0.
        self.started = self.entered = self.prev = time.monotonic()
        self.timeout = float(self.declare_parameter('preflight_timeout_s', 45.).value)
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('invalid timeout')
        if self.get_parameter('use_sim_time').value or any(self.resolve_topic_name(t) != t for t in (POSE, TWIST, STATE)):
            raise ValueError('hardware clock and unremapped Skybeta endpoints required')
        self.create_subscription(String, STATE, self.state_cb, 1)
        if passive:
            return  # Checker uses subscriptions only; never schedule a mission tick.
        self.pose_pub = self.create_publisher(PoseStamped, POSE, 1)
        self.twist_pub = self.create_publisher(TwistStamped, TWIST, 1)
        self.create_timer(.1, self.tick)
        self.get_logger().info('Waiting for disarmed ground checks; pilot arms in Altitude then selects Position.')

    def state_cb(self, msg):
        try:
            s = json.loads(msg.data)
            stamp = s['stamp']
            if not (type(stamp) in (float, int) and math.isfinite(stamp)
                    and all(type(s[k]) is bool for k in ('px4_link','armed','offboard_active','landed','frozen','envelope_ok'))
                    and xyz(s['origin_enu']) and xyz(s['local_enu']) and xyz(s['odom_enu'])
                    and isinstance(s['envelope'], dict)
                    and all(type(s['envelope'][k]) in (int, float) and math.isfinite(s['envelope'][k])
                            for k in ('min_altitude_m','max_altitude_m','max_speed_mps'))):
                raise ValueError()
            if stamp > self.fi_stamp:
                self.fi, self.fi_stamp = s, stamp
                self.authority_event('interface', s['armed'], s['offboard_active'])
        except (KeyError, TypeError, ValueError):
            self.fi = None

    def on_status(self, msg):
        super().on_status(msg)
        if self.status is msg:
            self.authority_event('px4', msg.arming_state == VehicleStatus.ARMING_STATE_ARMED,
                                 msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def authority_event(self, source, armed, offboard):
        if self.phase == 'READY':
            if source in self.seen_off and not offboard:
                self.abort('Lost Offboard during pilot handover')
            if offboard:
                self.seen_off.add(source)
        if self.phase == 'READY' and self.saw_armed and not armed:
            self.abort('Disarmed before handover; restart from preflight')
        if self.phase not in ('WAIT', 'READY', 'DONE', 'ABORT', 'OBSERVE') and not (armed and offboard):
            if self.phase == 'HANDOFF':
                self.change('OBSERVE', time.monotonic())
            else:
                self.abort('Pilot takeover or disarm before maneuver completion')

    def problem(self, now):
        s = self.fi
        if s is None or not -.005 <= now-self.fi_stamp <= .6 or not s['px4_link']:
            return 'missing/stale flight_interface state or PX4 link'
        e = s['envelope']
        if not (s['envelope_ok'] and not s['frozen'] and .3 <= e['min_altitude_m'] < e['max_altitude_m'] <= 2.5
                and 0 < e['max_speed_mps'] <= 1.):
            return 'interface frozen or envelope not within team limits'
        if self.count_publishers(STATE) != 1 or self.count_publishers(self.health_topic) != 1:
            return 'expected one interface and one health publisher'
        for topic in (POSE, TWIST):
            if self.count_publishers(topic) != (0 if self.passive else 1):
                return 'unexpected setpoint publisher on ' + topic
            if not any(e.node_name == 'flight_interface' and e.node_namespace == '/'
                       for e in self.get_subscriptions_info_by_topic(topic)):
                return 'flight_interface is not subscribed to ' + topic
        if self.status is None or self.status.failsafe or not 0 <= now-self.status_time <= 1.:
            return 'missing/stale PX4 status or failsafe'
        if not self.position_ready(now) or not 0 <= now-self.pos_time <= 1.:
            return 'missing/stale position or heading'
        return self.vision_problem(now)

    def change(self, phase, t):
        self.phase, self.entered, self.since = phase, t, None
        self.get_logger().info(phase + (': pilot take control and land; holding final pose' if phase == 'HANDOFF' else ''))

    def abort(self, why):
        self.get_logger().error(why + '; stopped publishing. Pilot has responsibility for recovery.')
        self.phase = 'ABORT'

    def tick(self):
        if self.phase in ('DONE', 'ABORT'):
            return
        t, now = time.monotonic(), self.clock_seconds()
        dt, self.prev = t-self.prev, t
        if self.phase == 'OBSERVE':  # Never publish again, even if Offboard returns.
            if (self.status is not None and self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED
                    and 0 <= now-self.status_time <= 1 and self.landed is True and 0 <= now-self.landed_time <= 1
                    and self.fi and self.fi['px4_link'] and not self.fi['armed'] and self.fi['landed']
                    and -.005 <= now-self.fi_stamp <= .6):
                self.change('DONE', t)
            elif t-self.entered > 60:
                self.abort('Pilot landing unconfirmed')
            return
        pre = self.phase in ('WAIT', 'READY')
        if pre and t-self.started >= self.timeout:
            return self.abort('Pilot handover timeout')
        reason = self.problem(now)
        if reason or not 0 <= dt <= .5:
            if self.phase == 'WAIT':
                if reason != self.last_problem:
                    self.get_logger().info('Waiting: ' + (reason or 'control loop gap'))
                self.last_problem = reason
                self.prime = None
                return
            return self.abort(reason or 'control loop gap')
        s, p = self.fi, self.pose
        armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        off = self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        if pre:
            if self.phase == 'WAIT' and (armed or s['armed'] or off or s['offboard_active']):
                return self.abort('Started or armed before disarmed preflight was complete')
            if not (self.landed is True and 0 <= now-self.landed_time <= 1 and s['landed']):
                if self.phase == 'READY':
                    return self.abort('Handover no longer on ground')
                self.prime = None
                return
            if self.phase == 'WAIT':
                self.prime = t if self.prime is None else self.prime
                if t-self.prime < 2:
                    return
                self.home = p
                self.origin = tuple(s['origin_enu'])
                height = p[2]+1-self.origin[2]
                if not s['envelope']['min_altitude_m'] < height <= s['envelope']['max_altitude_m']:
                    return self.abort('1 m climb outside configured envelope')
                self.change('READY', t)
                self.get_logger().info('READY: streaming stationary twist; awaiting pilot handover')
            if math.dist(p[:3], self.home[:3]) > .15 or abs(angle(p[3]-self.home[3])) > math.radians(6):
                return self.abort('Moved from launch pose before handover')
            self.saw_armed |= armed and s['armed']
            if not (armed and off and s['armed'] and s['offboard_active']):
                m = TwistStamped()  # Stationary; a ground pose would be clamped upward.
                m.header.frame_id, m.header.stamp = 'odom', self.get_clock().now().to_msg()
                self.twist_pub.publish(m)
                return
            self.change('CLIMB', t)
        if tuple(s['origin_enu']) != self.origin:
            return self.abort('Interface origin changed')
        if not (armed and off and s['armed'] and s['offboard_active']):
            return self.abort('Authority lost')
        if self.phase != 'HANDOFF' and t-self.entered > 30:
            return self.abort(self.phase + ' timed out')
        x, y, z0, yaw = self.home
        elapsed = t-self.entered
        phase = self.phase
        z = z0+1
        d = 0.
        if phase == 'CLIMB':
            z = min(z, max(z0+.3*elapsed, self.origin[2]+s['envelope']['min_altitude_m']))
        if phase == 'YAW' or (phase == 'HANDOFF' and self.plan == 'yaw'):
            yaw -= math.pi if phase == 'HANDOFF' else min(math.pi, math.radians(10)*elapsed)
        if phase in ('OUT', 'AT_FORWARD', 'RETURN'):
            d = min(2., .4*elapsed) if phase == 'OUT' else max(0., 2.-.4*elapsed) if phase == 'RETURN' else 2.
        x, y = x+d*math.cos(yaw), y+d*math.sin(yaw)
        if not (self.origin[2]+s['envelope']['min_altitude_m'] <= z <= self.origin[2]+s['envelope']['max_altitude_m']):
            return self.abort('Target outside reported altitude band')
        m = PoseStamped()
        m.header.frame_id, m.header.stamp = 'odom', self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.pose_pub.publish(m)
        if phase == 'HANDOFF':
            return
        final = {'CLIMB': z >= z0+1, 'YAW': elapsed >= 18., 'OUT': d == 2., 'RETURN': d == 0.}.get(phase, True)
        arrived = (final and math.hypot(p[0]-x,p[1]-y) <= .20 and abs(p[2]-z) <= .15
                   and abs(angle(p[3]-yaw)) <= math.radians(6))
        if not arrived:
            self.since = None
            return
        self.since = t if self.since is None else self.since
        wait, following = {
            'CLIMB': (1., 'HOVER'), 'HOVER': (10. if self.plan == 'hover' else 2.,
                       {'hover':'HANDOFF','yaw':'YAW','forward':'OUT'}[self.plan]),
            'YAW': (2., 'HANDOFF'), 'OUT': (1., 'AT_FORWARD'),
            'AT_FORWARD': (10., 'RETURN'), 'RETURN': (1., 'HANDOFF'),
        }[phase]
        if t-self.since >= wait:
            self.change(following, t)


def main(plan):
    rclpy.init()
    node = Mission(plan)
    try:
        while rclpy.ok() and node.phase not in ('DONE','ABORT'):
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        node.abort('Interrupted')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if node.phase == 'DONE' else 1
