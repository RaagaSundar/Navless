"""Passive interface and estimator monitor shared by preflight and the course runner."""
import json
import math

from std_msgs.msg import String
from px4_msgs.msg import VehicleStatus
from telemetry import Telemetry

POSE = '/skybeta/setpoint/pose'
TWIST = '/skybeta/setpoint/twist'
STATE = '/skybeta/state'


def xyz(v):
    return isinstance(v, list) and len(v) == 3 and all(type(x) in (int, float) and math.isfinite(x) for x in v)


class InterfaceMonitor(Telemetry):
    def __init__(self):
        super().__init__('real19_interface_monitor')
        self.fi, self.fi_stamp = None, 0.
        self.saw_armed = False
        self.seen_off = set()
        if self.get_parameter('use_sim_time').value or any(self.resolve_topic_name(t) != t for t in (POSE, TWIST, STATE)):
            raise RuntimeError('physical clock and standard Skybeta endpoints required')
        self.create_subscription(String, STATE, self.state_cb, 1)

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
            self.abort('Pilot takeover or disarm')

    def problem(self, now, expected_writers=0):
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
            if self.count_publishers(topic) != expected_writers:
                return 'unexpected setpoint publisher on ' + topic
            if not any(e.node_name == 'flight_interface' and e.node_namespace == '/'
                       for e in self.get_subscriptions_info_by_topic(topic)):
                return 'flight_interface is not subscribed to ' + topic
        if self.status is None or self.status.failsafe or not 0 <= now-self.status_time <= 1.:
            return 'missing/stale PX4 status or failsafe'
        if not self.position_ready(now) or not 0 <= now-self.pos_time <= 1.:
            return 'missing/stale position or heading'
        return self.vision_problem(now)

    def abort(self, why):
        self.get_logger().error(why + '; stopped publishing. Pilot has responsibility for recovery.')
        self.phase = 'ABORT'

