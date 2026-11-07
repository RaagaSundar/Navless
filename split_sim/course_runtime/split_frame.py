"""Stationary launch registration for the split, arbitrary-heading FRD bridge.

No truth or geographic yaw aid. The unchanged bridge swaps ROS x/y and
negates z: it reports heading +90deg for an identity ROS launch orientation.
PX4 EV-FRD yaw fusion explicitly clears yaw_align; heading_good_for_control
is therefore not evidence of *relative* yaw aiding. This narrow contract
requires fresh, active EV position/yaw aiding and the expected bridge axes.
"""
import math
from frames import launch_odom_to_ned


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw(q):
    if q is None or len(q) != 4 or not all(map(math.isfinite, q)):
        raise ValueError("missing or nonfinite orientation")
    n = math.hypot(*q)
    if abs(n - 1) > .005:
        raise ValueError("invalid quaternion norm")
    w, x, y, z = (v / n for v in q)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


class SplitFrame:
    def __init__(self, course_bearing_deg):
        self.course = course_bearing_deg
        self.bearing = None
        self.first = self.last = self.anchor = self.reset = None
        self.count = 0

    def observe(self, now, t_vo, q, p, flags, armed, landed, aligned_heading):
        """Return ready/wait/reject and reason; caller stops on any rejection."""
        good = (flags is not None and 0 <= now-flags.timestamp/1e6 <= 1.5
                and flags.cs_tilt_align and flags.cs_ev_pos and flags.cs_ev_yaw
                and not flags.cs_ev_yaw_fault and not flags.cs_inertial_dead_reckoning)
        resets = (p.heading_reset_counter, p.xy_reset_counter, p.z_reset_counter)
        def invalid(msg):
            self.first = self.last = self.anchor = None
            self.count = 0
            return ('reject' if self.bearing is not None or armed else 'wait', msg)
        if not good or not (p.xy_valid and p.z_valid and p.v_xy_valid and p.v_z_valid):
            return invalid('missing valid PX4 position/velocity or fresh EV position/yaw aiding')
        if self.bearing is not None and resets != self.reset:
            return invalid('PX4 control frame reset after launch registration')
        if self.bearing is not None and not math.isfinite(p.heading):
            return invalid('nonfinite PX4 heading after launch registration')
        if self.bearing is not None:
            return 'ready', 'registered'
        if armed or not landed:
            return invalid('launch registration requires landed and disarmed aircraft')
        if (t_vo is None or not 0 <= now-t_vo <= .5 or
                aligned_heading is None or not math.isfinite(aligned_heading)):
            return invalid('waiting for contemporaneous launch observations')
        try:
            vo_yaw = yaw(q)
        except ValueError as exc:
            return invalid(str(exc))
        speed = math.hypot(p.vx,p.vy,p.vz)
        if not math.isfinite(speed) or speed > .15 or not math.isfinite(p.heading):
            return invalid('launch pose is moving or invalid')
        if abs(vo_yaw) > math.radians(3):
            return invalid('route requires startup odometry aligned to launch nose')
        angle = wrap(aligned_heading + vo_yaw)
        if abs(wrap(angle-math.pi/2)) > math.radians(3):
            return invalid('PX4/VO registration disagrees with split EV bridge convention')
        if self.last == t_vo:
            return 'wait','waiting for distinct samples'
        if (self.last is None or not 0 < t_vo-self.last <= .30 or self.reset != resets or
                abs(wrap(angle-self.anchor)) > math.radians(1)):
            self.first, self.anchor, self.count = t_vo, angle, 0
        self.last, self.reset = t_vo, resets
        self.count += 1
        if t_vo-self.first < 2 or self.count < 10:
            return 'wait','qualifying stationary launch heading'
        self.bearing = math.degrees(self.anchor)
        return 'ready','registered'

    def course_position(self, xyz):
        return launch_odom_to_ned(xyz, self.course)

    def course_yaw_to_control(self, geographic_yaw):
        if self.bearing is None:
            raise ValueError('control frame unregistered')
        return wrap(geographic_yaw + math.radians(self.bearing-self.course))
