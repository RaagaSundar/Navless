"""ROS-independent, fail-closed camera odometry gate for local SITL only.

Input is continuous base_link pose in gravity-aligned ENU odom; map poses
(loop closures) and optical-frame poses are never accepted. No truth input.
"""
import math
try:
    from .frames import launch_odom_to_ned, launch_quaternion_to_ned, launch_covariance_diagonals
except ImportError:
    from frames import launch_odom_to_ned, launch_quaternion_to_ned, launch_covariance_diagonals

DIAG = (0, 7, 14, 21, 28, 35)


def covariance_valid(c):
    if len(c) != 36 or not all(math.isfinite(x) for x in c):
        return False
    if any(c[i] <= 0 or c[i] >= 9999 for i in DIAG):
        return False
    # Require symmetry and positive semidefiniteness (LDL decomposition).
    if any(abs(c[i*6+j]-c[j*6+i]) > 1e-6 for i in range(6) for j in range(6)):
        return False
    l = [[0.0]*6 for _ in range(6)]
    d = [0.0]*6
    for i in range(6):
        l[i][i] = 1.0
        d[i] = c[i*6+i]-sum(l[i][k]**2*d[k] for k in range(i))
        if d[i] < -1e-10:
            return False
        for j in range(i+1, 6):
            r = c[j*6+i]-sum(l[j][k]*l[i][k]*d[k] for k in range(i))
            if abs(d[i]) <= 1e-10:
                if abs(r) > 1e-8:
                    return False
            else:
                l[j][i] = r/d[i]
    return True


def multiply(a, b):
    w,x,y,z = a
    W,X,Y,Z = b
    return (w*W-x*X-y*Y-z*Z, w*X+x*W+y*Z-z*Y,
            w*Y-x*Z+y*W+z*X, w*Z+x*Y-y*X+z*W)


def convert(msg, now_ns, reset, calibrated_bearing_deg=None):
    """Return PX4 fields; pose covariance uses REP-103 fixed-axis world errors.

    Default FRD world has arbitrary heading. An explicit calibrated bearing
    enables NED using only the reviewed static launch fixture.
    Unknown velocity is NaN, never fabricated zero or finite differences.
    """
    p, q, c = msg.pose.pose.position, msg.pose.pose.orientation, msg.pose.covariance
    h = math.sqrt(0.5)
    quat = multiply(multiply((0,h,h,0), (q.w,q.x,q.y,q.z)), (0,1,0,0))
    fields = dict(timestamp=now_ns//1000, timestamp_sample=stamp_ns(msg)//1000,
                pose_frame=2, position=[p.y,p.x,-p.z], q=list(quat),
                velocity_frame=0, velocity=[math.nan]*3,
                angular_velocity=[math.nan]*3, velocity_variance=[math.nan]*3,
                position_variance=[c[7],c[0],c[14]],
                orientation_variance=[c[28],c[21],c[35]],
                reset_counter=reset, quality=0)
    if calibrated_bearing_deg is not None:
        fields['pose_frame'] = 1  # NED only with explicit known launch fixture.
        fields['position'] = launch_odom_to_ned([p.x,p.y,p.z],calibrated_bearing_deg)
        fields['q'] = launch_quaternion_to_ned((q.w,q.x,q.y,q.z),calibrated_bearing_deg)
        fields['position_variance'], fields['orientation_variance'] = launch_covariance_diagonals(c,calibrated_bearing_deg)
    return fields


def stamp_ns(msg):
    return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec


class Gate:
    def __init__(self):
        self.last_stamp = 0
        self.last_position = None
        self.last_accept_ns = 0
        self.tracking = False
        self.reset = 0
        self.seen = False
        self.reason = 'awaiting_estimate'

    def lost(self, reason):
        self.tracking = False
        self.reason = reason

    def watchdog(self, now_ns, wall_since_receipt):
        """Simulation-time freshness plus bounded independent wall stall check."""
        if self.seen and (now_ns-self.last_accept_ns > 250_000_000
                          or now_ns < self.last_accept_ns-50_000_000
                          or wall_since_receipt > 2.0):
            self.lost('no_fresh_pair')

    def accept(self, msg, now_ns, health_ok, health_age, sample_matched):
        stamp = stamp_ns(msg)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = [p.x,p.y,p.z,q.x,q.y,q.z,q.w]
        reason = None
        if not health_ok or not sample_matched or not 0 <= health_age <= 0.25:
            reason = 'tracking_health'
        elif not -50_000_000 <= now_ns-stamp <= 250_000_000 or stamp <= 0:
            reason = 'sample_age'
        elif stamp <= self.last_stamp:
            reason = 'nonmonotonic_sample'
        elif msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            reason = 'frame_contract'
        elif not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[3:])-1) > 0.02:
            reason = 'pose'
        elif not covariance_valid(msg.pose.covariance):
            reason = 'covariance'
        elif self.tracking and self.last_position is not None:
            dt = (stamp-self.last_stamp)*1e-9
            distance = math.dist(values[:3], self.last_position)
            if distance > 0.3+5*dt:
                reason = 'pose_jump'
        if reason:
            self.lost(reason)
            return False
        if not self.tracking and self.seen:
            self.reset = (self.reset+1)&255
        self.seen = self.tracking = True
        self.last_stamp = stamp
        self.last_accept_ns = now_ns
        self.last_position = values[:3]
        self.reason = 'accepted'
        return True
