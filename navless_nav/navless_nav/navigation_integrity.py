"""Source-time pose checks; no ROS, control outputs, reset service, or automatic recovery."""
import math


def _scalar(value):
    if isinstance(value, bool):
        raise ValueError('boolean is not a measurement')
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('nonfinite measurement')
    return value


def _vector(value, size):
    if isinstance(value, (str, bytes)):
        raise ValueError('expected a numeric sequence')
    values = tuple(_scalar(v) for v in value)
    if len(values) != size:
        raise ValueError('wrong measurement size')
    return values


class PoseIntegrity:
    """Only accepted samples advance continuity state. A latched fault requires a new instance.

    Position and quaternion (x, y, z, w) are checked against the last accepted source stamp,
    never against callback timing. Quaternion sign is immaterial; its full rotation angle is
    bounded by max_yaw_rate_rps. The caller still owns authority and the response to failure.
    """

    def __init__(self, max_age_s=.25, future_tolerance_s=.005, max_speed_mps=2.0,
                 max_yaw_rate_rps=3.0, position_slack_m=.05, yaw_slack_rad=.05,
                 frame_id='odom', child_frame_id='base_link'):
        self.max_age_s = _scalar(max_age_s)
        self.future_tolerance_s = _scalar(future_tolerance_s)
        self.max_speed_mps = _scalar(max_speed_mps)
        self.max_yaw_rate_rps = _scalar(max_yaw_rate_rps)
        self.position_slack_m = _scalar(position_slack_m)
        self.yaw_slack_rad = _scalar(yaw_slack_rad)
        if min(self.max_age_s, self.max_speed_mps, self.max_yaw_rate_rps) <= 0:
            raise ValueError('age, speed, and angular-rate limits must be positive')
        if min(self.future_tolerance_s, self.position_slack_m, self.yaw_slack_rad) < 0:
            raise ValueError('tolerances must be nonnegative')
        if not isinstance(frame_id, str) or not frame_id:
            raise ValueError('frame_id must be explicit')
        if not isinstance(child_frame_id, str) or not child_frame_id:
            raise ValueError('child_frame_id must be explicit')
        self.frame_id, self.child_frame_id = frame_id, child_frame_id
        self.reason = 'no_sample'
        self.fault = None
        self.last_stamp_s = None
        self._position = None
        self._quaternion = None
        self._reset_counter = None

    def _reject(self, reason, latch=False):
        if latch and self.last_stamp_s is not None and self.fault is None:
            self.fault = reason
        self.reason = self.fault or reason
        return False

    def observe(self, *, stamp_s, now_s, frame_id, child_frame_id, position, quaternion,
                reset_counter=None):
        if self.fault is not None:
            return self._reject(self.fault)
        try:
            stamp, now = _scalar(stamp_s), _scalar(now_s)
        except (TypeError, ValueError, OverflowError):
            return self._reject('invalid_time', latch=True)
        if frame_id != self.frame_id or child_frame_id != self.child_frame_id:
            return self._reject('frame_mismatch', latch=True)
        try:
            pos, quat = _vector(position, 3), _vector(quaternion, 4)
        except (TypeError, ValueError, OverflowError):
            return self._reject('invalid_pose', latch=True)
        norm = math.hypot(*quat)
        if abs(norm - 1.0) > .01:
            return self._reject('invalid_quaternion', latch=True)
        quat = tuple(q / norm for q in quat)
        if self.last_stamp_s is not None:
            if stamp < self.last_stamp_s:
                return self._reject('nonmonotonic_stamp', latch=True)
            if stamp == self.last_stamp_s:
                return self._reject('duplicate_stamp')
        age = now - stamp
        if age > self.max_age_s:
            return self._reject('stale_sample')
        if age < -self.future_tolerance_s:
            return self._reject('future_sample')
        if reset_counter is not None:
            try:
                counter = _scalar(reset_counter)
                if counter < 0 or not counter.is_integer():
                    raise ValueError('invalid reset counter')
            except (TypeError, ValueError, OverflowError):
                return self._reject('invalid_reset_counter', latch=True)
            if self._reset_counter is not None and counter != self._reset_counter:
                return self._reject('reset_counter_changed', latch=True)
        else:
            counter = None
        if self.last_stamp_s is not None:
            dt = stamp - self.last_stamp_s
            if math.dist(pos, self._position) > self.max_speed_mps * dt + self.position_slack_m:
                return self._reject('position_jump', latch=True)
            dot = abs(sum(a * b for a, b in zip(quat, self._quaternion)))
            angle = 2 * math.acos(min(1.0, dot))
            if angle > self.max_yaw_rate_rps * dt + self.yaw_slack_rad:
                return self._reject('orientation_jump', latch=True)
        self.last_stamp_s, self._position, self._quaternion = stamp, pos, quat
        if counter is not None:
            self._reset_counter = counter
        self.reason = 'ready'
        return True

    def fresh(self, now_s):
        if self.fault is not None:
            return self._reject(self.fault)
        if self.last_stamp_s is None:
            return self._reject('no_sample')
        try:
            age = _scalar(now_s) - self.last_stamp_s
        except (TypeError, ValueError, OverflowError):
            return self._reject('invalid_time')
        if age > self.max_age_s:
            return self._reject('stale_sample')
        if age < -self.future_tolerance_s:
            return self._reject('future_sample')
        self.reason = 'ready'
        return True
