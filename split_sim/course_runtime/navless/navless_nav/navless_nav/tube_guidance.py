"""Bounded lateral tube guidance; no ROS, sensor generation or estimator writes."""
import math


class TubeGuidance:
    def __init__(self, max_shift=0.20, rate=0.10, timeout=0.30, max_travel=1.5):
        if not all(math.isfinite(v) and v > 0 for v in (max_shift, rate, timeout)):
            raise ValueError('tube guidance limits must be finite and positive')
        self.max_shift, self.rate, self.timeout = max_shift, rate, timeout
        self.max_travel = max_travel
        self.clear()

    def clear(self):
        self.measurement = None
        self.shift = 0.0
        self.last_time = None

    def update(self, stamp, center, yaw, now):
        values = (stamp, now, *center, yaw)
        if (len(center) != 3 or not all(math.isfinite(v) for v in values)
                or not -1e-6 <= now - stamp <= self.timeout
                or (self.measurement is not None and stamp <= self.measurement[0])):
            self.clear()
            return False
        self.measurement = (stamp, tuple(center), yaw)
        return True

    def apply(self, nominal, position, prior_yaw, now, *, enabled, max_lead):
        """Replace only the normal component of the nominal route target.

        Invalid/lost data returns the original target immediately. No old correction
        is carried across a loss, exit, time reset or authority change.
        """
        m = self.measurement
        if (not enabled or m is None or not -1e-6 <= now - m[0] <= self.timeout
                or (self.last_time is not None and now < self.last_time)):
            self.clear()
            return tuple(nominal)
        _, center, yaw = m
        if math.dist(center[:2], position[:2]) > self.max_travel:
            self.clear()
            return tuple(nominal)
        angle = math.atan2(math.sin(yaw - prior_yaw), math.cos(yaw - prior_yaw))
        # The line is undirected; either camera/travel orientation is allowed.
        angle = (angle + math.pi / 2) % math.pi - math.pi / 2
        n = (-math.sin(prior_yaw), math.cos(prior_yaw))
        shift = sum((center[i] - nominal[i]) * n[i] for i in (0, 1))
        along = sum((nominal[i] - center[i]) * (math.cos(prior_yaw), math.sin(prior_yaw))[i]
                    for i in (0, 1))
        shift += math.tan(angle) * along
        if abs(angle) > math.radians(15) or abs(shift) > self.max_shift:
            self.clear()
            return tuple(nominal)
        dt = 0.0 if self.last_time is None else min(now - self.last_time, 0.2)
        self.last_time = now
        self.shift += max(-self.rate * dt, min(self.rate * dt, shift - self.shift))
        out = tuple(nominal[i] + self.shift * n[i] for i in (0, 1)) + (nominal[2],)
        # Preserve the route follower's lead cap, including lateral corrections.
        d = math.dist(out, position)
        if d > max_lead and d > 0:
            dz = out[2]-position[2]
            if abs(dz) >= max_lead:
                return tuple(nominal)
            scale = math.sqrt((max_lead*max_lead-dz*dz) / sum((out[i]-position[i])**2 for i in (0, 1)))
            out = tuple(position[i] + (out[i]-position[i])*scale for i in (0, 1)) + (out[2],)
        return out
