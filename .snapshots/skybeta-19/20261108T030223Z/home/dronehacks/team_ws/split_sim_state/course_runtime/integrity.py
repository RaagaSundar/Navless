"""Rejection checks for measured split-SITL odometry and PX4 local position."""

import math


class IntegrityGate:
    """Compare launch-frame VO (+up) to sample-aligned PX4 local z (+down)."""

    def __init__(self, max_speed=1.0, height_limit=0.15, settle=0.2):
        self.max_speed = max_speed
        self.height_limit = height_limit
        self.settle = settle
        self.previous = None
        self.height_bad_since = None
        self.latched = None

    def observe(self, t, xyz, px4_z, armed):
        if self.latched:
            return False, self.latched
        if not all(math.isfinite(value) for value in (t, *xyz, px4_z)):
            return self.reject("nonfinite source")
        if self.previous is not None:
            previous_time, previous_xyz = self.previous
            dt = t - previous_time
            if dt <= 0:
                return False, "duplicate or backward source stamp"
            speed = math.dist(previous_xyz, xyz) / dt
            if speed > self.max_speed:
                return self.reject(f"VO step speed {speed:.3f}m/s exceeds approved "
                                   f"{self.max_speed:.3f}m/s envelope")
        self.previous = (t, tuple(xyz))

        disagreement = abs(xyz[2] + px4_z)
        if armed and disagreement > self.height_limit:
            if self.height_bad_since is None:
                self.height_bad_since = t
            duration = t - self.height_bad_since
            # Source timestamps have microsecond precision. Float subtraction
            # at a large simulation epoch can round a 200 ms interval down.
            if round(duration * 1e6) >= round(self.settle * 1e6):
                return self.reject(f"VO/PX4 vertical disagreement {disagreement:.3f}m "
                                   f"for {duration:.3f}s")
        else:
            self.height_bad_since = None
        return True, "continuous and vertically consistent"

    def reject(self, reason):
        self.latched = reason
        return False, reason
