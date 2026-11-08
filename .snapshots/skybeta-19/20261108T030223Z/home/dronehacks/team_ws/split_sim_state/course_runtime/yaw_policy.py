"""Optional portable camera look policy for a known north-start course fixture.

Uses validated launch odometry plus published course geometry, never truth.
Only the first northwest exterior corridor uses an east/inward look. No claim
that geometric visibility is visual tracking qualification. Default remains off.
"""
import math

MAX_LOOK_DEG = 60.0
RATE_DEG_S = 10.0


def desired_first_exterior_bearing(east, north):
    """Compass bearing; taper out eastward before the next corridor."""
    if north < 0.8:
        return 0.0
    weight = min(1.0, max(0.0, (-1.3-east)/0.6))
    return math.radians(MAX_LOOK_DEG*weight)


class FirstExteriorLook:
    def __init__(self):
        self.yaw = 0.0
        self.last_time = None
        self.started = False
        self.finished = False

    def step(self, east, north, height_above_launch, now, sample_time, armed):
        """NED yaw or None on invalid/stale data; time is ROS clock seconds.

        Rig replay sets armed=True and supplies identical course coordinates,
        height above launch, timestamps. Once the first region is left, the
        policy returns north and cannot re-enable on a later path crossing.
        """
        values=(east,north,height_above_launch,now,sample_time)
        if not all(math.isfinite(v) for v in values) or not 0 <= now-sample_time <= .5:
            return None
        if self.last_time is not None and now < self.last_time:
            return None
        dt = 0.0 if self.last_time is None else min(now-self.last_time,.1)
        self.last_time=now
        if not armed:
            return self.yaw
        if height_above_launch > .15:
            self.started=True
        if self.started and (east >= -1.3 or north < .8):
            self.finished=True
        target = desired_first_exterior_bearing(east,north) if self.started and not self.finished else 0.0
        bound=math.radians(RATE_DEG_S)*dt
        self.yaw += min(bound,max(-bound,target-self.yaw))
        return self.yaw
