"""Continuity admission after the original VSLAM status/age/covariance gate."""
import math


class SourceAdmission:
    """Keep published odometry untouched. Never move the reference to an outlier."""
    def __init__(self):
        self.reference = None
        self.last_seen = None
        self.quarantined = False
        self.candidate = None
        self.reason = 'waiting for first source-qualified pose'

    def accept(self, stamp, xyz):
        if not all(math.isfinite(v) for v in (stamp,*xyz)):
            return False
        if self.last_seen is not None and stamp <= self.last_seen:
            return False
        self.last_seen = stamp
        if self.reference is not None:
            dt = stamp-self.reference[0]
            distance = math.dist(xyz,self.reference[1])
            if distance > dt * 1.0 or dt > .250001:
                self.quarantined = True
            if self.quarantined:
                # A persistent shift cannot eventually become admissible as dt grows.
                # Recovery is limited to two consistent measured samples within 10 cm
                # of the last trusted pose and within 0.5 s of its acquisition.
                if distance > min(.10, dt*1.0) or dt > .500001:
                    self.candidate = None
                    self.reason = 'outside bounded recovery region'
                    return False
                if (self.candidate is None or stamp-self.candidate[0] > .250001
                        or math.dist(xyz,self.candidate[1]) > stamp-self.candidate[0]):
                    self.candidate = (stamp,tuple(xyz))
                    self.reason = 'awaiting second consistent recovery sample'
                    return False
        self.reference = stamp,tuple(xyz)
        self.quarantined = False
        self.candidate = None
        self.reason = 'source qualified'
        return True


class LandedBootstrap:
    """First release requires fresh landed/disarmed PX4 and steady genuine VO."""
    def __init__(self):
        self.first = self.previous = self.origin = None
        self.count = 0
        self.ready = False

    def observe(self, now, stamp, xyz, status, land):
        if self.ready:
            return True
        grounded = (status is not None and land is not None and
                    0 <= now-status.timestamp/1e6 <= .75 and
                    0 <= now-land.timestamp/1e6 <= 1.5 and
                    status.arming_state == status.ARMING_STATE_DISARMED and
                    not status.failsafe and land.landed)
        if not grounded:
            self.first = self.previous = self.origin = None
            self.count = 0
            return False
        if self.previous is not None and stamp <= self.previous:
            return False
        if (self.previous is None or stamp-self.previous > .250001
                or math.dist(self.origin,xyz) > .05):
            self.first, self.origin, self.count = stamp, tuple(xyz), 0
        self.previous = stamp
        self.count += 1
        self.ready = self.count >= 10 and stamp-self.first >= 2
        return self.ready
