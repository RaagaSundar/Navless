"""RC handover state, independent of ROS. A loss after handover is latched."""
from dataclasses import dataclass


@dataclass
class Handover:
    phase: str = "WAIT"
    since: float = None
    reason: str = "waiting for disarmed preflight"

    def stop(self, reason):
        self.phase, self.reason = "STOP", reason
        return self.phase

    def update(self, t, fault, landed, px_armed, fi_armed, px_off, fi_off):
        if self.phase == "STOP":
            return self.phase
        if fault:
            if self.phase != "WAIT":
                return self.stop(fault)
            self.since, self.reason = None, fault
            return self.phase
        if self.phase == "ACTIVE":
            if not all((px_armed, fi_armed, px_off, fi_off)):
                return self.stop("pilot takeover or disarm")
            return self.phase
        if not landed:
            if self.phase == "READY":
                return self.stop("left ground before pilot handover")
            self.since = None
            return self.phase
        if self.phase == "WAIT":
            if any((px_armed, fi_armed, px_off, fi_off)):
                return self.stop("expected disarmed ground start")
            if self.since is None:
                self.since = t
            if t - self.since >= 2:
                self.phase, self.reason = "READY", "awaiting RC handover"
        elif all((px_armed, fi_armed, px_off, fi_off)):
            self.phase, self.reason = "ACTIVE", "pilot handover confirmed"
        return self.phase
