"""Stationary registration of independent launch-VO and PX4 local position origins."""
from collections import deque
import math
from statistics import median


class LaunchDatum:
    """Freeze a measured NED position translation before the first setpoint stream."""

    def __init__(self):
        self.window = deque()
        self.offset = None

    def observe(self, stamp, px4_ned, vision_ned, stationary):
        if self.offset is not None:
            return
        if not stationary or not all(math.isfinite(v) for v in (stamp,*px4_ned,*vision_ned)):
            self.window.clear()
            return
        delta = tuple(a-b for a,b in zip(px4_ned,vision_ned))
        # Preserve the upstream 0.15 m prearm height check. An arbitrary large
        # discrepancy is never accepted just because the aircraft is stationary.
        if abs(delta[2]) > .15 or math.hypot(*delta[:2]) > .15:
            self.window.clear()
            return
        if self.window and stamp == self.window[-1][0]:
            return
        if self.window and not 0 < stamp-self.window[-1][0] <= .3:
            self.window.clear()
        self.window.append((stamp,delta))
        while len(self.window)>1 and self.window[1][0] <= stamp-5:
            self.window.popleft()

    def candidate(self, now):
        if (len(self.window)<10 or self.window[-1][0]-self.window[0][0]<5-1e-6
                or not 0<=now-self.window[-1][0]<=.5):
            return None
        axes=list(zip(*(delta for _,delta in self.window)))
        if any(max(axis)-min(axis)>.05 for axis in axes):
            return None
        return tuple(median(axis) for axis in axes)

    def freeze(self, now):
        if self.offset is None:
            self.offset=self.candidate(now)
        return self.offset is not None

    def to_px4(self, position):
        return list(position) if self.offset is None else [x+d for x,d in zip(position,self.offset)]

    def from_px4(self, position):
        return list(position) if self.offset is None else [x-d for x,d in zip(position,self.offset)]
