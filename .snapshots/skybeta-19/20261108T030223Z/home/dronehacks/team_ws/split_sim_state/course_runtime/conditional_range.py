"""Check actual down-range health across a mapped window sill in fixed-height flight."""
from collections import deque
import math


class WindowRangeHealth:
    def __init__(self, wall):
        self.samples = deque(maxlen=220)
        self.step = wall['window']['bottom_above_floor_in'] * .0254
        self.half_opening = wall['window']['width_in'] * .0254 / 2
        self.thickness = wall['thickness_in'] * .0254
        self.center = tuple(v*.0254-3.048 for v in wall['window']['center_in'])
        a,b = wall['start_in'],wall['end_in']
        length=math.dist(a,b)
        self.axis=((b[0]-a[0])/length,(b[1]-a[1])/length)
        # One wall thickness at 0.10 m/s, plus one status period and 0.5 s transport margin.
        self.limit = self.thickness/.10 + 1.5
        self.saw_normal=False
        self.edge_time=None
        self.before=None

    def receive(self, stamp, value, frame, minimum, maximum):
        if not math.isfinite(stamp) or self.samples and stamp<=self.samples[-1][0]:
            return
        valid=(frame=='downward_range_link' and math.isfinite(value)
               and abs(minimum-.02)<1e-5 and abs(maximum-4.)<1e-5 and minimum<=value<=maximum)
        self.samples.append((stamp,value if valid else None))

    def in_aperture(self,east,north):
        de,dn=east-self.center[0],north-self.center[1]
        along=self.axis[0]*de+self.axis[1]*dn
        across=-self.axis[1]*de+self.axis[0]*dn
        return (all(map(math.isfinite,(along,across))) and abs(along)<=self.half_opening-.30
                and abs(across)<=.30+self.thickness/2+.15)

    def check(self, now, flags, east, north, airborne, authority, position_at=None):
        def reject(reason):
            return False,False,reason
        if flags is None or not 0<=now-flags.timestamp/1e6<=1.5:
            return reject('missing or stale estimator status')
        if (not flags.cs_ev_hgt or not flags.cs_baro_hgt or flags.cs_fake_hgt or flags.cs_baro_fault
            or flags.fs_bad_acc_vertical or flags.cs_rng_fault or flags.cs_rng_stuck):
            return reject('fixed EV/baro height or range/IMU health invalid')
        samples=[s for s in self.samples if s[0]<=now]
        if not samples or now-samples[-1][0]>.20 or samples[-1][1] is None:
            return reject('missing, stale, invalid or out-of-range measured down laser')
        stamp,reading=samples[-1]
        normal=flags.cs_rng_hgt and flags.cs_rng_terrain and flags.cs_rng_kin_consistent
        if self.edge_time is not None and now-self.edge_time>self.limit and normal:
            self.edge_time=self.before=None
        if self.saw_normal and airborne and authority and self.edge_time is None:
            # Register the physical event while fusion can still be healthy. The
            # estimator flags may arrive later. Pair each edge with its own
            # measured control-frame position, not the later status location.
            for (t0,r0),(t1,r1) in reversed(list(zip(samples,samples[1:]))):
                if now-t1>self.limit:break
                if r0 is None or r1 is None or not 0<t1-t0<=.10:continue
                if abs((r0-r1)-self.step)>6*math.sqrt(2)*.01+(t1-t0):continue
                position = position_at(t1) if position_at else ((east,north) if now-t1<.10 else None)
                if position is not None and self.in_aperture(*position):
                    self.edge_time,self.before=t1,r0
                    break
        if normal:
            self.saw_normal=True
            return True,False,'range height and terrain updates active'
        if not self.saw_normal or not airborne or not authority:
            return reject('conditional range unavailable without established airborne authority')
        if not self.in_aperture(east,north):
            return reject('conditional range unavailable outside reviewed window corridor')
        if reading<.30-.03445:
            return reject('insufficient measured clearance below the aircraft envelope')
        if self.edge_time is None or not 0<=now-self.edge_time<=self.limit:
            return reject('missing recent sill-sized down-range step or bounded transition expired')
        if min(abs(reading-self.before),abs(reading-(self.before-self.step)))>.15:
            return reject('range inconsistent with sill and adjacent surface')
        if flags.cs_rng_kin_consistent and not flags.cs_rng_terrain:
            # Resumption may wait for PX4's regular-sample requirement, but the
            # beam must have already measured the original surface again.
            if abs(reading-self.before)>.10:
                return reject('range fusion unavailable without matching transition evidence')
        return True,True,'fresh measured window step; conditional range update temporarily suspended'
