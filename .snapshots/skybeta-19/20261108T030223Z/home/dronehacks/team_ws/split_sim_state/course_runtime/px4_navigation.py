"""Opt-in PX4 navigation guard. No publishing, truth, or estimator refeeding.

A one-second vision-age limit is an experimental BRAKE duration bound, not a
validated accuracy guarantee. Existing evidence includes biased vision where
PX4 uncertainty/valid flags alone did not reveal the error.
"""
import math
from collections import deque

RESET_FIELDS=('xy_reset_counter','z_reset_counter','vxy_reset_counter','vz_reset_counter','heading_reset_counter')


class PoseHistory:
    """Actual PX4 state samples, aligned by timestamp_sample, never extrapolated."""
    def __init__(self):
        self.samples=deque(maxlen=256)

    def add(self,p):
        stamp=float(p.timestamp_sample)/1e6
        values=tuple(float(getattr(p,k)) for k in ('x','y','z','heading'))
        if stamp<=0 or not all(math.isfinite(v) for v in (stamp,*values)):
            return False
        if self.samples and stamp<=self.samples[-1][0]:return False
        resets=tuple(int(getattr(p,k)) for k in RESET_FIELDS)
        self.samples.append((stamp,values,resets))
        while len(self.samples)>2 and self.samples[1][0]<stamp-2.:self.samples.popleft()
        return True

    def at(self,stamp):
        if stamp is None or not math.isfinite(stamp):return None
        for sample in self.samples:
            if abs(sample[0]-stamp)<1e-9:return sample[1]
        for a,b in zip(self.samples,list(self.samples)[1:]):
            if a[0]<stamp<b[0] and b[0]-a[0]<=.10+1e-9 and a[2]==b[2]:
                f=(stamp-a[0])/(b[0]-a[0])
                xyz=tuple(x+(y-x)*f for x,y in zip(a[1][:3],b[1][:3]))
                delta=math.atan2(math.sin(b[1][3]-a[1][3]),math.cos(b[1][3]-a[1][3]))
                yaw=math.atan2(math.sin(a[1][3]+delta*f),math.cos(a[1][3]+delta*f))
                return (*xyz,yaw)
        return None


class FusedNavigationGate:
    def __init__(self, registered_split_frame=None):
        self.registered_split_frame = registered_split_frame
        self.ready_since=None
        self.initialized=False
        self.resets=None
        self.coast_since=None

    def evaluate(self,p,now,vision_stamp,vision_ned,vision_yaw,flags,flags_stamp,aligned_pose=None,measured_conditional_range=False):
        def fail(reason):
            self.ready_since=None
            return 'reject' if self.initialized else 'waiting',reason
        required = ('xy_valid','z_valid','v_xy_valid','v_z_valid')
        if not all(bool(getattr(p,k,False)) for k in required):
            return fail('PX4 invalid position/velocity flags')
        if self.registered_split_frame is None:
            if not p.heading_good_for_control:
                return fail('PX4 heading_good_for_control false')
        else:
            frame = self.registered_split_frame
            if (frame.bearing is None or frame.reset != (p.heading_reset_counter,p.xy_reset_counter,p.z_reset_counter)
                    or flags is None or not flags.cs_ev_yaw or not flags.cs_tilt_align
                    or flags.cs_ev_yaw_fault or flags.cs_inertial_dead_reckoning):
                return fail('unqualified arbitrary-heading FRD or EV yaw aiding lost')
            # Three standard deviations must fit the existing ten-degree yaw
            # residual bound. This checks uncertainty separately from agreement.
            if not 0 <= p.heading_var <= (math.radians(10) / 3)**2:
                return fail('uncertain relative EV heading')
        for key,limit in [('eph',.15),('epv',.20),('evh',.30),('evv',.30)]:
            v=float(getattr(p,key,math.nan))
            if not math.isfinite(v) or not 0<=v<=limit:return fail(f'PX4 {key}={v} exceeds {limit}')
        if not all(math.isfinite(float(getattr(p,k,math.nan))) for k in ('x','y','z','vx','vy','vz','heading')):
            return fail('nonfinite PX4 pose/velocity/heading')
        resets=tuple(int(getattr(p,k)) for k in ('xy_reset_counter','z_reset_counter','vxy_reset_counter','vz_reset_counter','heading_reset_counter'))
        if not self.initialized and resets!=self.resets:
            self.ready_since=None;self.resets=resets
        if self.initialized and resets!=self.resets:return fail(f'PX4 estimator reset {self.resets}->{resets}')
        if vision_stamp is None or not 0<=now-vision_stamp<=1.0:return fail('vision-age brake bound1.0s exceeded')
        flags_fresh=flags is not None and flags_stamp is not None and 0<=now-flags_stamp<=1.5
        if not flags_fresh:return fail('PX4 fusion-status stale')
        if not bool(getattr(flags,'cs_rng_hgt',False)) and not (
                measured_conditional_range and self.registered_split_frame is not None
                and flags.cs_ev_hgt and flags.cs_baro_hgt and not flags.cs_baro_fault):
            return fail('measured range height fusion inactive')
        if any(bool(getattr(flags,k,False)) for k in ('cs_rng_fault','cs_rng_stuck','cs_fake_hgt','fs_bad_acc_vertical')):
            return fail('range/vertical estimator integrity fault')
        # dist_bottom_valid is terrain validity, not range-height fusion:
        # it is false in the observed low flight despite cs_rng_hgt=true.
        vision_live=now-vision_stamp<=.25 and bool(flags.cs_ev_pos)
        if vision_live:
            if vision_ned is None or vision_yaw is None or not all(math.isfinite(v) for v in (*vision_ned[:2],vision_yaw)):return fail('no calibrated visual comparison')
            if aligned_pose is None or not all(math.isfinite(v) for v in aligned_pose):
                return fail('no acquisition-aligned PX4 pose history')
            residual=math.dist(aligned_pose[:2],vision_ned[:2])
            yaw_error=abs(math.atan2(math.sin(aligned_pose[3]-vision_yaw),math.cos(aligned_pose[3]-vision_yaw)))
            if residual>.20 or yaw_error>math.radians(10):return fail(f'EV/PX4 acquisition-aligned inconsistency XY={residual:.3f}m yaw={math.degrees(yaw_error):.2f}deg')
            if not self.initialized:
                self.ready_since=now if self.ready_since is None else self.ready_since
                if now-self.ready_since<1.0:return 'waiting','settling fresh fusion/vision1s'
                self.initialized=True;self.resets=resets
            self.coast_since=None
            return 'ready','fresh vision and fused local position'
        if not self.initialized:return fail('initialization requires fresh vision+active EV fusion')
        self.coast_since=now if self.coast_since is None else self.coast_since
        if now-self.coast_since>1.0:return fail('fusion-loss brake bound1.0s exceeded')
        return 'coast','braking only; route progression suspended; vision age<=1.0s'
