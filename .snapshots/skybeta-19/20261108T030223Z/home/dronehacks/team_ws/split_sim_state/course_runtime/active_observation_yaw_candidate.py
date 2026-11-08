"""Finite experimental SITL stationary yaw observation; no control publishers.

Measured FRONT sector + original known launch standoff support one2deg step.
Rear/occluded clearance is NOT certified. Aircraft center must stay inside the
reviewed hover cylinder. Existing PX4/SLAM/gyro/integrity checks dominate.
"""
from dataclasses import dataclass
import json,math
from pathlib import Path
import numpy as np
from anticipatory_tangent_yaw_candidate import wrap


@dataclass(frozen=True)
class ObservationDecision:
    yaw_rad: float
    speed_limit_m_s: float
    action: str
    state: str
    reason: str


class FiniteYawObservation:
    def __init__(self,config):
        self.config=config;self.increment=math.radians(float(config['increment_deg']));self.rate=math.radians(float(config['yaw_rate_deg_s']))
        if not 0<abs(self.increment)<=math.radians(2) or not 0<self.rate<=math.radians(2):raise ValueError('finite test limited to2deg/2deg-per-second')
        if not (0<float(config['hover_radius_m'])<=.15 and 0<float(config['timeout_s'])<=20 and float(config['stationary_settle_s'])>=.5 and float(config['post_step_settle_s'])>=.4 and float(config['minimum_front_ray_distance_m'])>=.5 and .8<=config['height_band_m'][0]<config['height_band_m'][1]<=1.1):raise ValueError('finite commissioning bounds cannot be weakened')
        if config['assurance_scope']!='limited_front_sector_and_known_launch_standoff':raise ValueError('explicit blind-sector scope required')
        self.state='WAIT_STABLE';self.yaw=None;self.target=None;self.last_time=None;self.started=None;self.stable_since=None;self.settled_since=None;self.qualified=[];self.last_quality_stamp=None;self.terminal=None

    @classmethod
    def from_file(cls,path):return cls(json.loads(Path(path).read_text()))

    def result(self,action,reason):return ObservationDecision(0. if self.yaw is None else wrap(self.yaw),0.,action,self.state,reason)

    def health_ok(self,health,now):
        try:
            for flag in ('px4_valid','slam_valid','gyro_valid','integrity_valid','authority_valid','range_valid'):
                if health[flag] is not True:return False
            for key in ('pose_stamp_s','yaw_stamp_s','gyro_stamp_s','status_stamp_s','range_stamp_s'):
                limit=.75 if key=='status_stamp_s' else .25
                if not math.isfinite(health[key]) or not 0<=now-health[key]<=limit:return False
            return True
        except (KeyError,TypeError,ValueError):return False

    def sector_supported(self,sector,now,current,target):
        try:
            if sector['state']=='measured_fixed_surface_standoff':
                return (bool(sector['evidence_id']) and 0<=now-sector['stamp_s']<=.25
                        and sector['minimum_observed_body_range_lower_bound_m']>=float(self.config.get('minimum_fixed_surface_body_range_m',.35))
                        and sector['scope']=='observed_fixed_surfaces_only_not_free_space')
            return (sector['state']=='supported_measured_front_sector' and bool(sector['evidence_id'])
                    and math.isfinite(sector['stamp_s']) and 0<=now-sector['stamp_s']<=.25
                    and sector['minimum_measured_ray_distance_m']>=float(self.config['minimum_front_ray_distance_m'])
                    and abs(wrap(current-sector['heading_center_rad']))<=sector['half_width_rad']
                    and abs(wrap(target-sector['heading_center_rad']))<=sector['half_width_rad'])
        except (KeyError,TypeError,ValueError):return False

    def update(self,now,position_enu,measured_compass_yaw,health,quality,front_sector,phase='OBSERVE'):
        """Decision only. Caller holds its existing measured hover setpoint.

        On weak/UNKNOWN view: stop yaw and all translation immediately.
        REJECTED or timeout: request PX4 auto-land if authority/health remain.
        Integrity/authority loss: return integrity_stop; caller follows its
        existing stop-publication/failsafe policy, never sends blind commands.
        COMPLETE requests land after this one step, not another route leg.
        """
        try:now=float(now);measured_compass_yaw=float(measured_compass_yaw)
        except (TypeError,ValueError):return self.result('integrity_stop','invalid time/yaw')
        if not math.isfinite(now) or not math.isfinite(measured_compass_yaw):return self.result('integrity_stop','nonfinite time/yaw')
        if self.last_time is not None and not 0<now-self.last_time<=.25:return self.result('integrity_stop','clock gap/reversal')
        dt=0. if self.last_time is None else now-self.last_time;self.last_time=now
        if self.yaw is None:self.yaw=measured_compass_yaw
        if not self.health_ok(health,now):return self.result('integrity_stop','existing PX4/SLAM/gyro/range/integrity/authority health failed')
        if self.terminal:return self.result(self.terminal,'finite observation ended; no route progression')
        if phase!='OBSERVE':return self.result('hold','outside commissioned observation phase')
        if self.started is None:self.started=now
        if now-self.started>float(self.config['timeout_s']):self.state='ABORT';self.terminal='auto_land_requested';return self.result(self.terminal,'finite observation timeout')
        try:
            x,y,z=map(float,position_enu);center=self.config['launch_center_enu_xy'];speed=float(health['speed_m_s']);gyro_rate=abs(float(health['gyro_yaw_rate_deg_s']))
            position_ok=(all(math.isfinite(v) for v in [x,y,z,speed,gyro_rate]) and speed>=0 and math.hypot(x-center[0],y-center[1])<=float(self.config['hover_radius_m']) and self.config['height_band_m'][0]<=z<=self.config['height_band_m'][1])
        except (KeyError,TypeError,ValueError):return self.result('integrity_stop','invalid measured hover state')
        if not position_ok:self.state='ABORT';self.terminal='auto_land_requested';return self.result(self.terminal,'outside finite reviewed hover cylinder/height band')
        if health.get('known_bad_sector',False):return self.result('hold','known-bad/net sector: no active yaw probe')
        state=quality.get('state') if isinstance(quality,dict) else None
        if state=='rejected':self.state='ABORT';self.terminal='auto_land_requested';return self.result(self.terminal,'causal stereo/gyro/model observation rejected')
        try:stamp=float(quality.get('stamp_s',float('nan'))) if isinstance(quality,dict) else float('nan')
        except (TypeError,ValueError):stamp=float('nan')
        fresh=math.isfinite(stamp) and 0<=now-stamp<=.25
        if state!='qualified' or not fresh:
            self.qualified=[];self.stable_since=None;self.settled_since=None
            if self.state=='OBSERVE_VIEW':self.state='SETTLE'
            return self.result('hold','weak/unknown/stale actual view: pause yaw and progression')
        if speed>=.03:
            self.stable_since=None;self.settled_since=None;self.qualified=[]
            return self.result('hold','brake/hold until genuine measured stationary hover')
        target=self.yaw+self.increment if self.target is None else self.target
        if front_sector is None:front_sector=measured_fixed_surface_standoff(quality)
        if not self.sector_supported(front_sector,now,measured_compass_yaw,target):return self.result('hold','measured front sector lacks current step support; unseen gaps not free')
        if self.state=='WAIT_STABLE':
            self.stable_since=now if self.stable_since is None else self.stable_since
            if now-self.stable_since<float(self.config['stationary_settle_s']):return self.result('hold','collect stationary qualified baseline')
            self.target=self.yaw+self.increment;self.state='SLEW'
        if self.state=='SLEW':
            delta=wrap(self.target-self.yaw);bound=self.rate*dt;self.yaw+=max(-bound,min(bound,delta))
            if abs(wrap(self.target-self.yaw))<1e-8:self.state='SETTLE'
            return self.result('observe','single bounded increment under current genuine stereo/PX4/gyro health')
        if self.state=='SETTLE':
            if abs(wrap(self.target-measured_compass_yaw))>math.radians(.5) or gyro_rate>.5:
                self.settled_since=None;self.qualified=[];return self.result('hold','hold yaw until measured attitude settles')
            self.settled_since=now if self.settled_since is None else self.settled_since
            if now-self.settled_since<float(self.config['post_step_settle_s']):return self.result('hold','measured settle interval')
            self.state='OBSERVE_VIEW';self.qualified=[];self.last_quality_stamp=None
        if self.state=='OBSERVE_VIEW':
            if stamp<self.settled_since+float(self.config['post_step_settle_s']):return self.result('hold','need images acquired after yaw settled')
            if self.last_quality_stamp is None or stamp>self.last_quality_stamp:
                if self.last_quality_stamp is not None and stamp-self.last_quality_stamp>.3:self.qualified=[]
                self.qualified.append(stamp);self.last_quality_stamp=stamp
            if len(self.qualified)>=3 and self.qualified[-1]-self.qualified[0]>=.4-1e-8:
                self.state='COMPLETE';self.terminal='complete_land_requested';return self.result(self.terminal,'three fresh post-settle qualified samples; one-step experiment complete')
            return self.result('hold','observe actual next view; need3 qualified samples spanning.4s')
        return self.result('hold','bounded observation hold')


def measured_fixed_surface_standoff(quality,sigma_disparity_px=1.):
    """Actual verified stereo surface ranges, NOT any free-space certificate.

    Uses genuine current four-view fixed-track points and measured calibration.
    Declared conservative1px disparity sigma/3sigma range deduction is an
    experimental sensor-error assumption, not a measured calibration accuracy.
    Rear/occluded/gaps and uncertain distant points remain unknown.
    """
    unknown=dict(state='unknown',scope='observed_fixed_surfaces_only_not_free_space',blind_zones=['rear','occluded','gaps','discarded_uncertain_range'])
    if not isinstance(quality,dict) or quality.get('state')!='qualified':return unknown
    try:
        K=np.asarray(quality['calibration']['K'],float).reshape(3,3);baseline=float(quality['calibration']['baseline_m']);T=np.asarray(quality['calibration']['body_from_left_optical'],float)
        xyz=np.array([x['xyz_m'] for x in quality['landmarks_current_camera']],float)
        if xyz.ndim!=2 or xyz.shape[1]!=3 or T.shape!=(4,4) or not np.isfinite(K).all() or K[0,0]<=0 or not math.isfinite(baseline) or baseline<=0 or not math.isfinite(sigma_disparity_px) or sigma_disparity_px<1:return unknown
        finite=np.isfinite(xyz).all(1)&(xyz[:,2]>.25)&(xyz[:,2]<=4.);xyz=xyz[finite]
        if not len(xyz) or not np.isfinite(T).all() or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-5) or not np.allclose(T[3],[0,0,0,1],atol=1e-6) or np.linalg.det(T[:3,:3])<.999:return unknown
        body=xyz@T[:3,:3].T+T[:3,3];sigma_z=xyz[:,2]**2/(K[0,0]*baseline)*float(sigma_disparity_px)
        sigma_range=sigma_z*np.linalg.norm(xyz,axis=1)/xyz[:,2];lower=np.linalg.norm(body,axis=1)-3*sigma_range
        return dict(state='measured_fixed_surface_standoff',scope='observed_fixed_surfaces_only_not_free_space',evidence_id=quality['evidence_id'],stamp_s=float(quality['stamp_s']),minimum_observed_body_range_lower_bound_m=float(min(lower)),minimum_observed_body_range_m=float(np.linalg.norm(body,axis=1).min()),observed_points=len(xyz),assumed_disparity_sigma_px=float(sigma_disparity_px),blind_zones=unknown['blind_zones'])
    except (KeyError,TypeError,ValueError,OverflowError):return unknown
