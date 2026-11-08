"""Staged planned-motion camera yaw with anticipation and translation speed cap.

Portable course geometry + validated estimated pose/yaw only; no truth, ROS,
control publisher or estimator substitution. Caller retains all safety gates.
Compass/NED yaw atan2(east,north); separate PX4-frame registration is required.
"""
from dataclasses import dataclass
import hashlib,json,math
from pathlib import Path
import numpy as np


def wrap(angle):return math.atan2(math.sin(angle),math.cos(angle))


def launch_flu_yaw_to_compass(measured_launch_flu_yaw,launch_compass_yaw=0.):
    """Measured FLU yaw CCW from launch nose → geographic compass CW north.

    Does not use raw PX4 arbitrary-heading FRD yaw. Caller validates source
    quaternion/freshness and separately registers output into PX4 control yaw.
    """
    if not math.isfinite(measured_launch_flu_yaw) or not math.isfinite(launch_compass_yaw):raise ValueError('nonfinite measured yaw')
    return wrap(launch_compass_yaw-measured_launch_flu_yaw)


@dataclass(frozen=True)
class TangentDecision:
    yaw_rad: float
    speed_limit_m_s: float
    hold: bool
    progress_m: float
    target_yaw_rad: float
    measured_yaw_error_rad: float
    reason: str


class AnticipatoryTangentYaw:
    def __init__(self,route_data,config):
        self.p=np.asarray(route_data['waypoints_enu'],float)
        if self.p.ndim!=2 or self.p.shape[1]!=3 or len(self.p)<2 or not np.isfinite(self.p).all():raise ValueError('invalid route')
        self.s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(self.p,axis=0),axis=1))]
        if np.any(np.diff(self.s)<=0):raise ValueError('repeated route point')
        self.cruise=float(config.get('cruise_m_s',.4));self.preview=float(config.get('lookahead_m',.35));self.chord=float(config.get('tangent_span_m',.24));self.rate=math.radians(float(config.get('max_yaw_rate_deg_s',10)))
        self.margin=float(config.get('yaw_rate_utilization',.65));self.accel=float(config.get('braking_accel_m_s2',.12));self.max_cross=float(config.get('max_cross_track_m',.65));self.initial=math.radians(float(config.get('initial_compass_deg',0)))
        if not (0<self.cruise<=.4 and 0<self.preview<=1 and .05<=self.chord<=1 and 0<self.rate<=math.radians(10) and 0<self.margin<1 and 0<self.accel<=.5):raise ValueError('invalid reviewed yaw/speed bounds')
        self.grid=np.linspace(0,self.s[-1],int(math.ceil(self.s[-1]/.02))+1)
        angles=[];last=self.initial
        for a in self.grid:
            center=min(self.s[-1],a+self.preview);lo=max(0,center-self.chord/2);hi=min(self.s[-1],center+self.chord/2)
            v=self.at(hi)[:2]-self.at(lo)[:2]
            if np.linalg.norm(v)>.015:last=math.atan2(v[0],v[1])
            lock=config.get('window_lock')
            if lock and float(lock['start_arc_m'])<=a<=float(lock['end_arc_m']):last=math.radians(float(lock['compass_deg']))
            angles.append(last)
        self.headings=np.unwrap(angles)
        slope=np.abs(np.gradient(self.headings,self.grid));self.speed=np.minimum(self.cruise,self.rate*self.margin/np.maximum(slope,1e-8))
        for i in range(len(self.speed)-2,-1,-1):self.speed[i]=min(self.speed[i],math.sqrt(self.speed[i+1]**2+2*self.accel*(self.grid[i+1]-self.grid[i])))
        for i in range(1,len(self.speed)):self.speed[i]=min(self.speed[i],math.sqrt(self.speed[i-1]**2+2*self.accel*(self.grid[i]-self.grid[i-1])))
        self.require_visual=bool(config.get('require_visual_evidence',True));self.reviewed_benchmarks=set(config.get('reviewed_visual_benchmark_sha256',[]));self.route_hash=config.get('route_sha256');self.last_qualified_yaw=self.initial
        self.progress_m=0.;self.yaw=self.initial;self.last_time=None;self.holding=False;self.finished=False

    @classmethod
    def from_files(cls,route_path,config_path):
        raw=Path(route_path).read_bytes();config=json.loads(Path(config_path).read_text())
        if hashlib.sha256(raw).hexdigest()!=config['route_sha256']:raise ValueError('tangent yaw route hash mismatch')
        return cls(json.loads(raw),config)

    def at(self,arc):return np.array([np.interp(arc,self.s,self.p[:,k]) for k in range(3)])

    def project(self,pos,dt):
        low=max(0,self.progress_m-.3);high=min(self.s[-1],self.progress_m+.35+.6*dt);best=(float('inf'),self.progress_m)
        start=max(0,int(np.searchsorted(self.s,low))-1);stop=min(len(self.p)-1,int(np.searchsorted(self.s,high))+1)
        for i in range(start,stop):
            a,b=self.p[i],self.p[i+1]
            lo=max(low,self.s[i]);hi=min(high,self.s[i+1])
            if lo>hi:continue
            d=b-a;l=self.s[i+1]-self.s[i];f=float(np.dot(pos-a,d)/np.dot(d,d));f=min((hi-self.s[i])/l,max((lo-self.s[i])/l,f));candidate=(float(np.linalg.norm(pos-a-f*d)),self.s[i]+f*l)
            if candidate<best:best=candidate
        if best[0]>self.max_cross:raise ValueError('position outside reviewed local route corridor')
        self.progress_m=max(self.progress_m,min(best[1],self.progress_m+.6*dt))

    def propose(self,position_enu,now):
        """Pure proposal for same-tick visual projection before update().

        Does not advance progress, timers or yaw. Producer must compensate
        its camera points to this pose/time; stale untranslated points cannot
        certify a moving future view.
        """
        pos=np.asarray(position_enu,float)
        if pos.shape!=(3,) or not np.isfinite(pos).all() or not math.isfinite(now):return None
        dt=0. if self.last_time is None else now-self.last_time
        if self.last_time is not None and not 0<dt<=.5:return None
        previous=self.progress_m
        try:
            self.project(pos,dt);arc=self.progress_m;target=float(np.interp(arc,self.grid,self.headings))
            return dict(progress_m=arc,target_yaw_rad=wrap(target))
        except ValueError:return None
        finally:self.progress_m=previous

    def update(self,position_enu,measured_compass_yaw,now,pose_stamp,yaw_stamp,phase='ROUTE',armed=True,visual_evidence=None):
        """Decision or None: stale/invalid data must trigger caller's stop policy.

        ROUTE needs independently fresh measured geographic yaw (not this
        command). Scale only Carrot advancement, never freshness timer dt:
        dt_scale=min(1,decision.speed_limit_m_s/carrot.speed_here()).
        Hold leaves the current carrot position fixed; do not advance/catch_up.
        COAST holds but can resume; LAND freezes for remaining lifecycle.
        """
        pos=np.asarray(position_enu,float);vals=[measured_compass_yaw,now,pose_stamp,yaw_stamp]
        if pos.shape!=(3,) or not np.isfinite(pos).all() or not all(math.isfinite(x) for x in vals) or not 0<=now-pose_stamp<=.5 or not 0<=now-yaw_stamp<=.5:return None
        if self.last_time is not None and not 0<now-self.last_time<=.5:return None
        dt=0.0 if self.last_time is None else now-self.last_time;self.last_time=now
        phase=phase.upper()
        if phase in ('LAND','LANDING','DONE','STOP'):self.finished=True
        if self.finished or phase!='ROUTE' or not armed:
            return TangentDecision(wrap(self.yaw),0.,True,self.progress_m,wrap(self.yaw),wrap(self.yaw-measured_compass_yaw),'lifecycle hold')
        try:self.project(pos,dt)
        except ValueError:return None
        target=float(np.interp(self.progress_m,self.grid,self.headings));target=self.yaw+wrap(target-self.yaw)
        visual_ok,visual_reason=self.visual_support(visual_evidence,target,now)
        if self.require_visual and not visual_ok:
            restore=self.yaw+wrap(self.last_qualified_yaw-self.yaw);bound=self.rate*dt
            self.yaw+=max(-bound,min(bound,restore))
            self.holding=True
            return TangentDecision(wrap(self.yaw),0.,True,self.progress_m,wrap(target),abs(wrap(target-measured_compass_yaw)),visual_reason)
        if visual_ok:self.last_qualified_yaw=measured_compass_yaw
        delta=target-self.yaw;bound=self.rate*dt
        # Proportional bounded approach avoids command jumps and low-speed flips.
        self.yaw+=max(-bound,min(bound,delta*min(1,3*dt)))
        error=abs(wrap(target-measured_compass_yaw))
        if error>math.radians(18):self.holding=True
        elif error<math.radians(8):self.holding=False
        cap=float(np.interp(self.progress_m,self.grid,self.speed))
        if self.holding:cap=0.
        return TangentDecision(wrap(self.yaw),cap,self.holding,self.progress_m,wrap(target),error,'align before advancing' if self.holding else 'anticipated tangent with yaw/braking cap')

    def visual_support(self,evidence,target,now):
        """Require a reviewed measured producer AND evidence of the future view.

        Healthy current images do not qualify unseen bearings. Thresholds and
        confidence must come from a benchmarked sensor-only producer/passive
        temporal index, never this yaw module or raw corner/net-knot counts.
        Empty reviewed-benchmark allowlist deliberately makes flight hold.
        """
        if not isinstance(evidence,dict):return False,'future-view support unknown: brake/retain qualified view'
        try:
            coverage=evidence['future_view_coverage'];stamp=float(evidence['stamp'])
            lo,hi=map(float,coverage['arc_range_m']);heading=float(coverage['heading_center_rad']);tolerance=float(coverage['heading_tolerance_rad'])
            good=(evidence['state']=='qualified' and evidence['source'] in ('live_fixed_landmark_projection','calibrated_passive_temporal_index')
                  and bool(evidence['evidence_id']) and evidence['benchmark_sha256'] in self.reviewed_benchmarks
                  and coverage['route_sha256']==self.route_hash
                  and all(math.isfinite(x) for x in [stamp,lo,hi,heading,tolerance])
                  and 0<=now-stamp<=.5 and lo<=self.progress_m<=hi and 0<tolerance<=math.radians(10)
                  and abs(wrap(heading-target))<=tolerance
                  and evidence['measured_checks']['nonperiodic_fixed_tracks_verified'] is True
                  and evidence['measured_checks']['independent_rotation_consistent'] is True
                  and evidence['measured_checks']['periodic_ambiguity_rejected'] is True)
            return bool(good),'reviewed future view supported' if good else 'future nonperiodic view unqualified: brake/retain view'
        except (KeyError,TypeError,ValueError,OverflowError):return False,'invalid visual evidence: brake/retain view'


def project_verified_fixed_landmarks(signal,target_bearing,measured_bearing,progress_m,
                                     route_sha256,benchmark_sha256,thresholds=None):
    """Future-view support from REAL verified four-view points, never new texture.

    Requires benchmarked thresholds supplied by the sensor-quality team. None
    returns unknown. Rotation-only projection assumes static landmarks and
    uses measured calibrated optical→FLU extrinsics; it cannot certify newly
    revealed/occluded surfaces or future translation. That limitation remains
    in evidence reasons and should be covered by a passive temporal index.
    """
    result=dict(state='unknown',source='live_fixed_landmark_projection',
                evidence_id=str(signal.get('evidence_id','unidentified')),
                benchmark_sha256=benchmark_sha256,stamp=signal.get('stamp_s',float('nan')),
                reasons=['future support requires reviewed thresholds and fixed landmarks'])
    if signal.get('state')!='qualified' or not isinstance(thresholds,dict):return result
    try:
        xyz=np.array([p['xyz_m'] for p in signal['landmarks_current_camera']],float)
        transform=np.asarray(signal['calibration']['body_from_left_optical'],float)
        K=np.asarray(signal['calibration']['K'],float).reshape(3,3)
        if xyz.ndim!=2 or xyz.shape[1]!=3 or transform.shape!=(4,4) or not np.isfinite(xyz).all() or not np.isfinite(transform).all():return result
        if not np.allclose(transform[3],[0,0,0,1],atol=1e-6) or not np.allclose(transform[:3,:3].T@transform[:3,:3],np.eye(3),atol=1e-5) or np.linalg.det(transform[:3,:3])<.999 or K[0,0]<=0 or K[1,1]<=0:return result
        delta=wrap(target_bearing-measured_bearing);c,s=math.cos(delta),math.sin(delta);rotation=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
        body=xyz@transform[:3,:3].T+transform[:3,3];future=(body@rotation.T-transform[:3,3])@transform[:3,:3]
        depth=future[:,2];uv=(future@K.T)[:,:2]/np.maximum(depth[:,None],1e-9)
        width=int(thresholds['width']);height=int(thresholds['height']);margin=float(thresholds['image_margin_px'])
        good=(depth>float(thresholds['minimum_depth_m']))&(depth<float(thresholds['maximum_depth_m']))&(uv[:,0]>margin)&(uv[:,0]<width-margin)&(uv[:,1]>margin)&(uv[:,1]<height-margin)
        cells=np.floor(uv[good]/[width/8,height/5]).astype(int);occupied=len(np.unique(cells,axis=0));count=int(good.sum())
        qualified=(count>=int(thresholds['minimum_tracks']) and occupied>=int(thresholds['minimum_grid_cells']) and abs(delta)<=math.radians(float(thresholds['max_projection_angle_deg'])))
        result.update(state='unknown',projection_tracks_sufficient=qualified,projected_tracks=count,projected_grid_cells=occupied,
                      current_arc_m=progress_m,proposed_heading_rad=target_bearing,
                      reasons=['rotation-only surviving verified fixed landmarks; unseen/occluded/new surfaces unqualified'])
        return result
    except (KeyError,TypeError,ValueError,OverflowError):return result
