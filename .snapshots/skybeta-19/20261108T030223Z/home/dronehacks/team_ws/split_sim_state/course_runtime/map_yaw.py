"""Portable arclength-based yaw schedule using only estimated ENU position.

Schedule is a reviewed route artifact. This module has no simulator, ROS or map
oracle dependency. Caller retains telemetry checks, authority and abort policy.
"""
import math
import numpy as np


class MapYaw:
    def __init__(self, points_enu, arc_m, bearing_unwrapped_deg,
                 max_rate_deg_s=7.0, max_progress_m_s=.6, max_cross_track_m=.65):
        self.points=np.asarray(points_enu,dtype=float)
        self.arc=np.asarray(arc_m,dtype=float)
        self.bearings=np.asarray(bearing_unwrapped_deg,dtype=float)
        if (self.points.ndim!=2 or self.points.shape[1]!=3 or len(self.points)<2
                or self.arc.ndim!=1 or len(self.arc)!=len(self.bearings)
                or len(self.arc)<2 or not np.isfinite(self.points).all()
                or not np.isfinite(self.arc).all() or not np.isfinite(self.bearings).all()
                or np.any(np.diff(self.arc)<=0) or self.arc[0]!=0):
            raise ValueError('invalid route/heading schedule')
        self.route_arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(self.points,axis=0),axis=1))]
        if np.any(np.diff(self.route_arc)<=0) or abs(self.route_arc[-1]-self.arc[-1])>.02:
            raise ValueError('heading schedule does not match route length')
        if not (0<max_rate_deg_s<=10 and max_progress_m_s>0 and max_cross_track_m>0):
            raise ValueError('invalid bounds')
        self.max_rate=math.radians(max_rate_deg_s)
        self.max_progress=max_progress_m_s
        self.max_cross_track=max_cross_track_m
        self.progress_m=0.0
        self.bearing_rad=math.radians(float(self.bearings[0]))

    def update(self, position_enu, dt):
        """Return bounded NED yaw radians. Raise on invalid/stale/off-route input.

        Progress is monotonic, projected only onto a local forward route window
        and capped by elapsed time. Crossing nearby route legs cannot jump yaw.
        dt must come from a fresh monotonic control update, not wall-clock gaps.
        """
        pos=np.asarray(position_enu,dtype=float)
        if pos.shape!=(3,) or not np.isfinite(pos).all() or not math.isfinite(dt) or not 0<dt<=.5:
            raise ValueError('yaw requires fresh finite position and 0<dt<=.5s')
        low=max(0.,self.progress_m-.30)
        high=min(self.route_arc[-1],self.progress_m+.35+self.max_progress*dt)
        candidates=[]
        for i,(a,b) in enumerate(zip(self.points,self.points[1:])):
            lo=max(low,self.route_arc[i]);hi=min(high,self.route_arc[i+1])
            if hi<lo:continue
            d=b-a;length=self.route_arc[i+1]-self.route_arc[i]
            f=float(np.dot(pos-a,d)/np.dot(d,d))
            f=min((hi-self.route_arc[i])/length,max((lo-self.route_arc[i])/length,f))
            candidates.append((float(np.linalg.norm(pos-(a+f*d))),self.route_arc[i]+f*length))
        distance,arc=min(candidates)
        if distance>self.max_cross_track:
            raise ValueError('estimated position outside reviewed yaw route corridor')
        self.progress_m=max(self.progress_m,min(arc,self.progress_m+self.max_progress*dt))
        target=math.radians(float(np.interp(self.progress_m,self.arc,self.bearings)))
        delta=target-self.bearing_rad
        self.bearing_rad+=max(-self.max_rate*dt,min(self.max_rate*dt,delta))
        return math.atan2(math.sin(self.bearing_rad),math.cos(self.bearing_rad))
