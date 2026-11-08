"""Optional calibrated-feature-informed hybrid yaw, no truth or flight commands.

The matching heading profile is partial visual evidence, not VIO qualification.
Caller owns the existing navigation, telemetry, integrity and arming checks.
"""
import hashlib
import json
import math
from pathlib import Path
from map_yaw import MapYaw
from yaw_policy import FirstExteriorLook


class HybridCameraLook:
    def __init__(self, route_data, config, launch_height_m=.07):
        self.first=FirstExteriorLook()
        self.map=MapYaw(route_data['waypoints_enu'],config['route_arc_m'],
                        config['bearing_unwrapped_deg'],max_rate_deg_s=config['rate_limit_deg_s'])
        self.trigger=float(config.get('hybrid_trigger_arc_m',4.4))
        self.launch_height=float(launch_height_m)
        self.last_time=None
        self.hybrid_active=False
        self.yaw=0.0
        self.finished=False

    @classmethod
    def from_files(cls, route_path, config_path, launch_height_m=.07):
        route_bytes=Path(route_path).read_bytes()
        config=json.loads(Path(config_path).read_text())
        if hashlib.sha256(route_bytes).hexdigest()!=config['route_sha256']:
            raise ValueError('hybrid heading profile route hash mismatch')
        return cls(json.loads(route_bytes),config,launch_height_m)

    def step(self,east,north,height_above_launch,now,sample_time,armed,phase='ROUTE'):
        """Return compass/NED yaw radians or None for stale/invalid route input.

        Same first six position/time inputs as FirstExteriorLook, plus optional
        phase. Caller must pass its actual lifecycle phase to hold heading in
        COAST/LAND; only ROUTE advances the map schedule. Physical north launch
        and arbitrary PX4 control-frame registration stay separate in caller.
        """
        values=(east,north,height_above_launch,now,sample_time)
        if not all(math.isfinite(x) for x in values) or not 0<=now-sample_time<=.5:
            return None
        if self.last_time is not None and (now<self.last_time or now-self.last_time>.5):
            return None
        dt=0.0 if self.last_time is None else now-self.last_time
        self.last_time=now
        if self.finished:return self.yaw
        if phase.upper() == 'COAST':
            return self.yaw
        if phase.upper() in ('LAND','LANDING','DONE','STOP'):
            self.finished=True
            return self.yaw
        if not armed:return self.yaw
        first=self.first.step(east,north,height_above_launch,now,sample_time,armed)
        if first is None:return None
        if phase.upper()!='ROUTE' or height_above_launch<=.15 or dt==0:
            if not self.hybrid_active:self.yaw=first
            return self.yaw
        try:
            candidate=self.map.update((east,north,self.launch_height+height_above_launch),dt)
        except ValueError:
            return None
        if not self.hybrid_active:
            self.yaw=first
            if self.map.progress_m>=self.trigger:
                self.hybrid_active=True
                self.map.bearing_rad=first
            return self.yaw
        self.yaw=candidate
        return self.yaw
