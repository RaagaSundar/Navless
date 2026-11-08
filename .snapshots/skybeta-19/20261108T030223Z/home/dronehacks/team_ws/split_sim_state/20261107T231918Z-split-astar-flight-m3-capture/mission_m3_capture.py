#!/usr/bin/env python3
"""Finite moving M3 sensor-capture wrapper; original Interface is sole controller.

No changed-world geometry/truth is loaded. Same frozen nominal prefix in both
worlds. This is a data-capture/stop experiment, NOT successful obstacle avoidance.
"""
import argparse,hashlib,importlib.util,json,math,sys,time
from pathlib import Path


def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def load_contract(path,runtime):
 c=json.loads(path.read_text())
 if c['schema']!='m3_capture/v1' or c['shadow_only'] is not True:raise ValueError('requires frozen shadow capture contract')
 if digest(Path(__file__))!=c['wrapper_sha256']:raise ValueError('wrapper changed after contract freeze')
 if digest(runtime/'mission.py')!=c['base_mission_sha256']:raise ValueError('base mission changed after contract freeze')
 route=path.parent/c['route_file']
 if digest(route)!=c['route_sha256']:raise ValueError('capture route SHA mismatch')
 if not c['independent_geometry_screen_passed']:raise ValueError('independent complete-prefix safety screen not passed')
 return c,route


def install_course(base,c,events):
 class CaptureCourse(base):
  def __init__(self,*args,**kwargs):
   super().__init__(*args,**kwargs)
   self.m3_land_reason=None;self.m3_hold_at=None;self.m3_hold_point=None;self.m3_stop_logged=False
   self.m3_brake_once=False;self.m3_brake_at=None;self.m3_brake_point=None;self.m3_low_speed_since=None
   self.m3_nav_stamp=None;self.m3_velocity=None;self.m3_last_pose_event=-math.inf
   self.m3_shadow=None;self.m3_shadow_wall=None;self.m3_last_shadow=-math.inf
   from std_msgs.msg import String
   self.create_subscription(String,'/split_course/obstacle_decision',self.m3_decision,10)
  def odometry_cb(self,msg):
   super().odometry_cb(msg)
   self.m3_nav_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
   v=msg.twist.twist.linear;self.m3_velocity=(float(v.x),float(v.y),float(v.z))
   if hasattr(self,'m3_last_pose_event') and (self.m3_brake_at is not None or self.m3_hold_at is not None or self.now_s()-self.m3_last_pose_event>=.10):
    self.m3_last_pose_event=self.now_s();self.m3_event('actual_navigation_sample',source_stamp_s=self.m3_nav_stamp,velocity_odom_mps=self.m3_velocity)
  def m3_event(self,event,**extra):
   row=dict(event=event,stamp_s=self.now_s(),source='actual_fused_PX4_navigation',position_odom_m=self.position,**extra)
   try:encoded=json.dumps(row,allow_nan=False)
   except (ValueError,TypeError):encoded=json.dumps(dict(event='unserializable_shadow_or_sample_rejected',stamp_s=self.now_s()))
   events.write(encoded+'\n');events.flush()
  def m3_decision(self,msg):
   try:d=json.loads(msg.data)
   except (ValueError,TypeError):return
   self.m3_shadow=d;self.m3_shadow_wall=time.monotonic()
   if self.now_s()-self.m3_last_shadow>.25:
    self.m3_last_shadow=self.now_s();self.m3_event('shadow_obstacle_prediction',decision=d)
  def start_route(self):
   super().start_route()
   if self.state!='ROUTE':return
   # Preserve upstream path/profile; only reduce prefix cruise and terminal caps.
   for i in range(len(self.carrot.limits)):
    self.carrot.limits[i]=min(self.carrot.limits[i],c['prefix_speed_cap_mps'] if i<c['slow_from_parent_index'] else c['terminal_speed_cap_mps'])
   self.m3_event('capture_route_started',goal='finite nominal-prefix capture, not full course',parent_last_index=c['parent_last_index'])
  def _state_route(self):
   now=self.now_s()
   # One actual commanded-stop transient early in original straight corridor.
   if not self.m3_brake_once and self.carrot.index>=c['braking_trigger_parent_index']:
    self.m3_brake_once=True;self.m3_brake_at=now;self.m3_brake_point=tuple(self.position)
    self.m3_event('commanded_open_corridor_stop',carrot_parent_index=self.carrot.index,stop_target_odom_m=self.m3_brake_point)
   if self.m3_brake_at is not None:
    self.last_tick=now;self.send(*self.m3_brake_point)
    speed=math.inf if self.m3_velocity is None else math.sqrt(sum(v*v for v in self.m3_velocity))
    if speed<c['stationary_speed_mps']:
     if self.m3_low_speed_since is None:self.m3_low_speed_since=now
    else:self.m3_low_speed_since=None
    elapsed=now-self.m3_brake_at
    settled=self.m3_low_speed_since is not None and now-self.m3_low_speed_since>=c['stationary_settle_s']
    if elapsed<c['braking_hold_s'] or not settled:
     if elapsed>=c['braking_timeout_s']:
      self.m3_land_reason='bounded actual stop did not settle; ordinary landing';self.goto('LAND',self.m3_brake_point);self.m3_hold_at=now;self.m3_hold_point=self.m3_brake_point
     return
    self.m3_event('open_corridor_stop_window_complete',duration_s=elapsed,actual_stationary_speed_mps=speed)
    self.m3_brake_at=None;self.last_tick=now
   if self.carrot.index>=c['slow_from_parent_index']:
    self.arrival_radius=min(self.arrival_radius,c['terminal_arrival_radius_m'])
   super()._state_route()
  def _state_land(self):
   # Route end is a real predeclared capture endpoint, not a fabricated pad.
   now=self.now_s()
   if self.m3_hold_at is None:
    self.m3_hold_at=now;self.m3_hold_point=tuple(self.target)
    self.m3_event('capture_endpoint_stop',stop_target_odom_m=self.m3_hold_point)
   self.send(*self.m3_hold_point)
   if now-self.m3_hold_at>=c['endpoint_hold_s'] and self.m3_land_reason is None:
    self.m3_land_reason='predeclared nominal-prefix sensor capture complete; ordinary landing, not course completion'
    self.m3_event('normal_autoland_intent',reason=self.m3_land_reason,shadow_receipt_present=self.m3_shadow is not None)
 return CaptureCourse


def main():
 p=argparse.ArgumentParser();p.add_argument('--runtime',required=True,type=Path);p.add_argument('--contract',required=True,type=Path);p.add_argument('--check-contract',action='store_true');a,remaining=p.parse_known_args();runtime=a.runtime.resolve();c,route=load_contract(a.contract.resolve(),runtime)
 if a.check_contract:print(json.dumps(c,indent=2));return 0
 if '--route'in remaining or '--duration'in remaining:raise ValueError('route and duration owned by frozen capture contract')
 sys.path[:0]=[str(runtime),str(runtime/'navless/navless_nav')]
 import navless_nav.course_mission as course
 evidence=next((remaining[i+1]for i,s in enumerate(remaining[:-1])if s=='--recorder-token'),None)
 runid=next((remaining[i+1]for i,s in enumerate(remaining[:-1])if s=='--run-id'),None)
 if not evidence or not runid:raise ValueError('actual flight recording token/run id mandatory')
 events=Path(evidence).with_name(runid+'-m3-capture-events.jsonl').open('x',buffering=1)
 course.CourseMission=install_course(course.CourseMission,c,events)
 spec=importlib.util.spec_from_file_location('m3_capture_native_base',runtime/'mission.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
 def capture_land_ready(mission,phase,nav_state,armed,offboard,failsafe,landed):
  return (phase=='ACTIVE' and bool(getattr(mission,'m3_land_reason',None)) and nav_state=='ready' and armed and offboard and not failsafe and not landed)
 module.route_land_ready=capture_land_ready
 sys.argv=[str(runtime/'mission.py'),*remaining,'--route',str(route),'--duration',str(c['duration_sim_s'])]
 if '--qualified-astar'not in remaining:sys.argv.append('--qualified-astar')
 try:return module.main()
 finally:events.close()
if __name__=='__main__':sys.exit(main())
