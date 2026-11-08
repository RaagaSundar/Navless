#!/usr/bin/env python3
"""Optional outbound observer wrapper: preserve exact accepted frame for RETURN.

No flight/controller changes: original methods run first and unchanged. Output
contains actual measured registration only; no simulator truth or new fix.
"""
import argparse,importlib.util,json,hashlib,os,sys
from pathlib import Path
from retained_registration import RESET_NAMES

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--runtime',type=Path,required=True);ap.add_argument('--registration-output',type=Path,required=True);ap.add_argument('--world-id',required=True);ap.add_argument('--outbound-run-id',required=True);a,remaining=ap.parse_known_args();runtime=a.runtime.resolve()
 if a.registration_output.exists():raise ValueError('registration output already exists')
 sys.path[:0]=[str(runtime),str(runtime/'navless/navless_nav')]
 import split_frame,launch_datum
 import navless_nav.course_mission as course
 from px4_msgs.msg import VehicleAttitude,VehicleOdometry
 from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
 state={}
 def save():
  need=('control_bearing_deg','reset_counters','datum_offset_ned_m','attitude_reset_counter','vision_reset_counter')
  if any(k not in state for k in need) or a.registration_output.exists():return
  result=dict(state,schema_version=1,outbound_run_id=a.outbound_run_id,world_id=a.world_id,source='actual accepted stationary SplitFrame and LaunchDatum, with actual PX4 attitude and bridge counters',base_mission_sha256=hashlib.sha256((runtime/'mission.py').read_bytes()).hexdigest())
  fd=os.open(a.registration_output,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
  with os.fdopen(fd,'w') as f:json.dump(result,f,indent=2);f.write('\n')
 class RecordingFrame(split_frame.SplitFrame):
  def observe(self,now,t_vo,q,p,flags,armed,landed,aligned_heading):
   answer=super().observe(now,t_vo,q,p,flags,armed,landed,aligned_heading)
   if answer[0]=='ready' and not armed and landed:
    state.update(control_bearing_deg=self.bearing,course_bearing_deg=self.course,reset_counters={k:int(getattr(p,k)) for k in RESET_NAMES},frame_source_stamp_s=t_vo);save()
   return answer
 class RecordingDatum(launch_datum.LaunchDatum):
  def freeze(self,now):
   ready=super().freeze(now)
   if ready:state.update(datum_offset_ned_m=list(self.offset),datum_frozen_sim_s=now);save()
   return ready
 class RecordingCourse(course.CourseMission):
  def __init__(self,*args,**kwargs):
   super().__init__(*args,**kwargs);qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
   def attitude(msg):state['attitude_reset_counter']=int(msg.quat_reset_counter);save()
   def vision(msg):state['vision_reset_counter']=int(msg.reset_counter);save()
   self.create_subscription(VehicleAttitude,'/fmu/out/vehicle_attitude',attitude,qos);self.create_subscription(VehicleOdometry,'/fmu/in/vehicle_visual_odometry',vision,qos)
 split_frame.SplitFrame=RecordingFrame;launch_datum.LaunchDatum=RecordingDatum;course.CourseMission=RecordingCourse
 spec=importlib.util.spec_from_file_location('observed_outbound_base',runtime/'mission.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);sys.argv=[str(runtime/'mission.py'),*remaining];return module.main()
if __name__=='__main__':sys.exit(main())
