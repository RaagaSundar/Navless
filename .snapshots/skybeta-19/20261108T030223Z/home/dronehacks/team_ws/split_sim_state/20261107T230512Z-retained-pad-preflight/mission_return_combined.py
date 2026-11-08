#!/usr/bin/env python3
"""Second-leg RETURN preserving exact outbound frame, datum and reset counters."""
import argparse,hashlib,importlib.util,json,sys
from pathlib import Path
from return_contract import load_contract
from mission_return import install_return_course
from retained_registration import retained_frame_class,retained_datum_class,CounterMonitor

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--runtime',type=Path,required=True);ap.add_argument('--contract',type=Path,required=True);ap.add_argument('--contract-sha256');ap.add_argument('--check-contract',action='store_true');a,remaining=ap.parse_known_args();runtime=a.runtime.resolve();c=load_contract(a.contract,a.contract_sha256,runtime)
 if c['mode']!='combined':raise ValueError('combined adapter requires retained-original-origin route')
 def artifact(key):
  entry=c[key];p=(a.contract.resolve().parent/entry['path']).resolve()
  if hashlib.sha256(p.read_bytes()).hexdigest()!=entry['sha256']:raise ValueError(key+' SHA mismatch')
  return json.loads(p.read_text())
 seed=artifact('retained_registration');grade=artifact('outbound_grade')
 if not grade.get('passed'):raise ValueError('original unchanged independent outbound grade did not pass')
 if seed.get('world_id')!=c.get('world_id') or seed.get('outbound_run_id')!=c.get('outbound_run_id'):raise ValueError('outbound world/run identity differs from retained registration')
 if seed.get('base_mission_sha256')!=c['base_mission_sha256']:raise ValueError('outbound source/return base differs')
 if a.check_contract:print(json.dumps(c,indent=2));return 0
 if '--route' in remaining or '--duration' in remaining:raise ValueError('route/duration come only from combined RETURN contract')
 sys.path[:0]=[str(runtime),str(runtime/'navless/navless_nav')]
 import split_frame,launch_datum
 import navless_nav.course_mission as course
 from px4_msgs.msg import VehicleAttitude,VehicleOdometry
 from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
 monitor=CounterMonitor(seed);base_return=install_return_course(course.CourseMission,c)
 class CombinedReturnCourse(base_return):
  def __init__(self,*args,**kwargs):
   super().__init__(*args,**kwargs);qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
   self.create_subscription(VehicleAttitude,'/fmu/out/vehicle_attitude',monitor.attitude_cb,qos);self.create_subscription(VehicleOdometry,'/fmu/in/vehicle_visual_odometry',monitor.vision_cb,qos)
 split_frame.SplitFrame=retained_frame_class(split_frame.SplitFrame,seed,monitor);launch_datum.LaunchDatum=retained_datum_class(launch_datum.LaunchDatum,seed);course.CourseMission=CombinedReturnCourse
 spec=importlib.util.spec_from_file_location('retained_nvidia_return_base',runtime/'mission.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
 sys.argv=[str(runtime/'mission.py'),*remaining,'--route',c['route_path'],'--duration',str(c['duration_sim_s'])]
 if '--qualified-astar' not in remaining:sys.argv.append('--qualified-astar')
 return module.main()
if __name__=='__main__':sys.exit(main())
