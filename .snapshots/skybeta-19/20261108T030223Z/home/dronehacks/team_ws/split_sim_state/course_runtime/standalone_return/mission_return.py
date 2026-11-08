#!/usr/bin/env python3
"""RETURN adapter over unchanged full-qualified NVIDIA native mission.

All ROS/estimator/admission/recorder/arming/landing logic remains in the pinned
base mission. This changes only route, approved controller and vertical pad
launch behavior, adding a stricter measured-pad start-location check.
"""
import argparse,importlib.util,json,math,sys
from pathlib import Path
from return_contract import load_contract,pad_start_ok

def install_return_course(base_class,contract):
 class ReturnCourse(base_class):
  def __init__(self,*args,**kwargs):
   from rclpy.parameter import Parameter
   overrides=kwargs.get('parameter_overrides',[])
   # Use public ROS Parameter construction; never touch PX4 parameters.
   kwargs['parameter_overrides']=[Parameter(p.name,value=0.0) if p.name=='takeoff_lead_m' else p for p in overrides]
   kwargs['parameter_overrides'] += [Parameter('arrival_radius_m',value=.10),Parameter('max_start_offset_m',value=.20)]
   super().__init__(*args,**kwargs)
   actual={k:float(self.get_parameter(k).value) for k in contract['controller']}
   if actual!=contract['controller']:raise ValueError('actual return controller differs from contract')
  def lead_m(self):return 0.0
  def takeoff_target(self):
   return (self.position[0],self.position[1],self.route[0][2])
  def climbed(self):
   need=self.target[2]-.10
   return self.position[2]>=need or (self.px4_z is not None and self.now_s()-self.px4_z_at<=1.0 and self.px4_z>=need)
  def _state_wait_for_offboard(self):
   if self.frame is None or not pad_start_ok(self.position,self.frame[0],self.frame[1],float(self.get_parameter('start_height_m').value)):
    self.info_once('return_not_on_pad','RETURN not streaming: measured landed pose must be within .20m of goal pad in retained/calibrated course frame')
    return
   return super()._state_wait_for_offboard()
 return ReturnCourse

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--runtime',type=Path,required=True);ap.add_argument('--contract',type=Path,required=True);ap.add_argument('--contract-sha256');ap.add_argument('--check-contract',action='store_true');a,remaining=ap.parse_known_args();runtime=a.runtime.resolve();contract=load_contract(a.contract,a.contract_sha256,runtime)
 if a.check_contract:print(json.dumps(contract,indent=2));return 0
 if contract['mode']!='standalone':raise ValueError('combined return requires separately qualified retained-registration adapter; standalone adapter refuses re-registration')
 if '--yaw-policy' in remaining and remaining[remaining.index('--yaw-policy')+1]!='off':raise ValueError('standalone baseline profile holds north; a different yaw requires a separately reviewed contract')
 if '--route' in remaining:raise ValueError('route is supplied only by pinned RETURN contract')
 if '--duration' in remaining:raise ValueError('duration is supplied only by pinned RETURN contract')
 sys.path[:0]=[str(runtime),str(runtime/'navless/navless_nav')]
 import navless_nav.course_mission as course
 course.CourseMission=install_return_course(course.CourseMission,contract)
 spec=importlib.util.spec_from_file_location('pinned_nvidia_return_base',runtime/'mission.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
 sys.argv=[str(runtime/'mission.py'),*remaining,'--route',contract['route_path'],'--duration',str(contract['duration_sim_s'])]
 if '--qualified-astar' not in remaining:sys.argv.append('--qualified-astar')
 # Original main retains explicit approval, fresh stationary landed/disarmed,
 # full fused-source qualifiers, recorder/startup token, authority and LAND.
 return module.main()
if __name__=='__main__':sys.exit(main())
