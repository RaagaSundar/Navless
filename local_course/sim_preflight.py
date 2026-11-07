#!/usr/bin/env python3
"""Passive >=30 wall-second telemetry evidence. Never sends PX4 inputs."""
import json
import statistics
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleStatus, VehicleLandDetected, VehicleLocalPosition, EstimatorStatusFlags, VehicleOdometry
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def main():
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--duration',type=float,default=35);a=p.parse_args()
    if a.duration < 30: p.error('At least 30 wall seconds required')
    rclpy.init(); n=Node('passive_preflight'); start=time.monotonic(); rows=[]; latest={}; counts={}; clock=[0.]
    def callback(key,msg):
        counts[key]=counts.get(key,0)+1; latest[key]=(time.monotonic(),msg)
    specs=[('status','/fmu/out/vehicle_status_v1',VehicleStatus),('land','/fmu/out/vehicle_land_detected',VehicleLandDetected),('position','/fmu/out/vehicle_local_position',VehicleLocalPosition),('estimator','/fmu/out/estimator_status_flags',EstimatorStatusFlags),('ev','/fmu/in/vehicle_visual_odometry',VehicleOdometry),('odom','/local_course/validated_odometry',Odometry),('bridge','/local_course/bridge_status',String)]
    subs=[n.create_subscription(typ,topic,lambda m,k=key:callback(k,m),qos_profile_sensor_data) for key,topic,typ in specs]
    subs.append(n.create_subscription(Clock,'/clock',lambda m:clock.__setitem__(0,m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data))
    next_sample=start
    while time.monotonic()-start<a.duration:
        rclpy.spin_once(n,timeout_sec=.05)
        now=time.monotonic()
        if now<next_sample:continue
        next_sample=now+1; row={'wall_elapsed':now-start,'sim_time':clock[0],'counts':dict(counts)}
        for key,(receipt,m) in latest.items():
            data={'receipt_age_wall':now-receipt}
            for field in ('timestamp','timestamp_sample','arming_state','nav_state','landed','xy_valid','z_valid','v_xy_valid','v_z_valid','heading_good_for_control','cs_ev_pos','cs_ev_yaw','cs_ev_hgt','cs_rng_hgt','x','y','z','heading'):
                if hasattr(m,field):data[field]=getattr(m,field)
            if key in ('status','failsafe'):
                for name,typ in m.get_fields_and_field_types().items():
                    if typ == 'boolean': data[name]=getattr(m,name)
            if key=='ev':data['sample_age_sim']=clock[0]-m.timestamp_sample/1e6
            if key=='bridge':data['diagnostic']=m.data
            if key=='position':data['timestamp_difference_sim']=clock[0]-m.timestamp/1e6
            row[key]=data
        rows.append(row)
    output={'duration_wall':time.monotonic()-start,'samples':rows,'counts':counts,'note':'Passive stationary evidence; freshness includes wall receipt and simulation sample age. No commands sent.'}
    Path(a.output).write_text(json.dumps(output,indent=2));print(json.dumps({'output':a.output,'counts':counts,'last':rows[-1]},indent=2));n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
