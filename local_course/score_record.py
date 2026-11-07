#!/usr/bin/env python3
"""Passive independent recorder; never commands PX4 or exposes truth to mission code."""
import argparse
import json
import math
from pathlib import Path
import signal
import threading
import time
from score_clock import TelemetryClock


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',required=True); ap.add_argument('--wall-timeout',type=float,default=1200)
    ap.add_argument('--sim-timeout',type=float,default=180)
    args=ap.parse_args()
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from px4_msgs.msg import VehicleStatus, VehicleLandDetected
    from gz.transport13 import Node
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.contacts_pb2 import Contacts
    rclpy.init(); ros=rclpy.create_node('independent_course_recorder')
    qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
    latest={}; lock=threading.Lock(); data={'samples':[],'contacts':[],'contact_topics_verified':False,
        'recorder':'passive independent observer; truth never supplied to mission', 'stop_reason':None}
    status_clock,land_clock=TelemetryClock(),TelemetryClock()
    def state(m):
        if 'pose' not in latest: return
        status_clock.observe(m.timestamp,latest['pose'][0],time.monotonic())
        latest['armed']=m.arming_state==VehicleStatus.ARMING_STATE_ARMED
    def land(m):
        if 'pose' not in latest: return
        land_clock.observe(m.timestamp,latest['pose'][0],time.monotonic())
        latest['landed']=bool(m.landed)
    subs=[]
    for topic in ('/fmu/out/vehicle_status','/fmu/out/vehicle_status_v1'):
        subs.append(ros.create_subscription(VehicleStatus,topic,state,qos))
    for topic in ('/fmu/out/vehicle_land_detected','/fmu/out/vehicle_land_detected_v1'):
        subs.append(ros.create_subscription(VehicleLandDetected,topic,land,qos))
    gz=Node()
    def odom(m):
        p=m.pose.position; q=m.pose.orientation; t=m.header.stamp.sec+m.header.stamp.nsec*1e-9
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        with lock: latest['pose']=(t,p.x,p.y,p.z,yaw)
    def contacts(m):
        t=m.header.stamp.sec+m.header.stamp.nsec*1e-9
        with lock:
            for c in m.contact:
                a,b=c.collision1.name,c.collision2.name
                other=b if 'dh_quad' in a else a
                if other.startswith('ground') or 'dh_quad' in other: continue
                data['contacts'].append({'t':t,'name':other})
    gz.subscribe(Odometry,'/model/dh_quad_0/odometry',odom)
    contact_topics=['/world/dronehacks/model/dh_quad_0/link/base_link/sensor/body_contact/contact']+[f'/world/dronehacks/model/dh_quad_0/link/rotor_{i}/sensor/prop{i}_contact/contact' for i in range(4)]
    contact_subscriptions={topic:bool(gz.subscribe(Contacts,topic,contacts)) for topic in contact_topics}
    data['contact_subscriptions']=contact_subscriptions
    import subprocess
    publisher_info={}
    for topic in contact_topics:
        result=subprocess.run(['gz','topic','-i','-t',topic],capture_output=True,text=True,timeout=15)
        publisher_info[topic]=result.stdout+result.stderr
    data['contact_publisher_info']=publisher_info
    data['contact_topics_verified']=all(contact_subscriptions.values()) and all('Publishers [' in info and 'gz.msgs.Contacts' in info
                                        for info in publisher_info.values())
    data['telemetry_age_clock']='simulation seconds; PX4 source timestamp directly compared with verified Gazebo epoch'
    data['telemetry_age_limit_sim_s']=2.5
    data['wall_stall_limit_s']=60
    stop=False
    def interrupted(*_):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGTERM,interrupted); signal.signal(signal.SIGINT,interrupted)
    output=Path(args.output); output.parent.mkdir(parents=True,exist_ok=True)
    def save():
        tmp=output.with_suffix('.partial'); tmp.write_text(json.dumps(data,indent=2)+'\n'); tmp.replace(output)
    begin=time.monotonic(); saved=begin; last_t=None; arm_t=None; finish_since=None
    try:
        while not stop and time.monotonic()-begin<args.wall_timeout:
            rclpy.spin_once(ros,timeout_sec=.02)
            now=time.monotonic()
            with lock: pose=latest.get('pose')
            if pose and pose[0]!=last_t and 'armed' in latest and 'landed' in latest:
                t,x,y,z,yaw=pose; last_t=t
                age=max(status_clock.age(t),land_clock.age(t))
                if not data['samples'] and age>2.5:
                    data.setdefault('startup_stale_packets',[]).append(dict(t=t,status_age=age))
                    continue
                data['samples'].append(dict(t=t,x=x,y=y,z=z,yaw=yaw,armed=latest['armed'],landed=latest['landed'],status_age=age,status_source_us=status_clock.stamp*1e6 if status_clock.stamp is not None else None,land_source_us=land_clock.stamp*1e6 if land_clock.stamp is not None else None))
                if latest['armed'] and arm_t is None: arm_t=t
                done=arm_t is not None and not latest['armed'] and latest['landed'] and age<2.5
                finish_since=(finish_since if finish_since is not None else t) if done else None
                if finish_since is not None and t-finish_since>=3:
                    data['stop_reason']='stable landed and disarmed'; break
                if arm_t is not None and t-arm_t>args.sim_timeout:
                    data['stop_reason']='simulation deadline'; break
            if arm_t is not None and (status_clock.stalled(now) or land_clock.stalled(now)):
                data['stop_reason']='PX4 source stalled for 60 wall seconds'; break
            if now-saved>1: save(); saved=now
        if data['stop_reason'] is None: data['stop_reason']='interrupted' if stop else 'wall deadline'
    finally:
        save()
        gz.unsubscribe('/model/dh_quad_0/odometry')
        for topic in contact_topics: gz.unsubscribe(topic)
        ros.destroy_node(); rclpy.shutdown()
    print(json.dumps({'output':str(output),'samples':len(data['samples']),'stop_reason':data['stop_reason']}),flush=True)


if __name__=='__main__': main()
