#!/usr/bin/env python3
"""Passive split-platform recorder; truth stays outside companion navigation."""
import argparse,json,signal,time,math
from pathlib import Path
from bisect import bisect_right

def merge(truth, telemetry):
    rows=[json.loads(s) for s in Path(telemetry).read_text().splitlines()]
    states={k:sorted((r for r in rows if r['kind']==k),key=lambda r:r['timestamp']) for k in ('status','land')}
    stamps={k:[row['timestamp']/1e6 for row in group] for k,group in states.items()}
    d=json.loads(Path(truth).read_text()); samples=[]
    for p in d['samples']:
        indices={k:bisect_right(series,p['t'])-1 for k,series in stamps.items()}
        if min(indices.values())<0:continue
        pick={k:states[k][idx] for k,idx in indices.items()}
        age=max(p['t']-r['timestamp']/1e6 for r in pick.values())
        samples.append(dict(p,armed=pick['status']['armed'],landed=pick['land']['landed'],status_age=age,status_source_us=pick['status']['timestamp'],land_source_us=pick['land']['timestamp']))
    d['samples']=samples;d['telemetry_merge']='latest source stamp <= Gazebo truth stamp; no interpolation or invented fix';return d

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('role',choices=['truth','telemetry','merge']);ap.add_argument('--output',required=True);ap.add_argument('--truth');ap.add_argument('--telemetry');ap.add_argument('--wall-timeout',type=float,default=660);a=ap.parse_args()
    if a.role=='merge':Path(a.output).write_text(json.dumps(merge(a.truth,a.telemetry),indent=2)+'\n');return
    import os
    if os.environ.get('ROS_DOMAIN_ID')!='219':raise SystemExit('requires isolated simulation domain219')
    stop=False
    def done(*_):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGTERM,done);signal.signal(signal.SIGINT,done)
    begin=time.monotonic();out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True)
    if a.role=='telemetry':
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.qos import qos_profile_sensor_data
        from px4_msgs.msg import VehicleStatus,VehicleLandDetected,VehicleOdometry
        from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
        from nav_msgs.msg import Odometry
        rclpy.init();n=rclpy.create_node('split_independent_telemetry_recorder');subscriptions=[]
        with out.open('x',buffering=1) as f:
            def cb(k,m):f.write(json.dumps(dict(kind=k,timestamp=int(m.timestamp),**({'armed':m.arming_state==2,'nav_state':int(m.nav_state),'failsafe':bool(m.failsafe)} if k=='status' else {'landed':bool(m.landed)})))+'\n')
            for k,typ,topics in [('status',VehicleStatus,['/fmu/out/vehicle_status_v1','/fmu/out/vehicle_status']),('land',VehicleLandDetected,['/fmu/out/vehicle_land_detected','/fmu/out/vehicle_land_detected_v1'])]:
                for topic in topics:subscriptions.append(n.create_subscription(typ,topic,lambda m,k=k:cb(k,m),qos_profile_sensor_data))
            def odometry(kind,m):
                h=m.header.stamp
                p=m.pose.pose.position
                q=m.pose.pose.orientation
                f.write(json.dumps(dict(kind=kind,timestamp=h.sec*1000000+h.nanosec//1000,
                                        stamp_ns=h.sec*10**9+h.nanosec,
                                        position=[p.x,p.y,p.z],orientation_xyzw=[q.x,q.y,q.z,q.w],
                                        frame_id=m.header.frame_id,child_frame_id=m.child_frame_id,
                                        pose_covariance_6x6=list(m.pose.covariance),
                                        variance_xyz=[m.pose.covariance[i] for i in (0,7,14)]))+'\n')
            def bridge(m):
                f.write(json.dumps(dict(kind='bridge', timestamp=int(m.timestamp),
                            sample_us=int(m.timestamp_sample),reset_counter=int(m.reset_counter),
                            pose_frame=int(m.pose_frame)))+'\n')
            def tracking(m):
                stamp=m.header.stamp
                f.write(json.dumps(dict(kind='nvidia_status', timestamp=stamp.sec*10**6+stamp.nanosec//1000,
                                        vo_state=int(m.vo_state),frame=m.header.frame_id))+'\n')
            subscriptions.append(n.create_subscription(VehicleOdometry,'/fmu/in/vehicle_visual_odometry',bridge,qos_profile_sensor_data))
            subscriptions.append(n.create_subscription(VisualSlamStatus,'/visual_slam/status',tracking,qos_profile_sensor_data))
            for kind,topic in [('vision_raw','/visual_slam/tracking/odometry'),('vision_admitted','/mapping/validated_odometry')]:
                subscriptions.append(n.create_subscription(Odometry,topic,lambda m,k=kind:odometry(k,m),qos_profile_sensor_data))
            try:
                while not stop and time.monotonic()-begin<a.wall_timeout:
                    rclpy.spin_once(n,timeout_sec=.1)
            except ExternalShutdownException:
                if rclpy.ok():
                    raise
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return
    from gz.transport13 import Node
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.contacts_pb2 import Contacts
    import subprocess,threading
    lock=threading.Lock();gz=Node();d={'samples':[],'contacts':[],'contact_topics_verified':False,'recorder':'native Gazebo independent observer; separate Jetson PX4 telemetry merge'}
    def odom(m):
        p=m.pose.position;q=m.pose.orientation
        sample = dict(t=m.header.stamp.sec+m.header.stamp.nsec*1e-9,
                      x=p.x,y=p.y,z=p.z,orientation_xyzw=[q.x,q.y,q.z,q.w],
                      yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        with lock:
            d['samples'].append(sample)
    def contact(m):
        with lock:
            for c in m.contact:
                x,y=c.collision1.name,c.collision2.name;other=y if 'dh_quad' in x else x
                if other.startswith('ground') or 'dh_quad' in other:continue
                d['contacts'].append(dict(t=m.header.stamp.sec+m.header.stamp.nsec*1e-9,name=other))
    gz.subscribe(Odometry,'/model/dh_quad_0/odometry',odom)
    topics=['/world/dronehacks/model/dh_quad_0/link/base_link/sensor/body_contact/contact']+[f'/world/dronehacks/model/dh_quad_0/link/rotor_{i}/sensor/prop{i}_contact/contact' for i in range(4)]
    checks={}
    for t in topics:
        sub=gz.subscribe(Contacts,t,contact);r=subprocess.run(['gz','topic','-i','-t',t],capture_output=True,text=True,timeout=15);checks[t]=dict(subscribed=bool(sub),publisher_info=r.stdout+r.stderr)
    d['contact_publisher_info']=checks;d['contact_topics_verified']=all(v['subscribed'] and 'Publishers [' in v['publisher_info'] and 'gz.msgs.Contacts' in v['publisher_info'] for v in checks.values())
    def save():
        with lock:text=json.dumps(d,indent=2)+'\n'
        tmp=out.with_suffix('.partial');tmp.write_text(text);tmp.replace(out)
    while not stop and time.monotonic()-begin<a.wall_timeout:save();time.sleep(.5)
    save()
if __name__=='__main__':main()
