#!/usr/bin/env python3
"""Record a bounded, passive 35-second end-to-end observation as JSON."""
import json
import time
from collections import defaultdict
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from px4_msgs.msg import VehicleOdometry, VehicleStatus, VehicleLocalPosition, EstimatorStatusFlags
from std_msgs.msg import String

rclpy.init()
n=Node('handheld_slam_passive_verification')
counts=defaultdict(int)
last={}
samples={}
positions=[]
sample_ages=[]
subscriptions=[]
started=time.monotonic()

def callback(key,msg):
    counts[key]+=1
    last[key]=time.monotonic()
    if key in ('tracking_odometry','validated_odometry'):
        p=msg.pose.pose.position
        sample=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        age=n.get_clock().now().nanoseconds*1e-9-sample
        samples[key]={'position':[p.x,p.y,p.z],'age_ms':round(age*1000,2)}
        if key=='tracking_odometry':
            positions.append([p.x,p.y,p.z])
        if key=='validated_odometry':
            sample_ages.append(age*1000)
    elif key=='px4_visual_odometry':
        samples[key]={'position':[float(x) for x in msg.position],'pose_frame':msg.pose_frame,'reset_counter':msg.reset_counter}
    elif key=='px4_status':
        samples[key]={'armed':msg.arming_state==2,'nav_state':msg.nav_state,'failsafe':msg.failsafe}
    elif key=='px4_local_position':
        samples[key]={'position':[msg.x,msg.y,msg.z],'xy_valid':msg.xy_valid,'z_valid':msg.z_valid,'eph':msg.eph,'epv':msg.epv}
    elif key=='landmarks':
        samples[key]={'count':msg.width*msg.height,'frame':msg.header.frame_id}
    elif key=='gate':
        samples[key]=json.loads(msg.data)
    elif key=='ekf':
        samples[key]={'vision_xy':msg.cs_ev_pos,'vision_yaw':msg.cs_ev_yaw,'vision_height':msg.cs_ev_hgt,'vision_velocity':msg.cs_ev_vel}

topics=[('tracking_odometry',Odometry,'/visual_slam/tracking/odometry'),('validated_odometry',Odometry,'/mapping/validated_odometry'),('px4_visual_odometry',VehicleOdometry,'/fmu/in/vehicle_visual_odometry'),('px4_status',VehicleStatus,'/fmu/out/vehicle_status_v1'),('px4_local_position',VehicleLocalPosition,'/fmu/out/vehicle_local_position'),('landmarks',PointCloud2,'/visual_slam/vis/landmarks_cloud'),('gate',String,'/mapping/gate_status')]
for key,typ,topic in topics:
    subscriptions.append(n.create_subscription(typ,topic,lambda m,k=key:callback(k,m),qos_profile_sensor_data))
subscriptions.append(n.create_subscription(EstimatorStatusFlags,'/fmu/out/estimator_status_flags',lambda m:callback('ekf',m),qos_profile_sensor_data))
while time.monotonic()-started<35:
    rclpy.spin_once(n,timeout_sec=0.1)
duration=time.monotonic()-started
result={'duration_s':round(duration,2),'rates_hz':{k:round(v/duration,2) for k,v in counts.items()},'last_age_s':{k:round(time.monotonic()-v,3) for k,v in last.items()},'samples':samples,'position_span_m':[round(max(p[i] for p in positions)-min(p[i] for p in positions),3) for i in range(3)] if positions else None,'visual_odometry_px4_subscribers':n.count_subscribers('/fmu/in/vehicle_visual_odometry')}
if sample_ages:
    ages=sorted(sample_ages)
    result['validated_age_ms']={name:round(ages[min(len(ages)-1,int(len(ages)*q))],2) for name,q in [('p50',0.5),('p95',0.95),('max',1)]}
print(json.dumps(result,indent=2))
n.destroy_node()
rclpy.try_shutdown()
