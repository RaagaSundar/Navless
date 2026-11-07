"""Passively inspect full-rate body IMU and camera acquisition timing on split SITL."""
import bisect
import json
import math
import os
import statistics
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu

if os.environ.get('ROS_DOMAIN_ID') != '219':
    raise SystemExit('requires simulation domain 219')
rclpy.init()
node=rclpy.create_node('split_inertial_timing_observer')
stamps={k:[] for k in ('imu','left','right','vision')}
gravity=[]
gyro=[]
def cb(key,msg):
    stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
    stamps[key].append(stamp)
    if key=='imu':
        assert msg.header.frame_id=='base_link' and msg.orientation_covariance[0]==-1
        v=msg.linear_acceleration
        w=msg.angular_velocity
        gravity.append((v.x,v.y,v.z));gyro.append(math.hypot(w.x,w.y,w.z))

subs=[]
for key,typ,topic in [('imu',Imu,'/split_camera/body_imu'),('left',Image,'/oak/left/image_raw'),
                      ('right',Image,'/oak/right/image_raw'),('vision',Odometry,'/mapping/validated_odometry')]:
    qos = QoSProfile(depth=256,reliability=ReliabilityPolicy.RELIABLE) if key=='imu' else qos_profile_sensor_data
    subs.append(node.create_subscription(typ,topic,lambda m,k=key:cb(k,m),qos))
start=time.monotonic()
while time.monotonic()-start<25 and (len(stamps['imu'])<1500 or len(stamps['vision'])<70):
    rclpy.spin_once(node,timeout_sec=.05)
imu=stamps['imu']
pair=sorted(set(stamps['left']).intersection(stamps['right'],stamps['vision']))
gaps=[(b-a)/1e6 for a,b in zip(imu,imu[1:])]
brackets=[]
for t in pair:
    j=bisect.bisect_right(imu,t)
    if 0<j<len(imu):
        brackets.append(max(t-imu[j-1],imu[j]-t)/1e6)
out=dict(counts={k:len(v) for k,v in stamps.items()},duplicates=len(imu)-len(set(imu)),
         imu_max_gap_ms=max(gaps,default=None), imu_min_gap_ms=min(gaps,default=None),
         stereo_vision_imu_brackets=len(brackets),max_bracket_side_ms=max(brackets,default=None),
         stationary_accel_median_flu=[statistics.median(c) for c in zip(*gravity)],
         stationary_gyro_norm_median=statistics.median(gyro) if gyro else None,
         tracker_mode='source observer only; inference mode is verified separately from running NVIDIA parameters')
print(json.dumps(out,indent=2))
node.destroy_node();rclpy.shutdown()
if not gaps or min(gaps)!=4 or max(gaps)!=4 or len(brackets)<50 or max(brackets)>4:
    raise SystemExit('full-rate synchronized IMU was not qualified')
