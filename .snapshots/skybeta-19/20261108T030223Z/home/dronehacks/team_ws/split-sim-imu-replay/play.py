"""Replay observed sensor packets by acquisition stamp and save actual tracker outputs."""
import collections
import json
import os
import sqlite3
import sys
import time
import zlib
import rclpy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, Image, CameraInfo
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from rclpy.qos import qos_profile_sensor_data

if os.environ.get("ROS_DOMAIN_ID")!="218":
    raise SystemExit("isolated replay requires domain218")
db = sqlite3.connect(f"file:{sys.argv[1]}?mode=ro", uri=True)
rclpy.init()
node=rclpy.create_node("ordered_sensor_replay")
clock=node.create_publisher(Clock,"/clock",10)
imu=node.create_publisher(Imu,"/replay/imu",100)
eyes=("left","right")
images={e:node.create_publisher(Image,f"/replay/{e}/image",10) for e in eyes}
infos={e:node.create_publisher(CameraInfo,f"/replay/{e}/camera_info",10) for e in eyes}
poses=[]
status=collections.Counter()
def odom(m):
    s=m.header.stamp
    p=m.pose.pose.position
    q=m.pose.pose.orientation
    poses.append([s.sec+s.nanosec/1e9,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
def update_status(m):
    status[m.vo_state]+=1
node.create_subscription(Odometry,"/visual_slam/tracking/odometry",odom,qos_profile_sensor_data)
node.create_subscription(VisualSlamStatus,"/visual_slam/status",update_status,qos_profile_sensor_data)
deadline=time.monotonic()+35
while not all(p.get_subscription_count() for p in [imu,*infos.values(),*images.values()]):
    if time.monotonic()>deadline:
        raise RuntimeError("no complete set of tracker input subscriptions")
    rclpy.spin_once(node,timeout_sec=.05)
def stamp(s,t):
    s.sec,s.nanosec=divmod(t,10**9)
info={}
for e in eyes:
    val=db.execute("SELECT DISTINCT frame,data FROM samples WHERE kind=?",(e+"_info",)).fetchall()
    if len(val)!=1:
        raise ValueError("missing or changing CameraInfo")
    frame,raw=val[0]
    v=json.loads(raw)
    m=CameraInfo()
    m.header.frame_id=frame
    m.width,m.height=v["width"],v["height"]
    m.distortion_model=("plumb_bob","rational_polynomial","equidistant")[v["model"]]
    m.k,m.p,m.r,m.d=v["k"],v["p"],v["r"],v["d"]
    info[e]=m
speed=float(os.getenv("REPLAY_RTF",".75"))
# Each received packet keeps its native timestamp. Only camera pairs present in
# both streams can initialize stereo. No value is synthesized for missing data.
first=None
wall=time.monotonic()
counts=collections.Counter()
for t,kind,frame,data in db.execute("SELECT * FROM samples WHERE kind IN ('imu','left','right') ORDER BY stamp,kind"):
    if first is None:
        first=t
    due=wall+(t-first)/1e9/speed
    while time.monotonic()<due:
        rclpy.spin_once(node,timeout_sec=min(.005,max(0,due-time.monotonic())))
    c=Clock()
    stamp(c.clock,t)
    clock.publish(c)
    if kind=="imu":
        if frame!="dh_quad_0::base_link::imu_sensor":
            raise ValueError("unexpected body IMU frame")
        m=Imu()
        m.header.frame_id="base_link"
        stamp(m.header.stamp,t)
        m.orientation_covariance[0]=-1
        w=m.angular_velocity
        a=m.linear_acceleration
        w.x,w.y,w.z,a.x,a.y,a.z=json.loads(data)
        imu.publish(m)
    else:
        m=Image()
        m.header.frame_id=frame
        stamp(m.header.stamp,t)
        m.width,m.height,m.step,m.encoding=640,400,640,"mono8"
        m.data=zlib.decompress(data)
        ci=info[kind]
        if ci.header.frame_id!=frame:
            raise ValueError("image/calibration frame mismatch")
        stamp(ci.header.stamp,t)
        infos[kind].publish(ci)
        images[kind].publish(m)
    counts[kind]+=1
    rclpy.spin_once(node,timeout_sec=0.)
end=time.monotonic()+1.
while time.monotonic()<end:
    rclpy.spin_once(node,timeout_sec=.02)
with open(sys.argv[2],"w") as stream:
    json.dump(dict(source_counts=counts,status_counts=status,poses=poses),stream)
print("source",dict(counts),"status",dict(status),"poses",len(poses),flush=True)
node.destroy_node()
rclpy.shutdown()
