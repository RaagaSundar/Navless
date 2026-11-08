#!/usr/bin/env python3
"""Simulation IO only: typed, allowlisted Gazebo camera/clock to standard ROS.

No pose, odometry, PX4, commands, host devices or calibration generation. L8
bytes, source stamps, frame IDs and actual CameraInfo arrays are preserved.
The process-local pure-Python protobuf setting handles the image's old Gazebo
bindings without changing packages. pause_images is a simulation loss-test hook.
"""
import json
import os
from pathlib import Path
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION']='python'
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from rosgraph_msgs.msg import Clock
from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.camera_info_pb2 import CameraInfo as GzCameraInfo
from gz.msgs10.clock_pb2 import Clock as GzClock

STATE=Path('/root/gazebo_state')
MODELS={0:'plumb_bob',1:'rational_polynomial',2:'equidistant'}


def set_time(dst, src):
    if src.sec<0 or not 0<=src.nsec<1000000000:
        raise ValueError('invalid native stamp')
    dst.sec=src.sec
    dst.nanosec=src.nsec


def set_header(dst, src):
    set_time(dst.stamp,src.stamp)
    frames=[value for item in src.data if item.key=='frame_id' for value in item.value]
    if len(frames)!=1 or not frames[0]:
        raise ValueError('native header must contain one frame_id')
    dst.frame_id=frames[0]


class SensorBridge(Node):
    def __init__(self):
        super().__init__('simulation_native_sensor_bridge')
        self.gz=GzNode()
        self.counts={}
        self.calibrations={}
        self.rejected=0
        self.image_pubs={}
        self.info_pubs={}
        self.clock_pub=self.create_publisher(Clock,'/clock',10)
        self.gz.subscribe(GzClock,'/clock',self.clock)
        for eye in ('left','right'):
            base='/oak/'+eye
            self.image_pubs[eye]=[self.create_publisher(Image,base+'/'+name,qos_profile_sensor_data)
                                  for name in ('image_raw','image_mono')]
            self.info_pubs[eye]=self.create_publisher(CameraInfo,base+'/camera_info',10)
            self.gz.subscribe(GzImage,base+'/image_raw',lambda msg,e=eye:self.image(e,msg))
            self.gz.subscribe(GzCameraInfo,base+'/camera_info',lambda msg,e=eye:self.info(e,msg))
        self.create_timer(5.0,self.report)

    def count(self, key):
        self.counts[key]=self.counts.get(key,0)+1

    def reject(self, error):
        self.rejected+=1
        if self.rejected<=10:
            self.get_logger().error(str(error))

    def clock(self, src):
        try:
            dst=Clock()
            set_time(dst.clock,src.sim)
            self.clock_pub.publish(dst)
            self.count('clock')
        except Exception as error:
            self.reject(error)

    def image(self, eye, src):
        try:
            if (STATE/'pause_images').exists():
                self.count('test_dropped_'+eye)
                return
            if src.pixel_format_type!=1 or src.width<=0 or src.height<=0 or src.step<src.width or len(src.data)!=src.height*src.step:
                raise ValueError('expected native L_INT8 image with valid dimensions/step/data')
            dst=Image()
            set_header(dst.header,src.header)
            dst.width=src.width
            dst.height=src.height
            dst.step=src.step
            dst.encoding='mono8'
            dst.is_bigendian=0 # A single byte per pixel has no endian conversion.
            dst.data=src.data
            for pub in self.image_pubs[eye]:
                pub.publish(dst)
            self.count('image_'+eye)
        except Exception as error:
            self.reject(error)

    def info(self, eye, src):
        try:
            if len(src.intrinsics.k)!=9 or len(src.projection.p)!=12 or len(src.rectification_matrix)!=9:
                raise ValueError('native CameraInfo arrays have unexpected dimensions')
            dst=CameraInfo()
            set_header(dst.header,src.header)
            dst.width=src.width
            dst.height=src.height
            dst.distortion_model=MODELS[src.distortion.model]
            dst.d=list(src.distortion.k)
            dst.k=list(src.intrinsics.k)
            dst.p=list(src.projection.p)
            dst.r=list(src.rectification_matrix)
            self.info_pubs[eye].publish(dst)
            self.count('camera_info_'+eye)
            if eye not in self.calibrations:
                self.calibrations[eye]={'frame_id':dst.header.frame_id,'stamp_sec':src.header.stamp.sec,
                    'stamp_nsec':src.header.stamp.nsec,'width':dst.width,'height':dst.height,
                    'distortion_model':dst.distortion_model,'d':list(dst.d),'k':list(dst.k),'p':list(dst.p),'r':list(dst.r),
                    'conversion':'actual native source arrays copied exactly'}
                (STATE/'adapter-calibration.json').write_text(json.dumps(self.calibrations,indent=2))
        except Exception as error:
            self.reject(error)

    def report(self):
        self.get_logger().info(json.dumps({'counts':self.counts,'rejected':self.rejected}))


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID')!='219':
        raise SystemExit('Refusing outside simulation ROS domain219')
    rclpy.init()
    node=SensorBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
