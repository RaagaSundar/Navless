#!/usr/bin/env python3
"""OpenCV stereo-only measured odometry profile. No autopilot commands.

Consumes exact synchronized rectified ROS stereo and CameraInfo, plus actual
calibrated TF. Emits a separately named raw estimator stream; never resets its
origin automatically after lost tracking. Run isolated offline/domain test first.
"""
import json
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import OdomInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import message_filters
try:
    from .replay_core import StereoVO
    from .vo_core import camera_calibration, transform_matrix, camera_to_body, quaternion_xyzw, StartupReadiness
except ImportError:
    from replay_core import StereoVO
    from vo_core import camera_calibration, transform_matrix, camera_to_body, quaternion_xyzw, StartupReadiness


class StereoNode(Node):
    def __init__(self):
        super().__init__('course_opencv_stereo_vo')
        self.declare_parameter('opencv_threads',1)
        cv2.setNumThreads(int(self.get_parameter('opencv_threads').value))
        self.cv=CvBridge();self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.image_group=MutuallyExclusiveCallbackGroup()
        self.declare_parameter('recovery_horizon_s',0.0)
        self.recovery_horizon=float(self.get_parameter('recovery_horizon_s').value)
        self.estimator=None;self.calibration=None;self.mount=None
        self.startup=StartupReadiness()
        self.counts={'pairs':0,'accepted':0,'rejected':0}
        self.pub=self.create_publisher(Odometry,'/local_course/opencv/odom',10)
        self.health=self.create_publisher(OdomInfo,'/local_course/opencv/odom_info',10)
        self.diag=self.create_publisher(String,'/local_course/opencv/status',10)
        latest_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
        self.inputs=[message_filters.Subscriber(self,typ,topic,qos_profile=latest_qos,callback_group=self.image_group)
            for typ,topic in [(Image,'/oak/left/image_raw'),(Image,'/oak/right/image_raw'),
                              (CameraInfo,'/oak/left/camera_info'),(CameraInfo,'/oak/right/camera_info')]]
        self.sync=message_filters.TimeSynchronizer(self.inputs,10)
        self.sync.registerCallback(self.pair)

    def report(self,header,valid,reason,extra=None):
        info=OdomInfo();info.header=header;info.lost=not valid
        self.health.publish(info)
        msg=String();msg.data=json.dumps(dict(self.counts,valid=valid,reason=reason,
                            covariance_model='four-view residual Hessian, n*sum increment covariance cross-correlation bound; excludes calibration/systematic error',
                            **(extra or {})))
        self.diag.publish(msg)

    def pair(self,left,right,li,ri):
        self.counts['pairs']+=1
        began=time.perf_counter();result=None
        try:
            stamp=Time.from_msg(left.header.stamp)
            now=self.get_clock().now().nanoseconds
            age=(now-stamp.nanoseconds)*1e-9
            if not -.05<=age<=.25:
                raise ValueError('stale_camera_pair')
            if left.header.frame_id!=li.header.frame_id or right.header.frame_id!=ri.header.frame_id:
                raise ValueError('image_calibration_frame')
            if (left.width,left.height)!=(li.width,li.height) or (right.width,right.height)!=(ri.width,ri.height):
                raise ValueError('image_calibration_dimensions')
            k,baseline=camera_calibration(li,ri)
            tf=self.tf.lookup_transform('base_link',left.header.frame_id,stamp).transform
            mount=transform_matrix([tf.translation.x,tf.translation.y,tf.translation.z],
                                  [tf.rotation.x,tf.rotation.y,tf.rotation.z,tf.rotation.w])
            stereo=self.tf.lookup_transform(left.header.frame_id,right.header.frame_id,stamp).transform
            rig=transform_matrix([stereo.translation.x,stereo.translation.y,stereo.translation.z],
                                [stereo.rotation.x,stereo.rotation.y,stereo.rotation.z,stereo.rotation.w])
            if not np.allclose(rig[:3,:3],np.eye(3),atol=1e-5) or not np.allclose(rig[:3,3],[baseline,0,0],atol=1e-4):
                raise ValueError('stereo_projection_tf_disagree')
            calibration=np.r_[k.ravel(),baseline]
            if self.estimator is None:
                if not self.startup.observe(stamp.nanoseconds*1e-9,now*1e-9):
                    self.counts['rejected']+=1
                    self.report(left.header,False,'waiting_consistent_stereo_clock',{'startup_pairs':self.startup.count})
                    return
                self.calibration=calibration;self.mount=mount
                self.estimator=StereoVO(k,baseline_m=baseline,max_dt_s=.25,recovery_horizon_s=self.recovery_horizon)
            elif not np.allclose(self.calibration,calibration,rtol=0,atol=1e-8) or not np.allclose(self.mount,mount,atol=1e-8):
                raise ValueError('calibration_changed_origin_restart_required')
            a=self.cv.imgmsg_to_cv2(left,desired_encoding='mono8')
            b=self.cv.imgmsg_to_cv2(right,desired_encoding='mono8')
            result=self.estimator.process(stamp.nanoseconds*1e-9,a,b)
            if not result['valid']:
                self.counts['rejected']+=1
                self.report(left.header,False,result['reason'],{'estimator':result.get('counters',{})})
                return
            age_after=(self.get_clock().now().nanoseconds-stamp.nanoseconds)*1e-9
            if not -.05<=age_after<=.25:
                raise ValueError('stale_after_stereo_processing')
            pose,cov=camera_to_body(result['T_origin_camera'],result['covariance_6x6'],self.mount)
            out=Odometry();out.header.stamp=left.header.stamp;out.header.frame_id='odom';out.child_frame_id='base_link'
            out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=map(float,pose[:3,3])
            q=quaternion_xyzw(pose[:3,:3])
            out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=map(float,q)
            out.pose.covariance=[float(v) for v in cov.ravel()]
            # Twist unavailable: PX4 bridge publishes no velocity for this profile.
            out.twist.covariance=[float('nan')]*36
            for vector in (out.twist.twist.linear,out.twist.twist.angular):
                vector.x=vector.y=vector.z=float('nan')
            self.pub.publish(out)
            self.counts['accepted']+=1
            self.report(left.header,True,'measured_stereo',{'estimator':result.get('counters',{}), 'elapsed_s':result.get('elapsed_s'), 'acquisition_age_s':age_after})
        except Exception as exc:
            if self.estimator is None:
                self.startup=StartupReadiness()
            self.counts['rejected']+=1
            self.report(left.header,False,str(exc),{'callback_elapsed_s':time.perf_counter()-began,
                'estimator_elapsed_s':None if result is None else result.get('elapsed_s')})


def main():
    rclpy.init();node=StereoNode();executor=MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:executor.spin()
    finally:executor.shutdown();node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
