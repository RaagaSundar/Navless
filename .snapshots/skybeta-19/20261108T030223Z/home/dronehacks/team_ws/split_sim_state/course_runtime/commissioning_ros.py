"""Optional finite yaw commission using native ROS images/IMU and existing PX4 adapter.

No simulator truth/API. The worker is read-only; the existing Interface remains
sole command publisher and retains all source/authority/height/recording gates.
"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import hashlib,json,math,threading,time
from pathlib import Path
import numpy as np
from active_observation_yaw_candidate import FiniteYawObservation,ObservationDecision
from anticipatory_tangent_yaw_candidate import wrap
from visual_quality import VisualQuality


def mono_array(msg):
    if msg.encoding not in ('mono8','8UC1') or msg.is_bigendian or msg.step<msg.width:
        raise ValueError('native mono8 calibration contract required')
    raw=np.frombuffer(bytes(msg.data),np.uint8)
    if len(raw)!=msg.height*msg.step:raise ValueError('image buffer size mismatch')
    return raw.reshape(msg.height,msg.step)[:,:msg.width].copy()


def transform_matrix(transform):
    q=transform.rotation;w,x,y,z=q.w,q.x,q.y,q.z
    if not all(math.isfinite(v) for v in [w,x,y,z]) or abs(w*w+x*x+y*y+z*z-1)>.005:raise ValueError('invalid TF quaternion')
    T=np.eye(4);T[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]]
    T[:3,3]=[transform.translation.x,transform.translation.y,transform.translation.z]
    return T


def commissioned_course_type(base):
    class CommissionCourse(base):
        def start_route(self):
            hover=tuple(self.target)
            super().start_route() # preserves original route-start offset/safety checks
            if self.state=='ROUTE':self.goto('OBSERVE',hover)
        def _state_observe(self):
            if self.vio_ok():self.send(*self.target)
    return CommissionCourse


class CommissioningRosBridge:
    def __init__(self,node,args):
        import cv2
        cv2.setNumThreads(1)
        from sensor_msgs.msg import Image,CameraInfo,Imu,Range
        from rclpy.qos import qos_profile_sensor_data
        from tf2_ros import Buffer,TransformListener
        self.node=node;self.args=args;self.lock=threading.Lock();self.images={'left':{},'right':{}};self.infos={};self.imu=deque(maxlen=5000);self.last_imu=None;self.imu_fault_until=-math.inf;self.range=None
        self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='commission-stereo-quality');self.future=None;self.last_submit=-math.inf;self.last_pair=-1;self.quality={'state':'unknown','stamp_s':None,'reasons':['waiting for actual stereo/IMU']};self.probe=None;self.calibration_fault=False;self.info_signature=None;self.tf_signature=None
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,node,spin_thread=False)
        self.config=json.loads(Path(__file__).with_name('active-observation-config.json').read_text());self.policy=FiniteYawObservation(self.config)
        self.log=Path(args.recorder_token).with_name(args.run_id+'-active-observation.jsonl').open('x') if args.recorder_token and args.run_id else None
        self.last_decision=None;self.acquiring_since=None;self.last_packet=None
        for side in ('left','right'):
            node.create_subscription(Image,f'/oak/{side}/image_raw',lambda m,s=side:self.image(s,m),qos_profile_sensor_data)
            node.create_subscription(CameraInfo,f'/oak/{side}/camera_info',lambda m,s=side:self.info(s,m),qos_profile_sensor_data)
        node.create_subscription(Imu,'/split_camera/body_imu',self.inertial,qos_profile_sensor_data)
        node.create_subscription(Range,'/split/downward_range',self.downrange,qos_profile_sensor_data)
        node.create_timer(.05,self.pump)
        self.record('commission_config',self.config)

    def record(self,kind,payload):
        if self.log:self.log.write(json.dumps(dict(kind=kind,receipt_sim_s=self.node.now(),payload=payload),allow_nan=False)+'\n');self.log.flush()

    def image(self,side,msg):
        stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        try:array=mono_array(msg)
        except ValueError as exc:self.quality={'state':'unknown','stamp_s':stamp/1e9,'reasons':[str(exc)]};return
        self.images[side][stamp]=(array,msg.header.frame_id,time.monotonic())
        for key in sorted(self.images[side])[:-16]:del self.images[side][key]

    def info(self,side,msg):
        item=dict(width=msg.width,height=msg.height,k=list(msg.k),p=list(msg.p),d=list(msg.d),frame=msg.header.frame_id)
        if self.info_signature is not None and side in self.infos and item!=self.infos[side]:self.calibration_fault=True
        self.infos[side]=item

    def inertial(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9;w=msg.angular_velocity;row=(stamp,w.x,w.y,w.z)
        if msg.header.frame_id!='base_link' or not all(math.isfinite(v) for v in row) or self.imu and stamp<=self.imu[-1][0]:
            self.imu_fault_until=self.node.now()+.5;return
        self.imu.append(row);self.last_imu=row

    def downrange(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        valid=(msg.header.frame_id=='downward_range_link' and math.isfinite(msg.range) and abs(msg.min_range-.02)<1e-5 and abs(msg.max_range-4)<1e-5 and msg.min_range<=msg.range<=msg.max_range)
        self.range=(stamp,bool(valid))

    def calibrate(self):
        from rclpy.time import Time
        if not all(s in self.infos for s in ('left','right')):return False
        left,right=self.infos['left'],self.infos['right'];K=np.array(left['k']).reshape(3,3)
        if left['width']!=right['width'] or left['height']!=right['height'] or not np.allclose(left['k'],right['k'],atol=1e-5) or any(abs(v)>1e-9 for v in left['d']+right['d']):return False
        try:
            T=transform_matrix(self.buffer.lookup_transform('base_link',left['frame'],Time()).transform)
            right_T=transform_matrix(self.buffer.lookup_transform('base_link',right['frame'],Time()).transform)
        except Exception:return False
        baseline=-right['p'][3]/K[0,0]
        delta=T[:3,:3].T@(right_T[:3,3]-T[:3,3])
        if not np.allclose(T[:3,:3],right_T[:3,:3],atol=1e-5) or not np.allclose(delta,[baseline,0,0],atol=.001):return False
        signature=hashlib.sha256(json.dumps([left,right,T.tolist()],sort_keys=True).encode()).hexdigest()
        if self.info_signature is not None and signature!=self.info_signature:self.calibration_fault=True;return False
        if self.probe is None:
            self.probe=VisualQuality(K,baseline,T);self.info_signature=signature
            self.record('actual_calibration',dict(left=left,right=right,body_from_left_optical=T.tolist(),baseline_m=baseline,sha256=signature))
        return not self.calibration_fault

    def pump(self):
        now=self.node.now()
        if self.future is not None and self.future.done():
            try:self.quality=self.future.result()
            except Exception as exc:self.quality={'state':'unknown','stamp_s':None,'reasons':['worker rejected:'+type(exc).__name__]}
            self.record('actual_sensor_quality',self.quality);self.future=None
        if self.future is not None or now-self.last_submit<.2:return
        try:
            if not self.calibrate():return
        except (ValueError,TypeError,ZeroDivisionError):self.calibration_fault=True;return
        common=sorted(set(self.images['left'])&set(self.images['right']))
        if len(common)<2 or common[-1]<=self.last_pair or not self.imu:return
        a,b=common[-2:];t0,t1=a/1e9,b/1e9
        if self.imu[-1][0]<t1 or not 0<=now-t1<=.25 or now<self.imu_fault_until:return
        if any(self.images[s][key][1]!=self.infos[s]['frame'] for s in ('left','right') for key in (a,b)):
            self.calibration_fault=True;return
        gyro=np.array([row for row in self.imu if t0-.012<=row[0]<=t1+.012])
        if len(gyro)<2:return
        images=[self.images[s][key][0] for key,s in [(a,'left'),(a,'right'),(b,'left'),(b,'right')]]
        self.last_packet=dict(previous_stamp_s=t0,stamp_s=t1,previous_left=images[0],previous_right=images[1],left=images[2],right=images[3],gyro_samples=gyro,K=self.probe.K.copy(),baseline_m=self.probe.baseline,body_from_left_optical=self.probe.extrinsic.copy(),received_wall_s=max(self.images[s][b][2] for s in ('left','right')))
        self.future=self.pool.submit(self.probe.evaluate,t0,t1,*images,gyro);self.last_submit=now;self.last_pair=b

    def latest_actual_pair(self):
        """Read-only actual adjacent-pair snapshot for another low-rate worker.

        Arrays/calibration are immutable snapshot references; caller must not
        edit them and must apply its own acquisition/wall-age/pose admission.
        No truth or fabricated images are produced.
        """
        if self.last_packet is None:return None
        return dict(self.last_packet,calibration_valid=not self.calibration_fault,imu_fault_active=self.node.now()<self.imu_fault_until)

    def ready(self,now):
        common=sorted(set(self.images['left'])&set(self.images['right']))
        return bool(self.probe is not None and not self.calibration_fault and common and 0<=now-common[-1]/1e9<=.25 and self.last_imu and 0<=now-self.last_imu[0]<=.25 and now>=self.imu_fault_until and self.range and self.range[1] and 0<=now-self.range[0]<=.25)

    def control(self,now,p,status,mission,interface):
        from navless_nav.route import odom_to_course
        from frames import ned_to_launch_odom
        from px4_msgs.msg import VehicleStatus
        if mission.state!='OBSERVE':return None
        launch=ned_to_launch_odom(interface.launch_datum.from_px4((p.x,p.y,p.z)),interface.control_bearing)
        origin=json.loads(Path(self.args.route).read_text())['launch_origin_enu'];position=odom_to_course(launch,origin,self.args.start_bearing,self.args.start_height)
        measured=wrap(p.heading-math.radians(interface.control_bearing-self.args.start_bearing));speed=math.sqrt(p.vx*p.vx+p.vy*p.vy+p.vz*p.vz)
        current_ready=self.ready(now);gyro_stamp=-1. if self.last_imu is None else self.last_imu[0]
        health=dict(px4_valid=bool(p.xy_valid and p.z_valid and p.v_xy_valid and p.v_z_valid),slam_valid=bool(interface.nav_state=='ready' and 0<=now-interface.source.get('odom',-1)<=.5),gyro_valid=bool(self.last_imu and now>=self.imu_fault_until and 0<=now-gyro_stamp<=.25),integrity_valid=True,authority_valid=bool(interface.phase=='ACTIVE' and status.arming_state==VehicleStatus.ARMING_STATE_ARMED and status.nav_state==VehicleStatus.NAVIGATION_STATE_OFFBOARD and not status.failsafe),range_valid=bool(self.range and self.range[1]),pose_stamp_s=p.timestamp_sample/1e6,yaw_stamp_s=p.timestamp_sample/1e6,gyro_stamp_s=gyro_stamp,status_stamp_s=status.timestamp/1e6,range_stamp_s=-1. if self.range is None else self.range[0],speed_m_s=speed,gyro_yaw_rate_deg_s=0. if self.last_imu is None else math.degrees(self.last_imu[3]),known_bad_sector=False)
        if interface.nav_state=='coast':
            self.policy.last_time=now;self.policy.qualified=[];self.policy.settled_since=None;self.policy.stable_since=None
            if self.policy.state=='OBSERVE_VIEW':self.policy.state='SETTLE'
            return ObservationDecision(measured,0.,'hold','NATIVE_COAST','existing bounded native brake; no yaw advancement')
        if not current_ready:return ObservationDecision(measured,0.,'auto_land_requested','SENSOR_STOP','observation sensor stream/calibration unavailable; existing native guards passed, request land')
        if self.acquiring_since is None:self.acquiring_since=now
        center=self.config['launch_center_enu_xy'];offset=math.hypot(position[0]-center[0],position[1]-center[1])
        # Keep original takeoff target while measured hover converges, no route.
        if self.policy.started is None and (speed>=.03 or offset>self.config['hover_radius_m'] or not self.config['height_band_m'][0]<=position[2]<=self.config['height_band_m'][1]):
            action='auto_land_requested' if now-self.acquiring_since>10 or offset>.25 else 'hold'
            result=ObservationDecision(measured,0.,action,'ACQUIRE_HOVER','hold original takeoff setpoint until actual stationary hover;10s acquisition bound')
        else:result=self.policy.update(now,position,measured,health,self.quality,None)
        payload=dict(position_enu=list(position),measured_compass_yaw_rad=measured,health=health,quality_state=self.quality.get('state'),quality_stamp_s=self.quality.get('stamp_s'),yaw_command_rad=result.yaw_rad,action=result.action,state=result.state,reason=result.reason)
        self.record('commission_decision',payload)
        if self.last_decision!=(result.action,result.state,result.reason):self.node.get_logger().info('active observation: '+json.dumps(payload));self.last_decision=(result.action,result.state,result.reason)
        return result

    def close(self):
        self.pool.shutdown(wait=True,cancel_futures=True)
        if self.log:self.log.close()
