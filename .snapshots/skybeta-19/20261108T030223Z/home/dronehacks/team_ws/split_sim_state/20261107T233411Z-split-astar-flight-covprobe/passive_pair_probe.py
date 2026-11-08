#!/usr/bin/env python3
"""SHA-pinned, receive-only paired covariance commissioning. No application publishers.
Pilot runs only in an already grounded/disarmed slot. No experiment is started
by importing this module. No GPU, transport reset, PX4 input or parameter API.
"""
import argparse,ast,base64,copy,hashlib,importlib.util,json,math,os,signal,time
from pathlib import Path

def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def load_pair(candidate, candidate_sha, original, original_sha):
    if digest(candidate)!=candidate_sha or digest(original)!=original_sha:
        raise ValueError('Pinned source SHA mismatch; stop and review actual installed files')
    tree=ast.parse(Path(original).read_text());fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='valid_odometry');scope={'math':math};exec(compile(ast.Module(body=[fn],type_ignores=[]),str(original),'exec'),scope)
    spec=importlib.util.spec_from_file_location('reviewed_roundoff',candidate);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module,scope['valid_odometry']

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--candidate',required=True);ap.add_argument('--candidate-sha',required=True);ap.add_argument('--original',required=True);ap.add_argument('--original-sha',required=True);ap.add_argument('--output',required=True);ap.add_argument('--wall-seconds',type=float,default=20);a=ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='219':raise SystemExit('requires isolated simulation ROS_DOMAIN_ID=219')
    if not 2<=a.wall_seconds<=30:raise SystemExit('bounded ground slot must be2–30 wall seconds')
    for key in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS']:os.environ[key]='1'
    if hasattr(os,'sched_getaffinity'):
        allowed=os.sched_getaffinity(0);os.sched_setaffinity(0,{min(allowed)})
    try:os.nice(10)
    except OSError:pass
    candidate,valid=load_pair(a.candidate,a.candidate_sha,a.original,a.original_sha)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import serialize_message
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from px4_msgs.msg import VehicleStatus,VehicleLandDetected
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
    from std_msgs.msg import String
    rclpy.init(args=[]);node=rclpy.create_node('passive_covariance_pair_probe',enable_rosout=False,start_parameter_services=False,use_global_arguments=False)
    output=Path(a.output);output.parent.mkdir(parents=True,exist_ok=True);clock_s=None;status=None;land=None;tracking=False;status_wall=None;stop=False;failure=None;seen={'positive':0,'roundoff_negative':0};controls_passed=None;last_stamp=None;count=0;latencies=[];subscriptions=[];started=time.monotonic()
    def halt(*_):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGINT,halt);signal.signal(signal.SIGTERM,halt)
    with output.open('x',buffering=1) as stream:
        def emit(row):
            def safe(value):
                if isinstance(value,float) and not math.isfinite(value):return None
                if isinstance(value,list):return [safe(v) for v in value]
                if isinstance(value,dict):return {k:safe(v) for k,v in value.items()}
                return value
            stream.write(json.dumps(safe(row),allow_nan=False)+'\n')
        emit({'kind':'probe_start','wall_unix_s':time.time(),'pid':os.getpid(),'domain':219,'source_sha':{'candidate':a.candidate_sha,'original':a.original_sha},'cpu_affinity':sorted(os.sched_getaffinity(0)) if hasattr(os,'sched_getaffinity') else None,'receive_only':True,'no_application_publishers':True,'scope':'Exact original valid_odometry vs covariance-canonicalized copy; downstream stateful admission is not reimplemented'})
        def clock_cb(m):
            nonlocal clock_s
            clock_s=m.clock.sec+m.clock.nanosec*1e-9
        def px4_cb(m):
            nonlocal status,stop,failure
            status=m
            if m.arming_state!=m.ARMING_STATE_DISARMED:
                failure='armed state observed; passive ground slot ended';stop=True
        def land_cb(m):
            nonlocal land,stop,failure
            land=m
            if not m.landed:failure='not-landed state observed; passive ground slot ended';stop=True
        def tracking_cb(m):
            nonlocal tracking,status_wall
            tracking=m.vo_state==1;status_wall=time.monotonic()
        def diagnostic(m):emit({'kind':'actual_original_gate_diagnostic','receipt_wall_unix_s':time.time(),'data':m.data})
        def noncov_bytes(m):
            clone=copy.deepcopy(m);clone.pose.covariance=[0.]*36;clone.twist.covariance=[0.]*36
            return serialize_message(clone)
        def raw(m):
            nonlocal last_stamp,count,stop,failure,controls_passed
            stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            if stamp==last_stamp:return
            last_stamp=stamp
            if clock_s is None or status is None or land is None:return
            ground=(status.arming_state==status.ARMING_STATE_DISARMED and not status.failsafe and land.landed and 0<=clock_s-status.timestamp*1e-6<=.75 and 0<=clock_s-land.timestamp*1e-6<=1.5)
            if not ground:return
            age_wall=math.inf if status_wall is None else time.monotonic()-status_wall
            if not math.isfinite(age_wall):return
            payload=serialize_message(m);raw_sha=hashlib.sha256(payload).hexdigest();start=time.perf_counter_ns();old=bool(valid(m,clock_s,tracking,age_wall));old_ns=time.perf_counter_ns()-start
            start=time.perf_counter_ns();fixed,report=candidate.canonicalize_message(m);new=bool(fixed is not None and valid(fixed,clock_s,tracking,age_wall));new_ns=time.perf_counter_ns()-start
            preserved=hashlib.sha256(serialize_message(m)).hexdigest()==raw_sha
            noncov_preserved=fixed is None or noncov_bytes(m)==noncov_bytes(fixed)
            if not preserved or not noncov_preserved:failure='payload preservation failed';stop=True
            negative=any(m.pose.covariance[i]<0 or m.twist.covariance[i]<0 for i in [0,7,14,21,28,35]);kind='roundoff_negative' if negative else 'positive'
            if preserved and noncov_preserved and new and ((not negative and old) or (negative and not old)):seen[kind]+=1
            count+=1;latencies.append(new_ns/1e6)
            emit({'kind':'actual_source_pair','source_acquisition_sim_s':stamp,'clock_sim_s':clock_s,'acquisition_age_sim_s':clock_s-stamp,'tracking':tracking,'tracking_status_receipt_age_wall_s':age_wall,'receipt_wall_unix_s':time.time(),'header_frame':m.header.frame_id,'child_frame':m.child_frame_id,'pose_covariance':list(m.pose.covariance),'twist_covariance':list(m.twist.covariance),'raw_cdr_sha256':raw_sha,'raw_cdr_b64':base64.b64encode(payload).decode(),'old_valid_odometry':old,'candidate_valid_odometry':new,'candidate_covariance_report':report,'old_decision_ms':old_ns/1e6,'candidate_decision_ms':new_ns/1e6,'original_payload_unchanged':preserved,'all_non_covariance_fields_exact':noncov_preserved,'actual_negative_covariance_diagonal':negative})
            if count==1:
                material=copy.deepcopy(m);material.pose.covariance[0]=-1e-6;bad,_=candidate.canonicalize_message(material)
                stale=copy.deepcopy(fixed if fixed is not None else m);old_ns_stamp=int((clock_s-.528)*1e9);stale.header.stamp.sec=old_ns_stamp//10**9;stale.header.stamp.nanosec=old_ns_stamp%10**9
                controls_passed=bool(bad is None and not valid(stale,clock_s,tracking,age_wall))
                if not controls_passed:failure='in-process hard-fail control failed';stop=True
                emit({'kind':'in_process_controls_never_published','source_raw_sha256':raw_sha,'material_negative_rejected':bad is None,'stale_0528s_still_rejected':not bool(valid(stale,clock_s,tracking,age_wall))})
            if seen['positive'] and seen['roundoff_negative']:stop=True
        subscriptions.append(node.create_subscription(Clock,'/clock',clock_cb,qos_profile_sensor_data))
        for topic in ['/fmu/out/vehicle_status_v1','/fmu/out/vehicle_status']:subscriptions.append(node.create_subscription(VehicleStatus,topic,px4_cb,qos_profile_sensor_data))
        for topic in ['/fmu/out/vehicle_land_detected','/fmu/out/vehicle_land_detected_v1']:subscriptions.append(node.create_subscription(VehicleLandDetected,topic,land_cb,qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(VisualSlamStatus,'/visual_slam/status',tracking_cb,qos_profile_sensor_data));subscriptions.append(node.create_subscription(Odometry,'/visual_slam/tracking/odometry',raw,qos_profile_sensor_data));subscriptions.append(node.create_subscription(String,'/mapping/gate_status',diagnostic,qos_profile_sensor_data))
        while not stop and time.monotonic()-started<a.wall_seconds:rclpy.spin_once(node,timeout_sec=.05)
        complete=bool(seen['positive'] and seen['roundoff_negative'] and controls_passed and not failure)
        emit({'kind':'probe_summary','paired_actual_source_complete':complete,'failure':failure,'in_process_controls_passed':controls_passed,'actual_source_packet_count':count,'paired_actual_categories':seen,'candidate_max_ms':max(latencies,default=None),'wall_elapsed_s':time.monotonic()-started,'source_sha':{'candidate':digest(a.candidate),'original':digest(a.original)},'no_application_ROS_publications_or_PX4_commands':True,'no_flight_qualification_claim':True})
    node.destroy_node();rclpy.shutdown()
    return 0 if complete else 2
if __name__=='__main__':raise SystemExit(main())
