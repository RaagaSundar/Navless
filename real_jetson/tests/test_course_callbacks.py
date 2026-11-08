"""Exercise the actual course class and its output callbacks without ROS transport."""
import importlib
import math
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'jetson'))


@pytest.fixture
def runtime(monkeypatch):
    clock = [100.]
    sent = []
    class Msg:
        ARMING_STATE_ARMED = 2
        NAVIGATION_STATE_OFFBOARD = 14
        def __init__(self, **kw):
            self.header = NS()
            self.pose = NS(position=NS(), orientation=NS())
            self.twist = NS(linear=NS(x=0.,y=0.,z=0.),angular=NS(z=0.))
            vars(self).update(kw)
    class Node:
        def __init__(self, name, parameter_overrides=()):
            self.params = {'use_sim_time': False}
            self.params.update({p.name:p.value for p in parameter_overrides})
        def declare_parameter(self, name, default, descriptor=None):
            return NS(value=self.params.setdefault(name,default))
        def get_parameter(self,name): return NS(value=self.params[name])
        def get_logger(self): return NS(info=lambda s:None,warn=lambda s:None,error=lambda s:None)
        def get_clock(self):
            return NS(now=lambda:NS(nanoseconds=round(clock[0]*1e9),to_msg=lambda:clock[0]))
        def create_subscription(self,*args): pass
        def create_timer(self,*args): pass
        def create_publisher(self,typ,topic,qos):
            assert topic in ('/skybeta/setpoint/pose','/skybeta/setpoint/twist')
            return NS(publish=lambda m:sent.append((topic,m)))
        def resolve_topic_name(self,topic): return topic
        def count_publishers(self,topic): return 1
        def get_subscriptions_info_by_topic(self,topic):
            return [NS(node_name='flight_interface',node_namespace='/')]
    for package, names in {
        'rclpy.node':['Node'], 'rclpy.parameter':['Parameter'], 'rclpy.executors':['SingleThreadedExecutor'],
        'rclpy.qos':['QoSProfile','DurabilityPolicy','HistoryPolicy','ReliabilityPolicy','qos_profile_sensor_data'],
        'std_msgs.msg':['Bool','String'],'nav_msgs.msg':['Odometry'],
        'px4_msgs.msg':['VehicleStatus','VehicleLocalPosition','VehicleLandDetected','EstimatorStatusFlags','VehicleOdometry'],
        'rcl_interfaces.msg':['ParameterDescriptor'],'isaac_ros_visual_slam_interfaces.msg':['VisualSlamStatus'],
        'geometry_msgs.msg':['PoseStamped','TwistStamped','PointStamped']}.items():
        parent=package.split('.')[0]
        if parent not in sys.modules: monkeypatch.setitem(sys.modules,parent,ModuleType(parent))
        module=ModuleType(package)
        for name in names: setattr(module,name,Msg)
        monkeypatch.setitem(sys.modules,package,module)
    sys.modules['rclpy.node'].Node=Node
    sys.modules['rclpy.parameter'].Parameter=lambda name,value:NS(name=name,value=value)
    qos=sys.modules['rclpy.qos']
    qos.DurabilityPolicy=qos.HistoryPolicy=qos.ReliabilityPolicy=NS(BEST_EFFORT=0,TRANSIENT_LOCAL=1,VOLATILE=2,KEEP_LAST=1)
    for key in ('course','interface_monitor','telemetry','navless_nav.course_mission'):
        monkeypatch.delitem(sys.modules,key,raising=False)
    c=importlib.import_module('course')
    monkeypatch.setattr(c,'time',NS(monotonic=lambda:clock[0]))
    m=c.InterfaceMonitor()
    node=c.HardwareCourse(m)
    def step(armed=False,off=False,ground=True,health=True):
        clock[0]=round(clock[0]+.1,1)
        sec=int(clock[0]);stamp=NS(sec=sec,nanosec=round((clock[0]-sec)*1e9))
        header=NS(frame_id='odom',stamp=stamp)
        odom=NS(header=header,child_frame_id='base_link',pose=NS(pose=NS(
            position=NS(x=0.,y=0.,z=0.),orientation=NS(w=1.,x=0.,y=0.,z=0.))))
        micro=round(clock[0]*1e6)
        pos=NS(timestamp=micro,x=0.,y=0.,z=-.19,heading=math.pi/2,
               xy_reset_counter=0,z_reset_counter=0,heading_reset_counter=0,xy_valid=True,z_valid=True,
               heading_good_for_control=True,dead_reckoning=False)
        node.odometry_cb(odom)
        node.px4_local_cb(pos)
        node.status_cb(NS(header=header,vo_state=1))
        m.on_slam_odometry(odom)
        m.on_position(pos)
        m.on_landed(NS(timestamp=micro,landed=ground))
        m.on_vision_health(NS(data=health))
        m.on_estimator_flags(NS(timestamp=micro,cs_yaw_align=True,cs_ev_pos=True,cs_ev_yaw=True,
               cs_ev_yaw_fault=False,cs_inertial_dead_reckoning=False,reject_hor_pos=False,reject_yaw=False))
        m.vision_time=m.vision_stamp=clock[0];m.vision_valid=True
        m.on_status(NS(timestamp=micro,arming_state=2 if armed else 1,nav_state=14 if off else 2,failsafe=False))
        import json
        m.state_cb(NS(data=json.dumps(dict(stamp=clock[0],landed=ground,armed=armed,offboard_active=off,
            px4_link=True,frozen=False,envelope_ok=True,origin_enu=[0,0,0],odom_enu=[0,0,0],
            local_enu=[0,0,.19],envelope=dict(min_altitude_m=.3,max_altitude_m=2.5,max_speed_mps=1.)))))
        node.tick()
    return node,sent,step


@pytest.mark.parametrize('loss',['rc','vision'])
def test_course_starts_after_handover_and_yields(runtime,loss):
    node,sent,step=runtime
    for _ in range(46): step()
    assert node.handover.phase=='READY',node.handover.reason
    assert sent and all(t.endswith('/twist') and m.twist.linear.z==0 for t,m in sent)
    step(True,False)
    step(True,True)
    topic,msg=sent[-1]
    assert topic.endswith('/pose') and msg.header.frame_id=='odom' and node.state=='TAKEOFF'
    assert msg.pose.position.z==pytest.approx(node.route[0][2]+.19)
    count=len(sent)
    step(True,loss!='rc',False,loss!='vision')
    step(True,True,False)
    assert node.handover.phase=='STOP' and len(sent)==count
