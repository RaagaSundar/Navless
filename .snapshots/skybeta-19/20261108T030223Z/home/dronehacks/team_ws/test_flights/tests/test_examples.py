"""Offline contract and maneuver tests. No ROS transport, flight or vehicle model."""
import importlib
import json
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest


class Msg:
    ARMING_STATE_DISARMED, ARMING_STATE_ARMED, NAVIGATION_STATE_OFFBOARD = 1, 2, 14
    POSE_FRAME_NED, POSE_FRAME_FRD = 1, 2

    def __init__(self):
        self.header = NS()
        self.pose = NS(position=NS(), orientation=NS())
        self.twist = NS(linear=NS(x=0., y=0., z=0.), angular=NS(z=0.))


class Node:
    def __init__(self, name):
        self.writers, self.timers, self.subs, self.counts = {}, [], {}, {}
        self.logs = []
        self.now = 100.

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=round(self.now*1e9), to_msg=lambda: self.now))

    def create_publisher(self, typ, topic, qos):
        assert topic in ('/skybeta/setpoint/pose', '/skybeta/setpoint/twist')
        data = self.writers[topic] = []
        return NS(publish=data.append)

    def create_subscription(self, typ, topic, cb, qos):
        self.subs[topic] = cb

    def create_timer(self, period, cb):
        self.timers.append(cb)

    def count_publishers(self, topic):
        return self.counts.get(topic, int(topic in self.writers or topic in ('/skybeta/state','/onboard/vision_health')))

    def get_subscriptions_info_by_topic(self, topic):
        return [NS(node_name='flight_interface',node_namespace='/')]

    def get_parameter(self, name):
        return NS(value=False)

    def resolve_topic_name(self, name):
        return name

    def declare_parameter(self, name, default):
        return NS(value=default)

    def get_logger(self):
        return NS(info=self.logs.append,error=self.logs.append)


@pytest.fixture
def harness(monkeypatch):
    for name in ['rclpy','rclpy.node','rclpy.qos','px4_msgs','px4_msgs.msg','geometry_msgs',
                 'geometry_msgs.msg','nav_msgs','nav_msgs.msg','std_msgs','std_msgs.msg']:
        monkeypatch.setitem(sys.modules,name,ModuleType(name))
    sys.modules['rclpy.node'].Node = Node
    qos = sys.modules['rclpy.qos']
    for key in ('ReliabilityPolicy','DurabilityPolicy','HistoryPolicy'):
        setattr(qos,key,NS(BEST_EFFORT=0,TRANSIENT_LOCAL=1,VOLATILE=2,KEEP_LAST=3))
    qos.QoSProfile = lambda **kw: kw
    for pkg,names in [('px4_msgs','VehicleStatus VehicleLandDetected VehicleLocalPosition EstimatorStatusFlags VehicleOdometry'),
                      ('geometry_msgs','PoseStamped TwistStamped'),('nav_msgs','Odometry'),('std_msgs','String Bool')]:
        for name in names.split():
            setattr(sys.modules[pkg+'.msg'],name,Msg)
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]/'examples'))
    for name in ('skybeta_test','telemetry'):
        monkeypatch.delitem(sys.modules,name,raising=False)
    mod = importlib.import_module('skybeta_test')
    clock = [100.]
    monkeypatch.setattr(mod,'time',NS(monotonic=lambda:clock[0]))
    def make(plan='hover',passive=False):
        n = mod.Mission(plan,passive)
        def step(armed=False, off=False, landed=True, pose=(4.,-3.,.1,math.pi/2), health=True, tick=True):
            n.now = clock[0] = round(clock[0]+.1,1)
            stamp = round(n.now*1e6)
            n.on_position(NS(timestamp=stamp,x=2.,y=1.,z=0.,heading=0.,xy_valid=True,z_valid=True,
                             xy_reset_counter=0,z_reset_counter=0,heading_reset_counter=0,
                             heading_good_for_control=True,dead_reckoning=False))
            n.on_status(NS(timestamp=stamp,arming_state=2 if armed else 1,nav_state=14 if off else 2,failsafe=False))
            n.on_landed(NS(timestamp=stamp,landed=landed))
            n.on_estimator_flags(NS(timestamp=stamp,cs_yaw_align=True,cs_ev_pos=True,cs_ev_yaw=True,
                                    cs_ev_yaw_fault=False,cs_inertial_dead_reckoning=False,reject_hor_pos=False,reject_yaw=False))
            p = NS(x=pose[0],y=pose[1],z=pose[2])
            q = NS(x=0.,y=0.,z=math.sin(pose[3]/2),w=math.cos(pose[3]/2))
            n.on_slam_odometry(NS(header=NS(frame_id='odom',stamp=NS(sec=int(n.now),nanosec=round((n.now%1)*1e9))),
                                  child_frame_id='base_link',pose=NS(pose=NS(position=p,orientation=q))))
            n.on_vision_odometry(NS(timestamp=stamp,timestamp_sample=stamp,position=[1.,2.,3.],q=[1.,0.,0.,0.],pose_frame=1,reset_counter=0))
            n.on_vision_health(NS(data=health))
            state=dict(stamp=n.now,px4_link=True,armed=armed,offboard_active=off,landed=landed,
                       frozen=False,envelope_ok=True,origin_enu=[4.,-3.,.1],odom_enu=list(pose[:3]),local_enu=[0.,0.,1.],
                       envelope=dict(min_altitude_m=.3,max_altitude_m=2.5,max_speed_mps=1.))
            n.state_cb(NS(data=json.dumps(state)))
            if tick:
                n.tick()
        return n,step
    return make,mod


@pytest.mark.parametrize('plan',['hover','yaw','forward'])
def test_full_plan_then_pilot_landing(harness,plan):
    make,m = harness
    n,step = make(plan)
    for _ in range(24):
        step()
    assert n.phase == 'READY' and n.writers[m.TWIST] and not n.writers[m.POSE]
    step(True)
    assert not n.writers[m.POSE]
    step(True,True)
    p=(4.,-3.,.1,math.pi/2)
    trail=[]
    for _ in range(700):
        step(True,True,False,p)
        msg=n.writers[m.POSE][-1].pose
        p=(msg.position.x,msg.position.y,msg.position.z,2*math.atan2(msg.orientation.z,msg.orientation.w))
        trail.append(p)
        if n.phase=='HANDOFF':
            break
    assert n.phase=='HANDOFF', n.logs
    assert p==pytest.approx((4.,-3.,1.1,-math.pi/2 if plan=='yaw' else math.pi/2))
    assert max(v[1] for v in trail)==pytest.approx(-1. if plan=='forward' else -3.)
    if plan=='hover':
        assert sum(abs(v[2]-1.1)<.001 for v in trail)>=100
    if plan=='yaw':
        assert all(-math.radians(1.01) <= b[3]-a[3] <= 1e-8 for a,b in zip(trail,trail[1:]))
    before=len(n.writers[m.POSE])
    step(True,False,False,p)
    assert n.phase=='OBSERVE'
    step(False,False,False,p)
    assert n.phase=='OBSERVE'
    step(False,False,True,p)
    assert n.phase=='DONE' and len(n.writers[m.POSE])==before


@pytest.mark.parametrize('loss',['takeover','stale_state','health','writer','reset','fusion','stuck'])
def test_no_more_setpoints(harness,loss):
    make,m=harness
    n,step=make()
    for _ in range(24): step()
    step(True,True)
    count=len(n.writers[m.POSE])
    step(True,loss!='takeover',False,health=loss!='health',tick=False)
    if loss=='stale_state': n.fi_stamp-=1
    if loss=='writer': n.counts[m.TWIST]=2
    if loss=='fusion': n.estimator_flags.cs_ev_yaw=False
    if loss=='reset':
        n.pos.xy_reset_counter+=1
        n.on_position(n.pos)
    if loss=='stuck': n.entered-=31
    n.tick()
    step(True,True,False)
    assert n.phase=='ABORT' and len(n.writers[m.POSE])==count


@pytest.mark.parametrize('fault',['late','no_interface','two_health','malformed','stale_odom','not_settled','passive'])
def test_no_commands_before_ready(harness,fault):
    make,m=harness
    n,step=make(passive=fault=='passive')
    for _ in range(25):
        step(fault=='late',fault=='late',tick=False)
        if fault=='no_interface': n.counts[m.STATE]=0
        if fault=='two_health': n.counts[n.health_topic]=2
        if fault=='malformed': n.state_cb(NS(data='{"armed":"false"}'))
        if fault=='stale_odom': n.slam_time-=1
        if fault=='not_settled': n.landed=False
        if fault!='passive': n.tick()
    assert not any(n.writers.values())
    if fault=='passive':
        assert not n.writers and not n.timers and n.problem(n.now) is None
