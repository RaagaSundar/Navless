"""Center the quad on repeated camera observations, cross, settle, then land."""
from collections import deque
import json
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from hover import FunnelHover
from passage import advance, midpoint


class FunnelTraverse(FunnelHover):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.buffer = Buffer()
        self.transforms = TransformListener(self.buffer, self)
        self.extrinsic = None
        self.estimates = deque(maxlen=35)
        self.center = self.axis = self.carrot = None
        self.phase_time = self.last_step = None
        self.settled = None
        self.latest_center = -math.inf

    def sample(self, msg, info, odom):
        super().sample(msg, info, odom)
        if self.measurement is None or self.state not in ('AIM','APPROACH'):
            return
        stamp, aim = self.measurement
        if not aim or abs(stamp-(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)) > 1e-7:
            return
        if odom.header.frame_id != 'odom' or odom.child_frame_id != 'base_link':
            return
        if self.extrinsic is None:
            try:
                self.extrinsic = self.buffer.lookup_transform('base_link', msg.header.frame_id, Time()).transform
            except TransformException:
                return
        point = midpoint(aim, info.p, self.extrinsic, odom.pose.pose)
        if point is None or (self.center is not None and np.linalg.norm(point-self.center) > .28):
            return
        self.estimates.append((stamp,point))
        recent = [p for t,p in self.estimates if 0 <= stamp-t < .7]
        if len(recent) >= 8 and np.max(np.ptp(recent,axis=0)) < .16:
            self.center = np.median(recent,axis=0)
            self.latest_center = stamp

    def phase(self, state, now, reason):
        self.state, self.phase_time = state, now
        print(json.dumps(dict(event='passage',state=state,t=now,reason=reason,
                              position=self.position, center=None if self.center is None else self.center.tolist())),flush=True)

    def step(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.position is None or not 0 <= now-self.position_stamp < .5 or now-self.state_stamp > .5:
            return
        xy = np.array(self.position[:2])
        if not self.active:
            if self.state != 'WAIT' or not self.measured(now):
                return
            self.home, self.carrot = self.position, xy
        if self.home is None:
            return
        dt = 0. if self.last_step is None else np.clip(now-self.last_step,0.,.15)
        self.last_step = now
        z = self.home[2]
        if self.active:
            z = min(z+1.,self.position[2]+.20)
            if self.state == 'WAIT':
                self.phase('CLIMB',now,'Offboard active')
            if self.state == 'CLIMB' and abs(self.position[2] - self.home[2] - 1.) < .08:
                self.phase('AIM',now,'at traversal height')
            if self.state == 'AIM' and now-self.phase_time > 3. and self.measured(now) and now-self.latest_center < .25:
                direction = self.center - self.home[:2]
                distance = np.linalg.norm(direction)
                if 2.5 < distance < 5.5 and abs(self.measurement[1]['bearing_right_deg']) < 2:
                    self.axis = direction / distance
                    self.phase('APPROACH',now,'metric opening acquired')
            if self.state == 'APPROACH':
                fresh = 0 <= now - self.latest_center < .30
                delta = self.center-xy
                along = float(delta @ self.axis)
                lateral = np.linalg.norm(delta - along*self.axis)
                if fresh and .75 <= along <= 1.2 and lateral < .09:
                    self.phase('CROSS',now,'center locked before edges leave view')
                elif fresh and along > .75 and lateral < .30:
                    self.carrot = advance(self.carrot,xy,self.center,dt)
                elif now-self.latest_center > 2. or along < .75 or lateral >= .30:
                    self.phase('LAND',now,'approach check failed')
            if self.state == 'CROSS':
                end = self.center+self.axis
                self.carrot = advance(self.carrot,xy,end,dt)
                at_end = np.linalg.norm(end-xy) < .09 and abs(self.position[2]-self.home[2]-1.) < .12
                if not at_end:
                    self.settled = None
                elif self.settled is None:
                    self.settled = now
                elif now-self.settled >= 1.:
                    self.phase('LAND',now,'far-side position held for one second')
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.pose.position.x,sp.pose.position.y = map(float,self.carrot)
        sp.pose.position.z = float(z)
        sp.pose.orientation.w = 1.
        self.publisher.publish(sp)

    def heading(self, now, p, history):
        if self.state not in ('APPROACH','CROSS','LAND'):
            return super().heading(now,p,history)
        return self.yaw_command
