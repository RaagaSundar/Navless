"""Launch-relative position hold and a bounded yaw correction using the optical bearing."""
import json
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from observe import Observer


class FunnelHover(Observer):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.position = self.home = None
        self.position_stamp = self.state_stamp = -math.inf
        self.active = False
        self.state = 'WAIT'
        self.hold_since = None
        self.yaw_command = self.initial_heading = None
        self.last_tick = None
        self.create_subscription(Odometry, '/split_course/px4_navigation', self.pose, 10)
        self.create_subscription(String, '/skybeta/state', self.status_update, 10)
        self.publisher = self.create_publisher(PoseStamped, '/skybeta/setpoint/pose', 10)
        self.create_timer(0.1, self.step)

    def pose(self, msg):
        p = msg.pose.pose.position
        self.position = (p.x,p.y,p.z)
        self.position_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def status_update(self, msg):
        self.active = json.loads(msg.data)['offboard_active']
        self.state_stamp = self.get_clock().now().nanoseconds * 1e-9

    def measured(self, now):
        return self.measurement and self.measurement[1] and 0 <= now-self.measurement[0] <= .25

    def step(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.position is None or not 0 <= now-self.position_stamp < .5 or now-self.state_stamp > .5:
            return
        if not self.active:
            if not self.measured(now) or self.state != 'WAIT':
                return
            self.home = self.position
        if self.home is None:
            return
        x,y,z = self.home
        if self.active:
            if self.state == 'WAIT':
                self.state = 'CLIMB'
            z = min(z+1., self.position[2]+.20)
            if abs(self.position[2]-self.home[2]-1.) < .10 and self.hold_since is None:
                self.hold_since = now
                self.state = 'AIM'
            if self.hold_since is not None and (now-self.hold_since >= 5.0 or not self.measured(now)):
                self.state = 'LAND'
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x,y,z
        msg.pose.orientation.w = 1.
        self.publisher.publish(msg)

    def heading(self, now, p, history):
        if self.initial_heading is None or not self.active:
            self.initial_heading = self.yaw_command = float(p.heading)
        dt = 0. if self.last_tick is None else max(0., min(.1, now-self.last_tick))
        self.last_tick = now
        if self.state == 'AIM' and self.measured(now):
            aligned = history.at(self.measurement[0])
            if aligned is not None:
                # Image-right and PX4 positive NED yaw both turn clockwise.
                desired = aligned[3] + math.radians(self.measurement[1]['bearing_right_deg'])
                delta = math.remainder(desired-self.initial_heading, 2*math.pi)
                target = self.initial_heading + max(-math.radians(10), min(math.radians(10), delta))
                step = math.radians(5)*dt
                self.yaw_command += max(-step, min(step, target-self.yaw_command))
        return self.yaw_command
