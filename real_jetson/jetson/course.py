"""Team course controller using the organiser's running interface on aircraft 19."""
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from px4_msgs.msg import VehicleStatus
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from handover import Handover
from navless_nav.course_mission import CourseMission
from interface_monitor import InterfaceMonitor

ROOT = Path(__file__).resolve().parent


class HardwareCourse(CourseMission):
    def __init__(self, monitor):
        self.monitor = monitor
        self.handover = Handover()
        self.pose_stamp = None
        self.tracking_stamp = None
        self.tick_at = None
        self.writing = False
        super().__init__(parameter_overrides=[
            Parameter('route_file', value=str(ROOT / 'window-route.json')),
            Parameter('net_margin_m', value=0.25),
            Parameter('cruise_speed_mps', value=0.4),
            Parameter('tof_tube_centering', value=True),
        ])
        if len(self.route or []) < 2 or self.get_parameter('use_sim_time').value:
            raise RuntimeError("missing route or hardware clock")
        self.last_report = None

    def odometry_cb(self, msg):
        header = msg.header
        stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if (header.frame_id != 'odom' or msg.child_frame_id != 'base_link'
                or not all(map(math.isfinite, (p.x, p.y, p.z, q.x, q.y, q.z, q.w)))
                or not 0.9 <= sum(v*v for v in (q.x, q.y, q.z, q.w)) <= 1.1
                or not 0 <= self.now_s() - stamp <= 0.5
                or (self.pose_stamp is not None and stamp <= self.pose_stamp)):
            self.position = None
            return
        self.pose_stamp = stamp
        super().odometry_cb(msg)

    def status_cb(self, msg):
        self.tracking_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        super().status_cb(msg)

    def state_cb(self, msg):
        # Use the monitor's checked, timestamped interface state in tick().
        pass

    def tick(self):
        m, now, t = self.monitor, self.now_s(), time.monotonic()
        fault = m.problem(now, expected_writers=1)
        if m.phase == 'ABORT':
            fault = 'estimator reset or authority lost'
        if self.pose_stamp is None or self.position is None or not 0 <= now-self.pose_stamp <= .5:
            fault = 'stale or invalid odometry'
        if self.tracking_stamp is None or self.vo_state != 1 or not 0 <= now-self.tracking_stamp <= .5:
            fault = 'tracking lost or stale'
        if m.landed is None or not 0 <= now-m.landed_time <= 1:
            fault = 'land detector missing or stale'
        if self.tick_at is not None and t - self.tick_at > .5:
            fault = 'course timer stalled'
        self.tick_at = t
        s = m.fi or {}
        px_armed = m.status is not None and m.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        px_off = m.status is not None and m.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ground = m.landed is True and s.get('landed') is True
        if (self.handover.phase in ('READY', 'ACTIVE') and s
                and tuple(s['origin_enu']) != self.interface_origin):
            fault = 'flight interface origin changed'
        floor_settling = False
        if not fault:
            super().state_cb(String(data=json.dumps(s)))
            if self.handover.phase != 'ACTIVE':
                if (math.hypot(*self.position[:2]) > self.max_start_offset
                        or self.px4_heading() is None or not self.headings_agree()):
                    fault = 'launch position or heading outside limits'
                elif not self.heights_agree():
                    floor_settling = True
                    fault = 'floor heights have not settled'
        if self.handover.phase == 'READY' and not fault:
            if (math.dist(m.pose[:3], self.ground_pose[:3]) > .15
                    or abs(math.remainder(m.pose[3] - self.ground_pose[3], 2*math.pi)) > math.radians(6)):
                fault = 'aircraft moved before RC handover'
        phase = self.handover.update(t, fault, ground, px_armed, s.get('armed'), px_off, s.get('offboard_active'))
        status = (phase, self.handover.reason)
        if status != self.last_report:
            self.get_logger().info(f'{phase}: {status[1]}')
            self.last_report = status
        if phase == 'WAIT':
            if fault:
                self.prearm_passed = False
                # heights_agree owns its settling clock when the floor check itself is pending.
                if not floor_settling:
                    self.heights_agree_since = None
                    self.z_samples.clear()
            return
        if phase == 'READY':
            if m.phase == 'WAIT':
                self.interface_origin = tuple(s['origin_enu'])
                self.ground_pose = m.pose
            m.phase = 'READY'
            m.saw_armed |= bool(px_armed and s.get('armed'))
            msg = TwistStamped()
            msg.header.frame_id = 'odom'
            msg.header.stamp = self.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
        elif phase == 'ACTIVE':
            m.phase = 'COURSE'
            self.writing = True
            try:
                super().tick()
            finally:
                self.writing = False

    def publish_pose(self, *args):
        # A PX4 heading-reset callback can also ask the base to republish.
        if self.writing:
            super().publish_pose(*args)

    def send_twist(self, *args):
        if self.writing:
            super().send_twist(*args)


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '19':
        raise SystemExit('requires physical ROS domain 19')
    rclpy.init(args=[])
    monitor = InterfaceMonitor()
    controller = HardwareCourse(monitor)
    executor = SingleThreadedExecutor()
    for node in (monitor, controller):
        executor.add_node(node)
    try:
        while rclpy.ok() and controller.handover.phase != 'STOP':
            executor.spin_once(timeout_sec=.2)
    finally:
        executor.shutdown()
        controller.destroy_node()
        monitor.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
