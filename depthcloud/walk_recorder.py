#!/usr/bin/env python3
"""Walk-session recorder: everything needed to judge localization afterwards, one CSV row every 0.1 s.

Started and stopped by walk_session.sh (don't run it by hand during a flight). Read-only: it only
subscribes. Writes <session>/track.csv, flushed every second, so a pulled battery keeps the data.

Columns (latest value of each source at the row's time; ages in seconds, '' = none yet):
  localizer  /team/localize/base_pose + /team/localize/status: status, x y z yaw, fitness, matches,
             measured x/y/heading, cropped, odom_source
  correction /team/localize/status -> odom_correction: valid, dx dy dyaw
  cuVSLAM    /visual_slam/tracking/odometry (base_link in odom) + /visual_slam/status vo_state
  PX4        /fmu/out/vehicle_local_position (EKF2, NED -> ENU), only if PX4 is linked
  stereo     /team/stereo/points message rate (clouds in the last second)

    python3 walk_recorder.py --ros-args -p out:=/root/team_ws/walk_sessions/<time>/track.csv
"""
import csv
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
try:
    from px4_msgs.msg import VehicleLocalPosition
except ImportError:
    VehicleLocalPosition = None
try:
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
except ImportError:
    VisualSlamStatus = None

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
COLUMNS = ['wall_time', 't',
           'loc_status', 'loc_x', 'loc_y', 'loc_z', 'loc_yaw_deg', 'loc_age', 'fitness', 'matches',
           'meas_x', 'meas_y', 'meas_heading', 'cropped', 'odom_source',
           'corr_valid', 'corr_dx', 'corr_dy', 'corr_dyaw_deg',
           'odom_x', 'odom_y', 'odom_z', 'odom_yaw_deg', 'odom_age', 'vo_state',
           'px4_x', 'px4_y', 'px4_z', 'px4_yaw_deg', 'px4_age',
           'clouds_per_s']


def yaw_deg(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


class WalkRecorder(Node):
    def __init__(self):
        super().__init__('walk_recorder')
        self.declare_parameter('out', '/root/team_ws/walk_sessions/track.csv')
        self.declare_parameter('rate_hz', 10.0)
        self.path = str(self.get_parameter('out').value)
        self.f = open(self.path, 'w', newline='')
        self.w = csv.DictWriter(self.f, fieldnames=COLUMNS, extrasaction='ignore')
        self.w.writeheader()
        self.last_flush = time.monotonic()
        self.loc = self.loc_at = None
        self.status = {}
        self.odom = self.odom_at = None
        self.vo = None
        self.px4 = self.px4_at = None
        self.cloud_times = []
        self.create_subscription(PoseStamped, '/team/localize/base_pose', self.on_loc, 10)
        self.create_subscription(String, '/team/localize/status', self.on_status, 10)
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/team/stereo/points', self.on_cloud, 2)
        if VisualSlamStatus is not None:
            self.create_subscription(VisualSlamStatus, '/visual_slam/status', self.on_vo, 10)
        if VehicleLocalPosition is not None:
            self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.on_px4, PX4_QOS)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self.row)
        self.get_logger().info(f'recording to {self.path}')

    def on_loc(self, m):
        p = m.pose.position
        self.loc, self.loc_at = (p.x, p.y, p.z, yaw_deg(m.pose.orientation)), time.monotonic()

    def on_status(self, m):
        try:
            self.status = json.loads(m.data)
        except ValueError:
            pass

    def on_odom(self, m):
        p = m.pose.pose.position
        self.odom, self.odom_at = (p.x, p.y, p.z, yaw_deg(m.pose.pose.orientation)), time.monotonic()

    def on_vo(self, m):
        self.vo = int(m.vo_state)

    def on_px4(self, m):
        if m.xy_valid and m.z_valid and all(math.isfinite(v) for v in (m.x, m.y, m.z)):
            yaw = math.degrees(math.pi / 2 - m.heading) if math.isfinite(m.heading) else float('nan')
            self.px4, self.px4_at = (m.y, m.x, -m.z, (yaw + 180) % 360 - 180), time.monotonic()

    def on_cloud(self, _m):
        self.cloud_times.append(time.monotonic())

    def row(self):
        now = time.monotonic()
        self.cloud_times = [t for t in self.cloud_times if now - t <= 1.0]
        s = self.status
        meas = s.get('measured') or {}
        corr = s.get('odom_correction') or {}
        v = {'wall_time': round(time.time(), 3), 't': round(now, 3), 'loc_status': s.get('status', ''),
             'fitness': s.get('fitness'), 'matches': s.get('matches'), 'cropped': s.get('cropped'),
             'odom_source': s.get('odom_source', ''),
             'meas_x': meas.get('x'), 'meas_y': meas.get('y'), 'meas_heading': meas.get('heading'),
             'corr_valid': int(bool(corr.get('valid'))) if corr else '', 'corr_dx': corr.get('dx'),
             'corr_dy': corr.get('dy'), 'corr_dyaw_deg': corr.get('dyaw_deg'),
             'vo_state': self.vo, 'clouds_per_s': len(self.cloud_times)}
        for name, val, at in (('loc', self.loc, self.loc_at), ('odom', self.odom, self.odom_at),
                              ('px4', self.px4, self.px4_at)):
            if val is not None:
                v.update({f'{name}_x': val[0], f'{name}_y': val[1], f'{name}_z': val[2],
                          f'{name}_yaw_deg': val[3], f'{name}_age': now - at})
        self.w.writerow({k: (round(x, 4) if isinstance(x, float) else ('' if x is None else x)) for k, x in v.items()})
        if now - self.last_flush >= 1.0:
            self.f.flush()
            self.last_flush = now

    def close(self):
        try:
            self.f.flush()
            self.f.close()
        except ValueError:
            pass


def main():
    rclpy.init()
    node = WalkRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
