#!/usr/bin/env python3
"""Walk-session recorder: everything needed to judge localization afterwards.

Started and stopped by walk_session.sh (don't run it by hand during a flight). Read-only: it only
subscribes, never publishes. All files are flushed every second, so a pulled battery keeps the data.

track.csv, one row every 0.1 s (latest value of each source at the row's time; ages in s, '' = none yet):
  localizer  /team/localize/base_pose + /team/localize/status: status, x y z yaw, fitness, matches,
             measured x/y/heading, cropped, odom_source
  correction /team/localize/status -> odom_correction: valid, dx dy dyaw
  cuVSLAM    /visual_slam/tracking/odometry (base_link in odom): pose + velocity; /visual_slam/status vo_state
  PX4        /fmu/out/vehicle_local_position (EKF2, NED -> ENU): position, velocity, acceleration, heading,
             reset counters, rangefinder, accuracy, dead reckoning; vehicle_status_v1: armed, nav_state;
             battery voltage; how fresh the vision feed into PX4 (/fmu/in/vehicle_visual_odometry) is
  IMU        how fresh the camera IMU is
  stereo     /team/stereo/points message rate (clouds in the last second)

raw/*.csv: EVERY message, every field, at the source's full rate (arrays as name_0, name_1, ...;
nested fields joined with '.'):
  px4_local_position.csv   /fmu/out/vehicle_local_position   EKF2 estimate (NED), all fields
  px4_status.csv           /fmu/out/vehicle_status_v1        arming, mode, failsafe, ...
  px4_battery.csv          /fmu/out/battery_status
  px4_vision_in.csv        /fmu/in/vehicle_visual_odometry   what the bridge feeds EKF2 (NED/FRD)
  cuvslam_odometry.csv     /visual_slam/tracking/odometry    pose, twist, covariances
  oak_imu.csv              /oak/imu/data                     camera IMU (BNO086, ~200 Hz)
19's PX4 publishes only those three /fmu/out topics over uXRCE-DDS: the Pixhawk's own IMU, attitude and
EKF2 odometry topics aren't available to ROS, and enabling them is a PX4 config change, which we don't do.
Each raw row starts with recv_wall (time.time()) and recv_t (monotonic, the same clock as track.csv's t).

    python3 walk_recorder.py --ros-args -p out:=/root/team_ws/walk_sessions/<time>/track.csv [-p raw:=false]
"""
import csv
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String
try:
    from px4_msgs.msg import BatteryStatus, VehicleLocalPosition, VehicleOdometry, VehicleStatus
except ImportError:
    BatteryStatus = VehicleLocalPosition = VehicleOdometry = VehicleStatus = None
try:
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
except ImportError:
    VisualSlamStatus = None

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
RAW_TOPICS = ('px4_local_position', 'px4_status', 'px4_battery', 'px4_vision_in', 'cuvslam_odometry', 'oak_imu')
COLUMNS = ['wall_time', 't',
           'loc_status', 'loc_x', 'loc_y', 'loc_z', 'loc_yaw_deg', 'loc_age', 'fitness', 'matches',
           'meas_x', 'meas_y', 'meas_heading', 'cropped', 'odom_source',
           'corr_valid', 'corr_dx', 'corr_dy', 'corr_dyaw_deg',
           'odom_x', 'odom_y', 'odom_z', 'odom_yaw_deg', 'odom_age', 'odom_vx', 'odom_vy', 'odom_vz', 'odom_wz',
           'vo_state',
           'px4_x', 'px4_y', 'px4_z', 'px4_yaw_deg', 'px4_age', 'px4_vx', 'px4_vy', 'px4_vz',
           'px4_ax', 'px4_ay', 'px4_az', 'px4_xy_valid', 'px4_z_valid', 'px4_xy_resets', 'px4_z_resets',
           'px4_heading_resets', 'px4_heading_good', 'px4_dist_bottom', 'px4_dist_bottom_valid',
           'px4_eph', 'px4_epv', 'px4_dead_reckoning',
           'px4_armed', 'px4_nav_state', 'battery_v', 'vision_in_age', 'imu_age',
           'clouds_per_s']


def yaw_deg(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def flatten(msg, prefix=''):
    """Every field of a ROS message as (name, value) pairs: nested messages joined with '.', arrays as _i."""
    out = []
    for name in msg.get_fields_and_field_types():
        v = getattr(msg, name)
        key = prefix + name
        if hasattr(v, 'get_fields_and_field_types'):
            out += flatten(v, key + '.')
        elif isinstance(v, str):
            out.append((key, v))
        elif isinstance(v, bytes):
            out.append((key, v.hex()))
        elif hasattr(v, '__len__'):
            out += [(f'{key}_{i}', x) for i, x in enumerate(v)]
        else:
            out.append((key, v))
    return out


def cell(v):
    if isinstance(v, bool):
        return int(v)
    if isinstance(v, float):
        return f'{v:.7g}'
    if hasattr(v, 'item'):                      # numpy scalar from an array field
        v = v.item()
        return f'{v:.7g}' if isinstance(v, float) else v
    return v


class RawLog:
    """One CSV per topic, one row per message, header from the first message."""

    def __init__(self, path):
        self.path, self.f, self.w, self.n = path, None, None, 0

    def write(self, msg):
        row = [('recv_wall', time.time()), ('recv_t', time.monotonic())] + flatten(msg)
        if self.f is None:
            self.f = open(self.path, 'w', newline='')
            self.w = csv.writer(self.f)
            self.w.writerow([k for k, _ in row])
        self.w.writerow([f'{row[0][1]:.4f}', f'{row[1][1]:.4f}'] + [cell(v) for _, v in row[2:]])
        self.n += 1

    def flush(self):
        if self.f:
            self.f.flush()

    def close(self):
        if self.f:
            try:
                self.f.flush()
                self.f.close()
            except ValueError:
                pass


class WalkRecorder(Node):
    def __init__(self):
        super().__init__('walk_recorder')
        self.declare_parameter('out', '/root/team_ws/walk_sessions/track.csv')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('raw', True)
        self.path = str(self.get_parameter('out').value)
        self.f = open(self.path, 'w', newline='')
        self.w = csv.DictWriter(self.f, fieldnames=COLUMNS, extrasaction='ignore')
        self.w.writeheader()
        self.last_flush = time.monotonic()
        self.loc = self.loc_at = None
        self.status = {}
        self.odom = self.odom_at = self.odom_twist = None
        self.vo = None
        self.px4 = self.px4_at = None
        self.px4_extra, self.px4_status, self.battery_v = {}, {}, None
        self.vision_in_at = self.imu_at = None
        self.cloud_times = []
        self.raw = {}
        if bool(self.get_parameter('raw').value):
            rawdir = os.path.join(os.path.dirname(os.path.abspath(self.path)), 'raw')
            os.makedirs(rawdir, exist_ok=True)
            self.raw = {n: RawLog(os.path.join(rawdir, n + '.csv')) for n in RAW_TOPICS}
        self.create_subscription(PoseStamped, '/team/localize/base_pose', self.on_loc, 10)
        self.create_subscription(String, '/team/localize/status', self.on_status, 10)
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/team/stereo/points', self.on_cloud, 2)
        self.create_subscription(Imu, '/oak/imu/data', self.on_imu, qos_profile_sensor_data)
        if VisualSlamStatus is not None:
            self.create_subscription(VisualSlamStatus, '/visual_slam/status', self.on_vo, 10)
        if VehicleLocalPosition is not None:
            self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.on_px4, PX4_QOS)
            self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.on_px4_status, PX4_QOS)
            self.create_subscription(BatteryStatus, '/fmu/out/battery_status', self.on_battery, PX4_QOS)
            # what the bridge feeds EKF2; a best-effort subscriber only listens, it doesn't change the feed
            self.create_subscription(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', self.on_vision_in,
                                     qos_profile_sensor_data)
        else:
            self.get_logger().warn('px4_msgs not found: no PX4 data recorded')
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self.row)
        self.get_logger().info(f'recording to {self.path}' + (' (+ raw/)' if self.raw else ''))

    def log_raw(self, name, m):
        r = self.raw.get(name)
        if r is not None:
            r.write(m)

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
        v, w = m.twist.twist.linear, m.twist.twist.angular
        self.odom_twist = (v.x, v.y, v.z, w.z)
        self.log_raw('cuvslam_odometry', m)

    def on_vo(self, m):
        self.vo = int(m.vo_state)

    def on_px4(self, m):
        self.log_raw('px4_local_position', m)
        # NED -> ENU: east = y, north = x, up = -z
        self.px4_extra = {'px4_vx': m.vy, 'px4_vy': m.vx, 'px4_vz': -m.vz, 'px4_ax': m.ay, 'px4_ay': m.ax,
                          'px4_az': -m.az, 'px4_xy_valid': int(m.xy_valid), 'px4_z_valid': int(m.z_valid),
                          'px4_xy_resets': m.xy_reset_counter, 'px4_z_resets': m.z_reset_counter,
                          'px4_heading_resets': m.heading_reset_counter,
                          'px4_heading_good': int(m.heading_good_for_control),
                          'px4_dist_bottom': m.dist_bottom, 'px4_dist_bottom_valid': int(m.dist_bottom_valid),
                          'px4_eph': m.eph, 'px4_epv': m.epv, 'px4_dead_reckoning': int(m.dead_reckoning)}
        if m.xy_valid and m.z_valid and all(math.isfinite(v) for v in (m.x, m.y, m.z)):
            yaw = math.degrees(math.pi / 2 - m.heading) if math.isfinite(m.heading) else float('nan')
            self.px4, self.px4_at = (m.y, m.x, -m.z, (yaw + 180) % 360 - 180), time.monotonic()

    def on_px4_status(self, m):
        self.px4_status = {'px4_armed': int(m.arming_state == 2), 'px4_nav_state': int(m.nav_state)}
        self.log_raw('px4_status', m)

    def on_battery(self, m):
        self.battery_v = float(m.voltage_v)
        self.log_raw('px4_battery', m)

    def on_vision_in(self, m):
        self.vision_in_at = time.monotonic()
        self.log_raw('px4_vision_in', m)

    def on_imu(self, m):
        self.imu_at = time.monotonic()
        self.log_raw('oak_imu', m)

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
             'vo_state': self.vo, 'clouds_per_s': len(self.cloud_times), 'battery_v': self.battery_v,
             'vision_in_age': None if self.vision_in_at is None else now - self.vision_in_at,
             'imu_age': None if self.imu_at is None else now - self.imu_at}
        v.update(self.px4_extra)
        v.update(self.px4_status)
        if self.odom_twist is not None:
            v.update(dict(zip(('odom_vx', 'odom_vy', 'odom_vz', 'odom_wz'), self.odom_twist)))
        for name, val, at in (('loc', self.loc, self.loc_at), ('odom', self.odom, self.odom_at),
                              ('px4', self.px4, self.px4_at)):
            if val is not None:
                v.update({f'{name}_x': val[0], f'{name}_y': val[1], f'{name}_z': val[2],
                          f'{name}_yaw_deg': val[3], f'{name}_age': now - at})
        self.w.writerow({k: ('' if x is None or (isinstance(x, float) and not math.isfinite(x))
                             else round(x, 4) if isinstance(x, float) else x) for k, x in v.items()})
        if now - self.last_flush >= 1.0:
            self.f.flush()
            for r in self.raw.values():
                r.flush()
            self.last_flush = now

    def close(self):
        for r in self.raw.values():
            r.close()
        try:
            self.f.flush()
            self.f.close()
        except ValueError:
            pass
        print('walk_recorder raw rows: ' + ', '.join(f'{k} {r.n}' for k, r in self.raw.items()), flush=True)


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
