#!/usr/bin/env python3
"""Flight-day checks for fly_course.sh. Runs INSIDE the skybeta-stack container (rclpy directly, so a stuck
`ros2` CLI daemon can't break it). Read-only: it only subscribes. Prints one JSON line and exits.

    python3 fly_check.py odom   [timeout_s]   -> {"ok":..,"x":..,"y":..,"z":..,"yaw_deg":..}
    python3 fly_check.py px4    [timeout_s]   -> {"ok":..} (vehicle_local_position arriving)
    python3 fly_check.py status [timeout_s]   -> cloud_localize's status JSON (+ "ok")
    python3 fly_check.py corr   [timeout_s]   -> odom_correction dx, dy, dyaw (+ "ok"; ok false = none published)
    python3 fly_check.py hz     [seconds]     -> {"ok":..,"rate_hz":..,"max_gap_s":..} for cuVSLAM odometry
"""
import json
import math
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data


def yaw_deg(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def first(node, msg_type, topic, qos, timeout):
    got = []
    node.create_subscription(msg_type, topic, lambda m: got.append(m), qos)
    t0 = time.monotonic()
    while not got and time.monotonic() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    return got[0] if got else None


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else 'odom'
    arg = float(sys.argv[2]) if len(sys.argv) > 2 else (12.0 if what == 'hz' else 6.0)
    rclpy.init()
    node = rclpy.create_node('fly_check_' + what)
    out = {'ok': False}
    try:
        if what == 'odom':
            from nav_msgs.msg import Odometry
            m = first(node, Odometry, '/visual_slam/tracking/odometry', qos_profile_sensor_data, arg)
            if m is not None:
                p = m.pose.pose.position
                out = {'ok': True, 'x': round(p.x, 3), 'y': round(p.y, 3), 'z': round(p.z, 3),
                       'yaw_deg': round(yaw_deg(m.pose.pose.orientation), 1)}
        elif what == 'px4':
            from px4_msgs.msg import VehicleLocalPosition
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
            m = first(node, VehicleLocalPosition, '/fmu/out/vehicle_local_position', qos, arg)
            if m is not None:
                out = {'ok': True, 'xy_valid': bool(m.xy_valid), 'z_valid': bool(m.z_valid),
                       'heading_deg': round(math.degrees(m.heading), 1) if math.isfinite(m.heading) else None,
                       'heading_resets': int(m.heading_reset_counter)}
        elif what == 'status':
            from std_msgs.msg import String
            m = first(node, String, '/team/localize/status', 10, arg)
            if m is not None:
                out = json.loads(m.data)
                out.pop('trail', None)
                out['ok'] = True
        elif what == 'corr':
            from geometry_msgs.msg import PoseStamped
            m = first(node, PoseStamped, '/team/localize/odom_correction', 5, arg)
            if m is not None:
                p = m.pose.position
                out = {'ok': True, 'dx': round(p.x, 3), 'dy': round(p.y, 3),
                       'dyaw_deg': round(yaw_deg(m.pose.orientation), 2)}
        elif what == 'hz':
            from nav_msgs.msg import Odometry
            stamps = []
            node.create_subscription(Odometry, '/visual_slam/tracking/odometry',
                                     lambda m: stamps.append(time.monotonic()), qos_profile_sensor_data)
            t0 = time.monotonic()
            while time.monotonic() - t0 < arg:
                rclpy.spin_once(node, timeout_sec=0.05)
            if len(stamps) > 2:
                gaps = [b - a for a, b in zip(stamps, stamps[1:])]
                out = {'ok': True, 'rate_hz': round((len(stamps) - 1) / (stamps[-1] - stamps[0]), 1),
                       'max_gap_s': round(max(gaps), 3)}
        else:
            out = {'ok': False, 'error': f'unknown check {what!r}'}
    except Exception as e:                       # never hang or crash the shell script
        out = {'ok': False, 'error': f'{type(e).__name__}: {e}'}
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps(out))


if __name__ == '__main__':
    main()
