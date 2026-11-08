#!/usr/bin/env python3
"""Passive check of the same telemetry, limits and endpoints as the three missions."""
import argparse
import json
import math
import time
import rclpy
from px4_msgs.msg import VehicleStatus
from interface_monitor import InterfaceMonitor


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=30.)
    options, ros_args = parser.parse_known_args()
    if not math.isfinite(options.seconds) or options.seconds < 30:
        parser.error('--seconds must be at least 30')
    rclpy.init(args=ros_args)
    n = InterfaceMonitor()
    start = report = time.monotonic()
    since = None
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=.1)
            t, now = time.monotonic(), n.clock_seconds()
            problem = n.problem(now)
            if not problem and (n.status.arming_state != VehicleStatus.ARMING_STATE_DISARMED
                    or n.fi['armed'] or n.fi['offboard_active'] or not n.fi['landed']
                    or not n.landed or not 0 <= now-n.landed_time <= 1):
                problem = 'expected fresh disarmed and landed reports'
            since = None if problem else since if since is not None else t
            if t-report >= 1:
                print(json.dumps({'elapsed_s': round(t-start,2), 'reason': problem,
                                  'ready': problem is None}), flush=True)
                report = t
            if t-start >= options.seconds:
                passed = since is not None and t-since >= 5
                print(json.dumps({'passed': passed, 'flight_commands_sent': 0}), flush=True)
                return 0 if passed else 1
        return 1
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
