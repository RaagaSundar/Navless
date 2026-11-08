#!/usr/bin/env python3
"""Passive 30+ second readiness check. Creates subscriptions, never flight publishers."""
import argparse
import json
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent / 'examples'))
from hover_land import TestFlight, ros_arguments
import rclpy


class NoPublish:
    def publish(self, msg):
        raise RuntimeError('Passive checker attempted publication')


class OdometryCheck(TestFlight):
    def create_publisher(self, *args, **kwargs):
        # TestFlight's three control publishers are inert objects: no DDS writers.
        return NoPublish()

    def __init__(self, seconds):
        super().__init__()
        self.seconds = seconds
        self.started = time.monotonic()
        self.ready_since = None
        self.last_report = -math.inf
        self.passed = False

    def problem(self):
        now = self.clock_seconds()
        if self.pos is None or self.status is None:
            return 'PX4 telemetry missing'
        if not 0 <= now-self.pos_time <= 1.0 or not 0 <= now-self.status_time <= 1.0:
            return 'PX4 telemetry stale'
        if self.status.arming_state != self.status.ARMING_STATE_DISARMED:
            return 'vehicle is not disarmed'
        if self.status.failsafe:
            return 'PX4 failsafe is active'
        if self.landed is not True or not 0 <= now-self.landed_time <= 1.0:
            return 'fresh landed state missing'
        if not self.position_ready(now):
            return 'position/heading invalid'
        return self.vision_problem(now)

    def tick(self):
        wall = time.monotonic()
        problem = self.problem()
        if problem is not None:
            self.ready_since = None
        elif self.ready_since is None:
            self.ready_since = wall
        if wall-self.last_report >= 1.0:
            print(json.dumps({'elapsed_s': round(wall-self.started, 2), 'ready': problem is None,
                              'reason': problem, 'kind': 'passive preflight; no flight commands'}), flush=True)
            self.last_report = wall
        if wall-self.started >= self.seconds:
            self.passed = self.ready_since is not None and wall-self.ready_since >= 5.0
            self.phase = 'DONE' if self.passed else 'ABORT'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.0)
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.seconds) or args.seconds < 30:
        parser.error('--seconds must be finite and at least 30')
    sys.argv = [sys.argv[0], *ros_args]
    rclpy.init(args=ros_arguments())
    node = OdometryCheck(args.seconds)
    try:
        while rclpy.ok() and node.phase not in ('DONE', 'ABORT'):
            rclpy.spin_once(node, timeout_sec=.1)
            # The wall deadline also bounds a stopped /clock; no timer or control fallback.
            if time.monotonic()-node.started > args.seconds+2:
                node.phase = 'ABORT'
    except KeyboardInterrupt:
        node.phase = 'ABORT'
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps({'passed': node.passed, 'flight_commands_sent': 0}), flush=True)
    return 0 if node.passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
