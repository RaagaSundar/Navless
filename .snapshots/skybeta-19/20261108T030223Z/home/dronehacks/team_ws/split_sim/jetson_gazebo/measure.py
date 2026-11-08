#!/usr/bin/env python3
"""Passive wall-time throughput and same-host callback latency measurement.

Image-to-odom is callback arrival delta for equal source stamps, NOT render-to-
odom or network latency. /clock age uses simulation time and is labeled as such.
"""
import argparse
from collections import Counter
import json
import statistics
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus


def stamp(msg):
    return msg.sec * 1000000000 + msg.nanosec


def summary(values):
    if not values:
        return {'n': 0}
    values = sorted(values)
    return {'n': len(values), 'median': statistics.median(values),
            'p95': values[min(len(values)-1, int(len(values)*.95))],
            'min': values[0], 'max': values[-1]}


class Measure(Node):
    def __init__(self):
        super().__init__('passive_gazebo_benchmark')
        self.images = {}
        self.odoms = {}
        self.arrivals = {'image': [], 'odom': [], 'validated': []}
        self.sim_stamps = {'image': [], 'odom': []}
        self.states = Counter()
        self.gates = []
        self.clocks = []
        self.subs = [
            self.create_subscription(Image, '/oak/left/image_mono', self.image, qos_profile_sensor_data),
            self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.odom, qos_profile_sensor_data),
            self.create_subscription(Odometry, '/mapping/validated_odometry', self.validated, qos_profile_sensor_data),
            self.create_subscription(VisualSlamStatus, '/visual_slam/status', self.status, qos_profile_sensor_data),
            self.create_subscription(String, '/mapping/gate_status', self.gate, 10),
            self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)]

    def image(self, msg):
        now = time.monotonic()
        key = stamp(msg.header.stamp)
        self.images[key] = now
        self.arrivals['image'].append(now)
        self.sim_stamps['image'].append(key)

    def odom(self, msg):
        now = time.monotonic()
        key = stamp(msg.header.stamp)
        self.odoms[key] = now
        self.arrivals['odom'].append(now)
        self.sim_stamps['odom'].append(key)

    def validated(self, msg):
        self.arrivals['validated'].append(time.monotonic())

    def status(self, msg):
        self.states[msg.vo_state] += 1

    def gate(self, msg):
        self.gates.append(json.loads(msg.data))

    def clock(self, msg):
        self.clocks.append((time.monotonic(), stamp(msg.clock)*1e-9))

    def result(self, seconds):
        latency = [(self.odoms[k]-v)*1000 for k,v in self.images.items() if k in self.odoms]
        clock_rate = None
        if len(self.clocks)>1:
            a,b=self.clocks[0],self.clocks[-1]
            clock_rate=(b[1]-a[1])/(b[0]-a[0])
        return {
            'duration_wall_s': seconds,
            'counts': {k:len(v) for k,v in self.arrivals.items()},
            'rates_wall_hz': {k:(len(v)-1)/(v[-1]-v[0]) if len(v)>1 else 0 for k,v in self.arrivals.items()},
            'rates_sim_hz': {k:(len(v)-1)*1e9/(v[-1]-v[0]) if len(v)>1 and v[-1]>v[0] else 0 for k,v in self.sim_stamps.items()},
            'same_stamp_image_to_odom_callback_ms': summary(latency),
            'simulation_to_wall_clock_rate': clock_rate,
            'tracking_vo_state_counts': dict(self.states),
            'gate_simulation_age_ms': summary([x['sample_age_ms'] for x in self.gates if x.get('sample_age_ms') is not None]),
            'first_gate':self.gates[0] if self.gates else None,
            'last_gate':self.gates[-1] if self.gates else None,
            'measurement_note': 'Callback delta is same-host arrival time for matching image/odom stamps, not render-to-odom or network latency. Gate age is simulation-clock time.'}


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--seconds',type=float,default=35)
    args=parser.parse_args()
    rclpy.init()
    node=Measure()
    started=time.monotonic()
    try:
        while time.monotonic()-started<args.seconds:
            rclpy.spin_once(node,timeout_sec=.05)
        print(json.dumps(node.result(time.monotonic()-started),indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
