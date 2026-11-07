#!/usr/bin/env python3
"""SIMULATION SENSOR MODEL ONLY: Gazebo truth -> noisy downward laser reading.

Uses collision ray casting from the existing team sensor model. It never emits
visual odometry, position fixes, commands or control setpoints. Assumptions:
ideal single ray, measured mount, Gaussian 1 cm noise, 0.02–4 m range, 40 Hz.
"""
import math
import os
import sys
import threading
import time

import numpy as np
os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil
sys.path.insert(0, '/workspace/Navless/gazebo_sim')
from dh.geom import RayCaster
from sensor_bridge import quat_to_mat, RNG_OFFSET_FLU
from gz.transport13 import Node
from gz.msgs10.odometry_pb2 import Odometry


def main():
    node, lock, state = Node(), threading.Lock(), {}
    def capture(msg):
        q, p = msg.pose.orientation, msg.pose.position
        with lock:
            state.update(t=msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9,
                         p=np.array([p.x, p.y, p.z]), R=quat_to_mat(q.w, q.x, q.y, q.z))
    node.subscribe(Odometry, '/model/dh_quad_0/odometry', capture)
    mav = mavutil.mavlink_connection('udpout:127.0.0.1:14280', source_system=2, source_component=197)
    caster, noise, last = RayCaster(), np.random.default_rng(19), -1.0
    while True:
        with lock:
            sample = dict(state)
        if not sample or sample['t'] - last < 1 / 40:
            time.sleep(0.002)
            continue
        last, p, R = sample['t'], sample['p'], sample['R']
        distance = caster.cast(p + R @ RNG_OFFSET_FLU, R @ np.array([0., 0., -1.]))
        if math.isfinite(distance):
            distance += noise.normal(0, 0.01)
        cm = round(min(max(distance, 0.02), 4.05) * 100)
        mav.mav.distance_sensor_send(
            int(last * 1000) & 0xffffffff, 2, 400, cm,
            mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, 0,
            mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, 1,
            0, 0, [0, 0, 0, 0], 100)


if __name__ == '__main__':
    main()
