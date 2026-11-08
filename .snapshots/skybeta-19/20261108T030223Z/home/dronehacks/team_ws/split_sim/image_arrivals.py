#!/usr/bin/env python3
"""Passive camera arrival timestamps for matching frames across two machines."""
import json
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

rclpy.init(args=[])
node = rclpy.create_node('split_camera_arrival_probe')
rows = []
def capture(msg):
    rows.append({'stamp_ns': msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec,
                 'wall_s': time.time(), 'bytes': len(msg.data)})
sub = node.create_subscription(Image, '/oak/left/image_raw', capture, qos_profile_sensor_data)
started = time.monotonic()
while time.monotonic() - started < 20:
    rclpy.spin_once(node, timeout_sec=0.1)
print(json.dumps(rows))
node.destroy_node()
rclpy.try_shutdown()
