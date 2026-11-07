"""Passive 20-second ROS check; sends no messages or commands."""
import json
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image

rclpy.init()
node = rclpy.create_node("local_sim_passive_sensor_check")
samples = {}


def observe(topic, message):
    stamp = message.clock if isinstance(message, Clock) else message.header.stamp
    seconds = stamp.sec + stamp.nanosec * 1e-9
    row = samples.setdefault(topic, {"count": 0, "first_stamp": seconds})
    row["count"] += 1
    row["last_stamp"] = seconds
    if isinstance(message, (Image, CameraInfo)):
        row.update(width=message.width, height=message.height,
                   frame_id=message.header.frame_id)
    if isinstance(message, Image):
        row.update(encoding=message.encoding, bytes=len(message.data),
                   pixel_std=float(np.frombuffer(message.data, dtype=np.uint8).std()))
    elif isinstance(message, CameraInfo):
        row["K"] = list(message.k)


expected = {"/clock": Clock}
for side in ("left", "right"):
    expected[f"/oak/{side}/image_raw"] = Image
    expected[f"/oak/{side}/camera_info"] = CameraInfo
subscriptions = [node.create_subscription(
    kind, topic, lambda msg, topic=topic: observe(topic, msg), qos_profile_sensor_data
) for topic, kind in expected.items()]
start = time.monotonic()
while time.monotonic() - start < 20:
    rclpy.spin_once(node, timeout_sec=0.1)
ok = all(topic in samples and samples[topic]["count"] >= 2
         and samples[topic]["last_stamp"] > samples[topic]["first_stamp"]
         for topic in expected)
for topic, kind in expected.items():
    if kind is not Clock:
        row = samples.get(topic, {})
        ok = ok and (row.get("width"), row.get("height")) == (1024, 640)
print(json.dumps({"passed": ok, "wall_seconds": time.monotonic() - start,
                  "topics": samples}, indent=2))
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if ok else 1)
