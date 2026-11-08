#!/usr/bin/env python3
"""Read-only ROS SLAM viewer and validated odometry relay. No flight commands.

Run inside the existing Sky Beta container with its underlay sourced. HTTP binds
to loopback: use SSH forwarding. cuVSLAM's map is a sparse landmark map, not an
occupancy map. Camera/IMU calibration and the original body TF remain untouched.
"""
import json
import math
import os
from pathlib import Path
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from isaac_ros_visual_slam_interfaces.srv import FilePath
from px4_msgs.msg import VehicleOdometry, VehicleStatus, VehicleLocalPosition, EstimatorStatusFlags
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parent
MAPS = ROOT / 'maps'


def valid_odometry(msg, now, tracking, status_age):
    """Fail closed on stale/lost tracking, wrong frames or invalid estimates."""
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if not tracking or status_age > 0.25:
        return False
    if not -0.05 <= now - stamp <= 0.25:
        return False
    if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
        return False
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    v, w = msg.twist.twist.linear, msg.twist.twist.angular
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
              v.x, v.y, v.z, w.x, w.y, w.z]
    if not all(math.isfinite(x) for x in values):
        return False
    if abs(sum(x*x for x in [q.x, q.y, q.z, q.w]) - 1) > 0.02:
        return False
    for cov in (msg.pose.covariance, msg.twist.covariance):
        if not all(math.isfinite(x) for x in cov):
            return False
        if any(cov[i] < 0 for i in (0, 7, 14, 21, 28, 35)):
            return False
    # cuVSLAM's identity is the documented covariance-unavailable sentinel.
    if all(msg.pose.covariance[i] == 1 for i in (0, 7, 14, 21, 28, 35)):
        return False
    return True


class MapNode(Node):
    def __init__(self):
        super().__init__('handheld_slam_viewer')
        self.lock = threading.RLock()
        self.data = {'clouds': {}, 'path': [], 'pose': None, 'tracking': False,
                     'accepted': 0, 'rejected': 0, 'saved': None, 'camera_seq': 0}
        self.times = {}
        self.jpeg = b''
        self.last_image = 0
        self.subs = []
        self.status_time = 0
        self.tracking = False
        self.last_path = 0
        self.subscribe(String, '/mapping/gate_status', self.gate_status)
        self.subscribe(VisualSlamStatus, '/visual_slam/status', self.status)
        self.subscribe(Odometry, '/visual_slam/tracking/odometry', self.odom)
        # A 10k-pose path at 30 Hz is expensive to deserialize in Python. Receive
        # bytes and deserialize only the 1 Hz display sample, outside the relay.
        self.subs.append(self.create_subscription(RosPath,
            '/visual_slam/tracking/slam_path', self.raw_path,
            qos_profile_sensor_data, raw=True))
        self.subscribe(Odometry, '/visual_slam/vis/slam_odometry', self.slam_pose)
        for name in ('landmarks_cloud', 'loop_closure_cloud', 'localizer_map_cloud'):
            self.subscribe(PointCloud2, '/visual_slam/vis/' + name,
                           lambda m, n=name: self.cloud(m, n))
        self.subscribe(Image, '/oak/left/image_raw', self.image)
        self.subscribe(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', self.ev)
        self.subscribe(VehicleStatus, '/fmu/out/vehicle_status_v1', self.fc)
        self.subscribe(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local)
        self.subscribe(EstimatorStatusFlags, '/fmu/out/estimator_status_flags', self.ekf)
        self.save_client = self.create_client(FilePath, '/visual_slam/save_map')
        self.pending_save = None
        self.create_timer(1.0, self.save_tick)

    def subscribe(self, typ, topic, cb):
        self.subs.append(self.create_subscription(typ, topic, cb, qos_profile_sensor_data))

    def touch(self, name):
        self.times[name] = time.monotonic()

    def status(self, msg):
        with self.lock:
            self.tracking = msg.vo_state == 1
            self.status_time = time.monotonic()
            self.data['tracking'] = self.tracking
            self.data['track_ms'] = msg.track_execution_time * 1000
            self.touch('tracking')

    def odom(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        p = msg.pose.pose.position
        with self.lock:
            self.data['odom_pose'] = [p.x, p.y, p.z]
            self.data['odom_frames'] = [msg.header.frame_id, msg.child_frame_id]
            self.data['sample_age_ms'] = round(1000*(now-msg.header.stamp.sec-msg.header.stamp.nanosec*1e-9), 1)
            self.touch('odometry')

    def gate_status(self, msg):
        with self.lock:
            self.data['gate'] = json.loads(msg.data)
            self.data['accepted'] = self.data['gate']['accepted']
            self.data['rejected'] = self.data['gate']['rejected']
            self.touch('gate')

    def slam_pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        with self.lock:
            self.data['pose'] = [p.x, p.y, p.z]
            self.data['orientation'] = [q.x, q.y, q.z, q.w]
            self.touch('slam_pose')

    def path(self, msg):
        if time.monotonic() - self.last_path < 0.7:
            return
        self.last_path = time.monotonic()
        points = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses]
        with self.lock:
            self.data['path'] = points
            self.data['path_frame'] = msg.header.frame_id
            self.touch('path')

    def raw_path(self, data):
        if time.monotonic() - self.last_path >= 0.7:
            self.path(deserialize_message(data, RosPath))

    def cloud(self, msg, name):
        if time.monotonic() - self.times.get(name, 0) < 0.7:
            return
        # Preserve the publisher's map coordinates; never invent geometry.
        raw = point_cloud2.read_points(msg, field_names=('x','y','z'), skip_nans=True)
        points = np.array([[float(p[0]), float(p[1]), float(p[2])] for p in raw])
        if points.size:
            points = points[np.isfinite(points).all(axis=1)]
            # Bound transfer size, while preserving uniform coverage.
            points = points[::max(1, math.ceil(len(points)/35000))]
        with self.lock:
            self.data['clouds'][name] = {'frame': msg.header.frame_id,
                'points': points.round(4).tolist(), 'total': msg.width*msg.height}
            self.touch(name)

    def image(self, msg):
        now = time.monotonic()
        if now-self.last_image < 0.5:
            return
        self.last_image = now
        if msg.encoding not in ('bgr8', 'rgb8', 'mono8'):
            return
        channels = 1 if msg.encoding == 'mono8' else 3
        rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        frame = rows[:, :msg.width*channels].reshape(msg.height, msg.width, channels)
        if channels == 1:
            frame = frame[:, :, 0]
        elif msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        frame = cv2.resize(frame, (512, 320))
        ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            with self.lock:
                self.jpeg = jpg.tobytes()
                self.data['camera_seq'] += 1
                self.touch('camera')

    def ev(self, msg):
        with self.lock:
            self.data['ev'] = {'position': [float(x) for x in msg.position], 'pose_frame': msg.pose_frame,
                               'reset_counter': msg.reset_counter, 'timestamp_sample': msg.timestamp_sample}
            self.touch('ev_published')

    def fc(self, msg):
        with self.lock:
            self.data['fc'] = {'armed': msg.arming_state == 2, 'nav_state': msg.nav_state,
                               'failsafe': msg.failsafe}
            self.touch('px4')

    def local(self, msg):
        with self.lock:
            self.data['local_position'] = {'position': [msg.x,msg.y,msg.z],
                'xy_valid': msg.xy_valid, 'z_valid': msg.z_valid,
                'eph': msg.eph, 'epv': msg.epv}
            self.touch('local_position')

    def ekf(self, msg):
        with self.lock:
            self.data['ekf'] = {'vision_xy':msg.cs_ev_pos, 'vision_yaw':msg.cs_ev_yaw,
                                'vision_height':msg.cs_ev_hgt, 'vision_velocity':msg.cs_ev_vel}
            self.touch('ekf')

    def snapshot(self):
        with self.lock:
            d = dict(self.data)
            d['ages'] = {k: round(time.monotonic()-v, 3) for k,v in self.times.items()}
            return d

    def request_save(self):
        with self.lock:
            if self.pending_save:
                return {'error': 'A save is already in progress'}
            if not self.save_client.service_is_ready():
                return {'error': 'cuVSLAM save service unavailable'}
            folder = MAPS / time.strftime('%Y%m%d-%H%M%S')
            folder.mkdir(parents=True, exist_ok=False)
            snapshot = self.snapshot()
            (folder/'snapshot.json').write_text(json.dumps(snapshot, allow_nan=False))
            pts = snapshot['clouds'].get('landmarks_cloud', {}).get('points', [])
            with (folder/'landmarks.ply').open('w') as f:
                f.write('ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nend_header\n' % len(pts))
                for p in pts:
                    f.write('%s %s %s\n' % tuple(p))
            req = FilePath.Request()
            req.file_path = str(folder/'cuvslam')
            future = self.save_client.call_async(req)
            self.pending_save = (future, folder, time.monotonic())
            self.data['saved'] = {'state': 'saving', 'folder': str(folder)}
            return self.data['saved']

    def save_tick(self):
        with self.lock:
            if self.pending_save:
                future, folder, started = self.pending_save
                if future.done():
                    try:
                        result = future.result()
                        ok = result.success
                        self.data['saved'] = {'state': 'saved' if ok else 'failed', 'folder': str(folder)}
                    except Exception as exc:
                        self.data['saved'] = {'state':'failed', 'error': str(exc), 'folder':str(folder)}
                    (folder/'save_result.json').write_text(json.dumps(self.data['saved']))
                    self.pending_save = None
                elif time.monotonic()-started > 60:
                    future.cancel()
                    self.data['saved'] = {'state':'failed', 'error':'Save timed out', 'folder':str(folder)}
                    self.pending_save = None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def send(self, code, content_type, data):
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        route = self.path.split('?')[0]
        if route == '/':
            self.send(200, 'text/html; charset=utf-8', (ROOT/'viewer.html').read_bytes())
        elif route == '/state':
            self.send(200, 'application/json', json.dumps(self.server.node.snapshot(), allow_nan=False).encode())
        elif route == '/camera.jpg':
            self.send(200, 'image/jpeg', self.server.node.jpeg)
        elif route == '/download':
            self.send(200, 'application/json', json.dumps(self.server.node.snapshot(), allow_nan=False).encode())
        else:
            self.send(404, 'text/plain', b'Not found')

    def do_POST(self):
        # Only local viewer requests; never expose a generic ROS command API.
        if self.headers.get('Origin') and self.headers['Origin'] != 'http://'+self.headers.get('Host',''):
            self.send(403, 'text/plain', b'Origin rejected')
            return
        if self.path == '/save':
            self.send(200, 'application/json', json.dumps(self.server.node.request_save()).encode())
        else:
            self.send(404, 'text/plain', b'Not found')


def main():
    rclpy.init()
    node = MapNode()
    server = ThreadingHTTPServer(('127.0.0.1', 8765), Handler)
    server.node = node
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info('SLAM viewer: http://127.0.0.1:8765 (SSH forward required)')
    try:
        rclpy.spin(node)
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
