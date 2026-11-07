"""Ordered original body IMU measurements for the simulation-only stereo-inertial profile."""
import argparse
import math
import os
import queue
import socket
import struct
import threading
import time

PACKET = struct.Struct('!Q6d')
FRAME = 'dh_quad_0::base_link::imu_sensor'
TOPIC = '/world/dronehacks/model/dh_quad_0/link/base_link/sensor/imu_sensor/imu'
ROS_TOPIC = '/split_camera/body_imu'


def unpack(data, previous):
    stamp, *axes = PACKET.unpack(data)
    if stamp <= 0 or not all(map(math.isfinite, axes)):
        raise ValueError('invalid inertial measurement')
    if previous is not None and stamp - previous != 4_000_000:
        raise ValueError(f'noncontiguous 250 Hz IMU stamps: {previous} -> {stamp}')
    return stamp, axes


def sender():
    from gz.transport13 import Node
    from gz.msgs10.imu_pb2 import IMU

    if os.environ.get('GZ_PARTITION') != 'drone19_split':
        raise RuntimeError('wrong Gazebo partition')
    pending = queue.Queue(256)
    enabled = threading.Event()
    invalid = threading.Event()
    def capture(msg):
        if not enabled.is_set():
            return
        frames = [v for d in msg.header.data if d.key == 'frame_id' for v in d.value]
        if frames != [FRAME] or msg.entity_name != FRAME:
            invalid.set()
            return
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nsec
        axes = [getattr(v, dim) for v in (msg.angular_velocity, msg.linear_acceleration) for dim in 'xyz']
        try:
            pending.put_nowait(PACKET.pack(stamp, *axes))
        except queue.Full:
            invalid.set()

    node = Node()
    if not node.subscribe(IMU, TOPIC, capture):
        raise RuntimeError('cannot subscribe to original body IMU')
    with socket.socket() as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0',7448))
        server.listen(1)
        while True:
            conn, _ = server.accept()
            with conn:
                conn.settimeout(2)
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                invalid.clear()
                while not pending.empty():
                    pending.get_nowait()
                previous = None
                enabled.set()
                try:
                    while not invalid.is_set():
                        data = pending.get(timeout=30 if previous is None else 2)
                        previous, _ = unpack(data, previous)
                        conn.sendall(data)
                except (OSError, ValueError, queue.Empty) as error:
                    print(f'IMU stream closed: {error}', flush=True)
                finally:
                    enabled.clear()


def receiver():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu
    rclpy.init()
    node = rclpy.create_node('split_ordered_body_imu')
    qos = QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(Imu, ROS_TOPIC, qos)
    deadline = time.monotonic() + 60
    # The SSH forwarding socket may accept before the renderer endpoint binds.
    # Retry only before receiving the first real packet. Never join a broken
    # live stream across a gap or fabricate missing samples.
    while time.monotonic() < deadline:
        conn = data = None
        try:
            conn = socket.create_connection(('127.0.0.1',17448),timeout=2)
            conn.settimeout(5)
            data = conn.makefile('rb')
            packet = data.read(PACKET.size)
            if len(packet) != PACKET.size:
                raise ConnectionError('renderer not ready')
            break
        except OSError:
            if data: data.close()
            if conn: conn.close()
            time.sleep(.25)
    else:
        raise RuntimeError('no original IMU received within startup deadline')
    count = 0
    previous = None
    with conn, data:
        while rclpy.ok():
            if len(packet) != PACKET.size:
                raise RuntimeError('ordered IMU link closed')
            stamp, axes = unpack(packet, previous)
            if previous is None:
                conn.settimeout(2)
            previous = stamp
            msg = Imu()
            msg.header.frame_id = 'base_link'
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
            msg.orientation_covariance[0] = -1.
            w, a = msg.angular_velocity, msg.linear_acceleration
            w.x,w.y,w.z,a.x,a.y,a.z = axes
            pub.publish(msg)
            count += 1
            if count % 2500 == 0:
                print(f'{count} source samples; last stamp {stamp}',flush=True)
            packet = data.read(PACKET.size)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('role', choices=('sender','receiver'))
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '219':
        parser.error('requires split simulation domain219')
    (sender if args.role == 'sender' else receiver)()
