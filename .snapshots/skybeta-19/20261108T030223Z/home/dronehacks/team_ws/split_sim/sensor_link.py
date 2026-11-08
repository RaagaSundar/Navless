#!/usr/bin/env python3
"""Allowlisted ROS sensor transport over an SSH-forwarded TCP connection.

CDR preserves standard ROS messages across the Humble/Jazzy boundary. Only the
declared sensor topics can travel; no commands, truth or PX4 input topics.
Both ends must run in isolated ROS domain 219. No credentials live here.
"""
import argparse
import collections
import json
import os
import socket
import struct
import threading
import time
import zlib

TOPICS = [
    ('/clock', 'rosgraph_msgs/msg/Clock'),
    ('/oak/left/image_raw', 'sensor_msgs/msg/Image'),
    ('/oak/right/image_raw', 'sensor_msgs/msg/Image'),
    ('/oak/left/camera_info', 'sensor_msgs/msg/CameraInfo'),
    ('/oak/right/camera_info', 'sensor_msgs/msg/CameraInfo'),
    ('/split/downward_range', 'sensor_msgs/msg/Range'),
]
MAX_BYTES = 4 * 1024 * 1024
HEADER = struct.Struct('!BI')


def encode(index, payload):
    if index not in range(len(TOPICS)) or len(payload) > MAX_BYTES:
        raise ValueError('invalid topic or oversized message')
    body = zlib.compress(payload, 1)
    if len(body) > MAX_BYTES:
        raise ValueError('oversized wire message')
    return HEADER.pack(index, len(body)) + body


def decode(index, body):
    if index not in range(len(TOPICS)) or not 0 < len(body) <= MAX_BYTES:
        raise ValueError('invalid topic or wire size')
    decompressor = zlib.decompressobj()
    result = decompressor.decompress(body, MAX_BYTES + 1)
    if len(result) > MAX_BYTES or not decompressor.eof or decompressor.unused_data:
        raise ValueError('invalid or oversized compressed message')
    return result


def read_exact(conn, count):
    result = bytearray()
    while len(result) < count:
        part = conn.recv(count - len(result))
        if not part:
            raise ConnectionError('sensor link closed')
        result.extend(part)
    return bytes(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role', choices=['laptop', 'jetson'])
    parser.add_argument('--port', type=int, default=7447)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '219':
        raise SystemExit('Refusing to run outside isolated ROS_DOMAIN_ID=219')
    import rclpy
    from rclpy.serialization import serialize_message, deserialize_message
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message
    rclpy.init(args=[])
    node = rclpy.create_node('split_sensor_' + args.role)
    types = [get_message(type_name) for _, type_name in TOPICS]
    pending = collections.OrderedDict()
    condition = threading.Condition()
    counts = collections.Counter()
    metrics = open('/tmp/split_link_metrics.jsonl', 'a', buffering=1)
    stop = threading.Event()
    publishers = []
    subscriptions = []

    if args.role == 'laptop':
        def capture(index, msg):
            # One latest message per topic bounds latency and memory during outages.
            # Keep CameraInfo for the next connection; no stale image is replayed.
            with condition:
                pending.pop(index, None)
                stamp = (msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
                         if hasattr(msg, 'header') else None)
                pending[index] = (serialize_message(msg), stamp, time.time())
                condition.notify()
        for index, (topic, _) in enumerate(TOPICS):
            subscriptions.append(node.create_subscription(
                types[index], topic, lambda msg, i=index: capture(i, msg),
                qos_profile_sensor_data))

        def worker():
            server = socket.socket()
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('0.0.0.0', args.port))  # Docker publishes loopback on the Mac.
            server.listen(1)
            server.settimeout(1)
            while not stop.is_set():
                try:
                    conn, _ = server.accept()
                except socket.timeout:
                    continue
                conn.settimeout(5)
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print('SSH sensor connection accepted', flush=True)
                try:
                    while not stop.is_set():
                        with condition:
                            if not pending:
                                condition.wait(0.2)
                                continue
                            index, (payload, stamp, captured) = pending.popitem(last=False)
                        packet = encode(index, payload)
                        sent = time.time()
                        conn.sendall(packet)
                        if index in (1, 2):
                            metrics.write(json.dumps({'role': 'send', 'index': index,
                                'stamp_ns': stamp, 'captured_s': captured,
                                'send_start_s': sent, 'send_done_s': time.time(),
                                'wire_bytes': len(packet)}) + '\n')
                        counts[TOPICS[index][0]] += 1
                except (OSError, ValueError) as error:
                    print(str(error), flush=True)
                finally:
                    conn.close()
            server.close()
    else:
        for index, (topic, _) in enumerate(TOPICS):
            # Simulated camera is already mono8; expose the normal raw topic for
            # downstream depth processing and the SLAM mono endpoint as well.
            pubs = [node.create_publisher(types[index], topic, 10)]
            if index in (1, 2):
                pubs.append(node.create_publisher(
                    types[index], topic.replace('image_raw', 'image_mono'), 10))
            publishers.append(pubs)

        def worker():
            while not stop.is_set():
                try:
                    with socket.create_connection(('127.0.0.1', args.port), timeout=5) as conn:
                        conn.settimeout(10)
                        while not stop.is_set():
                            index, size = HEADER.unpack(read_exact(conn, HEADER.size))
                            if index not in range(len(TOPICS)) or not 0 < size <= MAX_BYTES:
                                raise ValueError('invalid packet header')
                            body = read_exact(conn, size)
                            received = time.time()
                            msg = deserialize_message(decode(index, body), types[index])
                            if index in (1, 2):
                                stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
                                metrics.write(json.dumps({'role': 'receive', 'index': index,
                                    'stamp_ns': stamp, 'received_s': received,
                                    'publish_start_s': time.time(),
                                    'wire_bytes': size + HEADER.size}) + '\n')
                            for pub in publishers[index]:
                                pub.publish(msg)
                            counts[TOPICS[index][0]] += 1
                except (OSError, ValueError, RuntimeError) as error:
                    print('sensor link: ' + str(error), flush=True)
                    stop.wait(1)

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    node.create_timer(5, lambda: print(json.dumps(dict(counts)), flush=True))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        metrics.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
