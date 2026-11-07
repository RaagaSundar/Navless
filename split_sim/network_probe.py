#!/usr/bin/env python3
"""Bounded timestamp echo probe; no ROS, control topics or aircraft interfaces.

Small-packet four-timestamp exchanges estimate clock offset. Bulk probes measure
SSH/TCP upload time separately from that offset. One-way estimates assume the
minimum-RTT small-packet path is symmetric; this is an estimate, not a trace of
the actual camera messages. Run through a loopback SSH port forward only.
"""
import argparse
import json
import os
import socket
import statistics
import struct
import time

HEADER = struct.Struct('!I')
TIMES = struct.Struct('!dd')


def read_exact(conn, length):
    result = bytearray()
    while len(result) < length:
        part = conn.recv(length - len(result))
        if not part:
            raise EOFError()
        result.extend(part)
    return result


def summary(values):
    values = sorted(values)
    return {'min': values[0], 'median': statistics.median(values),
            'p95': values[min(len(values)-1, int(len(values)*0.95))], 'max': values[-1]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role', choices=['server', 'client'])
    parser.add_argument('--port', type=int, default=17449)
    parser.add_argument('--host', default='127.0.0.1')
    args = parser.parse_args()
    if args.role == 'server':
        with socket.socket() as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('127.0.0.1', args.port))
            server.listen(1)
            server.settimeout(60)
            conn, _ = server.accept()
            with conn:
                conn.settimeout(15)
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                try:
                    while True:
                        length, = HEADER.unpack(read_exact(conn, HEADER.size))
                        if not 0 < length <= 1024*1024:
                            raise ValueError('invalid probe size')
                        read_exact(conn, length)
                        received = time.time()
                        sent = time.time()
                        conn.sendall(TIMES.pack(received, sent))
                except EOFError:
                    pass
        return
    rows = []
    with socket.create_connection((args.host, args.port), timeout=10) as conn:
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        for size, count in [(1, 30), (128*1024, 12), (256*1024, 12), (512*1024, 12)]:
            payload = os.urandom(size)
            for _ in range(count):
                before = time.time()
                conn.sendall(HEADER.pack(size) + payload)
                received, sent = TIMES.unpack(read_exact(conn, TIMES.size))
                after = time.time()
                rows.append({'size': size, 't1': before, 't2': received,
                             't3': sent, 't4': after,
                             'rtt_ms': ((after-before)-(sent-received))*1000,
                             'offset_ms': ((received-before)+(sent-after))*500})
                time.sleep(0.05)
    calibration = min((r for r in rows if r['size'] == 1), key=lambda r:r['rtt_ms'])
    offset = calibration['offset_ms'] / 1000
    result = {'minimum_rtt_clock_offset_ms': calibration['offset_ms'],
              'clock_offset_uncertainty_bound_ms': calibration['rtt_ms']/2,
              'small_packet_rtt_ms': summary([r['rtt_ms'] for r in rows if r['size'] == 1]),
              'estimated_upload_ms': {}, 'raw': rows}
    for size in [128*1024, 256*1024, 512*1024]:
        result['estimated_upload_ms'][str(size)] = summary(
            [(r['t2']-r['t1']-offset)*1000 for r in rows if r['size']==size])
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
