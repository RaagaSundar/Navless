#!/usr/bin/env python3
"""Local telemetry station: transmits only a standard GCS heartbeat at 1 Hz.

Run inside isolated local-course-sim. No requests, commands, RC, mode changes,
parameter writes, or sensor injection. Incoming command ACKs are observations.
"""
import argparse
import collections
import json
import signal
import time
from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=0, help='0 runs until stopped')
    args = parser.parse_args()
    link = mavutil.mavlink_connection('udpin:127.0.0.1:14550', source_system=255,
                                     source_component=190)
    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    start = time.monotonic()
    next_hb = start
    counts = collections.Counter()
    sent = 0
    def emit(event, **data):
        print(json.dumps(dict(event=event, wall_time=time.time(),
                              elapsed=time.monotonic()-start, **data)), flush=True)
    emit('start', endpoint='127.0.0.1:14550', tx_allowlist=['HEARTBEAT'])
    while running and (not args.seconds or time.monotonic()-start < args.seconds):
        now = time.monotonic()
        # udpin learns the actual PX4 peer from received packets; do not broadcast.
        if now >= next_hb and link.clients:
            link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                    mavutil.mavlink.MAV_STATE_ACTIVE)
            sent += 1
            emit('tx_heartbeat', count=sent)
            next_hb = now + 1
        msg = link.recv_match(blocking=True, timeout=0.05)
        if msg is None:
            continue
        typ = msg.get_type()
        counts[typ] += 1
        if typ in ('HEARTBEAT', 'STATUSTEXT', 'COMMAND_ACK', 'SYS_STATUS', 'EXTENDED_SYS_STATE'):
            emit('rx', system=msg.get_srcSystem(), component=msg.get_srcComponent(), message=msg.to_dict())
    emit('stop', tx_heartbeats=sent, rx_counts=dict(counts))
    link.close()

if __name__ == '__main__':
    main()
