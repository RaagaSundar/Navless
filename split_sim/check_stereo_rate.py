"""Measure actual synchronized stereo and validated odometry on simulation domain219."""
import json
import os
import statistics
import sys
import time

def check_fast_camera(stamps):
    """Check the steady overlap. Discard only unmatched startup/shutdown edges."""
    left, right = stamps['left'], stamps['right']
    if not left or not right:
        return {'qualified': False, 'pairs': 0}
    first = max(left[0], right[0])
    last = min(left[-1], right[-1])
    eyes = [[t for t in stream if first <= t <= last] for stream in (left, right)]
    seq = eyes[0]
    intervals = [t1 - t0 for t0, t1 in zip(seq, seq[1:])]
    jitter_ns = max((abs(dt - 32_000_000) for dt in intervals), default=None)
    paired = len(set(seq).intersection(eyes[1]))
    return {
        'target_hz': 31.25, 'pairs': paired, 'identical_stamps': eyes[0] == eyes[1],
        'max_interval_error_ms': jitter_ns / 1e6 if jitter_ns is not None else None,
        'qualified': (paired >= 35 and eyes[0] == eyes[1] and
                      jitter_ns is not None and jitter_ns <= 2_000_000),
    }


def main():
    import argparse
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from nav_msgs.msg import Odometry
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera-hz', type=float, choices=(10, 15, 31.25))
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '219':
        raise SystemExit('requires split simulation domain')
    rclpy.init(args=[])
    node = rclpy.create_node('split_stereo_rate_probe')
    stamps = {k: [] for k in ('left','right','odom')}
    def record(kind, msg):
        if kind != 'odom' and (msg.height,msg.width,len(msg.data)) != (400,640,256000):
            raise ValueError('unexpected image geometry or payload size')
        s = msg.header.stamp
        stamps[kind].append(s.sec*10**9+s.nanosec)
    subs = [node.create_subscription(Image, f'/oak/{side}/image_raw',
              lambda msg, side=side: record(side,msg), qos_profile_sensor_data) for side in ('left','right')]
    subs.append(node.create_subscription(Odometry,'/mapping/validated_odometry',
                                         lambda msg: record('odom',msg),qos_profile_sensor_data))
    start = time.monotonic()
    while min(map(len,stamps.values())) < 45 and time.monotonic()-start < 25:
        rclpy.spin_once(node,timeout_sec=.1)
    result = {}
    for kind, values in stamps.items():
        seq = sorted(set(values))
        delta = [(b-a)/1e9 for a,b in zip(seq,seq[1:])]
        result[kind] = dict(samples=len(values), unique=len(seq),
                            median_hz=1/statistics.median(delta) if delta else 0,
                            delivered_hz=(len(seq)-1)*1e9/(seq[-1]-seq[0]) if delta else 0)
    paired=set(stamps['right']).intersection(stamps['left'])
    result['paired']=len(paired)
    result['paired_odom']=len(paired.intersection(stamps['odom']))
    if args.camera_hz == 31.25:
        result['fast_camera'] = check_fast_camera(stamps)
    print(json.dumps(result,indent=2),flush=True)
    node.destroy_node()
    rclpy.shutdown()
    enough_odometry = result['paired_odom'] >= 30 and result['odom']['delivered_hz'] > 12
    enough_images = result['paired'] >= 35 and min(result[k]['delivered_hz'] for k in ('left','right')) > 12
    return enough_odometry and enough_images and result.get('fast_camera', {}).get('qualified', True)


if __name__ == '__main__':
    sys.exit(0 if main() else 1)
