"""Local console. In live mode, read only the existing organiser status file over SSH."""
import argparse
import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import subprocess
import threading
import time

ROOT = Path(__file__).resolve().parent
REMOTE = """test "$(hostname)" = skybeta-19 || exit 75
cat /run/skybeta-status/ros_summary.json || exit 1
echo
date +%s.%N"""
LIMIT = 3.0


def project(raw, remote_now, transit):
    if raw.get('schema_version') != 1 or raw.get('env', {}).get('ros_domain_id') != 19:
        raise ValueError('Wrong source schema or ROS domain')
    age = remote_now - raw['written_at'] + transit
    if not math.isfinite(age) or not 0 <= age <= LIMIT:
        raise ValueError('Aircraft status file is stale')
    topics = raw['topics']

    def recent(name, limit):
        arrival = topics.get(name, {}).get('age_s')
        return type(arrival) in (float, int) and 0 <= arrival and arrival + age <= limit

    def vec(v):
        if isinstance(v, list) and len(v) == 3 and all(type(x) in (int, float) and math.isfinite(x) for x in v):
            return v

    fc, vio, fi = raw['fc'], raw['vio'], raw.get('skybeta_state') or {}
    link = fc.get('link') is True and recent('vehicle_status', 2)
    fi_live = link and fi.get('px4_link') is True and recent('skybeta_state', 2)
    tracking = vio.get('vo_state_name') if recent('slam_status', 2) else None
    return dict(status='live', sequence=raw['seq'], stamp=raw['written_at'], age_s=age,
                link=link, armed=fc.get('armed') if link else None, mode=fc.get('nav_state_name') if link else None,
                offboard=fi.get('offboard_active') if fi_live else None, landed=fi.get('landed') if fi_live else None,
                tracking=tracking, pose=vec(vio.get('odom_position_enu')) if tracking == 'tracking' and recent('slam_odometry', 2) else None,
                px4=vec(fi.get('local_enu')) if fi_live and recent('vehicle_local_position', 2) else None,
                rate=vio.get('odom_rate_hz') if recent('slam_odometry', 2) else None)


class Feed:
    def __init__(self):
        self.lock = threading.Lock()
        self.set(dict(status='offline', reason='Live source disabled'))

    def set(self, sample):
        with self.lock:
            self.sample, self.received = sample, time.monotonic()

    def get(self):
        with self.lock:
            if self.sample['status'] == 'live' and self.sample['age_s'] + time.monotonic() - self.received > LIMIT:
                return dict(status='offline', reason='Telemetry expired')
            return self.sample

    def poll(self, control):
        ssh = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=3']
        if control:
            ssh += ['-S', control]
        ssh += ['dronehacks@skybeta-19.local', REMOTE]
        while True:
            started = time.monotonic()
            try:
                result = subprocess.run(ssh, capture_output=True, text=True, timeout=4)
                if result.returncode == 75:
                    self.set(dict(status='offline', reason='Incorrect aircraft hostname. Check assignment.'))
                    return
                result.check_returncode()
                payload, stamp = result.stdout.rstrip().rsplit('\n', 1)
                sample = project(json.loads(payload), float(stamp), time.monotonic()-started)
            except (OSError, subprocess.SubprocessError, KeyError, TypeError, ValueError, AttributeError) as exc:
                reason = str(exc) if isinstance(exc, ValueError) else 'SSH or telemetry unavailable'
                sample = dict(status='offline', reason=reason)
            self.set(sample)
            time.sleep(max(.1, 1 - (time.monotonic()-started)))


def serve(live=False, control=None, port=8093):
    feed = Feed()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/':
                body, mime = (ROOT/'index.html').read_bytes(), 'text/html; charset=utf-8'
            elif self.path == '/api/state':
                body, mime = json.dumps(feed.get(), allow_nan=False).encode(), 'application/json'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1',port), Handler)
    if live:
        threading.Thread(target=feed.poll, args=(control,), daemon=True).start()
    print(f'Mac dashboard http://127.0.0.1:{server.server_port}/', flush=True)
    server.serve_forever()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--live', action='store_true', help='Enable SSH reads from aircraft 19')
    parser.add_argument('--control-socket', help='Optional path to an authenticated OpenSSH control socket')
    parser.add_argument('--port', type=int, default=8093)
    args = parser.parse_args()
    serve(args.live, args.control_socket, args.port)
