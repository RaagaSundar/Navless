#!/usr/bin/env python3
"""Loopback, read-only run evidence viewer. No simulator/flight interfaces."""
import argparse
import json
import math
import time
import re
import test_index
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlsplit

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
RUNS = ROOT / 'local_course/state/runs'
NVIDIA_RUNS = ROOT / 'local_course/state/nvidia-runs'
FLYX_RUNS = ROOT / 'local_course/state/flyx-runs'
COURSE = ROOT / 'gazebo_sim/course'
WALL_TESTS = ROOT / 'local_course/state/wall-rotation'
WALL_STEREO = WALL_TESTS / '20261107T224200Z-experimental-m3-tangent-pair'
WALL_FIXTURE = WALL_TESTS / '20261107T222450Z-m3-north-minus18-passive-v1'
EXTENSIONS = {'.json', '.jsonl', '.log', '.txt', '.yaml', '.png', '.jpg', '.csv', '.mp4'}
LIMIT = 16 * 1024 * 1024
FLIGHT_CACHE = {}

def read_json(path):
    try:
        if path.is_symlink() or not any(path.resolve().is_relative_to(root.resolve()) for root in (RUNS, NVIDIA_RUNS, FLYX_RUNS)):
            return None
        return json.loads(path.read_text()) if path.stat().st_size <= LIMIT else None
    except (OSError, ValueError):
        return None

def evidence_files(run):
    # Never follow links outside a run, and omit parameters, binary files and secrets.
    return sorted(p for p in run.rglob('*') if p.is_file() and not p.is_symlink()
                  and p.suffix in EXTENSIONS and p.resolve().is_relative_to(run.resolve())
                  and not any(s in p.name.lower() for s in ('credential', 'secret', 'token', 'password')))

def observed_flight(track):
    """Require armed+airborne+height in the same recorded sample, never a grade alone.

    Cache only the derived boolean keyed by size/mtime, so repeated refreshes do
    not parse unchanged multi-megabyte tracks. No cache is written to evidence.
    """
    try:
        if track.is_symlink() or not any(track.resolve().is_relative_to(root.resolve()) for root in (RUNS, NVIDIA_RUNS, FLYX_RUNS)):
            return False
        stat = track.stat()
        key = (stat.st_mtime_ns, stat.st_size)
    except OSError:
        return False
    cached = FLIGHT_CACHE.get(track)
    if cached and cached[0] == key:
        return cached[1]
    data = read_json(track)
    samples = data.get('samples', []) if isinstance(data, dict) else []
    observed = any(isinstance(s, dict) and s.get('armed') is True
                   and s.get('landed') is False and isinstance(s.get('z'), (int, float))
                   and math.isfinite(s['z']) and s['z'] > .25 for s in samples)
    FLIGHT_CACHE[track] = (key, observed)
    return observed

def recent_activity(run, now=None):
    """Fresh runtime output is activity evidence, not proof of airborne flight."""
    now = time.time() if now is None else now
    candidates = ('station.jsonl', 'native-vo.log', 'flight/mission.log',
                  'flight/recorder.log', 'map-observer/stream.jsonl')
    recent = []
    for name in candidates:
        path = run / name
        try:
            if path.is_symlink() or not path.resolve().is_relative_to(run.resolve()):
                continue
            age = now - path.stat().st_mtime
            if path.stat().st_size and 0 <= age <= 120:
                recent.append((age, name))
        except OSError:
            pass
    if not recent:
        return None
    age, name = min(recent)
    return dict(file=name, age_seconds=round(age, 1), window_seconds=120)

def nvidia_live_phase(run, flight, now=None):
    """Use source log timestamps, so copying an old log cannot revive a session."""
    now = time.time() if now is None else now
    for name in ('flight/mission.log', 'flight/preflight.log'):
        path = run / name
        try:
            if path.is_symlink() or not path.resolve().is_relative_to(run.resolve()):
                continue
            with path.open('rb') as stream:
                stream.seek(max(0, path.stat().st_size - 16384))
                tail = stream.read().decode('utf-8', errors='replace')
            stamps = re.findall(r'\[(\d{10}(?:\.\d+)?)\]', tail)
            if not stamps:
                continue
            stamp = float(stamps[-1])
            age = now - stamp
            if not 0 <= age <= 120:
                continue
            phase = 'flight / awaiting grade' if flight else 'preflight / in progress'
            return dict(file=name, age_seconds=round(age, 1), window_seconds=120,
                        updated_at_utc=datetime.fromtimestamp(stamp, timezone.utc).isoformat(), phase=phase)
        except OSError:
            pass
    return None

def wall_bench_available():
    required=[WALL_STEREO/name for name in ('experiment.json','paired-camera.json','paired-camera.mp4','approach-route.json','actual-left-paired-preview.png')]
    required.append(WALL_FIXTURE/'fault-fixture.json')
    try:
        if not all(p.is_file() and not p.is_symlink() and p.stat().st_size>0 and p.resolve().is_relative_to(WALL_TESTS.resolve()) for p in required):
            return False
        sidecar=WALL_STEREO/'paired-camera.json'
        if sidecar.stat().st_size>128*1024:
            return False
        metadata=json.loads(sidecar.read_text())
        return metadata.get('status')=='complete' and isinstance(metadata.get('frame_count'),int) and metadata['frame_count']>0
    except (OSError,ValueError):
        return False

def catalog():
    rows = []
    sources = [(RUNS, 'Mac SIM', ''), (NVIDIA_RUNS, 'NVIDIA SIM', 'nvidia:'), (FLYX_RUNS, 'FlyX SIM', 'flyx:')]
    entries = [(run, label, prefix) for root, label, prefix in sources if root.exists()
               for run in root.iterdir()]
    for run, label, prefix in sorted(entries, key=lambda row: row[0].name, reverse=True):
        if not run.is_dir() or run.is_symlink():
            continue
        platform = read_json(run / 'platform.json')
        if prefix == 'nvidia:' and (not isinstance(platform, dict) or platform.get('verified') is not True):
            continue
        if prefix == 'flyx:':
            assessment = read_json(run / 'assessment.json')
            if not isinstance(assessment, dict):
                continue
            files = evidence_files(run)
            has_track = 'course-truth-track.json' in [str(p.relative_to(run)) for p in files]
            rows.append(dict(id=prefix + run.name, run_id=run.name, platform_label=label,
                             platform=platform, status='trajectory recorded / ungraded' if has_track else 'evidence / incomplete',
                             grade=None, assessment=assessment, observed_flight=False, course_pass=False,
                             trajectory_archive=has_track, kind='native trajectory · no managed independent grade',
                             activity=None, in_progress=False, review=read_json(run / 'review.json'),
                             has_track=has_track, track_file='course-truth-track.json',
                             estimated_track_file='course-estimated-track.json',
                             files=[str(p.relative_to(run)) for p in files]))
            continue
        grade = read_json(run / 'flight/grade.json')
        files = evidence_files(run)
        track = run / 'flight/track.json'
        flight = observed_flight(track)
        course_pass = flight and isinstance(grade, dict) and grade.get('passed') is True
        state = 'course passed' if course_pass else 'course failed' if flight and isinstance(grade, dict) and grade.get('passed') is False else 'flight / ungraded' if flight else 'setup / interrupted'
        activity = recent_activity(run) if grade is None else None
        if grade is None and activity:
            state = 'flight / awaiting grade' if flight else 'in progress / recent runtime evidence'
        elif grade is None and flight:
            state = 'flight / awaiting grade'
        flight_kind = 'observed flight' if flight else 'session only · no observed flight'
        if not flight and isinstance(platform, dict) and platform.get('setup_status') in ('blocked', 'partial'):
            state = 'setup / ' + platform['setup_status']
        if prefix == 'nvidia:' and grade is None:
            live_phase = nvidia_live_phase(run, flight)
            if live_phase:
                activity = live_phase
                state = live_phase['phase']
        bounded = read_json(run / 'm3-capture-result.json')
        bounded_pass = flight and isinstance(bounded, dict) and bounded.get('capture_met_declared_goals') is True
        if flight and isinstance(bounded, dict) and isinstance(bounded.get('capture_met_declared_goals'), bool):
            state = 'bounded capture passed' if bounded_pass else 'bounded capture not met'
        rows.append(dict(id=prefix + run.name, run_id=run.name, platform_label=label, platform=platform, status=state, grade=grade, observed_flight=flight, bounded_pass=bounded_pass,
                         course_pass=course_pass, kind=flight_kind, activity=activity, in_progress=bool(activity) and state.startswith(('in progress', 'preflight / in progress', 'flight / awaiting')), 
                         review=read_json(run / 'review.json'), has_track=track.exists(), track_file='flight/track.json',
                         files=[str(p.relative_to(run)) for p in files]))
    if wall_bench_available():
        rows.insert(0, dict(id='bench:m3-wall-rotation', run_id=WALL_STEREO.name,
                            display_name='Unexpected wall · M3 rotated 18°', platform_label='Mac Gazebo',
                            experiment=True, status='paired stereo test complete', kind='camera bench',
                            observed_flight=False, course_pass=False, in_progress=False,
                            grade=None, has_track=False, files=[]))
    return rows

def api_json(value):
    """Strict JSON at the browser boundary; preserve original receipt files."""
    def finite(item):
        if isinstance(item,float) and not math.isfinite(item): return None
        if isinstance(item,dict): return {k:finite(v) for k,v in item.items()}
        if isinstance(item,(list,tuple)): return [finite(v) for v in item]
        return item
    return json.dumps(finite(value),allow_nan=False).encode()

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = unquote(urlsplit(self.path).path)
        if path == '/api/runs':
            return self.send(api_json(catalog()), 'application/json')
        if path == '/api/tests':
            return self.send(api_json(test_index.inventory(catalog())), 'application/json')
        if path.startswith('/test-evidence/'):
            parts = path[len('/test-evidence/'):].split('/')
            target = test_index.resolve_file(parts[0], '/'.join(parts[1:])) if len(parts) > 1 else None
            if target is None:
                return self.send_error(404)
            if target.stat().st_size > LIMIT:
                return self.send_error(413, 'Evidence exceeds 16 MiB viewing limit')
            mime = 'image/png' if target.suffix == '.png' else 'image/jpeg' if target.suffix in {'.jpg','.jpeg'} else 'application/json' if target.suffix == '.json' else 'text/plain; charset=utf-8'
            return self.send(target.read_bytes(), mime)
        fixed = {'/': HERE / 'index.html', '/app.js': HERE / 'app.js',
                 '/tests': HERE / 'tests.html', '/tests/': HERE / 'tests.html',
                 '/tests.js': HERE / 'tests.js', '/all_tests_picker.js': HERE / 'all_tests_picker.js',
                 '/api/course': COURSE / 'course_v3.json',
                 '/api/ideal': COURSE / 'route_centreline_window.json'}
        target = fixed.get(path)
        wall_files = {
            '/wall-bench.js': HERE / 'wall-bench.js',
            '/wall-test/fixture.json': WALL_FIXTURE / 'fault-fixture.json',
            '/wall-test/route.json': WALL_STEREO / 'approach-route.json',
            '/wall-test/cameras.png': WALL_STEREO / 'actual-left-paired-preview.png',
            '/wall-test/cameras.mp4': WALL_STEREO / 'paired-camera.mp4',
            '/wall-test/reference.jsonl': WALL_STEREO / 'changed/20261107T224309Z-m3-tangent-changed-passive/reference.jsonl',
            '/wall-test/results.json': ROOT / 'local_course/evidence/obstacle-v1/tangent-nominal-residual-commanded-v1.json',
        }
        if path in wall_files:
            target = wall_files[path]
        if path.startswith('/evidence/'):
            parts = path[len('/evidence/'):].split('/')
            if len(parts) < 2 or any(p in ('', '.', '..') for p in parts):
                return self.send_error(404)
            name = parts[0]
            root = NVIDIA_RUNS if name.startswith('nvidia:') else FLYX_RUNS if name.startswith('flyx:') else RUNS
            name = name.split(':', 1)[1] if root != RUNS else name
            if not name or ':' in name:
                return self.send_error(404)
            run = root / name
            if root == NVIDIA_RUNS:
                platform = read_json(run / 'platform.json')
                if not isinstance(platform, dict) or platform.get('verified') is not True:
                    return self.send_error(404)
            if root == FLYX_RUNS and not isinstance(read_json(run / 'assessment.json'), dict):
                return self.send_error(404)
            candidate = run.joinpath(*parts[1:])
            if run.is_dir() and not run.is_symlink() and candidate in evidence_files(run):
                target = candidate
        if target is None or not target.is_file():
            return self.send_error(404)
        if target.suffix == '.mp4':
            return self.send_video(target)
        if target.stat().st_size > LIMIT:
            return self.send_error(413, 'Evidence exceeds 16 MiB viewing limit')
        mime = 'image/jpeg' if target.suffix == '.jpg' else 'image/png' if target.suffix == '.png' else 'text/html; charset=utf-8' if target.suffix == '.html' else 'text/javascript; charset=utf-8' if target.suffix == '.js' else 'application/json' if target.suffix == '.json' else 'text/csv; charset=utf-8' if target.suffix == '.csv' else 'text/plain; charset=utf-8'
        return self.send(target.read_bytes(), mime)

    def send_video(self, target):
        """Stream bounded chunks and implement a single RFC byte range for seeking."""
        with target.open('rb') as stream:
            size = target.stat().st_size
            start, end = 0, size - 1
            requested = self.headers.get('Range')
            if requested:
                match = re.fullmatch(r'bytes=(\d*)-(\d*)', requested.strip())
                try:
                    if not match or not any(match.groups()):
                        raise ValueError
                    first, last = match.groups()
                    if first:
                        start = int(first)
                        end = min(int(last), end) if last else end
                    else:
                        length = int(last)
                        if length <= 0:
                            raise ValueError
                        start = max(0, size - length)
                    if start > end or start >= size:
                        raise ValueError
                except ValueError:
                    self.send_response(416)
                    self.send_header('Content-Range', f'bytes */{size}')
                    self.send_header('Content-Length', '0')
                    self.end_headers()
                    return
            self.send_response(206 if requested else 200)
            self.send_header('Content-Type', 'video/mp4')
            self.send_header('Accept-Ranges', 'bytes')
            self.send_header('Content-Length', str(max(0, end - start + 1)))
            if requested:
                self.send_header('Content-Range', f'bytes {start}-{end}/{size}')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            stream.seek(start)
            remaining = end - start + 1
            try:
                while remaining > 0:
                    chunk = stream.read(min(256 * 1024, remaining))
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    remaining -= len(chunk)
            except (BrokenPipeError, ConnectionResetError):
                pass  # Browser switched run or cancelled its seek.

    def send(self, data, mime):
        self.send_response(200)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(data)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8774)
    parser.add_argument('--runs', type=Path, default=RUNS, help='Mac run archive root')
    parser.add_argument('--nvidia-runs', type=Path, default=NVIDIA_RUNS)
    parser.add_argument('--flyx-runs', type=Path, default=FLYX_RUNS)
    parser.add_argument('--course', type=Path, default=COURSE, help='Directory containing course_v3.json and route_centreline_window.json')
    parser.add_argument('--test-evidence-root', type=Path, default=ROOT, help='Workspace root for explicit saved-test allowlist')
    parser.add_argument('--wall-tests', type=Path, default=WALL_TESTS, help='Archived wall-test fixture root; optional, not generated')
    args = parser.parse_args()
    RUNS, NVIDIA_RUNS, FLYX_RUNS, COURSE = (p.resolve() for p in (args.runs, args.nvidia_runs, args.flyx_runs, args.course))
    WALL_TESTS=args.wall_tests.resolve()
    WALL_STEREO=WALL_TESTS/'20261107T224200Z-experimental-m3-tangent-pair'
    WALL_FIXTURE=WALL_TESTS/'20261107T222450Z-m3-north-minus18-passive-v1'
    test_index.ROOT=args.test_evidence_root.resolve()
    test_index.RUN_ROOTS={'mac':RUNS,'nvidia':NVIDIA_RUNS,'flyx':FLYX_RUNS}
    print(f'Run viewer: http://127.0.0.1:{args.port}', flush=True)
    ThreadingHTTPServer(('127.0.0.1', args.port), Handler).serve_forever()
