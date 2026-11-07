#!/usr/bin/env python3
"""Loopback, read-only run evidence viewer. No simulator/flight interfaces."""
import argparse
import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlsplit

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
RUNS = ROOT / 'local_course/state/runs'
COURSE = ROOT / 'gazebo_sim/course'
EXTENSIONS = {'.json', '.jsonl', '.log', '.txt', '.yaml', '.png'}
LIMIT = 16 * 1024 * 1024
FLIGHT_CACHE = {}

def read_json(path):
    try:
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
        if track.is_symlink() or not track.resolve().is_relative_to(RUNS.resolve()):
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

def catalog():
    rows = []
    for run in sorted(RUNS.iterdir(), reverse=True) if RUNS.exists() else []:
        if not run.is_dir() or run.is_symlink():
            continue
        grade = read_json(run / 'flight/grade.json')
        files = evidence_files(run)
        track = run / 'flight/track.json'
        flight = observed_flight(track)
        course_pass = flight and isinstance(grade, dict) and grade.get('passed') is True
        state = 'course passed' if course_pass else 'course failed' if flight and isinstance(grade, dict) and grade.get('passed') is False else 'flight / ungraded' if flight else 'setup / interrupted'
        flight_kind = 'observed flight' if flight else 'session only · no observed flight'
        rows.append(dict(id=run.name, status=state, grade=grade, observed_flight=flight,
                         course_pass=course_pass, kind=flight_kind,
                         review=read_json(run / 'review.json'), has_track=track.exists(),
                         files=[str(p.relative_to(run)) for p in files]))
    return rows

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = unquote(urlsplit(self.path).path)
        if path == '/api/runs':
            return self.send(json.dumps(catalog()).encode(), 'application/json')
        fixed = {'/': HERE / 'index.html', '/app.js': HERE / 'app.js',
                 '/api/course': COURSE / 'course_v3.json',
                 '/api/ideal': COURSE / 'route_centreline_window.json'}
        target = fixed.get(path)
        if path.startswith('/evidence/'):
            parts = path[len('/evidence/'):].split('/')
            if len(parts) < 2 or any(p in ('', '.', '..') for p in parts):
                return self.send_error(404)
            run = RUNS / parts[0]
            candidate = run.joinpath(*parts[1:])
            if run.is_dir() and not run.is_symlink() and candidate in evidence_files(run):
                target = candidate
        if target is None or not target.is_file():
            return self.send_error(404)
        if target.stat().st_size > LIMIT:
            return self.send_error(413, 'Evidence exceeds 16 MiB viewing limit')
        mime = 'image/png' if target.suffix == '.png' else 'text/html; charset=utf-8' if target.suffix == '.html' else 'text/javascript; charset=utf-8' if target.suffix == '.js' else 'application/json' if target.suffix == '.json' else 'text/plain; charset=utf-8'
        return self.send(target.read_bytes(), mime)

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
    args = parser.parse_args()
    print(f'Run viewer: http://127.0.0.1:{args.port}', flush=True)
    ThreadingHTTPServer(('127.0.0.1', args.port), Handler).serve_forever()
