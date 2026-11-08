#!/usr/bin/env python3
"""Bounded Jetson planner simulations. Geometry oracle, no ROS/PX4/sensors.

The changed map is supplied directly to the planner for this layer. This is
not evidence that cameras detect a change. Runtime deadlines are unchanged.
"""
import argparse
from collections import Counter
import copy
import json
import os
from pathlib import Path
import platform
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'navless_nav'))
from navless_nav.adaptive_route import (Bounds, CollisionMap, ConfirmedDepth,
    NoDetour, adapt_route, anchors_for, source_identity, survey_anchors, surveyed_course)


def evaluate(collision, route, anchors, budget):
    start = time.monotonic()
    try:
        result = adapt_route(collision, route, anchors, route[0], 0, budget_s=budget)
        elapsed = time.monotonic()-start
        clear = collision.route_free(result.points)
        preserved = np.allclose(np.asarray(result.points)[list(result.anchors)],
                                np.asarray(route)[list(anchors)])
        return dict(outcome='replanned' if result.changed else 'unchanged',
                    elapsed_wall_s=elapsed, collision_free=bool(clear),
                    anchors_preserved=bool(preserved), points=len(result.points)), result
    except NoDetour as exc:
        reason = str(exc)
        return dict(outcome='hold_budget' if 'budget' in reason else 'hold_no_route',
                    elapsed_wall_s=time.monotonic()-start, reason=reason), None


def run(output, repeats):
    output.mkdir(parents=True, exist_ok=False)
    survey = ROOT/'gazebo_sim/course/course_asbuilt.json'
    route_path = ROOT/'gazebo_sim/course/route_centreline_window_asbuilt.json'
    original = json.loads(survey.read_text())
    route = json.loads(route_path.read_text())['waypoints_enu']
    anchors = survey_anchors(route, survey)
    prior = surveyed_course(survey)
    manifest = dict(scope='Jetson CPU geometry-oracle planner simulation; no rendered cameras, ROS, PX4 or flight',
        architecture=platform.machine(), python=platform.python_version(), numpy=np.__version__,
        cpu_quota=os.environ.get('BENCH_CPU_QUOTA'), nice=os.nice(0),
        source_sha256={str(p.relative_to(ROOT)): source_identity(p) for p in
            (survey, route_path, ROOT/'navless_nav/navless_nav/adaptive_route.py', Path(__file__).resolve())},
        concurrent_workload='shared Jetson; simulation/physical services may be active; CPU capped and nice 19',
        runtime_budget_s=.25, startup_budget_s=2., repeats=repeats)
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    initial, repair = evaluate(prior, route, anchors, 2.)
    (output/'baseline.json').write_text(json.dumps(initial, indent=2)+'\n')
    if repair is not None:
        route, anchors = repair.points, repair.anchors
    results = []
    with (output/'cases.jsonl').open('w') as stream:
        def record(row):
            results.append(row)
            stream.write(json.dumps(row, allow_nan=False)+'\n')
            stream.flush()
            print(row['name'], row['outcome'], round(row['elapsed_wall_s'], 4), flush=True)
        for repeat in range(repeats):
            for wall in ('M1', 'M2', 'M3', 'A1', 'C1'):
                for centimeters in (3, 5, 10):
                    for axis in (0, 1):
                        for sign in (-1, 1):
                            data = copy.deepcopy(original)
                            w = next(w for w in data['walls'] if w['name'] == wall)
                            delta = sign*centimeters/100/.0254
                            for key in ('start_in', 'end_in'):
                                w[key][axis] += delta
                            path = output/'perturbed-survey.json'
                            path.write_text(json.dumps(data))
                            collision = surveyed_course(path)
                            result, update = evaluate(collision, route, anchors, .25)
                            result.update(name=f'{wall}-{centimeters}cm-{axis}-{sign:+d}-r{repeat}',
                                          wall=wall, shift_m=sign*centimeters/100, axis=axis,
                                          repeat=repeat, synthetic_changed_map=True)
                            record(result)
        # A local open-space detour using the actual assumed vehicle envelope.
        for repeat in range(repeats*3):
            for shift in (.03, .05, .10):
                collision = CollisionMap([[.6, .34-shift, 1]], [np.eye(3)],
                                         [[.05, .10, .5]], ['shifted_wall'], Bounds())
                local = [(float(x), 0., 1.) for x in np.linspace(0, 1.25, 26)]
                result, _ = evaluate(collision, local, anchors_for(local), .25)
                result.update(name=f'local-wall-{shift:g}-r{repeat}', shift_m=shift, repeat=repeat)
                record(result)
    path.unlink(missing_ok=True)
    timing = [r['elapsed_wall_s'] for r in results]
    violations = [r['name'] for r in results if r.get('collision_free') is False
                  or r.get('anchors_preserved') is False]
    rates = []
    for hz in (2, 5, 10, 15):
        for latency in (.02, .10, .30):
            d = ConfirmedDepth()
            accepted, rejected = 0, 0
            for t in np.arange(1., 2.5, 1/hz):
                try:
                    d.add([[0., 0., 1.]], float(t), float(t+latency))
                    accepted += 1
                except ValueError:
                    rejected += 1
            rates.append(dict(hz=hz, latency_s=latency, frames_accepted=accepted,
                              frames_rejected=rejected, confirmed_voxels=len(d.confirmed)))
    summary = dict(cases=len(results), outcomes=dict(Counter(r['outcome'] for r in results)),
                   invariant_violations=violations, baseline=initial, rate_confirmation= rates,
                   timing_wall_s=dict(p50=float(np.percentile(timing, 50)),
                                      p95=float(np.percentile(timing, 95)), maximum=max(timing)),
                   scope=manifest['scope'])
    (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2), flush=True)
    return 1 if violations else 0


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--repeats', type=int, default=3, choices=range(1, 6))
    args = ap.parse_args()
    raise SystemExit(run(args.output, args.repeats))
