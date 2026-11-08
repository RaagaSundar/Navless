#!/usr/bin/env python3
"""Offline survey/route check. Writes geometry evidence; never starts ROS/flight."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'navless_nav'))
from navless_nav.adaptive_route import (Bounds, adapt_route, source_identity,
                                       survey_anchors, surveyed_course)


def inspect(survey, route):
    data = json.loads(route.read_text())
    points = np.asarray(data['waypoints_enu'])
    collision = surveyed_course(survey)
    anchors = survey_anchors(points, survey)
    blocked = np.flatnonzero(collision.blocked_edges(points[:-1], points[1:])).tolist()
    started = time.monotonic()
    result = adapt_route(collision, points, anchors, points[0], 0, budget_s=2.)
    elapsed = time.monotonic()-started
    clear = collision.route_free(result.points)
    same_anchors = np.allclose(np.asarray(result.points)[list(result.anchors)], points[list(anchors)])
    if not clear or not same_anchors:
        raise ValueError('offline candidate failed collision/mandatory-anchor check')
    return dict(schema=1, scope='offline assumed-geometry bench; no ROS, sensing, PX4 or flight qualification',
        survey_sha256=source_identity(survey), baseline_route_sha256=source_identity(route),
        source_sha256={str(p.relative_to(ROOT)): source_identity(p) for p in
                       (ROOT/'navless_nav/navless_nav/adaptive_route.py', Path(__file__).resolve())},
        assumptions=asdict(Bounds()), planning_budget_s=2., elapsed_wall_s=elapsed,
        baseline_blocked_segments=blocked,
        baseline_blocked_segment_starts=[data['waypoints_enu'][i] for i in blocked],
        baseline_points=len(points), candidate_points=len(result.points),
        candidate_clear=bool(clear), required_anchors_preserved=bool(same_anchors),
        baseline_anchor_indices=list(anchors), candidate_anchor_indices=list(result.anchors),
        route_type='EXPERIMENTAL_ADAPTIVE_ASBUILT_GEOMETRY',
        frame_convention=data['frame_convention'], launch_origin_enu=data['launch_origin_enu'],
        waypoints_enu=result.points,
        note='Candidate includes conservative downward-range keep-outs; this file is diagnostic and is not selected by a launch.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--survey', type=Path, default=ROOT/'gazebo_sim/course/course_asbuilt.json')
    parser.add_argument('--route', type=Path, default=ROOT/'gazebo_sim/course/route_centreline_window_asbuilt.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = inspect(args.survey, args.route)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: result[k] for k in ('baseline_blocked_segments', 'candidate_points',
          'candidate_clear', 'required_anchors_preserved', 'elapsed_wall_s')}, indent=2))
