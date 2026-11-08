"""Repair blocked portions of the seed route using the live inflated course grid."""
import copy
import math
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from plan_source.route_lib.route import create_live_course_planner
from planner.fsm_router import CourseStage


def blocked_segments(grid, path, margin=.25, occupied=None):
    """Check the connecting segments as well as the sparse waypoints."""
    a = np.asarray(path, dtype=float)
    if a.ndim != 2 or a.shape[1] != 3 or len(a) < 2 or not np.isfinite(a).all():
        raise ValueError('invalid route')
    delta = np.diff(a, axis=0)
    steps = np.maximum(1, np.ceil(np.linalg.norm(delta, axis=1)/(grid.resolution/4)).astype(int))
    ids = np.repeat(np.arange(len(delta)), steps)
    offsets = np.arange(len(ids)) - np.repeat(np.cumsum(steps)-steps, steps)
    samples = np.vstack((a[:-1][ids] + delta[ids]*(offsets/steps[ids])[:, None], a[-1]))
    cells = np.floor((samples-grid.bounds_min)/grid.resolution).astype(int)
    outside = np.any((cells < 0) | (cells >= grid.shape), axis=1)
    obstructed = outside | (np.abs(samples[:, :2]) > 3.048-margin).any(axis=1)
    occupancy = grid.inflated_grid if occupied is None else occupied
    obstructed[~outside] |= occupancy[tuple(cells[~outside].T)]
    return np.unique(np.append(ids, len(delta)-1)[obstructed])


def repair(router, path, margin):
    """Join the original route on each side of the obstruction and retain its destination."""
    grid = router.grid
    blocked = blocked_segments(grid, path, margin)
    if not len(blocked):
        return path
    profile = router.profile_for_stage(CourseStage.STAGE_1_MAZE)
    distances = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    repaired = list(path)
    # Repair from the end so earlier indices stay valid. Merge overlapping join intervals.
    spans = []
    for i in blocked:
        left = max(0, int(np.searchsorted(distances, distances[i]-.45))-1)
        right = min(len(path)-1, int(np.searchsorted(distances, distances[i+1]+.45)))
        if spans and left <= spans[-1][1]:
            spans[-1] = (spans[-1][0], right)
        else:
            spans.append((left, right))
    for left, right in reversed(spans):
        # Preserve mandatory gates when the original segment traverses their neighbourhood.
        anchors = []
        for target in (router.config.checkpoints.gate_a_exit, router.config.checkpoints.window_aperture):
            segment = np.asarray(path[left:right+1])
            ds = np.linalg.norm(segment - target, axis=1)
            index = int(np.argmin(ds))
            if ds[index] <= .20 and 0 < index < len(segment)-1:
                anchors.append((index, path[left+index]))
        spline, _ = router.optimizer.plan_spline(
            path[left], path[right], profile, intermediate_waypoints=[p for _,p in sorted(anchors)])
        if spline is None:
            return None
        points = [tuple(p) for p in spline.sample_points(max(32, math.ceil(spline.duration*16)))]
        if any(not np.isfinite(p).all() or not profile.z_min_enu <= p[2] <= profile.z_max_enu for p in points):
            return None
        if any(min(math.dist(anchor,p) for p in points) > .20 for _,anchor in anchors):
            return None
        repaired[left:right+1] = [path[left], *points, path[right]]
    if len(blocked_segments(grid, repaired, margin)):
        return None
    return repaired


class AdaptiveRoute:
    def __init__(self):
        _, self.grid, self.router = create_live_course_planner()
        self.prior_inflated = self.grid.inflated_grid.copy()
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='route-repair')
        self.pending = None
        self.failed = False

    def choose(self, path, margin):
        """Return (action, candidate). Only this thread touches the live grid."""
        if self.failed:
            return 'blocked', None
        if self.pending is not None:
            if not self.pending.done():
                return 'planning', None
            job, self.pending = self.pending, None
            try:
                candidate = job.result()
            except Exception:
                self.failed = True
                return 'planner error', None
            if candidate is None:
                self.failed = True
                return 'blocked', None
            # Include drift while paused and any obstacle inserted during planning.
            if not len(blocked_segments(self.grid, [path[0], *candidate], margin)):
                return 'replace', candidate
        if not len(blocked_segments(self.grid, path, margin)):
            return 'follow', None
        self.pending = self.worker.submit(repair, copy.deepcopy(self.router), path, margin)
        return 'planning', None

    def close(self):
        self.worker.shutdown(wait=False, cancel_futures=True)
