"""Experimental local detours against a surveyed course and confirmed depth.

Pure NumPy: no ROS, simulator geometry, truth, commands or raw-scan alignment.
The course JSON is the team's survey product, in its declared metric frame.
Vehicle envelopes and uncertainty allowances are assumptions, not qualification.
"""
from dataclasses import dataclass
import hashlib
import heapq
import itertools
import json
import math
from pathlib import Path
import time

import numpy as np


def xyz(value):
    p = np.asarray(value, dtype=float)
    if p.shape != (3,) or not np.isfinite(p).all():
        raise ValueError('expected finite XYZ')
    return p


@dataclass(frozen=True)
class Bounds:
    # Spheres in ENU at nose north, enclosing the team's existing vehicle model.
    # Physical vehicle dimensions/mounts still require independent review.
    parts: tuple = ((.160, .08, .03, .0775), (-.140, -.15, .03, .0775),
                    (-.160, .08, .03, .0775), (.140, -.15, .03, .0775),
                    (0, .07, .01, .055), (0, 0, .01, .055), (0, -.07, .01, .055),
                    (.12, .10, -.05, .02), (-.12, .10, -.05, .02),
                    (.11, -.10, -.05, .02), (-.11, -.10, -.05, .02))
    map_error_m: float = 0.02
    tracking_error_m: float = 0.02
    attitude_deg: float = 5.0

    def envelope(self):
        parts = np.asarray(self.parts, float)
        if (parts.ndim != 2 or parts.shape[1] != 4 or not len(parts)
                or not np.isfinite(parts).all() or np.any(parts[:, 3] <= 0)
                or not all(math.isfinite(v) and v >= 0 for v in
                (self.map_error_m, self.tracking_error_m, self.attitude_deg))
                or self.attitude_deg > 15):
            raise ValueError('invalid vehicle/uncertainty bounds')
        # Any body point moves <= 2*r*sin(theta/2) under bounded attitude.
        radius = np.linalg.norm(parts[:, :3], axis=1)
        reserve = (self.map_error_m + self.tracking_error_m
                   + 2*radius*np.sin(math.radians(self.attitude_deg)/2))
        return parts[:, :3], parts[:, 3]+reserve


class CollisionMap:
    """Sphere vehicle parts swept continuously against oriented obstacle boxes."""
    def __init__(self, centers, rotations, half_sizes, names, bounds=Bounds(), rangefinder=False):
        self.centers = np.asarray(centers, float).reshape(-1, 3)
        self.rotations = np.asarray(rotations, float).reshape(-1, 3, 3)
        self.half_sizes = np.asarray(half_sizes, float).reshape(-1, 3)
        self.names = tuple(names)
        if (not len(self.centers) or not len(self.centers) == len(self.rotations)
                == len(self.half_sizes) == len(self.names)
                or not all(np.isfinite(a).all() for a in
                           (self.centers, self.rotations, self.half_sizes))
                or np.any(self.half_sizes <= 0)
                or not np.allclose(self.rotations.transpose(0, 2, 1) @ self.rotations,
                                   np.eye(3), atol=1e-6)
                or not np.allclose(np.linalg.det(self.rotations), 1, atol=1e-6)):
            raise ValueError('invalid obstacle boxes')
        self.bounds = bounds
        self.rangefinder = rangefinder
        offsets, radii = bounds.envelope()
        n = len(offsets)
        self.offset = np.tile(offsets, (len(self.centers), 1))
        self.check_centers = np.repeat(self.centers, n, axis=0)
        self.check_rotations = np.repeat(self.rotations, n, axis=0)
        self.check_halves = np.repeat(self.half_sizes, n, axis=0)
        self.radii = np.tile(radii, len(self.centers))
        self.expanded = (self.half_sizes[:, None, :]+radii[None, :, None]).reshape(-1, 3)
        # Keep the downward height beam off raised objects even when the
        # prop/body envelope could physically fly over them. Assume a <=1.1m
        # route, <=5deg tilt, and a <=2cm lateral sensor mount uncertainty.
        beam = np.repeat([name == 'range_keepout' for name in self.names], n)
        if beam.any():
            radius = (1.1*math.tan(math.radians(bounds.attitude_deg))+.02
                      +bounds.map_error_m+bounds.tracking_error_m)
            self.offset[beam] = 0
            self.radii[beam] = radius
            self.expanded[beam] = self.check_halves[beam]+radius

    def blocked_edges(self, starts, ends):
        a, b = np.atleast_2d(starts).astype(float), np.atleast_2d(ends).astype(float)
        if (a.shape != b.shape or a.shape[1:] != (3,)
                or not np.isfinite(a).all() or not np.isfinite(b).all()):
            raise ValueError('invalid segments')
        out = np.zeros(len(a), bool)
        # Bound intermediate allocations even for a large confirmed cloud.
        for first in range(0, len(self.check_centers), 256):
            c, r, h, offset = (v[first:first+256] for v in
                       (self.check_centers, self.check_rotations, self.expanded, self.offset))
            local = np.einsum('mji,nmj->nmi', r, a[:, None]+offset-c)
            direction = np.einsum('mji,nj->nmi', r, b-a)
            parallel = np.abs(direction) < 1e-12
            safe = np.where(parallel, 1.0, direction)
            t1, t2 = (-h-local)/safe, (h-local)/safe
            low = np.where(parallel, -np.inf, np.minimum(t1, t2)).max(axis=2)
            high = np.where(parallel, np.inf, np.maximum(t1, t2)).min(axis=2)
            outside = (parallel & (np.abs(local) > h)).any(axis=2)
            hit = ~outside & (np.maximum(low, 0) <= np.minimum(high, 1))
            # Broad boxes are insufficient near wall ends. Use exact minimum
            # segment-to-box distance for the potentially colliding pairs.
            rows, cols = np.nonzero(hit)
            if len(rows):
                box_half = self.check_halves[first:first+256][cols]
                distances = segment_box_distance(local[rows, cols], direction[rows, cols], box_half)
                collide = distances <= self.radii[first:first+256][cols]+1e-9
                out[rows[collide]] = True
        return out

    def free(self, a, b=None):
        p = xyz(a)
        return not bool(self.blocked_edges([p], [p if b is None else xyz(b)])[0])

    def route_free(self, route):
        pts = np.asarray(route, float)
        if len(pts) == 1:
            return self.free(pts[0])
        return not self.blocked_edges(pts[:-1], pts[1:]).any()

    def with_points(self, points, voxel_m):
        points = np.asarray(points, float).reshape(-1, 3)
        if not len(points):
            return self
        centers, sizes = points, np.full(points.shape, voxel_m/2)
        names = ('confirmed_depth',)*len(points)
        if self.rangefinder:
            low = points[(points[:, 2] > .03) & (points[:, 2] < 1.1)]
            columns = low.copy()
            columns[:, 2] = .55
            centers = np.vstack((centers, columns))
            sizes = np.vstack((sizes, np.tile((voxel_m/2, voxel_m/2, .55), (len(low), 1))))
            names += ('range_keepout',)*len(low)
        return CollisionMap(np.vstack((self.centers, centers)),
                            np.concatenate((self.rotations, np.repeat(np.eye(3)[None], len(centers), axis=0))),
                            np.vstack((self.half_sizes, sizes)),
                            self.names + names, self.bounds, self.rangefinder)

    def novel_points(self, points):
        """Discard only known geometry within the explicit map error budget.

        Unlike the old 20 cm mask, points beyond the configured survey error
        can become obstacles even directly alongside an existing wall.
        """
        pts = np.asarray(points, float).reshape(-1, 3)
        keep = []
        for start in range(0, len(pts), 256):
            p = pts[start:start+256]
            local = np.einsum('mji,nmj->nmi', self.rotations, p[:, None]-self.centers)
            q = np.abs(local)-self.half_sizes
            dist = np.linalg.norm(np.maximum(q, 0), axis=2)+np.minimum(q.max(axis=2), 0)
            keep.extend((dist.min(axis=1) > self.bounds.map_error_m).tolist())
        return pts[np.asarray(keep, bool)]

    def near(self, route, margin):
        pts = np.asarray(route)
        offset, radii = self.bounds.envelope()
        vehicle = np.max(np.abs(offset)+radii[:, None], axis=0)
        extent = np.einsum('mij,mj->mi', np.abs(self.rotations), self.half_sizes)
        lo, hi = pts.min(axis=0)-margin-vehicle, pts.max(axis=0)+margin+vehicle
        keep = ((self.centers+extent >= lo) & (self.centers-extent <= hi)).all(axis=1)
        if not keep.any():
            return self
        return CollisionMap(self.centers[keep], self.rotations[keep], self.half_sizes[keep],
                            np.asarray(self.names)[keep], self.bounds, self.rangefinder)


def segment_box_distance(start, delta, half):
    """Exact convex piecewise-quadratic minimum for local segments and AABBs.

    Each box-face crossing partitions [0,1]. Inside an interval the set of
    active distance components is fixed, so its minimum is analytic.
    """
    parallel = np.abs(delta) < 1e-12
    safe = np.where(parallel, 1., delta)
    roots = np.concatenate(((-half-start)/safe, (half-start)/safe), axis=1)
    roots = np.where(np.concatenate((parallel, parallel), axis=1), 0, roots)
    cuts = np.sort(np.column_stack((np.zeros(len(start)), np.clip(roots, 0, 1), np.ones(len(start)))), axis=1)
    low, high = cuts[:, :-1], cuts[:, 1:]
    mid = start[:, None]+delta[:, None]*((low+high)/2)[..., None]
    active = np.abs(mid) > half[:, None]
    boundary = np.sign(mid)*half[:, None]
    offset = (start[:, None]-boundary)*active
    slope = delta[:, None]*active
    optimum = -(slope*offset).sum(axis=2)/np.maximum((slope*slope).sum(axis=2), 1e-30)
    at = np.clip(optimum, low, high)
    residual = np.maximum(np.abs(start[:, None]+delta[:, None]*at[..., None])-half[:, None], 0)
    return np.sqrt((residual*residual).sum(axis=2).min(axis=1))


def surveyed_course(path, bounds=Bounds()):
    """Build portable planning geometry from the as-built survey JSON.

    Walls/window are oriented boxes, tube a conservative polygonal shell.
    Boulders use bounding boxes (deliberately conservative). Net/floor/roof
    remain solid keep-outs regardless of whether optical depth sees them.
    """
    data = json.loads(Path(path).read_text())
    centers, rotations, halves, names = [], [], [], []
    def box(name, center, size, yaw=0, pitch=0):
        cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
        centers.append(center)
        halves.append(np.asarray(size)/2)
        rotations.append([[cy*cp, -sy, cy*sp], [sy*cp, cy, sy*sp], [-sp, 0, cp]])
        names.append(name)
    def world(pair):
        return np.asarray(pair)*0.0254 - 3.048
    for wall in data['walls']:
        p, q = world(wall['start_in']), world(wall['end_in'])
        length = np.linalg.norm(q-p)
        direction = (q-p)/length
        yaw = math.atan2(direction[1], direction[0])
        height, thickness = wall['height_in']*.0254, wall['thickness_in']*.0254
        def segment(s0, s1, z0, z1, suffix):
            if s1 <= s0 or z1 <= z0:
                raise ValueError('invalid wall/window dimensions')
            pos = p + direction*(s0+s1)/2
            box(wall['name']+suffix, (*pos, (z0+z1)/2), (s1-s0, thickness, z1-z0), yaw)
        if 'window' in wall:
            win = wall['window']
            at = float((world(win['center_in'])-p) @ direction)
            hw = win['width_in']*.0254/2
            zb, zt = win['bottom_above_floor_in']*.0254, win['top_above_floor_in']*.0254
            segment(0, at-hw, 0, height, '_a')
            segment(at+hw, length, 0, height, '_b')
            segment(at-hw, at+hw, 0, zb, '_below')
            segment(at-hw, at+hw, zt, height, '_above')
        else:
            segment(0, length, 0, height, '')
    t = data['tunnel']
    f = t['footprint_in']
    cx, cy = world(((f['x_min']+f['x_max'])/2, (f['y_min']+f['y_max'])/2))
    zlo, zhi = np.asarray(t['interior_z_in'])*.0254
    a, b = t['interior_oval_in']['width']*.0254/2, t['interior_oval_in']['height']*.0254/2
    for i in range(48):
        u, v = 2*math.pi*i/48, 2*math.pi*(i+1)/48
        p, q = np.array([a*math.cos(u), b*math.sin(u)]), np.array([a*math.cos(v), b*math.sin(v)])
        mid = (p+q)/2
        angle = math.atan2(q[1]-p[1], q[0]-p[0])
        # Chords lie inside the ideal ellipse, making the aperture conservative.
        box(f'tube_{i}', (cx+mid[0], cy, (zlo+zhi)/2+mid[1]),
            (np.linalg.norm(q-p)*1.02, (f['y_max']-f['y_min'])*.0254, .024), pitch=-angle)
    # Low supports have measured model dimensions, not live detections.
    for yy in (108.5, 138, 167.5):
        box('tube_support', (cx, yy*.0254-3.048, 10.9*.0254/2), (40*.0254, .0254, 10.9*.0254))
    for boulder in data['boulders_in']:
        x, y = world(boulder['center'])
        h, d = boulder['height']*.0254, boulder['diameter']*.0254
        box('boulder_bound', (x, y, h/2), (d, d, h))
        box('range_keepout', (x, y, .55), (d, d, 1.1))
    net = data['net_in']
    xmax, ymax, roof = np.asarray([net['x'], net['y'], net['height']])*.0254
    for axis, width in ((0, xmax), (1, ymax)):
        for sign in (-1, 1):
            c, size = [0., 0., roof/2], [xmax+.2, ymax+.2, roof]
            c[axis], size[axis] = sign*(width/2+.01), .02
            box('net', c, size)
    box('floor', (0, 0, -.05), (xmax+.2, ymax+.2, .1))
    box('roof', (0, 0, roof+.05), (xmax+.2, ymax+.2, .1))
    return CollisionMap(centers, rotations, halves, names, bounds, rangefinder=True)


class ConfirmedDepth:
    """Count distinct source frames, not rays; never erase unseen prior walls.

    No near-prior suppression: a 5 cm protrusion matters. Pending hits expire;
    committed obstacles remain until a reviewed map reset, including on loss.
    """
    def __init__(self, voxel_m=.05, frames=3, window_s=.5, max_voxels=12000):
        if voxel_m <= 0 or frames < 2 or window_s <= 0 or max_voxels < 1:
            raise ValueError('invalid depth confirmation settings')
        self.voxel, self.frames, self.window, self.limit = voxel_m, frames, window_s, max_voxels
        self.pending, self.confirmed = {}, set()
        self.last_stamp = -math.inf
        self.revision = 0

    def add(self, points, stamp, now):
        pts = np.asarray(points, float).reshape(-1, 3)
        if (not math.isfinite(stamp) or not math.isfinite(now) or stamp <= self.last_stamp
                or not 0 <= now-stamp <= .25 or not np.isfinite(pts).all()):
            raise ValueError('stale, repeated or invalid cloud')
        self.last_stamp = stamp
        self.pending = {key: times for key, times in self.pending.items()
                        if stamp-times[-1] <= self.window}
        keys = {tuple(v) for v in np.floor(pts/self.voxel).astype(np.int64)}
        if len(keys | set(self.pending) | self.confirmed) > self.limit:
            raise ValueError('depth map capacity exceeded')
        added = 0
        for key in keys-self.confirmed:
            times = [t for t in self.pending.get(key, []) if stamp-t <= self.window]+[stamp]
            if len(times) >= self.frames:
                self.confirmed.add(key)
                self.pending.pop(key, None)
                added += 1
            else:
                self.pending[key] = times
        if added:
            self.revision += 1
        return added

    def points(self):
        return (np.asarray(sorted(self.confirmed), float).reshape(-1, 3)+.5)*self.voxel


class NoDetour(ValueError):
    pass


def anchors_for(route, spacing=1.25):
    """Ordered immutable local goals; callers can add explicit gate anchors."""
    pts = np.asarray(route, float)
    if (not math.isfinite(spacing) or spacing <= 0 or pts.ndim != 2
            or pts.shape[1] != 3 or len(pts) < 2 or not np.isfinite(pts).all()):
        raise ValueError('invalid baseline route')
    arc, last, anchors = 0., 0., []
    for i in range(1, len(pts)):
        arc += float(np.linalg.norm(pts[i]-pts[i-1]))
        if arc-last >= spacing or i == len(pts)-1:
            anchors.append(i)
            last = arc
    return tuple(anchors)


def survey_anchors(route, survey_path):
    """Keep the baseline's ordered corridor/M2/Gate A/tube/window crossings.

    Both ends of each existing crossing segment remain immutable. The local
    planner may route around a protrusion between stages; it cannot choose
    a direct-to-pad shortcut or relocate an opening from ambiguous depth.
    """
    pts = np.asarray(route, float)
    data = json.loads(Path(survey_path).read_text())
    world = lambda p: np.asarray(p)*.0254-3.048
    walls = {w['name']: w for w in data['walls']}
    line = data['floor_line_in']
    m2 = walls['M2']
    p, q = world(m2['start_in']), world(m2['end_in'])
    normal = np.array([q[1]-p[1], p[0]-q[0]])
    gate = data['gates_in']['gate_A']['between']
    t = data['tunnel']['footprint_in']
    win = next(w for w in data['walls'] if 'window' in w)
    wp, wq = world(win['start_in']), world(win['end_in'])
    planes = [(world((line[1][0], line[1][1]-20)), np.array([0, 1.])),
              (p, normal), (world(gate[0]), np.array([1., 0])),
              (world((0, t['y_max'])), np.array([0, -1.])),
              (world((0, t['y_min'])), np.array([0, -1.])),
              (world(win['window']['center_in']), np.array([wp[1]-wq[1], wq[0]-wp[0]]))]
    anchors, after = {len(pts)-1}, 0
    for origin, direction in planes:
        signed = (pts[:, :2]-origin) @ direction
        crossing = next((i for i in range(after, len(pts)-1)
                         if signed[i] <= 0 < signed[i+1]), None)
        if crossing is None:
            raise ValueError('baseline lacks an ordered required course crossing')
        if crossing:
            anchors.add(crossing)
        anchors.add(crossing+1)
        after = crossing+1
    return tuple(sorted(anchors))


def _distance_to_polyline(points, route):
    p = np.asarray(points)[:, None, :]
    a, d = route[:-1], np.diff(route, axis=0)
    t = np.clip(((p-a)*d).sum(axis=2)/np.maximum((d*d).sum(axis=1), 1e-12), 0, 1)
    return np.linalg.norm(p-(a+t[..., None]*d), axis=2).min(axis=1)


def local_detour(collision, route, deadline, resolution=.05, max_deviation=.45, max_expansions=6000):
    """Bounded 3D A* with continuous edge checks; exact start/goal, no escape.

    Stays near this unfinished local course leg and within its existing
    altitude range (+/- one resolution). Smoothing uses checked shortcuts,
    never an unchecked spline. No approximate blocked-goal fallback.
    """
    route = np.asarray(route, float)
    collision = collision.near(route, max_deviation+resolution)
    start, goal = route[0], route[-1]
    if not collision.free(start) or not collision.free(goal):
        raise NoDetour('current pose or required route anchor is blocked')
    steps = np.array([v for v in itertools.product((-1, 0, 1), repeat=3) if v != (0, 0, 0)])
    costs = np.linalg.norm(steps, axis=1)*resolution
    initial = (0, 0, 0)
    queue, best, parent = [(float(np.linalg.norm(goal-start)), 0, initial)], {initial: 0.}, {}
    serial, expanded, closed = 0, 0, set()
    zlo, zhi = route[:, 2].min()-resolution, route[:, 2].max()+resolution
    while queue:
        if time.monotonic() > deadline or expanded >= max_expansions:
            raise NoDetour('local planning budget exceeded')
        _, _, key = heapq.heappop(queue)
        if key in closed:
            continue
        closed.add(key)
        p = start+np.asarray(key)*resolution
        if np.linalg.norm(p-goal) <= 1.8*resolution and collision.free(p, goal):
            path = [goal, p]
            while key in parent:
                key = parent[key]
                path.append(start+np.asarray(key)*resolution)
            path = list(reversed(path))
            # Shortcuts are checked as whole segments, including diagonals.
            result, i = [path[0]], 0
            while i < len(path)-1:
                j = len(path)-1
                while j > i+1 and not collision.free(path[i], path[j]):
                    j -= 1
                result.append(path[j])
                i = j
            if not collision.route_free(result):
                raise NoDetour('detour validation failed')
            return result
        expanded += 1
        neighbors = np.asarray(key)+steps
        ends = start+neighbors*resolution
        allowed = ((_distance_to_polyline(ends, route) <= max_deviation)
                   & (ends[:, 2] >= zlo) & (ends[:, 2] <= zhi))
        indices = np.flatnonzero(allowed)
        blocked = collision.blocked_edges(np.repeat(p[None], len(indices), axis=0), ends[indices])
        for i in indices[~blocked]:
            nxt = tuple(neighbors[i])
            cost = best[key]+costs[i]
            if cost+1e-10 >= best.get(nxt, math.inf):
                continue
            parent[nxt], best[nxt] = key, cost
            serial += 1
            heapq.heappush(queue, (cost+float(np.linalg.norm(ends[i]-goal)), serial, nxt))
    raise NoDetour('no collision-free local route to the next required anchor')


@dataclass
class RouteUpdate:
    points: list
    anchors: tuple
    changed: bool


def repair_leg(collision, leg, deadline):
    """Replace only the obstructed neighborhood, preserving clear prefix/suffix."""
    points = np.asarray(leg, float)
    changed = False
    for _ in range(8):
        if time.monotonic() > deadline:
            raise NoDetour('local planning budget exceeded')
        hits = collision.blocked_edges(points[:-1], points[1:])
        if not hits.any():
            return points, changed
        first = int(np.flatnonzero(hits)[0])
        last = first
        while last+1 < len(hits) and hits[last+1]:
            last += 1
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
        lo = max(0, int(np.searchsorted(arc, arc[first]-.40))-1)
        hi = min(len(points)-1, int(np.searchsorted(arc, arc[last+1]+.40)))
        detour = local_detour(collision, points[lo:hi+1], deadline)
        points = np.vstack((points[:lo], detour, points[hi+1:]))
        changed = True
    raise NoDetour('too many separate local obstructions')


def adapt_route(collision, route, anchors, current, index, budget_s=.25):
    """Keep every remaining anchor in order instead of planning straight to pad."""
    pts = np.asarray(route, float)
    current = xyz(current)
    if not 0 <= index < len(pts):
        raise ValueError('invalid route progress')
    remaining = [i for i in anchors if i > index]
    if not remaining or remaining[-1] != len(pts)-1:
        remaining.append(len(pts)-1)
    deadline = time.monotonic()+budget_s
    out, new_anchors, changed, previous = [current], [], False, index
    for end in remaining:
        if time.monotonic() > deadline:
            raise NoDetour('local planning budget exceeded')
        leg = np.vstack((out[-1], pts[previous+1:end+1]))
        leg, repaired = repair_leg(collision, leg, deadline)
        changed |= repaired
        out.extend(leg[1:])
        new_anchors.append(len(out)-1)
        previous = end
    return RouteUpdate([tuple(float(v) for v in p) for p in out], tuple(new_anchors), changed)


def source_identity(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def direction_observed(current, target, camera_origin, returns):
    """Require depth returns looking into the intended travel direction.

    This is a necessary visibility gate, not proof of full free-space volume.
    A fresh forward-facing cloud does not justify flying sideways/backwards.
    Full calibrated depth coverage and braking dynamics need qualification.
    """
    delta = xyz(target)-xyz(current)
    step = np.linalg.norm(delta)
    if step < 1e-6:
        return True
    if camera_origin is None:
        return False
    rays = np.asarray(returns, float).reshape(-1, 3)-xyz(camera_origin)
    lengths = np.linalg.norm(rays, axis=1)
    along = rays @ (delta/step)
    aligned = along/np.maximum(lengths, 1e-9) >= math.cos(math.radians(10))
    # Explicit conservative lookahead assumption at the capped .30 m/s.
    valid = aligned & (along >= max(.40, step))
    angular_cells = np.floor((rays[valid]/lengths[valid, None])/.01).astype(np.int64)
    return len(np.unique(angular_cells, axis=0)) >= 10
