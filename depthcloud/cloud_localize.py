#!/usr/bin/env python3
"""
cloud_localize.py: find the drone's camera inside a known course model (.obj) from the live stereo cloud.

Runs ALONGSIDE stereo_cloud.py (which it does not change or import). It subscribes to
stereo_cloud's /team/stereo/points (left camera optical frame), matches each cloud against the
.obj with point-to-plane ICP, starting from the previous pose, and outputs:

    /team/localize/pose     geometry_msgs/PoseStamped: camera pose in the .obj frame
                            (frame_id "course", Z up, metres; orientation is the camera's
                            forward-left-up body frame, so x = where the camera looks)
    http://<jetson>:8082/   3D viewer: the course model (ceiling hidden), the live cloud placed
                            by the current estimate, and the drone as a dot with heading + trail.
                            Has a form to reset the pose if tracking is lost.

It needs a starting guess: where the camera is and which way it faces (init_* parameters, or the
viewer's reset form). From there it tracks frame to frame, so move slowly (walking pace, slow turns)
until we add odometry. It reports the CAMERA's position, not the drone's centre.

Run INSIDE the stack container, with stereo_cloud.py already running (host/run.sh shell):
    python3 /root/team_ws/src/dronehacks/cloud_localize.py --ros-args \\
        -p obj:=/root/team_ws/src/dronehacks/test_room.obj \\
        -p init_x:=1.83 -p init_y:=1.0 -p init_z:=0.5 -p init_yaw_deg:=90.0

Check the matching maths offline (no ROS, no drone; needs numpy + OpenCV, scipy optional):
    python3 cloud_localize.py --selftest test_room.obj

Coordinates: the .obj frame, converted to Z up (obj_up:=y for a Y-up file). For test_room.obj
the origin is the marked floor corner, x along the 30 ft wall, y along the 34 ft wall.
Yaw 0 = camera looking along +x, yaw 90 = along +y. Pitch is positive when looking down.
"""
import json
import math
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

# camera optical frame (x right, y down, z forward) -> camera body frame (x forward, y left, z up)
R_OPT_TO_FLU = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
FT = 0.3048


# ----------------------------------------------------------------------------------------------
# Course model
# ----------------------------------------------------------------------------------------------

def load_obj(path, up="z", scale=1.0):
    """Vertices (N,3) in metres, Z up; polygons as vertex-index lists; and the `g` group name
    each polygon belongs to (e.g. "Boulder 1", "Net side east")."""
    V, F, G = [], [], []
    group = ""
    with open(path) as fh:
        for line in fh:
            t = line.split()
            if not t:
                continue
            if t[0] == "v":
                V.append([float(a) for a in t[1:4]])
            elif t[0] == "g":
                group = " ".join(t[1:])
            elif t[0] == "f":
                idx = []
                for a in t[1:]:
                    i = int(a.split("/")[0])
                    idx.append(i - 1 if i > 0 else len(V) + i)
                if len(idx) >= 3:
                    F.append(idx)
                    G.append(group)
    V = np.asarray(V, float) * scale
    if up == "y":                                   # Y-up file: (x, y, z) -> (x, -z, y)
        V = np.stack([V[:, 0], -V[:, 2], V[:, 1]], axis=1)
    elif up != "z":
        raise ValueError("obj_up must be 'z' or 'y'")
    return V, F, G


def poly_normal(V, poly):
    n = np.zeros(3)
    for i in range(1, len(poly) - 1):
        n += np.cross(V[poly[i]] - V[poly[0]], V[poly[i + 1]] - V[poly[0]])
    ln = np.linalg.norm(n)
    return n / ln if ln > 0 else n


def is_ceiling(V, F, tol=0.02):
    """Per polygon: True for horizontal faces at the model's very top (room ceiling / net roof)."""
    ztop = V[:, 2].max()
    return [all(V[i, 2] > ztop - tol for i in poly) and abs(poly_normal(V, poly)[2]) > 0.9 for poly in F]


def triangulate(F):
    """Fan-triangulate polygons; also returns which polygon each triangle came from."""
    tris, parent = [], []
    for k, p in enumerate(F):
        for i in range(1, len(p) - 1):
            tris.append([p[0], p[i], p[i + 1]])
            parent.append(k)
    return np.array(tris, dtype=np.int64).reshape(-1, 3), np.array(parent, dtype=np.int64)


def sample_surface(V, tris, spacing, rng):
    """Random points on the mesh surface, about one per spacing^2 of AREA (so finely tessellated
    parts like boulders aren't over-weighted), with face normals and the source triangle index."""
    a, b, c = V[tris[:, 0]], V[tris[:, 1]], V[tris[:, 2]]
    n = np.cross(b - a, c - a)
    area = 0.5 * np.linalg.norm(n, axis=1)
    tri_idx = np.nonzero(area > 1e-9)[0]
    a, b, c, n, area = a[tri_idx], b[tri_idx], c[tri_idx], n[tri_idx], area[tri_idx]
    n = n / (2.0 * area)[:, None]
    counts = np.floor(area / spacing ** 2 + rng.random(len(area))).astype(np.int64)   # unbiased rounding
    idx = np.repeat(np.arange(len(a)), counts)
    r1, r2 = np.sqrt(rng.random(len(idx))), rng.random(len(idx))
    p = (1 - r1)[:, None] * a[idx] + (r1 * (1 - r2))[:, None] * b[idx] + (r1 * r2)[:, None] * c[idx]
    return p, n[idx], tri_idx[idx]


def feature_edges(V, F, min_angle_deg=35.0):
    """Edges worth drawing: outlines and creases sharper than min_angle (not the diagonals of
    flat faces, and not the thousands of edges on a smooth boulder)."""
    normals = [poly_normal(V, p) for p in F]
    faces_of = {}
    for k, poly in enumerate(F):
        for i in range(len(poly)):
            e = (min(poly[i], poly[(i + 1) % len(poly)]), max(poly[i], poly[(i + 1) % len(poly)]))
            faces_of.setdefault(e, []).append(k)
    cos_t = math.cos(math.radians(min_angle_deg))
    out = []
    for e, ks in faces_of.items():
        if len(ks) != 2 or abs(float(normals[ks[0]] @ normals[ks[1]])) < cos_t:
            out.append(e)
    return out


class NearestNeighbour:
    """Nearest model sample for each query point: scipy's KD-tree if present, else OpenCV FLANN."""

    def __init__(self, pts):
        self.pts = np.ascontiguousarray(pts, dtype=np.float32)
        try:
            from scipy.spatial import cKDTree
            self.tree, self.kind = cKDTree(self.pts), "scipy"
        except ImportError:
            self.tree, self.kind = cv2.flann_Index(self.pts, dict(algorithm=1, trees=4)), "opencv-flann"

    def query(self, q, max_d=None):
        """Nearest sample index and distance. With max_d, points farther than that come back with
        distance inf (and an out-of-range index): much faster, since far queries are the slow ones."""
        q = np.ascontiguousarray(q, dtype=np.float32)
        if self.kind == "scipy":
            d, i = self.tree.query(q, k=1, distance_upper_bound=np.inf if max_d is None else max_d)
            return np.asarray(i), np.asarray(d)
        i, d2 = self.tree.knnSearch(q, 1, params=dict(checks=64))
        return i.ravel().astype(np.int64), np.sqrt(np.maximum(d2.ravel(), 0.0))


class CourseModel:
    def __init__(self, path, up="z", scale=1.0, spacing=0.04, match_ceiling=True, seed=0,
                 see_through=("net",), ignore=()):
        """Matching surface samples + what the viewer draws.

        - The viewer never draws the ceiling (or net roof). Matching keeps it by default: indoors
          the ceiling is often a third of the cloud.
        - `see_through`: groups whose name contains one of these (case-insensitive), e.g. the net.
          Stereo mostly sees through netting, so they're kept for matching (any strands it does
          see will match) but the self-test's simulated camera looks straight through them.
        - `ignore`: groups left out entirely."""
        V, F, G = load_obj(path, up, scale)
        has = lambda g, keys: any(k.lower() in g.lower() for k in keys)
        keep = [k for k in range(len(F)) if not has(G[k], ignore)]
        F, G = [F[k] for k in keep], [G[k] for k in keep]
        ceil = is_ceiling(V, F)
        match = [k for k in range(len(F)) if match_ceiling or not ceil[k]]
        if not match:
            raise ValueError(f"{path}: no faces left")
        self.V, self.F, self.G = V, [F[k] for k in match], [G[k] for k in match]
        tris, parent = triangulate(self.F)
        self.pts, self.normals, tri = sample_surface(V, tris, spacing, np.random.default_rng(seed))
        face = parent[tri]
        self.groups = sorted(set(self.G))
        self.see_through = np.array([has(self.G[f], see_through) for f in face], dtype=bool)
        self.is_ceiling = np.array([ceil[match[f]] for f in face], dtype=bool)
        self.nn = NearestNeighbour(self.pts)
        used = np.unique(np.concatenate([np.asarray(p) for p in F]))
        self.lo, self.hi = V[used].min(0), V[used].max(0)
        F_view = [F[k] for k in range(len(F)) if not ceil[k]]
        self.edges = feature_edges(V, F_view)
        self.view_pts = self.pts[~self.see_through & ~self.is_ceiling]      # faint dots in the viewer

    def edges_json(self, max_dots=15000):
        segs = [float(round(t, 4)) for i, j in self.edges for t in (*self.V[i], *self.V[j])]
        d = self.view_pts[:: max(1, int(math.ceil(len(self.view_pts) / max_dots)))]
        dots = [float(round(t, 3)) for t in d.ravel()]
        return json.dumps({"segments": segs, "dots": dots,
                           "lo": self.lo.round(4).tolist(), "hi": self.hi.round(4).tolist()}).encode()

    def group_centre(self, key):
        """Centre of the vertices of the first group whose name contains `key` (e.g. 'launch')."""
        for g in self.groups:
            if key.lower() in g.lower():
                vi = np.unique(np.concatenate([np.asarray(p) for p, gg in zip(self.F, self.G) if gg == g]))
                return (self.V[vi].min(0) + self.V[vi].max(0)) / 2.0
        return None


# ----------------------------------------------------------------------------------------------
# Pose maths + ICP
# ----------------------------------------------------------------------------------------------

def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def camera_pose(x, y, z, yaw_deg, pitch_deg=0.0, roll_deg=0.0):
    """4x4 transform taking camera OPTICAL-frame points into the course frame."""
    T = np.eye(4)
    T[:3, :3] = (rot_z(math.radians(yaw_deg)) @ rot_y(math.radians(pitch_deg))
                 @ rot_x(math.radians(roll_deg)) @ R_OPT_TO_FLU)
    T[:3, 3] = (x, y, z)
    return T


def pose_angles(T):
    """(yaw, pitch, roll) in degrees of the camera body frame; pitch > 0 = looking down."""
    R = T[:3, :3] @ R_OPT_TO_FLU.T
    f = R[:, 0]
    yaw, pitch = math.atan2(f[1], f[0]), math.atan2(-f[2], math.hypot(f[0], f[1]))
    Rr = (rot_z(yaw) @ rot_y(pitch)).T @ R
    return math.degrees(yaw), math.degrees(pitch), math.degrees(math.atan2(Rr[2, 1], Rr[1, 1]))


def quat_to_mat(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def quat_xyzw(R):
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0, R[1, 0] - R[0, 1])
    return x, y, z, w


def voxel_down(p, voxel):
    if len(p) == 0:
        return p
    _, i = np.unique(np.floor(p / voxel).astype(np.int64), axis=0, return_index=True)
    return p[np.sort(i)]


def transform(T, p):
    return p @ T[:3, :3].T + T[:3, 3]


def huber_w(r, k):
    a = np.abs(r)
    return np.where(a < k, 1.0, k / np.maximum(a, 1e-12))


def tukey_w(r, c):
    """Tukey biweight: full weight near 0, falling to exactly 0 at |r| >= c (clutter is ignored)."""
    u = np.clip(np.abs(r) / c, 0.0, 1.0)
    return (1.0 - u * u) ** 2


# ICP settings; the self-test can override them (icp.kernel=tukey icp.c=0.1 icp.schedule=0.3,0.2,0.1)
# Tukey + a tight schedule keeps people/chairs near modelled surfaces from dragging the pose
# (self-test with clutter=1.0: BAD 0 for tracking-sized and reset-sized start errors).
ICP_OPTS = {"schedule": (0.3, 0.2, 0.12, 0.08), "kernel": "tukey", "c": 0.06, "keep": 0.03, "anneal": 0.5}
# anneal > 0: the robust cut-off for each pass is max(c, anneal x that pass's search radius), so the
# coarse passes can still pull a heading that's 10-15 deg off before the tight passes refine it.
# 0.5 beat 0 on every comparison: window-route flights (median ~9 vs ~28 cm) and the clutter=1.0
# single-frame test (PASS 23 / BAD 1 vs PASS 17 / BAD 2 of 40).


def icp(src, T0, model, schedule=None, iters=5, huber=None, min_pts=150,
        lever=2.0, keep_ratio=None):
    """Point-to-plane ICP of camera-frame points `src` onto the model, starting at T0.

    Degeneracy-aware: each step only moves along directions the visible surfaces actually pin down
    (eigenvalues of the scaled Hessian above keep_ratio x the largest). A view of just floor, or of
    floor plus table tops, can't fix horizontal position, so those directions keep the starting
    value instead of drifting on noise. `lever` (m) puts rotations and translations on one scale.
    Returns (T, info). info['ok'] is False if there weren't enough matches; info['free'] is how many
    of the 6 directions were left unconstrained."""
    schedule = ICP_OPTS["schedule"] if schedule is None else schedule
    c_k = ICP_OPTS["c"] if huber is None else huber
    weight = tukey_w if ICP_OPTS["kernel"] == "tukey" else huber_w
    keep_ratio = ICP_OPTS["keep"] if keep_ratio is None else keep_ratio
    T = T0.copy()
    nn, mn = model.nn, model.normals
    Dinv = np.diag([1.0 / lever] * 3 + [1.0] * 3)
    free, ratio, Vk = 6, 0.0, np.zeros((6, 0))
    for max_d in schedule:
        for _ in range(iters):
            p = transform(T, src)
            idx, d = nn.query(p, max_d)
            m = d < max_d
            if m.sum() < min_pts:
                return T0, {"ok": False, "matches": int(m.sum()), "reason": "too few matches"}
            P, Q, N = p[m], model.pts[idx[m]].astype(float), mn[idx[m]]
            # Rotate about the CAMERA, not the cloud centroid: then a heading correction doesn't
            # swing the camera sideways, and holding an unconstrained direction really holds it.
            c = T[:3, 3].copy()
            r = np.einsum("ij,ij->i", N, P - Q)                       # r = n.(p - q)
            J = np.hstack([np.cross(P - c, N), N])                     # d r / d[rot, trans]
            w = weight(r, max(c_k, ICP_OPTS["anneal"] * max_d))
            Hs = Dinv @ (J.T @ (J * w[:, None])) @ Dinv                # scaled Hessian / gradient
            gs = Dinv @ (J.T @ (w * r))
            lam, V = np.linalg.eigh(Hs)
            keep = lam > keep_ratio * lam[-1]
            free, ratio = int((~keep).sum()), float(lam[0] / max(lam[-1], 1e-12))
            Vk = V[:, keep]
            delta = Dinv @ (-Vk @ ((Vk.T @ gs) / lam[keep]))
            R = cv2.Rodrigues(delta[:3].reshape(3, 1))[0]
            dT = np.eye(4)
            dT[:3, :3] = R
            dT[:3, 3] = c + delta[3:] - R @ c                         # rotate about the camera
            T = dT @ T
            if np.linalg.norm(delta) < 1e-5:
                break
    p = transform(T, src)
    _, d = nn.query(p, 0.08)
    inl = d < 0.08
    # How much of each world quantity this frame actually measured: the length of its direction's
    # projection onto the subspace ICP was allowed to move in (1 = fully measured, 0 = held at the
    # previous value). Order of the 6 directions: [rot x, rot y, rot z, x, y, z].
    probe = {"x": 3, "y": 4, "height": 5, "heading": 2}
    proj = {k: float(np.linalg.norm(Vk[i])) for k, i in probe.items()}
    measured = {k: v > 0.9 for k, v in proj.items()}
    return T, {"ok": True, "fitness": float(inl.mean()), "rmse": float(np.sqrt(np.mean(d[inl] ** 2))) if inl.any() else 1.0,
               "matches": int(inl.sum()), "free": free, "constraint": ratio, "measured": measured, "proj": proj}


class Localizer:
    """Frame-to-model tracking with sanity gates; keeps the last good pose when a frame is rejected."""

    def __init__(self, model, T0, max_range=5.0, voxel=0.05, max_points=1500,
                 min_fitness=0.35, min_matches=300, max_rmse=0.06, max_jump=0.6, max_turn_deg=30.0,
                 max_speed=3.0, max_turn_rate=360.0):
        """A frame is accepted if its matched points fit tightly (rmse) AND there are enough of them:
        either min_fitness of the cloud, or at least min_matches points. The absolute count matters in
        a room full of people and chairs that aren't in the model, where most points can't match."""
        self.model, self.T = model, T0.copy()
        self.max_range, self.voxel, self.max_points = max_range, voxel, max_points
        self.min_fitness, self.min_matches, self.max_rmse = min_fitness, min_matches, max_rmse
        self.max_jump, self.max_turn = max_jump, math.radians(max_turn_deg)
        self.rng = np.random.default_rng(1)
        self.status, self.info, self.rejects = "waiting", {}, 0
        self.last_good, self.measured = None, {}                   # monotonic time, what was measured
        self.trail = deque(maxlen=600)
        self.searching, self.odom = False, False
        self.max_speed, self.max_turn_rate = max_speed, max_turn_rate      # m/s, deg/s: odometry sanity
        self.odom_rejects, self.odom_note, self.last_delta = 0, "", None

    def reset(self, T, search_heading=False):
        """New start pose. With search_heading, the next frames try every heading from this position
        (the drone's position is known, e.g. the launch mark, but not which way it points)."""
        self.T, self.status, self.rejects = T.copy(), "searching" if search_heading else "reset", 0
        self.searching = search_heading
        self.trail.clear()

    def search_heading(self, src, step_deg=20.0, ambiguity=0.75):
        """ICP from every heading (step_deg apart) at the current position/height/tilt. Returns
        (T, info) of the best, or (None, info) if nothing fits or two different headings fit about
        equally well (second best facing > 30 deg away has >= ambiguity x the best's matches)."""
        x, y, z = self.T[:3, 3]
        yaw0, pitch0, roll0 = pose_angles(self.T)
        cands = []
        for k in range(int(round(360.0 / step_deg))):
            T, info = icp(src, camera_pose(x, y, z, yaw0 + k * step_deg, pitch0, roll0), self.model)
            if info["ok"] and info["rmse"] <= self.max_rmse:
                cands.append((info["matches"], T, info))
        if not cands:
            return None, {"ok": False, "reason": "no heading fits"}
        cands.sort(key=lambda c: -c[0])
        best_m, best_T, best_info = cands[0]
        best_yaw = pose_angles(best_T)[0]
        rival = next((c for c in cands[1:]
                      if abs((pose_angles(c[1])[0] - best_yaw + 180) % 360 - 180) > 30), None)
        best_info = dict(best_info, heading_candidates=len(cands),
                         rival_ratio=None if rival is None else round(rival[0] / max(best_m, 1), 2))
        enough = best_info["fitness"] >= self.min_fitness or best_m >= self.min_matches
        if not enough:
            return None, dict(best_info, ok=False, reason="poor fit at every heading")
        if rival is not None and rival[0] >= ambiguity * best_m:
            return None, dict(best_info, ok=False,
                              reason=f"ambiguous heading ({best_yaw:.0f}° vs {pose_angles(rival[1])[0]:.0f}°)")
        return best_T, best_info

    def prepare(self, pts_cam):
        z = pts_cam[:, 2]
        p = pts_cam[np.isfinite(z) & (z > 0.3) & (z < self.max_range)]
        if len(p) > 8 * self.max_points:                              # live clouds are big: thin first
            p = p[self.rng.choice(len(p), 8 * self.max_points, replace=False)]
        p = voxel_down(p, self.voxel)
        if len(p) > self.max_points:
            p = p[self.rng.choice(len(p), self.max_points, replace=False)]
        return p

    def odom_plausible(self, delta, dt):
        """Reject odometry steps the drone can't physically make (a VSLAM reset / relocalization jump
        looks like a teleport). dt = seconds since the previous frame."""
        dt = max(dt if dt is not None else 1.0 / 3.0, 0.05)
        step = float(np.linalg.norm(delta[:3, 3]))
        turn = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(delta[:3, :3]) - 1) / 2))))
        return step <= self.max_speed * dt + 0.05 and turn <= self.max_turn_rate * dt + 2.0, step, turn

    def update(self, pts_cam, delta=None, dt=None):
        """One frame. `delta` (optional, 4x4) is the camera's motion since the previous frame from
        odometry (VSLAM), in the previous camera's optical frame: T_now = T_prev @ delta. It's applied
        first as a prediction, so ICP starts from where the drone should be, directions the view can't
        pin down follow odometry instead of freezing, and frames with nothing to match (looking through
        the net) still move the estimate. Without it, the tracker holds the last pose.
        A delta faster than max_speed / max_turn_rate is treated as a VSLAM reset and ignored."""
        self.odom_note = ""
        if delta is not None:
            ok, step, turn = self.odom_plausible(delta, dt)
            if ok:
                self.last_delta = delta
            else:
                self.odom_rejects += 1
                self.odom_note = f"odometry jump rejected ({100 * step:.0f} cm, {turn:.0f}°): VSLAM reset?"
                # Don't assume the drone stopped: it was probably mid-move (or mid-turn, which the
                # camera may not be able to correct later). Repeat the last good step instead.
                delta = self.last_delta
        if delta is not None:
            self.T = self.T @ delta
        self.odom = delta is not None
        src = self.prepare(pts_cam)
        if len(src) < 150:
            self.status = "odometry" if self.odom else "no data"
            self.info = {"ok": False, "matches": len(src), "reason": "too few points in range"}
            if self.odom:
                self.trail.append(self.T[:3, 3].round(3).tolist())
            return False
        if self.searching:
            T, info = self.search_heading(src)
            self.info = info
            if T is None:
                self.status = "searching"
                return False
            self.searching, self.T, self.rejects = False, T, 0
            self.last_good, self.measured = time.monotonic(), info.get("proj", {})
            self.status = "tracking" if info["free"] == 0 else "weak"
            self.trail.append(self.T[:3, 3].round(3).tolist())
            return True
        T, info = icp(src, self.T, self.model)
        self.info = info
        good = (info["ok"] and info["rmse"] <= self.max_rmse
                and (info["fitness"] >= self.min_fitness or info["matches"] >= self.min_matches))
        if good:
            dR = T[:3, :3] @ self.T[:3, :3].T
            turn = math.acos(max(-1.0, min(1.0, (np.trace(dR) - 1) / 2)))
            if np.linalg.norm(T[:3, 3] - self.T[:3, 3]) > self.max_jump or turn > self.max_turn:
                good, info["reason"] = False, "jump too large"
        elif info["ok"]:
            info["reason"] = "poor fit"
        if good:
            self.T, self.rejects = T, 0
            self.last_good, self.measured = time.monotonic(), info.get("proj", {})
            self.status = "tracking" if info["free"] == 0 else "weak"
            self.trail.append(self.T[:3, 3].round(3).tolist())
        else:
            self.rejects += 1
            if self.odom:                                             # keep moving with odometry
                self.status = "odometry"
                self.trail.append(self.T[:3, 3].round(3).tolist())
            else:
                self.status = "lost" if self.rejects >= 5 else "holding"
        return good


# ----------------------------------------------------------------------------------------------
# Viewer
# ----------------------------------------------------------------------------------------------

VIEWER_HTML = r"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Course localization</title>
<style>
body{margin:0;background:#111;color:#ddd;font:14px sans-serif;overflow:hidden}
#gl{position:fixed;inset:0;width:100%;height:100%;display:block;touch-action:none;cursor:grab}
.panel{position:fixed;background:rgba(0,0,0,.72);padding:8px 10px;border-radius:6px;line-height:1.6}
#hud{top:8px;left:8px;min-width:250px;max-width:calc(100% - 32px)}
#ctl{bottom:8px;right:8px;max-width:270px}
#ctl summary{cursor:pointer;color:#bbb}
#status{font-weight:bold}
#big{font-size:20px;font-weight:bold;letter-spacing:.3px}
#est{color:#bbb}
.ok{color:#6d6}.held{color:#dd6}.none{color:#888}
@media (max-width:640px){body{font-size:12px}#big{font-size:16px}.panel{padding:5px 7px;line-height:1.45}
  #hud small,#pos,#fit{display:none}#ctl small{display:none}}
.tracking{color:#6d6}.weak{color:#dd6}.holding{color:#e93}.lost,.no-data{color:#f55}.waiting,.reset,.searching{color:#9cf}.odometry{color:#6af}
button{background:#333;color:#ddd;border:1px solid #555;border-radius:3px;padding:2px 8px;margin:2px 2px 2px 0;cursor:pointer}
input,select{background:#222;color:#ddd;border:1px solid #555;border-radius:3px;width:52px;padding:1px 3px}
label{display:inline-block;margin-right:4px}
.key{display:inline-block;width:9px;height:9px;border-radius:2px;margin:0 3px 0 6px}
small{color:#999}
</style></head><body>
<canvas id="gl"></canvas>
<div id="hud" class="panel">
<div>best estimate · <span id="status" class="waiting">waiting</span> <small id="reason"></small> <small id="age"></small></div>
<div id="big">-</div>
<div id="pos">position: -</div>
<div id="ang">heading: -</div>
<div id="est">measured: -</div>
<div id="simerr" style="display:none;color:#fff"></div>
<div id="fit"><small>fit: -</small></div>
<small>legend:<span class="key" style="background:#3ddc6b"></span>measured
<span class="key" style="background:#e6d24a"></span>partly held <span class="key" style="background:#5aaaff"></span>odometry only <span class="key" style="background:#888"></span>stale
· cloud: <span class="key" style="background:#50dc78"></span>matched <span class="key" style="background:#ff8c3c"></span>unmatched</small>
</div>
<div id="ctl" class="panel">
<button id="top">top view</button><button id="persp">3D view</button><button id="follow">follow: off</button><button id="clear">clear trail</button>
<details><summary>reset pose</summary>
<small>where the camera is now</small><br>
<label>x <input id="rx" value="24"></label><label>y <input id="ry" value="28"></label><label>h <input id="rz" value="2.8"></label>
<select id="ru"><option value="ft">ft</option><option value="m">m</option></select><br>
<label>yaw° <input id="ryaw" value="-90"></label><label>pitch° <input id="rp" value="0"></label><br>
<label><input type="checkbox" id="rsearch" style="width:auto"> heading unknown: search it</label><br>
<button id="reset">reset here</button><button id="usecur">fill from current</button><br>
<small>yaw 0 = looking along +x, 90 = along +y.</small>
</details>
<small>drag: orbit · shift/right-drag: pan · scroll: zoom</small>
</div>
<script>
"use strict";
const $ = id => document.getElementById(id);
const cv = $('gl'), gl = cv.getContext('webgl');
if (!gl) { $('status').textContent = 'WebGL not available'; throw new Error('no webgl'); }
const FT = 0.3048;

function shader(type, src) { const s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s);
  if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s)); return s; }
const prog = gl.createProgram();
// course frame (x, y, z up, metres) -> GL (x, y up, -z): (x, z, -y)
gl.attachShader(prog, shader(gl.VERTEX_SHADER, `
attribute vec3 p; attribute vec3 col; uniform mat4 m; uniform float psize; varying vec3 c;
void main(){ gl_Position = m * vec4(p.x, p.z, -p.y, 1.); gl_PointSize = psize; c = col; }`));
gl.attachShader(prog, shader(gl.FRAGMENT_SHADER, `precision mediump float; varying vec3 c; uniform float round_;
void main(){ if (round_ > 0.5 && length(gl_PointCoord - 0.5) > 0.5) discard; gl_FragColor = vec4(c, 1.); }`));
gl.linkProgram(prog);
const pL = gl.getAttribLocation(prog, 'p'), cL = gl.getAttribLocation(prog, 'col');
const mL = gl.getUniformLocation(prog, 'm'), sL = gl.getUniformLocation(prog, 'psize'), rL = gl.getUniformLocation(prog, 'round_');

function mkbuf() { return {pos: gl.createBuffer(), col: gl.createBuffer(), n: 0}; }
function fill(b, pos, cols) {
  gl.bindBuffer(gl.ARRAY_BUFFER, b.pos); gl.bufferData(gl.ARRAY_BUFFER, pos, gl.DYNAMIC_DRAW);
  gl.bindBuffer(gl.ARRAY_BUFFER, b.col); gl.bufferData(gl.ARRAY_BUFFER, cols, gl.DYNAMIC_DRAW);
  b.n = pos.length / 3;
}
function solid(n, rgb) { const c = new Uint8Array(n * 3); for (let i = 0; i < n; i++) c.set(rgb, i * 3); return c; }
const B = {model: mkbuf(), dots: mkbuf(), grid: mkbuf(), cloud: mkbuf(), trail: mkbuf(), drone: mkbuf(),
           arrow: mkbuf(), stem: mkbuf(), wedge: mkbuf(), ring: mkbuf(), truth: mkbuf(), truthHead: mkbuf()};
const COL = {ok: [61, 220, 107], held: [230, 210, 74], odom: [90, 170, 255], stale: [136, 136, 136]};
const ARROWS = ['→', '↗', '↑', '↖', '←', '↙', '↓', '↘'];          // as seen in top view (+x right, +y up)

// --- camera ---
const sub=(a,b)=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]], dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const norm=a=>{const l=Math.hypot(...a)||1;return [a[0]/l,a[1]/l,a[2]/l];};
function lookAt(e,t,u){const z=norm(sub(e,t)),x=norm(cross(u,z)),y=cross(z,x);
  return [x[0],y[0],z[0],0, x[1],y[1],z[1],0, x[2],y[2],z[2],0, -dot(x,e),-dot(y,e),-dot(z,e),1];}
function persp(f,a,n,fa){const t=1/Math.tan(f/2);return [t/a,0,0,0, 0,t,0,0, 0,0,(fa+n)/(n-fa),-1, 0,0,2*fa*n/(n-fa),0];}
function mul(a,b){const r=new Array(16);for(let i=0;i<4;i++)for(let j=0;j<4;j++){let s=0;
  for(let k=0;k<4;k++)s+=a[k*4+j]*b[i*4+k];r[i*4+j]=s;}return r;}
let lo = [0,0,0], hi = [9,10,3], cam, follow = false, drone = null;
const centre = () => [(lo[0]+hi[0])/2, 0.5, -(lo[1]+hi[1])/2];         // GL coords
const size = () => Math.max(hi[0]-lo[0], hi[1]-lo[1], 2);
function topView(){ cam = {t: centre(), yaw: 0, pitch: 1.5, dist: size() * 1.25}; }
function perspView(){ cam = {t: centre(), yaw: -0.6, pitch: 0.75, dist: size() * 1.2}; }
perspView();
function eye(){ const cp=Math.cos(cam.pitch);
  return [cam.t[0]+cam.dist*cp*Math.sin(cam.yaw), cam.t[1]+cam.dist*Math.sin(cam.pitch), cam.t[2]+cam.dist*cp*Math.cos(cam.yaw)]; }
let drag = null;
cv.addEventListener('contextmenu', e => e.preventDefault());
cv.addEventListener('pointerdown', e => { drag = {x:e.clientX, y:e.clientY, pan:e.button===2||e.shiftKey}; cv.setPointerCapture(e.pointerId); });
cv.addEventListener('pointerup', () => drag = null);
cv.addEventListener('pointermove', e => { if (!drag) return;
  const dx=e.clientX-drag.x, dy=e.clientY-drag.y; drag.x=e.clientX; drag.y=e.clientY;
  if (drag.pan) { const sy=Math.sin(cam.yaw), cy=Math.cos(cam.yaw), sp=Math.sin(cam.pitch), cp=Math.cos(cam.pitch);
    const r=[cy,0,-sy], u=[-sp*sy,cp,-sp*cy], k=cam.dist*0.0015;
    for (let i=0;i<3;i++) cam.t[i] += (-dx*r[i] + dy*u[i]) * k;
  } else { cam.yaw -= dx*0.006; cam.pitch = Math.max(-0.2, Math.min(1.55, cam.pitch + dy*0.006)); } });
cv.addEventListener('wheel', e => { e.preventDefault(); cam.dist = Math.max(0.5, Math.min(60, cam.dist*Math.exp(e.deltaY*0.001))); }, {passive:false});
$('top').onclick = topView; $('persp').onclick = perspView;
$('follow').onclick = e => { follow = !follow; e.target.textContent = 'follow: ' + (follow ? 'on' : 'off'); };
$('clear').onclick = () => fetch('/clear_trail');

// --- data ---
async function loadModel() {
  const m = await (await fetch('/model.json')).json();
  lo = m.lo; hi = m.hi; perspView();
  fill(B.model, new Float32Array(m.segments), solid(m.segments.length / 3, [200, 200, 210]));
  if (m.dots) fill(B.dots, new Float32Array(m.dots), solid(m.dots.length / 3, [95, 95, 108]));  // surfaces, faint
  const g = [];                                                       // 1 m floor grid
  for (let x = Math.ceil(lo[0]); x <= hi[0]; x++) g.push(x, lo[1], lo[2], x, hi[1], lo[2]);
  for (let y = Math.ceil(lo[1]); y <= hi[1]; y++) g.push(lo[0], y, lo[2], hi[0], y, lo[2]);
  fill(B.grid, new Float32Array(g), solid(g.length / 3, [55, 55, 62]));
}
async function pollCloud() {
  try { const r = await fetch('/cloud.bin', {cache:'no-store'});
    if (r.status === 200) { const buf = await r.arrayBuffer(), n = new DataView(buf).getUint32(0, true);
      const q = new Int16Array(buf, 4, n * 3), f = new Uint8Array(buf, 4 + n * 6, n);
      const pos = new Float32Array(n * 3), col = new Uint8Array(n * 3);
      for (let i = 0; i < n * 3; i++) pos[i] = q[i] / 1000;
      for (let i = 0; i < n; i++) col.set(f[i] ? [80, 220, 120] : [255, 140, 60], i * 3);
      fill(B.cloud, pos, col); } } catch (e) {}
  setTimeout(pollCloud, 350);
}
const fmt = (v, d=2) => (v >= 0 ? ' ' : '') + v.toFixed(d);
async function pollState() {
  try { const s = await (await fetch('/state.json', {cache:'no-store'})).json();
    $('status').textContent = s.status; $('status').className = s.status.replace(' ', '-');
    $('reason').textContent = s.reason ? '(' + s.reason + ')' : '';
    const [x, y, z] = s.position, m = s.measured || {};
    const odomOnly = s.status === 'odometry';
    const stale = !odomOnly && (s.age == null || s.age > 2.0);
    // m.* = fraction of that quantity the latest frame measured: >= 0.7 measured, 0.3-0.7 partly, < 0.3 held
    const posOk = m.x >= 0.7 && m.y >= 0.7, allOk = posOk && m.heading >= 0.7;
    const col = odomOnly ? COL.odom : stale ? COL.stale : allOk ? COL.ok : COL.held;
    const arrowCh = ARROWS[((Math.round(s.yaw / 45) % 8) + 8) % 8];
    $('big').textContent = `${(x/FT).toFixed(1)} ft, ${(y/FT).toFixed(1)} ft   ${arrowCh} ${s.yaw.toFixed(0)}°`;
    $('big').style.color = `rgb(${col})`;
    $('age').textContent = odomOnly ? (s.age == null ? '(following odometry; no course match yet)'
        : `(following odometry; course matched ${s.age.toFixed(0)} s ago)`)
      : s.age == null ? '(no accepted frame yet)' : stale ? `(last update ${s.age.toFixed(0)} s ago)` : '';
    $('pos').textContent = `x ${x.toFixed(2)} m  y ${y.toFixed(2)} m  camera height ${z.toFixed(2)} m`;
    $('ang').textContent = `heading ${s.yaw.toFixed(0)}°  pitch ${s.pitch.toFixed(0)}°  roll ${s.roll.toFixed(0)}°`;
    const tag = (k, label) => odomOnly ? `<span class="none">${label} odometry</span>`
      : stale || !(k in m) ? `<span class="none">${label} ?</span>`
      : m[k] >= 0.7 ? `<span class="ok">${label} ✓</span>`
      : m[k] >= 0.3 ? `<span class="held">${label} partly (${(100*m[k]).toFixed(0)}%)</span>`
      : `<span class="held">${label} held</span>`;
    $('est').innerHTML = `measured: ${tag('x', 'x')} ${tag('y', 'y')} ${tag('height', 'height')} ${tag('heading', 'heading')}`;
    $('fit').innerHTML = s.fitness == null ? '<small>fit: -</small>' :
      `<small>fit ${(100*s.fitness).toFixed(0)}%  rmse ${(100*s.rmse).toFixed(1)} cm  ${s.ms.toFixed(0)} ms  ${s.hz.toFixed(1)} Hz</small>`;
    if (s.odom_source || s.odom_rejects)
      $('fit').innerHTML += `<br><small>odometry: ${s.odom_source || '-'}${s.odom_rejects ? ` · ${s.odom_rejects} jump(s) rejected` : ''}` +
        `${s.odom_note ? ` · <span class="held">${s.odom_note}</span>` : ''}</small>`;
    drone = s;
    fill(B.drone, new Float32Array(s.position), new Uint8Array(col));
    // heading arrow on the floor
    const a = s.yaw * Math.PI / 180, f2 = [Math.cos(a), Math.sin(a)], n2 = [-f2[1], f2[0]], fz = 0.02;
    const pt = (u, v) => [x + f2[0]*u + n2[0]*v, y + f2[1]*u + n2[1]*v, fz];
    fill(B.arrow, new Float32Array([...pt(0.85, 0), ...pt(-0.25, 0.32), ...pt(0.05, 0),
                                    ...pt(0.85, 0), ...pt(0.05, 0), ...pt(-0.25, -0.32)]), solid(6, col));
    fill(B.stem, new Float32Array([x, y, z, x, y, fz]), solid(2, col));
    // camera field of view (75 x 51 deg), 1.2 m deep
    const F = s.forward, R = norm(cross(F, [0, 0, 1])), U = cross(R, F), D = 1.2, wv = [];
    const cn = [[1, 1], [1, -1], [-1, -1], [-1, 1]].map(([i, j]) =>
      [0, 1, 2].map(k => s.position[k] + D * (F[k] + 0.767 * i * R[k] + 0.477 * j * U[k])));
    for (let i = 0; i < 4; i++) wv.push(...s.position, ...cn[i], ...cn[i], ...cn[(i + 1) % 4]);
    fill(B.wedge, new Float32Array(wv), solid(16, col));
    // dashed ring = horizontal position held (not measured) by the latest frame
    const rv = [];
    if (!stale && !posOk) for (let i = 0; i < 24; i += 2) {
      const t0 = i / 24 * 2 * Math.PI, t1 = (i + 1) / 24 * 2 * Math.PI;
      rv.push(x + 0.45 * Math.cos(t0), y + 0.45 * Math.sin(t0), fz, x + 0.45 * Math.cos(t1), y + 0.45 * Math.sin(t1), fz);
    }
    fill(B.ring, new Float32Array(rv), solid(rv.length / 3, COL.held));
    fill(B.trail, new Float32Array(s.trail.flat()), solid(s.trail.length, [255, 120, 160]));
    if (s.truth) {                                                    // simulation only: the true pose
      const tp = s.truth.position, tf = s.truth.forward;
      const err = Math.hypot(tp[0] - x, tp[1] - y, tp[2] - z);
      const ty = Math.atan2(tf[1], tf[0]) * 180 / Math.PI, herr = Math.abs(((s.yaw - ty + 540) % 360) - 180);
      $('simerr').style.display = '';
      $('simerr').textContent = `SIMULATION · true pose (white): error ${(100 * err).toFixed(0)} cm, heading ${herr.toFixed(1)}°`;
      fill(B.truth, new Float32Array(tp), new Uint8Array([255, 255, 255]));
      fill(B.truthHead, new Float32Array([...tp, tp[0] + 0.6 * tf[0], tp[1] + 0.6 * tf[1], tp[2] + 0.6 * tf[2]]),
           solid(2, [255, 255, 255]));
    }
  } catch (e) { $('status').textContent = 'connection lost'; $('status').className = 'lost'; }
  setTimeout(pollState, 250);
}
$('usecur').onclick = () => { if (!drone) return; const u = $('ru').value === 'ft' ? FT : 1;
  $('rx').value = (drone.position[0]/u).toFixed(2); $('ry').value = (drone.position[1]/u).toFixed(2);
  $('rz').value = (drone.position[2]/u).toFixed(2); $('ryaw').value = drone.yaw.toFixed(0); $('rp').value = drone.pitch.toFixed(0); };
$('reset').onclick = () => { const u = $('ru').value === 'ft' ? FT : 1, v = id => parseFloat($(id).value);
  if ([v('rx'), v('ry'), v('rz'), v('ryaw'), v('rp')].some(isNaN)) { alert('fill in every field'); return; }
  fetch(`/reset?x=${v('rx')*u}&y=${v('ry')*u}&z=${v('rz')*u}&yaw=${v('ryaw')}&pitch=${v('rp')}&search=${$('rsearch').checked ? 1 : 0}`); };

function draw(b, mode, psize, round) {
  if (!b.n) return;
  gl.bindBuffer(gl.ARRAY_BUFFER, b.pos); gl.vertexAttribPointer(pL, 3, gl.FLOAT, false, 0, 0);
  gl.bindBuffer(gl.ARRAY_BUFFER, b.col); gl.vertexAttribPointer(cL, 3, gl.UNSIGNED_BYTE, true, 0, 0);
  gl.uniform1f(sL, psize); gl.uniform1f(rL, round ? 1 : 0); gl.drawArrays(mode, 0, b.n);
}
function frame() {
  const dpr = window.devicePixelRatio || 1, w = Math.round(cv.clientWidth*dpr), h = Math.round(cv.clientHeight*dpr);
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  if (follow && drone) cam.t = [drone.position[0], drone.position[2], -drone.position[1]];
  gl.viewport(0, 0, w, h); gl.clearColor(0.07, 0.07, 0.08, 1); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  gl.enable(gl.DEPTH_TEST); gl.useProgram(prog);
  gl.enableVertexAttribArray(pL); gl.enableVertexAttribArray(cL);
  gl.uniformMatrix4fv(mL, false, new Float32Array(mul(persp(0.9, w/Math.max(h,1), 0.05, 200), lookAt(eye(), cam.t, [0,1,0]))));
  draw(B.grid, gl.LINES, 1, false); draw(B.dots, gl.POINTS, 1.5*dpr, false); draw(B.model, gl.LINES, 1, false);
  draw(B.cloud, gl.POINTS, 2*dpr, false); draw(B.trail, gl.LINE_STRIP, 1, false);
  gl.disable(gl.DEPTH_TEST);                                          // estimate always visible
  draw(B.arrow, gl.TRIANGLES, 1, false); draw(B.ring, gl.LINES, 1, false);
  draw(B.stem, gl.LINES, 1, false); draw(B.wedge, gl.LINES, 1, false);
  draw(B.drone, gl.POINTS, 14*dpr, true);
  draw(B.truthHead, gl.LINES, 1, false); draw(B.truth, gl.POINTS, 8*dpr, true);
  requestAnimationFrame(frame);
}
loadModel().then(() => { pollCloud(); pollState(); requestAnimationFrame(frame); });
</script></body></html>
""".encode("utf-8")


class WebServer:
    def __init__(self, port, model, node):
        self.model_json = model.edges_json()
        self.node, srv = node, self

        class H(BaseHTTPRequestHandler):
            def _send(self, ctype, body, code=200):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                u = urlparse(self.path)
                if u.path == "/":
                    self._send("text/html; charset=utf-8", VIEWER_HTML)
                elif u.path == "/model.json":
                    self._send("application/json", srv.model_json)
                elif u.path == "/state.json":
                    self._send("application/json", json.dumps(srv.node.state()).encode())
                elif u.path == "/cloud.bin":
                    body = srv.node.cloud_bytes()
                    if body is None:
                        self.send_response(204)                         # no cloud yet: nothing to draw
                        self.send_header("Cache-Control", "no-store")
                        self.end_headers()
                    else:
                        self._send("application/octet-stream", body)
                elif u.path == "/reset":
                    try:
                        q = {k: float(v[0]) for k, v in parse_qs(u.query).items()}
                        srv.node.reset(q["x"], q["y"], q["z"], q["yaw"], q.get("pitch", 0.0),
                                       search=q.get("search", 0.0) > 0)
                        self._send("application/json", b'{"ok":true}')
                    except (KeyError, ValueError):
                        self.send_error(400, "need numeric x, y, z, yaw (pitch optional)")
                elif u.path == "/clear_trail":
                    srv.node.clear_trail()
                    self._send("application/json", b'{"ok":true}')
                else:
                    self.send_error(404)

            def log_message(self, *a):
                pass

        self.httpd = ThreadingHTTPServer(("0.0.0.0", port), H)
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()


class Tracker:
    """Everything shared between the ROS callback and the web server (ROS-free, so it is testable)."""

    def __init__(self, model, T0, **loc_args):
        self.model, self.loc = model, Localizer(model, T0, **loc_args)
        self.lock = threading.Lock()
        self.ms, self.hz, self.last_t, self.view_req = 0.0, 0.0, None, 0.0
        self.display = None                                        # (camera-frame points, matched flags)

    def process(self, pts_cam, delta=None, dt=None):
        t0 = time.perf_counter()
        with self.lock:
            self.loc.update(pts_cam, delta, dt)
            T = self.loc.T.copy()
        disp = None
        if time.monotonic() - self.view_req < 3.0:                    # only when someone is watching
            d = pts_cam[np.isfinite(pts_cam[:, 2]) & (pts_cam[:, 2] > 0.3) & (pts_cam[:, 2] < 5.0)]
            if len(d) > 12000:                                          # the viewer only needs a sketch
                d = d[:: int(math.ceil(len(d) / 12000))]
            _, dist = self.model.nn.query(transform(T, d), 0.08)
            disp = (d, dist < 0.08)
        now = time.monotonic()
        with self.lock:
            if disp is not None:
                self.display = disp
            self.ms = 0.8 * self.ms + 0.2 * (time.perf_counter() - t0) * 1000.0
            if self.last_t is not None:
                self.hz = 0.8 * self.hz + 0.2 / max(now - self.last_t, 1e-3)
            self.last_t = now
            return self.loc.T.copy(), self.loc.status

    def reset(self, x, y, z, yaw, pitch=0.0, search=False):
        with self.lock:
            self.loc.reset(camera_pose(x, y, z, yaw, pitch), search_heading=search)

    def clear_trail(self):
        with self.lock:
            self.loc.trail.clear()

    def state(self):
        with self.lock:
            T, loc = self.loc.T, self.loc
            yaw, pitch, roll = pose_angles(T)
            info = loc.info
            return {"status": loc.status, "reason": info.get("reason", "") if loc.status not in ("tracking", "weak") else "",
                    "position": T[:3, 3].round(3).tolist(), "forward": (T[:3, :3] @ R_OPT_TO_FLU.T)[:, 0].round(3).tolist(),
                    "yaw": yaw, "pitch": pitch, "roll": roll,
                    "fitness": info.get("fitness"), "rmse": info.get("rmse"), "ms": self.ms, "hz": self.hz,
                    "measured": loc.measured, "odom": loc.odom, "truth": getattr(self, "truth", None),
                    "odom_source": getattr(self, "odom_source", ""), "odom_note": loc.odom_note,
                    "odom_rejects": loc.odom_rejects,
                    "age": None if loc.last_good is None else time.monotonic() - loc.last_good,
                    "trail": list(loc.trail)}

    def cloud_bytes(self):
        with self.lock:
            self.view_req = time.monotonic()
            if self.display is None:
                return None
            d, ok = self.display
            T = self.loc.T.copy()
        p = transform(T, d)
        q = np.clip(np.round(p * 1000.0), -32767, 32767).astype("<i2")
        return (np.array([len(p)], "<u4").tobytes() + np.ascontiguousarray(q).tobytes()
                + ok.astype(np.uint8).tobytes())


# ----------------------------------------------------------------------------------------------
# Offline self-test: simulated views of the model, perturbed start, check ICP recovers the pose
# ----------------------------------------------------------------------------------------------

def simulate_view(model, T_cam, rng, w=96, h=60, hfov_deg=72.0, max_range=5.0, outliers=0.05):
    """Visible model surface points in the camera optical frame, with stereo-like noise."""
    Ti = np.linalg.inv(T_cam)
    pc = transform(Ti, model.pts[~model.see_through])                # stereo looks through the net
    z = pc[:, 2]
    f = (w / 2) / math.tan(math.radians(hfov_deg) / 2)
    m = (z > 0.3) & (z < max_range)
    pc, z = pc[m], z[m]
    u, v = np.floor(f * pc[:, 0] / z + w / 2).astype(int), np.floor(f * pc[:, 1] / z + h / 2).astype(int)
    m = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    pc, z, pix = pc[m], z[m], (v[m] * w + u[m])
    order = np.lexsort((z, pix))                                    # nearest surface per pixel
    _, first = np.unique(pix[order], return_index=True)
    vis = pc[order[first]]
    vis = vis + rng.normal(0, 1, vis.shape) * (0.004 * vis[:, 2:3] ** 2)      # noise grows with z^2
    n_out = int(outliers * len(vis))
    junk = rng.uniform([-1.5, -1.0, 0.5], [1.5, 1.0, 3.0], (n_out, 3))       # people, chairs, etc.
    return np.vstack([vis, junk])


def selftest(obj_path, up="z", scale=1.0, trials=20, max_range=5.0, perturb=0.4, points=1500, clutter=0.05,
             launch=False, search=True, launch_yaw=None):
    """Graded the way the live tracker behaves (including its accept/reject gates):
    PASS   accepted, < 5 cm and < 2 deg
    CLOSE  accepted, < 10 cm and < 3 deg
    HELD   rejected, or flagged 'weak' without getting worse: keeps the previous pose (safe)
    BAD    accepted as 'tracking' but wrong, or 'weak' and got worse: the case that must not happen
    launch=True: the camera sits on the model's launch mark (15-30 cm up).
      search=True:  it faces a RANDOM heading; the localizer starts facing 90 deg and must search.
      search=False: it faces launch_yaw (random if None) and the localizer is told that heading
                    to within ~8 deg (the drone is placed on the mark facing a known way)."""
    t0 = time.perf_counter()
    model = CourseModel(obj_path, up, scale)
    print(f"model: {len(model.F)} faces matched (ceiling hidden only in the viewer), {len(model.pts):,} surface samples, "
          f"nearest-neighbour: {model.nn.kind}, built in {time.perf_counter() - t0:.1f} s")
    print(f"bounds (m): x {model.lo[0]:.2f}..{model.hi[0]:.2f}  y {model.lo[1]:.2f}..{model.hi[1]:.2f}  "
          f"z {model.lo[2]:.2f}..{model.hi[2]:.2f}")
    rng = np.random.default_rng(7)
    loc = Localizer(model, np.eye(4), max_range=max_range, max_points=points)
    print(f"max_range {max_range} m, start-error scale {perturb}, max points {points}, clutter {clutter}"
          + ((", LAUNCH heading search" if search else ", LAUNCH known heading") if launch else ""))
    lm = model.group_centre("launch") if launch else None
    if launch and lm is None:
        raise SystemExit("launch=1 needs a group with 'launch' in its name")
    if launch:
        print(f"launch mark at ({lm[0]:.3f}, {lm[1]:.3f}); localizer starts there facing 90°")
    counts, n = {"PASS": 0, "CLOSE": 0, "HELD": 0, "BAD": 0}, 0
    print(" #   true x,y,h (m)     yaw  | start error    | final error     fit   rmse    ms  free status    result")
    while n < trials:
        if launch:
            x, y = lm[0], lm[1]
            zc, pitch = rng.uniform(0.15, 0.3), rng.uniform(-3, 3)
            yaw = rng.uniform(-180, 180) if (search or launch_yaw is None) else launch_yaw
        else:
            x = rng.uniform(model.lo[0] + 0.5, model.hi[0] - 0.5)
            y = rng.uniform(model.lo[1] + 0.5, model.hi[1] - 0.5)
            zc, yaw, pitch = rng.uniform(0.4, 1.6), rng.uniform(-180, 180), rng.uniform(0, 20)
            if np.isfinite(model.nn.query(np.array([[x, y, zc]]), 0.35)[1][0]):
                continue                                              # inside or touching an obstacle
        T_true = camera_pose(x, y, zc, yaw, pitch)
        cloud = simulate_view(model, T_true, rng, max_range=max(5.0, max_range), outliers=clutter)
        if len(cloud) < 300:
            continue                                                  # facing something too close
        n += 1
        if launch and search:
            T0 = camera_pose(x + rng.normal(0, 0.05), y + rng.normal(0, 0.05), 0.2, 90.0, 0.0)
            loc.reset(T0, search_heading=True)
        elif launch:
            T0 = camera_pose(x + rng.normal(0, 0.05), y + rng.normal(0, 0.05), 0.2, yaw + rng.normal(0, 8), 0.0)
            loc.reset(T0)
        else:
            dx, dy = rng.normal(0, 0.15 * perturb, 2)
            T0 = camera_pose(x + dx, y + dy, zc + rng.normal(0, 0.05 * perturb),
                             yaw + rng.normal(0, 6 * perturb), pitch + rng.normal(0, 3 * perturb))
            loc.reset(T0)
        t1 = time.perf_counter()
        accepted = loc.update(cloud)
        ms = (time.perf_counter() - t1) * 1000
        info, status = loc.info, loc.status

        def err(Ta):
            dR = Ta[:3, :3] @ T_true[:3, :3].T
            return (np.linalg.norm(Ta[:3, 3] - T_true[:3, 3]),
                    math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(dR) - 1) / 2)))))
        e0, e1 = err(T0), err(loc.T)
        if launch and search:
            # the heading search starts ~90 deg off on purpose: judge heading absolutely, position vs start
            worse = e1[0] > e0[0] + 0.03 or e1[1] > 3.0
        else:
            worse = e1[0] > e0[0] + 0.03 or e1[1] > e0[1] + 1.0
        if accepted and e1[0] < 0.05 and e1[1] < 2.0:
            grade = "PASS"
        elif accepted and e1[0] < 0.10 and e1[1] < 3.0:
            grade = "CLOSE"
        elif not accepted or (status == "weak" and not worse):
            grade = "HELD"
        else:
            grade = "BAD"
        counts[grade] += 1
        fit = f"{info['fitness']:.2f}  {100 * info['rmse']:4.1f}cm" if info.get("ok") else "  -     -   "
        print(f"{n:2d}  {x:5.2f} {y:5.2f} {zc:4.2f}  {yaw:5.0f}° | {100 * e0[0]:5.1f} cm {e0[1]:4.1f}° | "
              f"{100 * e1[0]:5.1f} cm {e1[1]:4.1f}°  {fit} {ms:4.0f}  {info.get('free', '-'):>3}  {status:8s}  {grade}")
    print(f"\nPASS {counts['PASS']}  CLOSE {counts['CLOSE']}  HELD {counts['HELD']}  BAD {counts['BAD']}  (of {trials})")
    print("HELD = the view didn't pin the pose down, so the tracker kept its previous estimate (safe).")
    print("BAD  = accepted but wrong. This should be 0." if counts["BAD"] else "BAD  = 0: no confidently wrong answers.")
    return counts["BAD"] == 0


def simflight(obj_path, route_path, speed=0.3, rate=3.0, yawmode="fixed", clutter=0.3, max_range=5.0,
              points=1500, seed=11, every=10, odom=True, odom_scale=0.03, odom_yaw_dps=0.3, odom_only=False,
              view=0, odom_drop=0.0, odom_resets=(), max_speed=3.0):
    """Fly a route in simulation, frame by frame, the way the live node tracks: the localizer starts on
    the launch mark facing 90 deg (north, along the guideline) and each frame starts from the previous
    estimate. The drone sits on the mark (camera ~0.2 m up), climbs to the route height, then follows
    the route's waypoints_enu at `speed` m/s, one frame every 1/rate s.
    yawmode: 'fixed' keeps heading 90 deg (mission.py today); 'travel' faces the direction of travel.
    odom: feed the tracker odometry deltas with realistic drift (odom_scale distance error, odom_yaw_dps
    heading drift, plus per-frame noise), like the stack's VSLAM.
    view: an HTTP port. Serves the same 3D viewer as the live node (plus the TRUE pose as a white
    marker), runs in real time, and repeats the route until stopped (Ctrl+C).
    odom_drop: chance per frame that VSLAM isn't trusted (lost tracking / gate rejects it): no delta that
    frame, and the odometry chain restarts, exactly like the live node.
    odom_resets: frame numbers where VSLAM 'resets' and its odometry jumps 1.5 m + 40 deg; the tracker
    must reject that step instead of teleporting."""
    model = CourseModel(obj_path)
    route = json.load(open(route_path))
    W = np.asarray(route["waypoints_enu"], float)
    lm = model.group_centre("launch")
    rng = np.random.default_rng(seed)
    step = speed / rate
    # path: 3 frames sitting on the mark, climb to the first waypoint's height, then the route
    pts = [np.array([lm[0], lm[1], 0.2])] * 3
    for zc in np.arange(0.2, W[0, 2], step)[1:]:
        pts.append(np.array([lm[0], lm[1], zc]))
    for a, b in zip(W[:-1], W[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        pts += [a + (b - a) * k / n for k in range(n)]
    pts.append(W[-1])
    P = np.asarray(pts)
    yaws = np.full(len(P), 90.0)
    if yawmode == "travel":
        for i in range(1, len(P)):
            d = P[min(i + 2, len(P) - 1), :2] - P[i - 1, :2]
            want = math.degrees(math.atan2(d[1], d[0])) if np.linalg.norm(d) > 1e-3 else yaws[i - 1]
            turn = (want - yaws[i - 1] + 180) % 360 - 180            # a real drone yaws <= ~90 deg/s
            yaws[i] = yaws[i - 1] + max(-90.0 / rate, min(90.0 / rate, turn))
    start = lambda: camera_pose(lm[0] + rng.normal(0, 0.05), lm[1] + rng.normal(0, 0.05), 0.2,
                                90 + rng.normal(0, 5), 0.0)
    tracker = Tracker(model, start(), max_range=max_range, max_points=points,
                      max_speed=max_speed, max_turn_rate=360.0 if max_speed < 100 else 1e9)
    loc = tracker.loc
    loc.status = "reset"
    if odom_only:                                                     # baseline: never accept a course match
        loc.min_fitness, loc.min_matches = 2.0, 10 ** 9
        print("BASELINE: odometry only, course matching disabled")
    if view:
        WebServer(int(view), model, tracker)
        print(f"viewer: http://<this machine>:{int(view)}/  (white = true pose, coloured = estimate; Ctrl+C stops)")
    print(f"route {route_path}: {len(W)} waypoints, {len(P)} frames at {speed} m/s, {rate} Hz "
          f"({100 * step:.0f} cm/frame), yaw {yawmode}, clutter {clutter}, "
          + (f"odometry with {100 * odom_scale:.0f}% scale + {odom_yaw_dps}°/s heading drift" if odom else "no odometry"))
    lap = 0
    while True:
        lap += 1
        if lap > 1:
            loc.reset(start())
            print(f"\n--- lap {lap} ---")
        print("   t(s)  true x,y,h        est x,y,h         pos err  hdg err  status    measured x/y/h/hdg")
        perr, herr, stat = [], [], {}
        T_prev, gap, odt = None, 1, 1.0 / rate
        for i, (p, yw) in enumerate(zip(P, yaws)):
            t_frame = time.monotonic()
            T_true = camera_pose(p[0], p[1], p[2], yw, rng.normal(0, 2), rng.normal(0, 2))
            delta = None
            # untrusted frame (lost tracking / gate rejected it): no odometry now, but VSLAM's frame is
            # continuous, so the next trusted frame bridges the gap from the last trusted one (live node too)
            dropped = odom and odom_drop > 0 and rng.random() < odom_drop
            if odom and T_prev is not None and not dropped:
                delta = np.linalg.inv(T_prev) @ T_true                # true camera motion since last trusted...
                delta[:3, 3] = delta[:3, 3] * (1 + odom_scale) + rng.normal(0, 0.005, 3)   # ...distance error
                drift = math.radians(odom_yaw_dps * gap / rate) + rng.normal(0, math.radians(0.1))
                delta[:3, :3] = rot_y(drift) @ delta[:3, :3]           # heading drift (optical y = down)
                if i in odom_resets:                                  # VSLAM reset: odometry teleports
                    jump = np.eye(4)
                    jump[:3, :3] = rot_y(math.radians(40))
                    jump[:3, 3] = (1.2, 0.0, 0.9)
                    delta = delta @ jump
                    print(f"   [frame {i}: injected VSLAM reset jump 1.5 m / 40°]")
            if not dropped:
                odt, T_prev, gap = gap / rate, T_true, 1
            else:
                gap += 1
            tracker.truth = {"position": T_true[:3, 3].round(3).tolist(),
                             "forward": (T_true[:3, :3] @ R_OPT_TO_FLU.T)[:, 0].round(3).tolist()}
            tracker.process(simulate_view(model, T_true, rng, max_range=max(5.0, max_range), outliers=clutter),
                            delta, odt if delta is not None else 1.0 / rate)
            e = np.linalg.norm(loc.T[:3, 3] - T_true[:3, 3])
            h = abs((pose_angles(loc.T)[0] - yw + 180) % 360 - 180)
            perr.append(e)
            herr.append(h)
            stat[loc.status] = stat.get(loc.status, 0) + 1
            if i % every == 0 or i == len(P) - 1:
                m = loc.measured
                ms = "/".join(f"{100 * m.get(k, 0):.0f}" for k in ("x", "y", "height", "heading")) if m else "-"
                q = loc.T[:3, 3]
                print(f" {i / rate:6.1f}  {p[0]:5.2f} {p[1]:5.2f} {p[2]:4.2f}   {q[0]:5.2f} {q[1]:5.2f} {q[2]:4.2f}   "
                      f"{100 * e:5.1f} cm  {h:5.1f}°   {loc.status:8s}  {ms}", flush=True)
            if view:                                                  # real time, so it can be watched
                time.sleep(max(0.0, 1.0 / rate - (time.monotonic() - t_frame)))
        perr, herr = np.asarray(perr), np.asarray(herr)
        print(f"\nframes {len(P)}  status {stat}")
        print(f"position error: median {100 * np.median(perr):.0f} cm, 90% {100 * np.percentile(perr, 90):.0f} cm, "
              f"max {100 * perr.max():.0f} cm, final {100 * perr[-1]:.0f} cm")
        print(f"heading error:  median {np.median(herr):.1f}°, 90% {np.percentile(herr, 90):.1f}°, max {herr.max():.1f}°",
              flush=True)
        if not view:
            return perr, herr


# ----------------------------------------------------------------------------------------------
# ROS 2 node
# ----------------------------------------------------------------------------------------------

def cloud_to_xyz(msg):
    """sensor_msgs/PointCloud2 with float32 x, y, z fields -> (N,3) float array."""
    off = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                   "offsets": [off["x"], off["y"], off["z"]], "itemsize": msg.point_step})
    a = np.frombuffer(bytes(msg.data), dtype=dt, count=msg.width * msg.height)
    return np.stack([a["x"], a["y"], a["z"]], axis=1).astype(float)


def main():
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.duration import Duration
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    import tf2_ros
    try:                                                              # cuVSLAM tracking status (Isaac ROS)
        from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
    except ImportError:
        VisualSlamStatus = None

    class CloudLocalize(Node):
        def __init__(self):
            super().__init__("team_cloud_localize")

            def p(name, default):
                # dynamic typing so `-p init_yaw_deg:=90` (an int) works as well as 90.0
                v = self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True)).value
                return float(v) if isinstance(default, float) else v
            obj = p("obj", "")
            if not obj:
                raise SystemExit("set the course model: --ros-args -p obj:=/path/to/course.obj")
            t0 = time.perf_counter()
            model = CourseModel(obj, p("obj_up", "z"), p("obj_scale", 1.0),
                                p("model_spacing", 0.04), p("match_ceiling", True))
            self.get_logger().info(f"{obj}: {len(model.pts):,} samples ({model.nn.kind}) in "
                                   f"{time.perf_counter() - t0:.1f} s")
            # Start pose: defaults to the model's launch mark (if it has one), camera ~0.2 m up as when
            # the drone sits on the floor, facing 90 deg (north): the drone is always placed facing
            # forward along the guideline, which runs north from the launch mark. search_heading:=true
            # tries every heading instead (unreliable from the mark: walls 0.5 m away look alike).
            launch = model.group_centre("launch")
            ix, iy = p("init_x", float("nan")), p("init_y", float("nan"))
            if math.isnan(ix) or math.isnan(iy):
                if launch is None:
                    raise SystemExit("no 'Launch' group in the model: set -p init_x:=... -p init_y:=...")
                ix, iy = float(launch[0]), float(launch[1])
                self.get_logger().info(f"starting at the launch mark ({ix:.3f}, {iy:.3f})")
            search = bool(p("search_heading", False))
            T0 = camera_pose(ix, iy, p("init_z", 0.2), p("init_yaw_deg", 90.0), p("init_pitch_deg", 0.0))
            self.tracker = Tracker(model, T0, max_range=p("max_range", 5.0), max_points=int(p("max_points", 1500)),
                                   min_fitness=p("min_fitness", 0.35), min_matches=int(p("min_matches", 300)),
                                   max_rmse=p("max_rmse", 0.06))
            self.tracker.loc.reset(T0, search_heading=search)
            self.period = 1.0 / p("rate_hz", 3.0)
            self.last = 0.0
            # --- odometry prediction (camera motion between frames) ---------------------------------
            # Preferred: Koji's fail-closed gate, /mapping/validated_odometry (odom -> base_link): only
            # samples that passed its checks (cuVSLAM tracking, <= 250 ms old, right frames, finite,
            # valid covariance). Fallback: raw TF odom -> camera, used only while /visual_slam/status
            # says tracking. Either way, a step faster than the drone can fly is rejected as a reset.
            self.use_odom, self.odom_frame = bool(p("use_odometry", True)), p("odom_frame", "odom")
            self.base_frame = p("base_frame", "base_link")
            self.validated = deque(maxlen=300)                       # (stamp ns, 4x4 odom<-base_link)
            self.vo_tracking, self.vo_time = None, 0.0               # None = no status topic seen
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.prev_odom, self.prev_stamp, self.T_base_cam = None, None, None
            self.tracker.odom_source = "waiting"
            if self.use_odom:
                self.create_subscription(Odometry, p("validated_odom_topic", "/mapping/validated_odometry"),
                                         self.on_validated, qos_profile_sensor_data)
                if VisualSlamStatus is not None:
                    self.create_subscription(VisualSlamStatus, "/visual_slam/status", self.on_vo_status,
                                             qos_profile_sensor_data)
            port = int(p("http_port", 8082))
            self.web = WebServer(port, model, self.tracker)
            # --- outputs: fail-closed consumers should check /team/localize/status before using a pose
            self.pub = self.create_publisher(PoseStamped, "/team/localize/pose", 5)          # camera
            self.pub_base = self.create_publisher(PoseStamped, "/team/localize/base_pose", 5)  # base_link
            self.pub_status = self.create_publisher(String, "/team/localize/status", 5)
            self.create_subscription(PointCloud2, p("cloud_topic", "/team/stereo/points"), self.on_cloud, 2)
            self.get_logger().info("Waiting for /team/stereo/points (is stereo_cloud.py running?) ... "
                                   f"view at http://<jetson>:{port}")

        @staticmethod
        def to_mat(q, v):
            M = np.eye(4)
            M[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w)
            M[:3, 3] = (v.x, v.y, v.z)
            return M

        def on_validated(self, msg):
            st = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            self.validated.append((st, self.to_mat(msg.pose.pose.orientation, msg.pose.pose.position)))

        def on_vo_status(self, msg):
            self.vo_tracking, self.vo_time = msg.vo_state == 1, time.monotonic()

        def base_to_cam(self, cam_frame):
            """Static base_link -> camera optical transform (cached)."""
            if self.T_base_cam is None:
                try:
                    tf = self.tf_buffer.lookup_transform(self.base_frame, cam_frame, Time(), Duration(seconds=0.1))
                    self.T_base_cam = self.to_mat(tf.transform.rotation, tf.transform.translation)
                except tf2_ros.TransformException:
                    return None
            return self.T_base_cam

        def odom_pose(self, msg):
            """(4x4 camera optical pose in the odom frame at the cloud's timestamp, source) or (None, why)."""
            st = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            Tbc = self.base_to_cam(msg.header.frame_id)
            if Tbc is None:
                return None, "no base_link->camera TF"
            if self.validated:
                near = min(self.validated, key=lambda s: abs(s[0] - st))
                if abs(near[0] - st) <= 60_000_000:                    # validated sample within 60 ms
                    return near[1] @ Tbc, "validated (Koji gate)"
                if abs(self.validated[-1][0] - st) < 2_000_000_000:
                    return None, "validated odometry gap"            # gate is alive but rejected these
                # gate silent for > 2 s: assume it isn't running and fall back to TF below
            # fallback: raw TF, but only while cuVSLAM reports tracking (fail closed)
            if self.vo_tracking is False or (self.vo_tracking and time.monotonic() - self.vo_time > 0.25):
                return None, "VSLAM not tracking"
            try:
                tf = self.tf_buffer.lookup_transform(self.odom_frame, msg.header.frame_id,
                                                     Time.from_msg(msg.header.stamp), Duration(seconds=0.05))
                return self.to_mat(tf.transform.rotation, tf.transform.translation), \
                    "tf" + (" (status unchecked)" if self.vo_tracking is None else "")
            except tf2_ros.TransformException:
                return None, "no odom TF at cloud time"

        def pose_msg(self, stamp, M, R_flu):
            out = PoseStamped()
            out.header.stamp, out.header.frame_id = stamp, "course"
            out.pose.position.x, out.pose.position.y, out.pose.position.z = (float(t) for t in M[:3, 3])
            q = quat_xyzw(R_flu)
            out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = q
            return out

        def on_cloud(self, msg):
            now = time.monotonic()
            if now - self.last < self.period:
                return
            self.last = now
            delta, dt = None, None
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self.use_odom:
                O, src = self.odom_pose(msg)
                self.tracker.odom_source = src
                if O is not None:
                    # bridge short gaps: VSLAM's odom frame is continuous, so the motion since the last
                    # trusted sample is still right (a reset in between shows up as an impossible jump
                    # and is rejected by the tracker). Gaps over 3 s restart the chain.
                    if self.prev_odom is not None and st - self.prev_stamp < 3.0:
                        delta, dt = np.linalg.inv(self.prev_odom) @ O, st - self.prev_stamp
                    self.prev_odom, self.prev_stamp = O, st
            else:
                self.tracker.odom_source = "off"
            T, status = self.tracker.process(cloud_to_xyz(msg), delta, dt)
            s = self.tracker.state()
            s.pop("trail", None)
            s.pop("truth", None)
            self.pub_status.publish(String(data=json.dumps(s)))
            if status not in ("tracking", "weak", "odometry"):
                return                                               # fail closed: no pose
            self.pub.publish(self.pose_msg(msg.header.stamp, T, T[:3, :3] @ R_OPT_TO_FLU.T))
            if self.T_base_cam is not None:
                Tb = T @ np.linalg.inv(self.T_base_cam)              # course <- base_link (FLU)
                self.pub_base.publish(self.pose_msg(msg.header.stamp, Tb, Tb[:3, :3]))

    rclpy.init()
    node = CloudLocalize()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    if len(sys.argv) >= 3 and sys.argv[1] == "--simflight":
        # --simflight model.obj route=route.json [speed=0.3] [rate=3] [yaw=fixed|travel] [clutter=0.3] [icp.*=...]
        opts = dict(a.split("=", 1) for a in sys.argv[3:] if "=" in a)
        for k, v in opts.items():
            if k.startswith("icp."):
                key = k[4:]
                ICP_OPTS[key] = (tuple(float(t) for t in v.split(",")) if key == "schedule"
                                 else v if key == "kernel" else float(v))
        simflight(sys.argv[2], opts["route"], speed=float(opts.get("speed", 0.3)), rate=float(opts.get("rate", 3)),
                  yawmode=opts.get("yaw", "fixed"), clutter=float(opts.get("clutter", 0.3)),
                  every=int(opts.get("every", 10)), odom=opts.get("odom", "1") not in ("0", "false"),
                  odom_scale=float(opts.get("odom_scale", 0.03)), odom_yaw_dps=float(opts.get("odom_yaw", 0.3)),
                  odom_only=opts.get("odom_only", "0") not in ("0", "false"), view=int(opts.get("view", 0)),
                  odom_drop=float(opts.get("odom_drop", 0.0)),
                  odom_resets=tuple(int(t) for t in opts["odom_resets"].split(",")) if "odom_resets" in opts else (),
                  max_speed=float(opts.get("max_speed", 3.0)), seed=int(opts.get("seed", 11)))
        sys.exit(0)
    if len(sys.argv) >= 3 and sys.argv[1] == "--selftest":
        # --selftest model.obj [up=z|y] [trials=20] [range=5] [perturb=0.4] [points=1500] [clutter=0.05]
        # perturb scales the start error: 0.4 ~ frame-to-frame tracking, 1.0 ~ a rough manual reset.
        # clutter = unmodelled points (people, chairs) as a fraction of the visible model points.
        opts = dict(a.split("=", 1) for a in sys.argv[3:] if "=" in a)
        for k, v in opts.items():                                     # icp.kernel=tukey icp.c=0.1 ...
            if k.startswith("icp."):
                key = k[4:]
                ICP_OPTS[key] = (tuple(float(t) for t in v.split(",")) if key == "schedule"
                                 else v if key == "kernel" else float(v))
        print("ICP:", ICP_OPTS)
        sys.exit(0 if selftest(sys.argv[2], opts.get("up", "z"), trials=int(opts.get("trials", 20)),
                               max_range=float(opts.get("range", 5.0)), perturb=float(opts.get("perturb", 0.4)),
                               points=int(opts.get("points", 1500)), clutter=float(opts.get("clutter", 0.05)),
                               launch=opts.get("launch", "0") not in ("0", "false", ""),
                               search=opts.get("search", "1") not in ("0", "false", ""),
                               launch_yaw=float(opts["yaw"]) if "yaw" in opts else None) else 1)
    main()
