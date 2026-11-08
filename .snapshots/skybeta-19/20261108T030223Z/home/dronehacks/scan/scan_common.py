"""Shared: load the phone scan, level it on the floor, align it to the course on the parts that match."""
import math
import json
import struct
import zlib
import numpy as np
from scipy.spatial import cKDTree
import cloud_localize as cl

TRUST = ("Barrier", "Tube oval", "C2a", "Net side")      # parts where scan and plan agree


def load_level(ply):
    raw = open(ply, "rb").read()
    h = raw.index(b"end_header\n") + len(b"end_header\n")
    n = int([l for l in raw[:h].decode().splitlines() if l.startswith("element vertex")][0].split()[2])
    dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("r", "u1"), ("g", "u1"), ("b", "u1")])
    a = np.frombuffer(raw, dt, count=n, offset=h)
    P = np.stack([a["x"], a["y"], a["z"]], 1).astype(float)
    rng = np.random.default_rng(0)
    low = P[P[:, 2] < np.percentile(P[:, 2], 40)]
    best = (0, None)
    for _ in range(400):
        s = low[rng.choice(len(low), 3, replace=False)]
        nrm = np.cross(s[1] - s[0], s[2] - s[0])
        if np.linalg.norm(nrm) < 1e-9:
            continue
        nrm /= np.linalg.norm(nrm)
        if abs(nrm[2]) < 0.9:
            continue
        cnt = int((np.abs((low - s[0]) @ nrm) < 0.02).sum())
        if cnt > best[0]:
            best = (cnt, (nrm if nrm[2] > 0 else -nrm, s[0]))
    nrm, p0 = best[1]
    inl = low[np.abs((low - p0) @ nrm) < 0.02]
    c = inl.mean(0)
    _, _, vt = np.linalg.svd(inl - c, full_matrices=False)
    nrm = vt[2] if vt[2][2] > 0 else -vt[2]
    v = np.cross(nrm, [0, 0, 1.0])
    s_, c_ = np.linalg.norm(v), nrm[2]
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R = np.eye(3) + K + K @ K * ((1 - c_) / (s_ ** 2)) if s_ > 1e-9 else np.eye(3)
    return (P - c) @ R.T


def model_samples(obj, spacing=0.03):
    model = cl.CourseModel(obj, match_ceiling=False)
    tris, parent = cl.triangulate(model.F)
    pts, nrm, tri = cl.sample_surface(model.V, tris, spacing, np.random.default_rng(0))
    grp = np.array([model.G[parent[k]] for k in tri])
    return model, pts, nrm, grp


def apply(theta, t, pts):
    c_, s_ = math.cos(theta), math.sin(theta)
    out = pts.copy()
    out[:, 0] = c_ * pts[:, 0] - s_ * pts[:, 1] + t[0]
    out[:, 1] = s_ * pts[:, 0] + c_ * pts[:, 1] + t[1]
    return out


def align(P, pts, nrm, grp):
    """Planar ICP (yaw, x, y) of the scan's wall-height points onto the TRUSTED model parts."""
    trusted = np.array([any(t in g for t in TRUST) for g in grp]) & (pts[:, 2] > 0.25) & (pts[:, 2] < 1.6) \
        & (np.abs(nrm[:, 2]) < 0.3)
    Mw, Mn = pts[trusted], nrm[trusted]
    tree = cKDTree(Mw[:, :2])
    sw = P[(P[:, 2] > 0.25) & (P[:, 2] < 1.6)]

    def icp(theta, t):
        for md in (0.4, 0.25, 0.15, 0.10, 0.06, 0.04, 0.03):
            for _ in range(20):
                q = apply(theta, t, sw)
                box = (np.abs(q[:, 0]) < 3.25) & (np.abs(q[:, 1]) < 3.25)
                d, i = tree.query(q[box, :2], distance_upper_bound=md)
                m = np.isfinite(d)
                if m.sum() < 100:
                    return theta, t, np.full(1, 9.0)
                Q, M, N = q[box][m], Mw[i[m]], Mn[i[m]][:, :2]
                N = N / np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-9)
                r = np.einsum("ij,ij->i", Q[:, :2] - M[:, :2], N)
                cc = max(0.03, 0.5 * md)
                w = np.where(np.abs(r) < cc, (1 - (r / cc) ** 2) ** 2, 0.0)
                dq = np.c_[-(Q[:, 1] - t[1]), Q[:, 0] - t[0]]
                J = np.c_[np.einsum("ij,ij->i", dq, N), N]
                dx = -np.linalg.solve(J.T @ (J * w[:, None]) + 1e-9 * np.eye(3), J.T @ (w * r))
                theta += dx[0]
                t = t + dx[1:]
                if np.abs(dx).max() < 1e-6:
                    break
        q = apply(theta, t, sw)
        box = (np.abs(q[:, 0]) < 3.25) & (np.abs(q[:, 1]) < 3.25)
        d, _ = tree.query(q[box, :2])
        return theta, t, d

    best = None
    for yaw0 in (180.0, 182.0, 184.0, 186.0):
        for t0 in ((0.0, 0.0), (0.1, 0.1), (-0.1, -0.25), (0.1, -0.1)):
            th, t, d = icp(math.radians(yaw0), np.array(t0))
            score = int((d < 0.04).sum())
            if best is None or score > best[0]:
                best = (score, th, t)
    return best[1], best[2], trusted


def png(path, img):
    h, w, _ = img.shape
    raw = b"".join(b"\x00" + img[y].tobytes() for y in range(h))

    def chunk(t, d):
        c = struct.pack(">I", len(d)) + t + d
        return c + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff)
    open(path, "wb").write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                           + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


class Canvas:
    def __init__(self, half=3.4, scale=120):
        self.half, self.s = half, scale
        n = int(2 * half * scale)
        self.img = np.full((n, n, 3), 255, np.uint8)

    def put(self, pts, col, r=0):
        x = ((pts[:, 0] + self.half) * self.s).astype(int)
        y = ((self.half - pts[:, 1]) * self.s).astype(int)
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                ok = (x + dx >= 0) & (y + dy >= 0) & (x + dx < self.img.shape[1]) & (y + dy < self.img.shape[0])
                self.img[y[ok] + dy, x[ok] + dx] = col

    def line(self, p0, p1, col, r=1):
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        self.put(p0 + np.linspace(0, 1, 600)[:, None] * (p1 - p0), col, r)
