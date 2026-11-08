"""Align a phone LiDAR scan (.ply) of the course to the course .obj and measure how well they agree.

python3 scan_compare.py scan.ply course.obj  (run from depthcloud/, needs cloud_localize.py importable)
"""
import math
import sys
import numpy as np
import cloud_localize as cl

ply, obj = sys.argv[1], sys.argv[2]
raw = open(ply, "rb").read()
h = raw.index(b"end_header\n") + len(b"end_header\n")
n = int([l for l in raw[:h].decode().splitlines() if l.startswith("element vertex")][0].split()[2])
dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("r", "u1"), ("g", "u1"), ("b", "u1")])
a = np.frombuffer(raw, dt, count=n, offset=h)
P = np.stack([a["x"], a["y"], a["z"]], 1).astype(float)
print(f"scan: {n:,} points, extent {np.ptp(P, 0).round(2)} m")

# --- 1. level the floor: biggest horizontal plane in the lowest 40% of points (RANSAC) -------------
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
R_level = np.eye(3) + K + K @ K * ((1 - c_) / (s_ ** 2)) if s_ > 1e-9 else np.eye(3)
P = (P - c) @ R_level.T
print(f"floor: {len(inl):,} points, tilt {math.degrees(math.acos(min(1, c_))):.2f} deg, "
      f"flatness (std) {np.std(((inl - c) @ R_level.T)[:, 2]) * 100:.1f} cm")

# --- 2. planar alignment: yaw + x/y on the levelled floor ----------------------------------------
import zlib, struct

def png(path, img):                                   # img: HxWx3 uint8
    h, w, _ = img.shape
    raw = b"".join(b"\x00" + img[y].tobytes() for y in range(h))
    def chunk(t, d):
        c = struct.pack(">I", len(d)) + t + d
        return c + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff)
    open(path, "wb").write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                           + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))

model = cl.CourseModel(obj, match_ceiling=False)
mp, mn = model.pts, model.normals
solid = ~model.see_through
wall = solid & (mp[:, 2] > 0.25) & (mp[:, 2] < 1.6) & (np.abs(mn[:, 2]) < 0.3)
Mw, Mn = mp[wall], mn[wall]
sw = P[(P[:, 2] > 0.25) & (P[:, 2] < 1.6)]
G = 0.05
size = 512
lo = np.array([-size * G / 2] * 2)

def occ(pts):
    ij = np.floor((pts[:, :2] - lo) / G).astype(int)
    ok = (ij >= 0).all(1) & (ij < size).all(1)
    g = np.zeros((size, size))
    np.add.at(g, (ij[ok, 0], ij[ok, 1]), 1.0)
    return g

Mg = (occ(Mw) > 0).astype(float)
# blur the model a little so near-misses still score
from scipy.ndimage import uniform_filter, binary_dilation
Mb = uniform_filter(Mg, 3)
FM = np.conj(np.fft.rfft2(Mb))
sc = sw[rng.choice(len(sw), min(len(sw), 200000), replace=False)]
cands = []
for yaw in np.arange(0, 360, 1.0):
    r = math.radians(yaw)
    q = sc[:, :2] @ np.array([[math.cos(r), -math.sin(r)], [math.sin(r), math.cos(r)]]).T
    Sg = (occ(np.c_[q, sc[:, 2]]) > 0).astype(float)
    corr = np.fft.irfft2(np.fft.rfft2(Sg) * FM, s=(size, size))
    k = np.unravel_index(np.argmax(corr), corr.shape)
    di, dj = [kk - size if kk > size // 2 else kk for kk in k]
    cands.append((corr[k], yaw, di, dj))
cands.sort(reverse=True)
print("coarse candidates (score, yaw, shift):")
for sc_, yaw, di, dj in cands[:6]:
    print(f"  {sc_:8.1f}  yaw {yaw:5.1f}  shift ({di * G:+.2f}, {dj * G:+.2f})")


def apply(theta, t, pts):
    c_, s_ = math.cos(theta), math.sin(theta)
    out = pts.copy()
    out[:, 0] = c_ * pts[:, 0] - s_ * pts[:, 1] + t[0]
    out[:, 1] = s_ * pts[:, 0] + c_ * pts[:, 1] + t[1]
    return out


from scipy.spatial import cKDTree
tree = cKDTree(Mw[:, :2])


def planar_icp(theta, t, pts):
    for md in (0.4, 0.25, 0.15, 0.10, 0.06, 0.04, 0.03):
        for _ in range(15):
            q = apply(theta, t, pts)
            box = (np.abs(q[:, 0]) < 3.2) & (np.abs(q[:, 1]) < 3.2)
            d, i = tree.query(q[box, :2], distance_upper_bound=md)
            m = np.isfinite(d)
            if m.sum() < 200:
                return theta, t, 0, 1.0
            Q, M, N = q[box][m], Mw[i[m]], Mn[i[m]][:, :2]
            nl = np.linalg.norm(N, axis=1, keepdims=True)
            N = N / np.maximum(nl, 1e-9)
            r = np.einsum("ij,ij->i", Q[:, :2] - M[:, :2], N)
            cc = max(0.03, 0.5 * md)
            w = np.where(np.abs(r) < cc, (1 - (r / cc) ** 2) ** 2, 0.0)
            # d r / d[theta, tx, ty] ; rotation about the course origin
            dq = np.c_[-Q[:, 1] + t[1], Q[:, 0] - t[0]]          # derivative of rotated point (about origin)
            J = np.c_[np.einsum("ij,ij->i", dq, N), N]
            H = J.T @ (J * w[:, None]) + 1e-9 * np.eye(3)
            g = J.T @ (w * r)
            dx = -np.linalg.solve(H, g)
            theta += dx[0]
            t = t + dx[1:] + np.array([-dx[0] * t[1], dx[0] * t[0]]) * 0   # rotation applied in apply()
            if np.abs(dx).max() < 1e-6:
                break
    q = apply(theta, t, pts)
    box = (np.abs(q[:, 0]) < 3.2) & (np.abs(q[:, 1]) < 3.2)
    d, _ = tree.query(q[box, :2])
    return theta, t, float((d < 0.05).mean()), float(np.median(d))


best = None
for sc_, yaw, di, dj in cands[:6]:
    th, t, fit, med = planar_icp(math.radians(yaw), np.array([di * G, dj * G]), sw)
    print(f"  from yaw {yaw:5.1f}: -> yaw {math.degrees(th) % 360:7.2f} shift ({t[0]:+.3f}, {t[1]:+.3f})  "
          f"wall pts within 5 cm {100 * fit:.1f}%  median {med * 100:.1f} cm")
    if best is None or fit > best[2]:
        best = (th, t, fit)
th, t, _ = best
W = apply(th, t, P)

# top-view images: scan wall-height points (grey), model walls (red)
S2, half = 120, 3.6
img = np.full((int(2 * half * S2), int(2 * half * S2), 3), 255, np.uint8)
def put(pts, col):
    x = ((pts[:, 0] + half) * S2).astype(int)
    y = ((half - pts[:, 1]) * S2).astype(int)
    ok = (x >= 0) & (y >= 0) & (x < img.shape[1]) & (y < img.shape[0])
    img[y[ok], x[ok]] = col
ws = W[(W[:, 2] > 0.25) & (W[:, 2] < 1.6)]
put(ws, (90, 90, 90))
put(Mw, (220, 30, 30))
png("/tmp/scan/overlay.png", img)
img[:] = 255
low = W[(W[:, 2] > 0.02) & (W[:, 2] < 0.6)]
put(low, (60, 60, 160))
put(mp[solid & (mp[:, 2] > 0.02) & (mp[:, 2] < 0.6) & (np.abs(mn[:, 2]) < 0.5)], (220, 30, 30))
png("/tmp/scan/overlay_low.png", img)

# --- 3. per-surface agreement -----------------------------------------------------------------
tris, parent = cl.triangulate(model.F)
pts2, nrm2, tri2 = cl.sample_surface(model.V, tris, 0.04, np.random.default_rng(0))
grp = np.array([model.G[parent[k]] for k in tri2])
ok_s = ~np.array([any(w in g.lower() for w in ("net", "floor", "launch", "ring", "guideline", "pad", "cross"))
                  for g in grp])
t3 = cKDTree(pts2[ok_s])
idx_map = np.flatnonzero(ok_s)
inb = (np.abs(W[:, 0]) < 3.05) & (np.abs(W[:, 1]) < 3.05) & (W[:, 2] > 0.03) & (W[:, 2] < 2.95)
Q = W[inb]
d, i = t3.query(Q, distance_upper_bound=0.15)
m = np.isfinite(d)
k = idx_map[i[m]]
off = np.einsum("ij,ij->i", Q[m] - pts2[k], nrm2[k])
print(f"\nscan points inside the course: {len(Q):,}; within 15 cm of a solid surface: {100 * m.mean():.1f}%")
dd, _ = t3.query(Q)
for tt in (0.02, 0.03, 0.05, 0.10):
    print(f"  within {tt * 100:.0f} cm: {100 * (dd < tt).mean():.1f}%")
print("per surface: median offset along its normal (+ = scan is on the outside), spread, coverage")
for g in sorted(set(grp[ok_s])):
    sel = grp[k] == g
    tot = (grp == g).sum()
    if sel.sum() < 40:
        print(f"  {g:<30} too few points ({int(sel.sum())})")
        continue
    cov = len(np.unique(k[sel])) / tot
    o = off[sel]
    print(f"  {g:<30} {np.median(o) * 100:+5.1f} cm  IQR {np.subtract(*np.percentile(o, [75, 25])) * 100:4.1f} cm"
          f"  coverage {100 * cov:3.0f}%  n={int(sel.sum())}")
far = Q[dd > 0.10]
far = far[far[:, 2] > 0.05]
cells = np.floor(far[:, :2] / 0.25).astype(int)
u, cnt = np.unique(cells, axis=0, return_counts=True)
print(f"unmodelled inside the course (>10 cm from any solid surface): {len(far):,} pts. Biggest 25 cm cells:")
for j in np.argsort(-cnt)[:10]:
    cz = far[(cells[:, 0] == u[j][0]) & (cells[:, 1] == u[j][1])][:, 2]
    print(f"  ({(u[j][0] + .5) * .25:+.2f}, {(u[j][1] + .5) * .25:+.2f})  {cnt[j]:6d}  z {cz.min():.2f}-{cz.max():.2f}")
