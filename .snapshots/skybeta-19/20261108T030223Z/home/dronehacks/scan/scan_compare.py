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

# --- 2. coarse yaw + xy from wall points (2D occupancy correlation) ------------------------------
model = cl.CourseModel(obj, match_ceiling=False)
mp = model.pts
solid = ~model.see_through
mw = mp[solid & (mp[:, 2] > 0.3) & (mp[:, 2] < 1.6)]
sw = P[(P[:, 2] > 0.3) & (P[:, 2] < 1.6)]
G = 0.05


def grid(pts, lo, shape):
    ij = np.floor((pts[:, :2] - lo) / G).astype(int)
    ok = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < shape[0]) & (ij[:, 1] < shape[1])
    g = np.zeros(shape)
    np.add.at(g, (ij[ok, 0], ij[ok, 1]), 1.0)
    return np.minimum(g, 3.0)


size = 512                                      # 25.6 m square
lo = np.array([-size * G / 2] * 2)
Mg = grid(mw, lo, (size, size))
FM = np.conj(np.fft.rfft2(Mg))
bestc = (-1, 0, 0, 0)
sc = sw[rng.choice(len(sw), min(len(sw), 150000), replace=False)]
for yaw in np.arange(0, 360, 1.0):
    r = math.radians(yaw)
    Rz = np.array([[math.cos(r), -math.sin(r)], [math.sin(r), math.cos(r)]])
    q = sc[:, :2] @ Rz.T
    Sg = grid(np.c_[q, sc[:, 2]], lo, (size, size))
    corr = np.fft.irfft2(np.fft.rfft2(Sg) * FM, s=(size, size))
    k = np.unravel_index(np.argmax(corr), corr.shape)
    if corr[k] > bestc[0]:
        bestc = (corr[k], yaw, k[0], k[1])
_, yaw, di, dj = bestc
di = di - size if di > size // 2 else di
dj = dj - size if dj > size // 2 else dj
r = math.radians(yaw)
T0 = np.eye(4)
T0[:3, :3] = cl.rot_z(r) if callable(cl.rot_z) else np.eye(3)
T0[:3, 3] = [-di * G, -dj * G, 0.0]
print(f"coarse: yaw {yaw:.0f} deg, shift ({-di * G:+.2f}, {-dj * G:+.2f}) m")

# --- 3. refine with the localizer's own point-to-plane ICP -------------------------------------
Pw = cl.transform(T0, P)
inside = (np.abs(Pw[:, 0]) < 3.3) & (np.abs(Pw[:, 1]) < 3.3) & (Pw[:, 2] > -0.2) & (Pw[:, 2] < 3.2)
src = cl.voxel_down(P[inside], 0.03)
centre = src.mean(0)
Tc = np.eye(4)
Tc[:3, 3] = -centre
T = T0 @ np.linalg.inv(Tc)                       # src0 = P - centre: ICP rotates about the course centre
T, info = cl.icp(src - centre, T, model, schedule=(0.3, 0.15, 0.08, 0.04, 0.03), iters=15, min_pts=500)
T = T @ Tc
yaw2, pit, rol = cl.pose_angles(T)
print(f"ICP: ok {info['ok']}, fitness {info.get('fitness', 0):.3f}, rmse {info.get('rmse', 0) * 100:.1f} cm, "
      f"free dirs {info.get('free')}, yaw {yaw2:.2f} pitch {pit:.2f} roll {rol:.2f}")

# --- 4. agreement inside the course -------------------------------------------------------------
W = cl.transform(T, P)
box = (np.abs(W[:, 0]) < 3.0) & (np.abs(W[:, 1]) < 3.0) & (W[:, 2] > 0.03) & (W[:, 2] < 2.9)
Q = W[box]
solid_nn = cl.NearestNeighbour(mp[solid])
_, d = solid_nn.query(Q)
print(f"scan points inside the course (above the floor): {len(Q):,}")
for t in (0.02, 0.03, 0.05, 0.10):
    print(f"  within {t * 100:.0f} cm of a solid model surface: {100 * (d < t).mean():.1f}%")
print(f"  median distance {np.median(d) * 100:.1f} cm (points within 10 cm: median "
      f"{np.median(d[d < 0.1]) * 100:.1f} cm)")

# per-group signed offset: scan points near each solid group, along the model normal
idx, dd = model.nn.query(Q, 0.15)
okm = dd < 0.15
gi = np.array([model.G[f] for f in range(len(model.G))])
tris, parent = cl.triangulate(model.F)
# map each model sample to its group via nearest-sample index -> face; rebuild with the same rng
pts2, nrm2, tri2 = cl.sample_surface(model.V, tris, 0.04, np.random.default_rng(0))
grp = np.array([model.G[parent[t]] for t in tri2])
print("\nper surface (scan points within 15 cm): offset along the surface normal, median / spread")
rows = []
for g in sorted(set(grp)):
    if "net" in g.lower() or "floor" in g.lower() or "launch" in g.lower() or "ring" in g.lower() \
            or "guideline" in g.lower() or "pad" in g.lower() or "cross" in g.lower():
        continue
    sel = okm & (grp[np.minimum(idx, len(grp) - 1)] == g)
    if sel.sum() < 50:
        rows.append(f"  {g:<32} too few scan points ({int(sel.sum())})")
        continue
    k = idx[sel]
    off = np.einsum("ij,ij->i", Q[sel] - pts2[k], nrm2[k])
    cover = len(np.unique(k)) / max(1, (grp == g).sum())
    rows.append(f"  {g:<32} {np.median(off) * 100:+5.1f} cm  (IQR {np.subtract(*np.percentile(off, [75, 25])) * 100:4.1f} cm)"
                f"  coverage {100 * cover:3.0f}%  n={int(sel.sum())}")
print("\n".join(rows))

# unmodelled stuff inside the course: >10 cm from any solid surface, by 25 cm cell
far = Q[d > 0.10]
cells = np.floor(far[:, :2] / 0.25).astype(int)
u, cnt = np.unique(cells, axis=0, return_counts=True)
top = np.argsort(-cnt)[:12]
print(f"\nunmodelled points (>10 cm from the model): {len(far):,} ({100 * len(far) / len(Q):.1f}%). Biggest 25 cm cells:")
for k in top:
    cz = far[(cells[:, 0] == u[k][0]) & (cells[:, 1] == u[k][1])][:, 2]
    print(f"  ({(u[k][0] + .5) * .25:+.2f}, {(u[k][1] + .5) * .25:+.2f})  {cnt[k]:6d} pts  z {cz.min():.2f}-{cz.max():.2f}")
np.save("/tmp/scan_T.npy", T)
