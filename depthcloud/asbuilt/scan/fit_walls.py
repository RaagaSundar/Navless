"""Fit straight wall segments to the scan's walls that the plan got wrong. python3 fit_walls.py scan.ply course.obj"""
import json
import math
import sys
import numpy as np
from scipy.spatial import cKDTree
import scan_common as sc

ply, obj = sys.argv[1], sys.argv[2]
P = sc.load_level(ply)
model, pts, nrm, grp = sc.model_samples(obj)
th, t, trusted = sc.align(P, pts, nrm, grp)
W = sc.apply(th, t, P)
print(f"alignment on trusted parts: yaw {math.degrees(th) % 360:.2f} deg, shift ({t[0]:+.3f}, {t[1]:+.3f}) m")
tt = cKDTree(pts[trusted])
Q = W[(W[:, 2] > 0.25) & (W[:, 2] < 1.6) & (np.abs(W[:, 0]) < 3.2) & (np.abs(W[:, 1]) < 3.2)]
d, i = tt.query(Q, distance_upper_bound=0.12)
m = np.isfinite(d)
gq = grp[trusted][i[m]]
off = np.einsum("ij,ij->i", Q[m] - pts[trusted][i[m]], nrm[trusted][i[m]])
for g in sorted(set(gq)):
    o = off[gq == g]
    print(f"  {g:<28} offset {np.median(o) * 100:+5.1f} cm  IQR {np.subtract(*np.percentile(o, [75, 25])) * 100:4.1f} cm"
          f"  n={len(o)}")

# wall-height points not explained by the trusted parts, boulders or cradles
keep_out = np.array([any(k in g for k in sc.TRUST + ("Boulder", "Tube cradle")) for g in grp])
ko = cKDTree(pts[keep_out][:, :2])
C = W[(W[:, 2] > 0.35) & (W[:, 2] < 1.55) & (np.abs(W[:, 0]) < 2.95) & (np.abs(W[:, 1]) < 2.95)]
dk, _ = ko.query(C[:, :2])
C = C[dk > 0.10]
cells, cnt = np.unique(np.floor(C[:, :2] / 0.02).astype(int), axis=0, return_counts=True)
X = ((cells + 0.5) * 0.02)[cnt >= 2]
print(f"\ncandidate wall cells (2 cm): {len(X):,}")

rng = np.random.default_rng(1)
segs, rest = [], X.copy()
for it in range(16):
    if len(rest) < 60:
        break
    bestl = None
    for _ in range(3000):
        a, b = rest[rng.choice(len(rest), 2, replace=False)]
        L = np.linalg.norm(b - a)
        if L < 0.3:
            continue
        u = (b - a) / L
        nv = np.array([-u[1], u[0]])
        k = int((np.abs((rest - a) @ nv) < 0.06).sum())
        if bestl is None or k > bestl[0]:
            bestl = (k, a, nv)
    _, a, nv = bestl
    inl = np.abs((rest - a) @ nv) < 0.07
    P2 = rest[inl]
    c = P2.mean(0)
    _, _, vt = np.linalg.svd(P2 - c, full_matrices=False)
    u = vt[0]
    s = (P2 - c) @ u
    order = np.argsort(s)
    ss = s[order]
    gaps = np.flatnonzero(np.diff(ss) > 0.3)
    used = np.zeros(len(P2), bool)
    for st, en in zip(np.r_[0, gaps + 1], np.r_[gaps, len(ss) - 1]):
        idx = order[st:en + 1]
        if ss[en] - ss[st] < 0.35 or len(idx) < 40:
            continue
        sub = P2[idx]
        cc = sub.mean(0)
        _, _, vt2 = np.linalg.svd(sub - cc, full_matrices=False)
        u2 = vt2[0]
        n2 = np.array([-u2[1], u2[0]])
        a_ = (sub - cc) @ u2
        o_ = (sub - cc) @ n2
        mid = np.median(o_)
        p0 = cc + u2 * np.percentile(a_, 1) + n2 * mid
        p1 = cc + u2 * np.percentile(a_, 99) + n2 * mid
        if p1[1] < p0[1]:
            p0, p1 = p1, p0
        segs.append({"p0": p0.round(3).tolist(), "p1": p1.round(3).tolist(),
                     "length_m": round(float(np.linalg.norm(p1 - p0)), 3),
                     "bearing_deg": round(float(math.degrees(math.atan2(p1[0] - p0[0], p1[1] - p0[1])) % 180), 1),
                     "spread_cm": round(float(np.subtract(*np.percentile(o_, [95, 5])) * 100), 1),
                     "cells": int(len(idx))})
        used[idx] = True
    rest = np.concatenate([rest[~inl], P2[~used]])

segs.sort(key=lambda s_: -s_["cells"])
print("fitted segments (course metres, x east / y north; bearing clockwise from north):")
for k, s_ in enumerate(segs):
    print(f"  {k:2d}  ({s_['p0'][0]:+.2f}, {s_['p0'][1]:+.2f}) -> ({s_['p1'][0]:+.2f}, {s_['p1'][1]:+.2f})"
          f"  len {s_['length_m']:.2f}  bearing {s_['bearing_deg']:5.1f}  spread {s_['spread_cm']:4.1f} cm"
          f"  cells {s_['cells']}")
json.dump({"theta": th, "t": list(map(float, t)), "segments": segs}, open("fit.json", "w"), indent=1)

cv = sc.Canvas()
cv.put(W[(W[:, 2] > 0.35) & (W[:, 2] < 1.55)], (175, 175, 175))
cv.put(pts[trusted], (220, 30, 30))
for k, s_ in enumerate(segs):
    cv.line(s_["p0"], s_["p1"], (20, 60, 220), 2)
    # index marker: k+1 small dots next to the segment's middle
    mid = (np.array(s_["p0"]) + np.array(s_["p1"])) / 2
    for j in range(k + 1):
        cv.put(np.array([[mid[0] + 0.06 + 0.035 * j, mid[1] + 0.06]]), (0, 150, 0), 2)
sc.png("fit.png", cv.img)
print("wrote fit.json, fit.png")
