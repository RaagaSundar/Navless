"""Best-guess as-built course .obj from the phone scan's wall fits (fit.json).

python3 build_asbuilt.py scan.ply course.obj out.obj
Replaces M1, M2, M3, A1 and C1 with walls fitted to the scan. Everything else (barriers, window wall
C2a, C2b, tube, boulders, pad, net, launch mark) stays as in the plan: those either matched the scan
within ~2-6 cm or (C2b) weren't seen well enough to move.
"""
import json
import math
import sys
import numpy as np
from scipy.spatial import cKDTree
import scan_common as sc

ply, obj, out = sys.argv[1], sys.argv[2], sys.argv[3]
fit = json.load(open("fit.json"))
S = fit["segments"]
IN = 0.0254
BAR_N, BAR_S = 0.4572 + 2 * IN, 0.4572 - 2 * IN        # barrier line y = 138 in -> faces at 140 / 136 in
NET = 3.048
H, THK = 68 * IN, 4 * IN

# which fitted segments make up each wall (indices into fit.json), and where its ends attach
WALLS = {
    "M1 maze divider": dict(segs=[0, 16], y_from=BAR_N, y_to=None),
    "M2 maze divider": dict(segs=[14, 15, 19], y_from=None, y_to=NET),
    "M3 maze divider": dict(segs=[8, 20, 5, 1], y_from=BAR_N, y_to=None, clip=(BAR_N, NET)),
    "A1 approach wall": dict(segs=[4, 9], y_from=None, y_to=BAR_S),
    "C1 chicane wall": dict(segs=[2, 21, 18, 1, 5], y_from=None, y_to=BAR_S, clip=(-NET, BAR_S)),
}


def wall_line(spec):
    """Fit one straight centre line (x as a function of y) to the chosen segments, weighted by support."""
    lo, hi = spec.get("clip", (-NET, NET))
    pts, w = [], []
    for k in spec["segs"]:
        p0, p1 = np.array(S[k]["p0"]), np.array(S[k]["p1"])
        for f in np.linspace(0, 1, 50):
            p = p0 + f * (p1 - p0)
            if lo <= p[1] <= hi:
                pts.append(p)
                w.append(S[k]["cells"])
    pts, w = np.array(pts), np.array(w, float)
    A = np.c_[pts[:, 1], np.ones(len(pts))]
    sw = np.sqrt(w)
    (m, b), *_ = np.linalg.lstsq(A * sw[:, None], pts[:, 0] * sw, rcond=None)
    y0 = spec["y_from"] if spec["y_from"] is not None else pts[:, 1].min()
    y1 = spec["y_to"] if spec["y_to"] is not None else pts[:, 1].max()
    y0, y1 = min(y0, y1), max(y0, y1)
    return np.array([m * y0 + b, y0]), np.array([m * y1 + b, y1])


new = {name: wall_line(spec) for name, spec in WALLS.items()}

# --- write the .obj: original minus the replaced groups' faces, plus new wall boxes ---------------
nv = 0
with open(obj) as fin, open(out, "w") as fout:
    group = ""
    for line in fin:
        t = line.split()
        if t and t[0] == "v":
            nv += 1
        if t and t[0] == "g":
            group = " ".join(t[1:])
        if t and t[0] == "f" and group in new:
            continue                                   # drop the plan's version of this wall
        fout.write(line)
    fout.write("\n# --- as-built walls (best guess from the 2026-11-06 phone scan, depthcloud scan fit;"
               " NOT tape-measured) ---\n")
    idx = nv + 1
    for name, (p0, p1) in new.items():
        u = (p1 - p0) / np.linalg.norm(p1 - p0)
        n = np.array([-u[1], u[0]]) * THK / 2
        corners = [p0 - n, p1 - n, p1 + n, p0 + n]
        fout.write(f"g {name}\n")
        for z in (0.0, H):
            for c in corners:
                fout.write(f"v {c[0]:.5f} {c[1]:.5f} {z:.5f}\n")
        b, t_ = idx, idx + 4
        quads = [(b, b + 1, b + 2, b + 3), (t_ + 3, t_ + 2, t_ + 1, t_),
                 (b, b + 3, t_ + 3, t_), (b + 1, t_ + 1, t_ + 2, b + 2),
                 (b, t_, t_ + 1, b + 1), (b + 3, b + 2, t_ + 2, t_ + 3)]
        for q in quads:
            fout.write("f " + " ".join(map(str, q)) + "\n")
        idx += 8

# --- report + check against the scan ------------------------------------------------------------
to_in = lambda p: [round(p[0] / IN + 120, 1), round(p[1] / IN + 120, 1)]
P = sc.load_level(ply)
W = sc.apply(fit["theta"], np.array(fit["t"]), P)
wall_pts = W[(W[:, 2] > 0.35) & (W[:, 2] < 1.55)][:, :2]
tree = cKDTree(wall_pts)
plan = {"M1 maze divider": ([44, 140], [44, 203.5]), "M2 maze divider": ([88, 240], [88, 176.5]),
        "M3 maze divider": ([132, 140], [132, 203.5]), "A1 approach wall": ([81.6, 136], [81.6, 95.5]),
        "C1 chicane wall": ([158.322, 75.57], [180.04, 135.24])}
rows = []
for name, (p0, p1) in new.items():
    L = np.linalg.norm(p1 - p0)
    samp = p0 + np.linspace(0, 1, 60)[:, None] * (p1 - p0)
    d, _ = tree.query(samp)
    seen = float((d < THK / 2 + 0.05).mean())
    bear = math.degrees(math.atan2(p1[0] - p0[0], p1[1] - p0[1])) % 180
    q0, q1 = (np.array(plan[name][0]) - 120) * IN, (np.array(plan[name][1]) - 120) * IN
    mid_shift = np.linalg.norm((p0 + p1) / 2 - (q0 + q1) / 2)
    rows.append({"name": name, "start_m": p0.round(3).tolist(), "end_m": p1.round(3).tolist(),
                 "start_in": to_in(p0), "end_in": to_in(p1), "length_m": round(float(L), 3),
                 "bearing_deg": round(bear, 1), "scan_support": round(seen, 2),
                 "moved_from_plan_m": round(float(mid_shift), 3)})
    print(f"{name:<18} ({p0[0]:+.2f}, {p0[1]:+.2f}) -> ({p1[0]:+.2f}, {p1[1]:+.2f}) m  len {L:.2f}  bearing {bear:5.1f}"
          f"  scan along it {100 * seen:3.0f}%  centre moved {mid_shift * 100:.0f} cm from the plan")
json.dump({"note": "best guess from the phone scan, not tape-measured; course frame as the .obj "
                   "(m, origin floor centre) and course_v3.json (in, origin SW net corner)",
           "thickness_in": 4, "height_in": 68, "walls": rows}, open("asbuilt_walls.json", "w"), indent=1)

model, pts, nrm, grp = sc.model_samples(out)
cv = sc.Canvas()
cv.put(W[(W[:, 2] > 0.35) & (W[:, 2] < 1.55)], (175, 175, 175))
solid = ~np.array(["net" in g.lower() for g in grp]) & (pts[:, 2] > 0.3) & (pts[:, 2] < 1.6) & (np.abs(nrm[:, 2]) < 0.3)
cv.put(pts[solid], (220, 30, 30))
for name, (p0, p1) in new.items():
    cv.line(p0, p1, (20, 60, 220), 1)
sc.png("asbuilt.png", cv.img)
print(f"wrote {out}, asbuilt_walls.json, asbuilt.png")
