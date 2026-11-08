#!/usr/bin/env python3
"""Split tracking error into along-track (lag) and cross-track parts, and estimate the time lag.

  python3 tools/analyze_run.py runs/<run>/log.csv
"""
import csv
import sys

import numpy as np

rows = list(csv.DictReader(open(sys.argv[1])))
f = lambda k: np.array([float(r[k]) if r[k] not in ("", "nan", "None") else np.nan for r in rows])
t, sp = f("t"), np.stack([f("sp_x"), f("sp_y"), f("sp_z")], 1)
p = np.stack([f("x"), f("y"), f("z")], 1)
fly = np.isfinite(f("err"))
t, sp, p = t[fly], sp[fly], p[fly]
v = np.gradient(sp, t, axis=0)
spd = np.linalg.norm(v, axis=1)
tang = v / np.maximum(spd, 1e-6)[:, None]
e = p - sp
along = np.sum(e * tang, 1)
cross = np.linalg.norm(e - along[:, None] * tang, axis=1)
moving = spd > 0.3
print(f"samples {len(t)}  moving {moving.sum()}")
print(f"along-track: mean {along[moving].mean() * 100:+.1f} cm (negative = behind)  |  cross-track: mean {cross[moving].mean() * 100:.1f} cm, p95 {np.percentile(cross[moving], 95) * 100:.1f}, max {cross[moving].max() * 100:.1f} cm")
print(f"vertical error: mean {e[moving, 2].mean() * 100:+.1f} cm, max |.| {np.abs(e[moving, 2]).max() * 100:.1f} cm")
# best time shift: actual(t) ~ setpoint(t - lag)
best = None
for lag in np.arange(0, 0.6, 0.01):
    sps = np.stack([np.interp(t - lag, t, sp[:, k]) for k in range(3)], 1)
    err = np.linalg.norm(p - sps, axis=1)[moving].mean()
    if best is None or err < best[1]:
        best = (lag, err)
print(f"best-fit lag {best[0] * 1000:.0f} ms -> mean error after removing lag {best[1] * 100:.1f} cm")
