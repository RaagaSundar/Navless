#!/usr/bin/env python3
"""List where a run came closest to obstacles and where tracking error peaked.

  python3 tools/worst_points.py runs/<run>   (or no argument for the newest run)
"""
import csv
import glob
import os
import sys

import numpy as np

run = sys.argv[1] if len(sys.argv) > 1 else max(glob.glob(os.path.join(os.path.dirname(__file__), "..", "runs", "*/")), key=os.path.getmtime)
rows = list(csv.DictReader(open(os.path.join(run, "log.csv"))))
g = lambda k: np.array([float(r[k]) if r[k] not in ("", "nan", "None") else np.nan for r in rows])
t, c, e = g("t"), g("clearance"), g("err")
x, y, z, sx, sy, sz = g("x"), g("y"), g("z"), g("sp_x"), g("sp_y"), g("sp_z")
print(os.path.basename(os.path.normpath(run)))
print("closest approaches (local minima of clearance < 15 cm):")
for i in range(1, len(c) - 1):
    if c[i] < 0.15 and c[i] <= c[i - 1] and c[i] <= c[i + 1]:
        print(f"  t={t[i]:5.2f}  clearance {c[i] * 100:5.1f} cm  at ({x[i]:+.2f},{y[i]:+.2f},{z[i]:.2f})  setpoint ({sx[i]:+.2f},{sy[i]:+.2f},{sz[i]:.2f})  err {e[i] * 100:.1f} cm")
print("largest tracking errors:")
for i in np.argsort(-np.nan_to_num(e))[:200:25]:
    print(f"  t={t[i]:5.2f}  err {e[i] * 100:5.1f} cm  pos ({x[i]:+.2f},{y[i]:+.2f},{z[i]:.2f})  sp ({sx[i]:+.2f},{sy[i]:+.2f},{sz[i]:.2f})")
