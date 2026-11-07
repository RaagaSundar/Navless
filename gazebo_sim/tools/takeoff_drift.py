#!/usr/bin/env python3
"""Horizontal drift from the launch point during the first seconds of a run (takeoff behaviour).

  python3 tools/takeoff_drift.py runs/<run> [...] [--window 6]
"""
import csv
import os
import sys

import numpy as np

args = [a for a in sys.argv[1:] if not a.startswith("--")]
window = float(sys.argv[sys.argv.index("--window") + 1]) if "--window" in sys.argv else 6.0
for run in args:
    if run.replace(".", "").isdigit():
        continue
    rows = list(csv.DictReader(open(os.path.join(run, "log.csv"))))
    g = lambda k: np.array([float(r[k]) if r[k] not in ("", "nan", "None") else np.nan for r in rows])
    t, x, y, z = g("t"), g("x"), g("y"), g("z")
    m = t < window
    dx, dy = x[m] - x[0], y[m] - y[0]
    k = int(np.argmin(dy))
    print(f"{os.path.basename(os.path.normpath(run))}: max south {-dy.min() * 100:.1f} cm at t={t[m][k]:.2f}s (z {z[m][k]:.2f}), "
          f"max |east| {np.abs(dx).max() * 100:.1f} cm")
