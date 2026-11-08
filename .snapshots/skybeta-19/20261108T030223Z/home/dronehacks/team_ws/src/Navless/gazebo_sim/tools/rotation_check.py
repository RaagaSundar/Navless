#!/usr/bin/env python3
"""Does the flown path look like the planned path rotated about the launch point? (frame-alignment check)

  python3 tools/rotation_check.py runs/<run> [...]
"""
import csv
import os
import sys

import numpy as np

for run in sys.argv[1:]:
    rows = list(csv.DictReader(open(os.path.join(run, "log.csv"))))
    g = lambda k: np.array([float(r[k]) if r[k] not in ("", "nan", "None") else np.nan for r in rows])
    ok = np.isfinite(g("err"))
    ref = np.stack([g("sp_x"), g("sp_y")], 1)[ok]
    gt = np.stack([g("x"), g("y")], 1)[ok]
    p0 = gt[0]
    best = None
    for th in np.radians(np.arange(-10, 10.01, 0.1)):
        c, s = np.cos(th), np.sin(th)
        r = (ref - p0) @ np.array([[c, s], [-s, c]]) + p0
        e = np.linalg.norm(r - gt, axis=1).mean()
        if best is None or e < best[1]:
            best = (np.degrees(th), e)
    e0 = np.linalg.norm(ref - gt, axis=1).mean()
    print(f"{os.path.basename(os.path.normpath(run))}: horizontal err {e0 * 100:.1f} cm; best rotation {best[0]:+.1f} deg -> {best[1] * 100:.1f} cm")
