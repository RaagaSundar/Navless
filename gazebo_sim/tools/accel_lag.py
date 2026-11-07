#!/usr/bin/env python3
"""Compare commanded vs achieved horizontal acceleration for a run (delay and gain).

  python3 tools/accel_lag.py runs/<run>
"""
import csv
import os
import sys

import numpy as np

run = sys.argv[1]
rows = list(csv.DictReader(open(os.path.join(run, "log.csv"))))
g = lambda k: np.array([float(r[k]) if r[k] not in ("", "nan", "None") else np.nan for r in rows])
fly = np.isfinite(g("err"))
t = g("t")[fly]
sp = np.stack([g("sp_x"), g("sp_y")], 1)[fly]
p = np.stack([g("x"), g("y")], 1)[fly]
tu = np.arange(t[0], t[-1], 0.01)
spu = np.stack([np.interp(tu, t, sp[:, k]) for k in range(2)], 1)
pu = np.stack([np.interp(tu, t, p[:, k]) for k in range(2)], 1)


def smooth(x, n=7):
    k = np.ones(n) / n
    return np.stack([np.convolve(x[:, j], k, mode="same") for j in range(x.shape[1])], 1)


a_sp = np.gradient(np.gradient(spu, 0.01, axis=0), 0.01, axis=0)
a_gt = smooth(np.gradient(np.gradient(smooth(pu), 0.01, axis=0), 0.01, axis=0))
best = None
for lag in range(0, 60):
    a = a_sp[: len(a_sp) - lag] if lag else a_sp
    b = a_gt[lag:]
    m = slice(100, len(a) - 100)
    e = np.mean(np.linalg.norm(a[m] - b[m], axis=1))
    if best is None or e < best[1]:
        best = (lag, e)
lag = best[0]
a, b = a_sp[: len(a_sp) - lag], a_gt[lag:]
gain = np.sum(a * b) / np.sum(a * a)
print(f"acceleration delay {lag * 10} ms, gain {gain:.2f}, residual {best[1]:.2f} m/s^2, |a_sp| mean {np.linalg.norm(a_sp, axis=1).mean():.2f}")
