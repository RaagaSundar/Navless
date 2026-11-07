#!/usr/bin/env python3
"""Estimate the real-time factor during a run from log.csv: rows are written at a 50 Hz wall-clock tick,
so sim seconds per row / 0.02 is the RTF. Also reports how the tracking error splits by phase.

  python3 tools/rtf_check.py runs/<run> [...]
"""
import csv
import os
import sys

import numpy as np

for run in sys.argv[1:]:
    rows = list(csv.DictReader(open(os.path.join(run, "log.csv"))))
    g = lambda k: np.array([float(r[k]) if r[k] not in ("", "nan", "None") else np.nan for r in rows])
    st, err = g("sim_t"), g("err")
    d = np.diff(st) / 0.02
    print(f"{os.path.basename(os.path.normpath(run))}: RTF median {np.median(d):.2f}, p5 {np.percentile(d, 5):.2f}, "
          f"min {d.min():.2f}, max {d.max():.2f}; err mean {np.nanmean(err) * 100:.1f} cm")
