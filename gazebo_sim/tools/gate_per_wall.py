#!/usr/bin/env python3
"""Minimum clearance per obstacle (state ROUTE) in fly_navless.py runs: which wall each lap actually came
closest to, not just the overall minimum.

  cd gazebo_sim && python3 tools/gate_per_wall.py runs/*navless-asb[0-9]* [--top 6] [--sdf model.sdf]

Measure against the course the laps were flown in. By default that's models/dh_course/model.sdf under the
current directory (where the runs/ are), so run it from the gate clone's gazebo_sim: the repo's own model is
the organisers' plan, and as-built laps measured against it go through walls that aren't there.
"""
import argparse
import csv
import glob
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from dh.geom import Obstacles, clearance  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("runs", nargs="+", help="run directories (each with log.csv), or globs of them")
ap.add_argument("--top", type=int, default=6, help="how many of the closest obstacles to list")
ap.add_argument("--sdf", default=os.path.join("models", "dh_course", "model.sdf"),
                help="course model the laps were flown in (default: under the current directory)")
a = ap.parse_args()

if not os.path.exists(a.sdf):
    sys.exit(f"{a.sdf} not found: run from the gazebo_sim the laps were flown in, or pass --sdf")
obs = Obstacles(a.sdf)
print(f"model: {os.path.abspath(a.sdf)} ({len(obs.names)} collision primitives)")
for run in sorted({r for pattern in a.runs for r in (glob.glob(pattern) or [pattern])}):
    if not os.path.exists(os.path.join(run, "log.csv")):
        continue
    rows = [r for r in csv.DictReader(open(os.path.join(run, "log.csv"))) if r["state"] == "ROUTE"]
    if not rows:
        continue
    P = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows])
    yaw_ned = np.array([math.pi / 2 - float(r["yaw_enu"]) for r in rows])
    c, names = clearance(obs, P, yaw_ned)
    per = {}
    for ci, ni in zip(c, names):
        per[ni] = min(per.get(ni, math.inf), float(ci))
    worst = sorted(per.items(), key=lambda kv: kv[1])[:a.top]
    print(os.path.basename(run.rstrip("/")) + ": " + ", ".join(f"{k} {v * 100:.1f}" for k, v in worst))
