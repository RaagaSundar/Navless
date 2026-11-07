"""Flight metrics from tools/sim_truth_recorder.py CSVs (.csv or .csv.gz): armed time, wall clearance from the
course mesh (floor excluded), VIO dropouts, PX4-vs-truth error, touchdowns vs the pad and the launch mark.

  python tools/flyx_lab/flight_metrics.py est.csv [more.csv ...]      (from the repo root; needs numpy)
"""
import csv, gzip, math, sys
import numpy as np

sys.path.insert(0, ".")
from planner.config import PlannerConfig
from planner.grid_map import OccupancyGrid3D

ORIGIN = np.array([-2.515, 1.114, 0.126])           # course ENU of PX4's local origin in the FlyX lab
PAD, HOME = np.array([-2.042, -0.792]), np.array([-2.515, 1.114])

cfg = PlannerConfig()
g = OccupancyGrid3D(bounds_min_m=cfg.global_map_bounds_min_m, bounds_max_m=cfg.global_map_bounds_max_m,
                    resolution_m=0.03, inflation_radius_m=0.0)
g.load_prior_from_obj("gazebo_sim/models/dh_course/meshes/course_visual.obj")
occ = np.argwhere(g.prior_grid)
walls = g.bounds_min + (occ + 0.5) * g.resolution
walls = walls[walls[:, 2] > 0.06]                    # drop the floor: clearance is to walls/ceilings


def load(path):
    op = gzip.open if path.endswith(".gz") else open
    with op(path, "rt", encoding="utf-8", errors="ignore") as f:
        rows = [r for r in csv.DictReader(l for l in f if l[:1].isdigit() or l.startswith("sim_t")) if r.get("sim_t")]
    fl = lambda r, k: float(r[k]) if r.get(k) not in (None, "") else math.nan
    return rows, fl


def metrics(path):
    rows, fl = load(path)
    t = np.array([fl(r, "sim_t") for r in rows])
    pos = ORIGIN + np.array([[fl(r, "truth_e"), fl(r, "truth_n"), fl(r, "truth_u")] for r in rows])
    armed = np.array([r["arming_state"] == "2" for r in rows])
    if not armed.any():
        return print(f"{path}: never armed")
    t_arm, t_end = t[armed][0], t[armed][-1]
    air = armed & (pos[:, 2] > 0.25)
    # clearance at ~10 Hz while airborne
    idx = np.where(air)[0]
    idx = idx[np.minimum(np.searchsorted(t[idx], np.arange(t[idx][0], t[idx][-1], 0.1)), len(idx) - 1)] if len(idx) else idx
    clear = np.full(len(idx), np.inf)
    for k, i in enumerate(idx):
        near = walls[np.all(np.abs(walls - pos[i]) < 0.6, axis=1)]
        if len(near):
            clear[k] = np.min(np.linalg.norm(near - pos[i], axis=1))
    worst = int(np.argmin(clear)) if len(clear) else None
    err = np.array([fl(r, "err_xy") for r, a in zip(rows, air) if a])
    vo = [r["vo_state"] for r, a in zip(rows, armed) if a]
    drops = sum(1 for a, b in zip(vo, vo[1:]) if a == "1" and b not in ("1", ""))
    # touchdowns: armed->disarmed transitions
    downs = [pos[i, :2] for i in range(1, len(rows)) if armed[i - 1] and not armed[i]]
    print(f"== {path}")
    print(f"  armed {t_end - t_arm:5.1f} s (first arm -> last disarm), airborne {air.sum() and (t[air][-1] - t[air][0]):5.1f} s span")
    if worst is not None:
        w = pos[idx[worst]]
        print(f"  min wall clearance (centre) {clear[worst]:.3f} m at course ({w[0]:.2f}, {w[1]:.2f}, {w[2]:.2f}), t+{t[idx[worst]] - t_arm:.1f} s;"
              f"  <0.20 m for {np.sum(clear < 0.20) * 0.1:.1f} s")
    print(f"  VIO dropouts while armed: {drops};  PX4-vs-truth error max {np.nanmax(err):.3f} m, p95 {np.nanpercentile(err, 95):.3f} m")
    for k, d in enumerate(downs):
        print(f"  touchdown {k + 1} at ({d[0]:.2f}, {d[1]:.2f}): {np.linalg.norm(d - PAD):.2f} m from pad centre, {np.linalg.norm(d - HOME):.2f} m from launch mark")


for p in sys.argv[1:]:
    metrics(p)
