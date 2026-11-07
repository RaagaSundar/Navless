#!/usr/bin/env python3
"""Check the planned route offline: length, time, speed, and clearance to every course obstacle.

  python3 plan.py [--v-max 2.5 --a-lat 4 ...] [--png plan.png]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dh.geom import Obstacles, clearance           # noqa: E402
from dh.route import Limits, build_trajectory, window_route  # noqa: E402


# Speed presets for the 50 Hz pos+vel+acc follower (DH_PX4_PROFILE=sim). "tilt" is the PX4 tilt limit (deg)
# fly_course.py sets before arming: the faster presets need headroom above the planned tilt for feedback.
PRESETS = {
    "safe": dict(v_max=2.0, a_lat=3.0, a_long=2.5, v_window=1.5, v_tube=2.0),
    "fast": dict(v_max=4.0, a_lat=5.0, a_long=4.5, v_window=2.5, v_tube=4.0, vz_down=1.5, v_land=0.8),
    "race": dict(v_max=6.0, a_lat=8.0, a_long=7.0, v_window=3.5, v_tube=6.0, vz_up=2.5, vz_down=1.5, v_land=1.0, tilt=55),
    "max": dict(v_max=7.0, a_lat=10.0, a_long=9.0, v_window=4.0, v_tube=7.0, vz_up=3.0, vz_down=2.0, v_land=1.0, tilt=55),
}


def limits_from_args(a):
    lim = Limits(v_max=a.v_max, a_lat=a.a_lat, a_long=a.a_long, v_window=a.v_window, v_tube=a.v_tube,
                 vz_up=a.vz_up, vz_down=a.vz_down, v_land=a.v_land, smooth=a.smooth, a_vert=a.a_vert, a_total=a.a_total)
    for k, v in PRESETS.get(a.preset or "", {}).items():
        if hasattr(lim, k):
            setattr(lim, k, v)
    return lim


def add_limit_args(ap):
    L = Limits()
    ap.add_argument("--preset", choices=sorted(PRESETS), help="named speed limits (override the individual limit flags)")
    ap.add_argument("--v-max", type=float, default=L.v_max)
    ap.add_argument("--a-lat", type=float, default=L.a_lat)
    ap.add_argument("--a-long", type=float, default=L.a_long)
    ap.add_argument("--v-window", type=float, default=L.v_window)
    ap.add_argument("--v-tube", type=float, default=L.v_tube)
    ap.add_argument("--vz-up", type=float, default=L.vz_up)
    ap.add_argument("--vz-down", type=float, default=L.vz_down)
    ap.add_argument("--v-land", type=float, default=L.v_land)
    ap.add_argument("--smooth", type=float, default=L.smooth)
    ap.add_argument("--a-vert", type=float, default=L.a_vert)
    ap.add_argument("--a-total", type=float, default=L.a_total)


def report(traj, obs, every=2):
    idx = np.arange(0, len(traj.s), every)
    flying = traj.p[idx, 2] > 0.25
    c, names = clearance(obs, traj.p[idx], traj.yaw[idx])
    c = np.where(flying, c, np.inf)
    print(f"length {traj.length:.2f} m   planned time {traj.duration:.2f} s   peak speed {traj.speed.max():.2f} m/s")
    k = int(np.argmin(c))
    print(f"min clearance {c[k] * 100:.1f} cm to {names[k]} at s = {traj.s[idx][k]:.2f} m")
    # per-stage minimum clearance
    keys = sorted(traj.wp_s.items(), key=lambda kv: kv[1])
    for (n0, s0), (n1, s1) in zip(keys[:-1], keys[1:]):
        m = (traj.s[idx] >= s0) & (traj.s[idx] < s1) & np.isfinite(c)
        if m.any():
            j = np.where(m)[0][np.argmin(c[m])]
            t0 = np.interp(s0, traj.s, traj.t)
            print(f"  {n0:>16s} -> {n1:<16s} t={t0:5.2f}s  min {c[j] * 100:5.1f} cm ({names[j]})")
    return c, idx


def plot(traj, obs, path, run=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Circle
    fig, ax = plt.subplots(figsize=(9, 9))
    for i, name in enumerate(obs.names):
        if obs.kind[i] != "box" or name.startswith("net"):
            continue
        hx, hy, hz = obs.size[i]
        R = obs.R[i]
        cs = np.array([[sx * hx, sy * hy, 0] for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]) @ R.T + obs.c[i]
        col = "#3a7bd5" if name.startswith("C2a_below") or name.startswith("C2a_above") else ("#777" if name.startswith("tube") else "#222")
        ax.add_patch(Polygon(cs[:, :2], closed=True, color=col, alpha=0.35 if name.startswith("tube") else 0.9))
    for i, name in enumerate(obs.names):
        if obs.kind[i] == "ell":
            ax.add_patch(Circle(obs.c[i][:2], obs.size[i][0], color="#aab", alpha=0.6))
    ax.add_patch(Circle((39.6 * 0.0254 - 3.048, 88.8 * 0.0254 - 3.048), 30 * 0.0254, fill=False, color="#a33"))
    sc = ax.scatter(traj.p[::5, 0], traj.p[::5, 1], c=traj.speed[::5], cmap="viridis", s=4, label="plan")
    plt.colorbar(sc, ax=ax, fraction=0.04, label="planned speed (m/s)")
    if run is not None:
        ax.plot(run[:, 0], run[:, 1], color="#e4572e", lw=1.2, label="flown (ground truth)")
    ax.set_xlim(-3.1, 3.1); ax.set_ylim(-3.1, 3.1); ax.set_aspect("equal")
    ax.set_xlabel("east (m)"); ax.set_ylabel("north (m)")
    ax.set_title(f"DroneHacks course: planned route {traj.length:.1f} m, {traj.duration:.1f} s")
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f"saved {path}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    add_limit_args(ap)
    ap.add_argument("--png", default="plan.png")
    a = ap.parse_args()
    wps, zones = window_route()
    traj = build_trajectory(wps, zones, limits_from_args(a))
    obs = Obstacles()
    report(traj, obs)
    plot(traj, obs, a.png)
