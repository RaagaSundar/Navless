"""The course route and its time-parametrised trajectory.

Route = corner waypoints derived from the course map (corridor centrelines, maze openings, tube axis,
window normal, pad centre), each with a fillet radius. Corners become circular arcs, then a speed
profile is fitted under limits on speed, lateral acceleration (v^2 * curvature), longitudinal
acceleration and climb/descent rate, plus per-zone speed caps (window, tube, landing).

All geometry here is in the WORLD frame (x east, y north, z up, metres).
"""
import json
import math
import os
from dataclasses import dataclass, field

import numpy as np

from .geom import IN, HALF, map_to_world

HERE = os.path.dirname(os.path.abspath(__file__))
COURSE_JSON = os.path.join(HERE, "..", "course", "course_v3.json")

GROUND_Z = 0.07        # base_link height when the quad stand-in sits on its feet


@dataclass
class Limits:
    v_max: float = 2.0          # m/s
    a_lat: float = 3.0          # m/s^2, v^2 * curvature
    a_long: float = 2.5         # m/s^2, along the path
    vz_up: float = 1.5          # m/s
    vz_down: float = 1.0        # m/s
    v_window: float = 1.5       # m/s while crossing the window wall
    v_tube: float = 2.0         # m/s inside the tube
    v_land: float = 0.6         # m/s on the final vertical descent
    smooth: float = 0.08        # s, Gaussian smoothing of the timed trajectory (makes acceleration continuous)
    a_vert: float = 4.0         # m/s^2, cap on the vertical part of the along-path acceleration
    a_total: float = 0.0        # m/s^2, budget for lateral + along-path acceleration together (0 = max(a_lat, a_long))


@dataclass
class Waypoint:
    name: str
    p: tuple                    # world (x, y, z)
    r: float = 0.0              # fillet radius at this corner (m)
    yaw: float = None           # PX4 yaw (deg, 0 = north, cw) to reach by this waypoint; None = keep


@dataclass
class Trajectory:
    t: np.ndarray
    s: np.ndarray
    p: np.ndarray               # world positions (N,3)
    v: np.ndarray               # world velocities (N,3)
    a: np.ndarray               # world accelerations (N,3)
    yaw: np.ndarray             # PX4 yaw (rad)
    speed: np.ndarray
    wp_s: dict = field(default_factory=dict)   # waypoint name -> path length at the corner

    @property
    def duration(self):
        return float(self.t[-1])

    @property
    def length(self):
        return float(self.s[-1])

    def sample(self, t):
        """Setpoint at time t (s since start): position, velocity, acceleration (world), yaw (rad)."""
        t = min(max(t, 0.0), self.t[-1])
        i = int(np.clip(np.searchsorted(self.t, t) - 1, 0, len(self.t) - 2))
        f = (t - self.t[i]) / max(self.t[i + 1] - self.t[i], 1e-9)
        lerp = lambda X: X[i] + f * (X[i + 1] - X[i])
        return lerp(self.p), lerp(self.v), lerp(self.a), float(lerp(self.yaw))


# ---------------------------------------------------------------------------------------- the route
def window_route(course_path=COURSE_JSON, z_maze=0.94, z_window=0.85):
    """Launch -> corridor -> maze -> Gate A -> tube -> window -> pad. Every point comes from the map."""
    d = json.load(open(course_path))
    W = lambda x, y, z: (*map_to_world(x, y), z)
    lx, ly = d["launch_mark_in"]
    line = d["floor_line_in"]                      # corridor + maze centreline, ends at Gate A
    tube = d["tunnel"]["footprint_in"]
    tube_x = (tube["x_min"] + tube["x_max"]) / 2
    win_wall = next(w for w in d["walls"] if "window" in w)
    win = win_wall["window"]
    z_win = z_window if z_window is not None else (win["bottom_above_floor_in"] + win["top_above_floor_in"]) / 2 * IN
    b = math.radians(win_wall["bearing_deg"])
    n = (math.sin(math.radians(win["cross_heading_deg"])), math.cos(math.radians(win["cross_heading_deg"])))
    wc = win["center_in"]
    pad = d["landing_pad_in"]["center"]
    U = 0.57                                        # maze U-turn radius: half the 45 in corridor spacing

    wps = [
        Waypoint("launch", W(lx, ly, GROUND_Z), 0.0, yaw=0.0),
        Waypoint("liftoff", W(lx, ly, 0.45), 0.25),
        Waypoint("climb", W(lx, ly + 30, z_maze), 0.30),
        Waypoint("corridor_end", W(line[1][0], line[1][1], z_maze), U),     # (21, 221.75)
        Waypoint("m1_top", W(line[2][0], line[2][1], z_maze), U),           # (66, 221.75)
        Waypoint("m2_bottom_w", W(line[3][0], line[3][1], z_maze), U),      # (66, 158.25)
        Waypoint("m2_bottom_e", W(line[4][0], line[4][1], z_maze), U),      # (110, 158.25)
        Waypoint("m3_top", W(line[5][0], line[5][1], z_maze), 0.50),        # (110, 221.75)
        Waypoint("gate_a_out", W(line[6][0] + 18, line[6][1], z_maze), 1.0),  # past Gate A (132, 221.75)
        Waypoint("tube_turn", W(tube_x, 200, z_maze), 0.60),
        Waypoint("tube_exit", W(tube_x - 3, tube["y_min"] - 26, z_maze), 0.80, yaw=0.0),
        Waypoint("window_approach", W(wc[0] - n[0] * 30, wc[1] - n[1] * 30, z_win), 0.60,
                 yaw=win_wall["bearing_deg"]),
        Waypoint("window_exit", W(wc[0] + n[0] * 24, wc[1] + n[1] * 24, z_win), 0.60),
        Waypoint("pad_above", W(pad[0], pad[1], 0.35), 0.25),
        Waypoint("pad", W(pad[0], pad[1], GROUND_Z + 0.05), 0.0),
    ]
    zones = {
        "window": (W(wc[0], wc[1], z_win), 0.55),               # (centre, radius): cap speed near the window
        "tube": ((tube_x * IN - HALF, (tube["y_min"] + tube["y_max"]) / 2 * IN - HALF), (tube["y_max"] - tube["y_min"]) / 2 * IN + 0.2),
    }
    return wps, zones


# ---------------------------------------------------------------------------------------- geometry
def _fillet_path(wps, ds=0.01):
    """Polyline with circular fillets -> densely sampled points, unit tangents, curvature vectors."""
    P = np.array([w.p for w in wps], float)
    n = len(P)
    seg = P[1:] - P[:-1]
    L = np.linalg.norm(seg, axis=1)
    D = seg / L[:, None]
    # tangent lengths, clamped so neighbouring fillets never overlap
    T = np.zeros(n)
    phi = np.zeros(n)
    for i in range(1, n - 1):
        c = float(np.clip(D[i - 1] @ D[i], -1, 1))
        phi[i] = math.acos(c)
        T[i] = wps[i].r * math.tan(phi[i] / 2) if phi[i] > 1e-4 else 0.0
    for i in range(n - 1):
        avail = L[i]
        if T[i] + T[i + 1] > avail:
            k = avail / (T[i] + T[i + 1]) * 0.999
            T[i] *= k
            T[i + 1] *= k
    pts, tan, curv, wp_s = [], [], [], {wps[0].name: 0.0}
    s_acc = 0.0

    def add_line(a, b):
        nonlocal s_acc
        l = np.linalg.norm(b - a)
        if l < 1e-9:
            return
        m = max(int(math.ceil(l / ds)), 1)
        u = (b - a) / l
        for k in range(m):
            pts.append(a + u * (l * k / m)); tan.append(u); curv.append(np.zeros(3))
        s_acc += l

    start = P[0]
    for i in range(1, n - 1):
        A = P[i] - D[i - 1] * T[i]
        add_line(start, A)
        if T[i] > 0:
            r = T[i] / math.tan(phi[i] / 2)
            bis = D[i] - D[i - 1]
            bis /= np.linalg.norm(bis)
            C = P[i] + bis * (r / math.cos(phi[i] / 2))
            e1 = (A - C) / r
            e2 = D[i - 1]
            arc = r * phi[i]
            m = max(int(math.ceil(arc / ds)), 1)
            for k in range(m):
                th = phi[i] * k / m
                q = C + r * (math.cos(th) * e1 + math.sin(th) * e2)
                t = -math.sin(th) * e1 + math.cos(th) * e2
                pts.append(q); tan.append(t); curv.append((C - q) / r**2)
            wp_s[wps[i].name] = s_acc + arc / 2
            s_acc += arc
            start = P[i] + D[i] * T[i]
        else:
            wp_s[wps[i].name] = s_acc
            start = A
    add_line(start, P[-1])
    pts.append(P[-1]); tan.append(D[-1]); curv.append(np.zeros(3))
    wp_s[wps[-1].name] = s_acc
    pts, tan, curv = np.array(pts), np.array(tan), np.array(curv)
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    return s, pts, tan, curv, wp_s


def build_trajectory(wps, zones, lim=Limits(), ds=0.01):
    s, p, t_hat, kv, wp_s = _fillet_path(wps, ds)
    kappa = np.linalg.norm(kv, axis=1)
    N = len(s)

    # ---- speed limits along the path
    vlim = np.full(N, lim.v_max)
    with np.errstate(divide="ignore"):
        vlim = np.minimum(vlim, np.sqrt(lim.a_lat / np.maximum(kappa, 1e-9)))
    tz = t_hat[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        vlim = np.minimum(vlim, np.where(tz > 1e-3, lim.vz_up / tz, np.inf))
        vlim = np.minimum(vlim, np.where(tz < -1e-3, lim.vz_down / -tz, np.inf))
    wc, wr = zones["window"]
    near_win = np.linalg.norm(p[:, :2] - np.array(wc[:2]), axis=1) < wr
    vlim[near_win] = np.minimum(vlim[near_win], lim.v_window)
    (tx, ty), tr = zones["tube"]
    in_tube = (np.abs(p[:, 0] - tx) < 0.75) & (np.abs(p[:, 1] - ty) < tr)
    vlim[in_tube] = np.minimum(vlim[in_tube], lim.v_tube)
    final = s > wp_s["pad_above"]
    vlim[final] = np.minimum(vlim[final], lim.v_land)

    # ---- forward / backward passes under the longitudinal acceleration limit
    v = vlim.copy()
    v[0] = v[-1] = 0.0
    ds_ = np.diff(s)
    a_tot = lim.a_total or max(lim.a_lat, lim.a_long)
    abs_tz = np.abs(tz)

    def a_avail(i, vi):
        # along-path acceleration left after turning (friction circle) and capped vertically (thrust)
        a = min(lim.a_long, math.sqrt(max(a_tot ** 2 - (vi * vi * kappa[i]) ** 2, 0.25)))
        if abs_tz[i] > 1e-3:
            a = min(a, lim.a_vert / abs_tz[i])
        return a

    for i in range(N - 1):
        v[i + 1] = min(v[i + 1], math.sqrt(v[i] ** 2 + 2 * a_avail(i, v[i]) * ds_[i]))
    for i in range(N - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a_avail(i + 1, v[i + 1]) * ds_[i]))

    # ---- time, velocity, acceleration
    vm = np.maximum((v[1:] + v[:-1]) / 2, 1e-3)
    t = np.concatenate([[0], np.cumsum(ds_ / vm)])
    vel = t_hat * v[:, None]
    dvds = np.gradient(v, s)
    acc = t_hat * (v * dvds)[:, None] + kv * (v ** 2)[:, None]

    # ---- yaw keyframes: hold, then turn linearly over the segment leading into the keyframe
    keys = [(wp_s[w.name], math.radians(w.yaw)) for w in wps if w.yaw is not None]
    ks = np.array([k[0] for k in keys])
    ky = np.unwrap(np.array([k[1] for k in keys]))
    yaw = np.interp(s, ks, ky)
    if lim.smooth <= 0:
        return Trajectory(t=t, s=s, p=p, v=vel, a=acc, yaw=yaw, speed=v, wp_s=wp_s)

    # ---- resample on a uniform clock and Gaussian-smooth position (jerk-continuous setpoints)
    dt = 0.005
    pad = int(4 * lim.smooth / dt)
    tu = np.arange(0, t[-1] + dt, dt)
    pu = np.stack([np.interp(tu, t, p[:, k]) for k in range(3)], 1)
    su = np.interp(tu, t, s)
    yu = np.interp(tu, t, yaw)
    k = np.arange(-pad, pad + 1) * dt
    ker = np.exp(-0.5 * (k / lim.smooth) ** 2)
    ker /= ker.sum()
    ext = np.concatenate([np.repeat(pu[:1], pad, 0), pu, np.repeat(pu[-1:], pad, 0)])
    ps = np.stack([np.convolve(ext[:, j], ker, mode="valid") for j in range(3)], 1)
    # (the ends move by only a few mm: the path starts and ends at rest)
    vs = np.gradient(ps, dt, axis=0)
    as_ = np.gradient(vs, dt, axis=0)
    return Trajectory(t=tu, s=su, p=ps, v=vs, a=as_, yaw=yu, speed=np.linalg.norm(vs, axis=1), wp_s=wp_s)
