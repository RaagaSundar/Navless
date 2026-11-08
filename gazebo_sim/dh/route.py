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
def _tangent_corner(p, centre, rho, q):
    """Corner of two straight legs that both touch the circle (centre, rho): one from p with the circle on its
    right, one into q from the circle's left (so the path rounds the circle clockwise). A fillet of radius rho
    at this corner is that circle's arc. World x/y."""
    def heading(a, side):
        v = centre - a
        return math.atan2(v[1], v[0]) + side * math.asin(rho / float(np.linalg.norm(v)))
    hp, hq = heading(p, 1.0), heading(q, -1.0)
    dp, dq = np.array([math.cos(hp), math.sin(hp)]), np.array([math.cos(hq), math.sin(hq)])
    k = np.linalg.solve(np.array([dp, -dq]).T, q - p)
    return p + dp * k[0]


def maze_2026_11_07(d, z_maze=0.94, tip_radius_m=0.585, tube_turn_y_in=208.0):
    """The maze leg since the organisers' 2026-11-07 change: M1-M3 replaced by MN (along the north net), MD (the
    long diagonal, tip 1 m from the net) and MT (barrier line to the tube's NW corner). The floor tape is the
    old maze's and runs under MD, so nothing here follows it (the map has no floor_line_in any more).
    Waypoints from the launch mark's climb to the tube turn (md_tip, tube_turn).

    Two straight legs, both tangent to a circle of radius tip_radius_m centred on MD's tip: one from the launch
    mark north-east up corridor 1 beside MD (bearing 51 deg), one into the tube turn on the tube's axis. The
    corner between them is filleted with that radius, so the path rounds the tip on the circle, through Gate A
    (the gap between MD's tip and the north net). Nose north (course_mission holds the takeoff heading), the
    drone reaches 0.28 m towards a wall 45 deg off its axes but only 0.2275 m behind and 0.1575 m ahead
    (DRONE_PARTS): 0.585 m balances MD (25.2 cm planned) against the north net at the top of the arc (25.8 cm).
    The first leg heads away from the west net at once. On the launch mark the prop tips are 29.6 cm from that
    net, 19's liftoffs have slid 25-33 cm west, and course_mission leans the take-off takeoff_lead_m along this
    leg: north-east, the lean puts the climb 19 cm further from the net (a leg due north would lean along it).
    tube_turn_y_in puts the turn into the tube far enough north that the path is straight 0.27 m before the
    drone's back reaches the mouth (208 in; 200 in, the old maze's, left 0.12 m).
    """
    walls = {w["name"]: w for w in d["walls"]}
    lx, ly = d["launch_mark_in"]
    tube = d["tunnel"]["footprint_in"]
    tube_x = (tube["x_min"] + tube["x_max"]) / 2
    launch = np.array(map_to_world(lx, ly))
    tip = np.array(map_to_world(*walls["MD"]["end_in"]))
    turn = np.array(map_to_world(tube_x, tube_turn_y_in))
    corner = _tangent_corner(launch, tip, tip_radius_m, turn)
    return [
        Waypoint("md_tip", (float(corner[0]), float(corner[1]), z_maze), tip_radius_m),
        Waypoint("tube_turn", (float(turn[0]), float(turn[1]), z_maze), 0.60),
    ]


def window_route(course_path=COURSE_JSON, z_maze=0.94, z_window=0.85, window_shift_m=0.0,
                 tube_exit_drop_in=26.0, pad_from_south_in=0.0, maze_top_north_m=0.0, window_via_in=None,
                 tip_radius_m=0.585, tube_turn_y_in=208.0):
    """Launch -> corridor -> maze -> Gate A -> tube -> window -> pad. Every point comes from the map.

    A map without floor_line_in has the organisers' 2026-11-07 maze: maze_2026_11_07 plans that leg from
    its walls (tip_radius_m: the turn round MD's tip; tube_turn_y_in: where it turns into the tube, map
    inches) and maze_top_north_m does not apply.

    window_shift_m moves the window crossing along the wall (+ = towards the wall's north-east end, C2a_b).
    A carrot follower cuts the turn before the window, so it crosses off-centre towards C2a_a, and
    aiming this far the other way centres the flown crossing.

    The defaults are the plan's route. On the as-built course C1's south end and A1's south end sit
    in its last two legs: tube_exit_drop_in turns towards the window further below the tube (clear of
    C1), and pad_from_south_in > 0 adds a point that far south of the pad, so the descent comes in
    from the south, clear of A1, instead of diagonally past its end.
    maze_top_north_m moves the two crossings over M1's and M3's north ends towards the north net. The
    floor line puts them midway between wall end and net, but nose north the drone reaches 0.2275 m
    behind its centre and only 0.1575 m ahead (DRONE_PARTS), so midway leaves the wall end the
    smaller gap. On the as-built course those gaps are only ~0.6 m wide.
    window_via_in (x, y map inches) adds a point between the tube exit and the window approach. On the
    as-built route it keeps the downward rangefinder's beam off boulder 3: PX4 takes its height from
    that rangefinder, and over the boulder it reads 0.4 m short.
    """
    d = json.load(open(course_path))
    W = lambda x, y, z: (*map_to_world(x, y), z)
    lx, ly = d["launch_mark_in"]
    tube = d["tunnel"]["footprint_in"]
    tube_x = (tube["x_min"] + tube["x_max"]) / 2
    win_wall = next(w for w in d["walls"] if "window" in w)
    win = win_wall["window"]
    z_win = z_window if z_window is not None else (win["bottom_above_floor_in"] + win["top_above_floor_in"]) / 2 * IN
    b = math.radians(win_wall["bearing_deg"])
    n = (math.sin(math.radians(win["cross_heading_deg"])), math.cos(math.radians(win["cross_heading_deg"])))
    wc = win["center_in"]
    aim = (wc[0] + math.sin(b) * window_shift_m / IN, wc[1] + math.cos(b) * window_shift_m / IN)
    pad = d["landing_pad_in"]["center"]
    U = 0.57                                        # maze U-turn radius: half the 45 in corridor spacing
    top = maze_top_north_m / IN                     # inches: the map's unit
    if "floor_line_in" in d:                        # the maze before 2026-11-07 (M1-M3)
        line = d["floor_line_in"]                   # corridor + maze centreline, ends at Gate A
        maze = [
            Waypoint("climb", W(lx, ly + 30, z_maze), 0.30),
            Waypoint("corridor_end", W(line[1][0], line[1][1] + top, z_maze), U),     # (21, 221.75)
            Waypoint("m1_top", W(line[2][0], line[2][1] + top, z_maze), U),           # (66, 221.75)
            Waypoint("m2_bottom_w", W(line[3][0], line[3][1], z_maze), U),            # (66, 158.25)
            Waypoint("m2_bottom_e", W(line[4][0], line[4][1], z_maze), U),            # (110, 158.25)
            Waypoint("m3_top", W(line[5][0], line[5][1] + top, z_maze), 0.50),        # (110, 221.75)
            Waypoint("gate_a_out", W(line[6][0] + 18, line[6][1] + top, z_maze), 1.0),  # past Gate A (132, 221.75)
            Waypoint("tube_turn", W(tube_x, 200, z_maze), 0.60),
        ]
    else:
        maze = maze_2026_11_07(d, z_maze, tip_radius_m, tube_turn_y_in)

    wps = [
        Waypoint("launch", W(lx, ly, GROUND_Z), 0.0, yaw=0.0),
        Waypoint("liftoff", W(lx, ly, 0.45), 0.25),
    ] + maze + [
        Waypoint("tube_exit", W(tube_x - 3, tube["y_min"] - tube_exit_drop_in, z_maze), 0.80, yaw=0.0),
    ] + ([Waypoint("window_via", W(window_via_in[0], window_via_in[1], (z_maze + z_win) / 2), 0.50)]
         if window_via_in else []) + [
        Waypoint("window_approach", W(aim[0] - n[0] * 30, aim[1] - n[1] * 30, z_win), 0.60,
                 yaw=win_wall["bearing_deg"]),
        Waypoint("window_exit", W(aim[0] + n[0] * 24, aim[1] + n[1] * 24, z_win), 0.60),
    ] + ([Waypoint("pad_south", W(pad[0], pad[1] - pad_from_south_in, 0.60), 0.35)] if pad_from_south_in > 0 else []) + [
        Waypoint("pad_above", W(pad[0], pad[1], 0.35), 0.25),
        Waypoint("pad", W(pad[0], pad[1], GROUND_Z + 0.05), 0.0),
    ]
    zones = {
        "window": (W(wc[0], wc[1], z_win), 0.55),               # (centre, radius): cap speed near the window
        "tube": ((tube_x * IN - HALF, (tube["y_min"] + tube["y_max"]) / 2 * IN - HALF), (tube["y_max"] - tube["y_min"]) / 2 * IN + 0.2),
    }
    return wps, zones


def gate_b_turns_route(course_path, turns_in, z_maze=0.94, maze_top_north_m=0.0, pad_from_south_in=26.0, r=0.45,
                       tip_radius_m=0.585, tube_turn_y_in=208.0):
    """The Gate B route from explicit turns, for maps where gate_b_route's parallel-chicane geometry does not
    hold (the as-built course: C1 leans 347 deg against C2b's 20 deg, and A1's ends are recorded the other way).

    turns_in = (y_w, x_ch, y_g, x_s), map inches: the leg west from the tube at y_w (south of C1's end), the
    chicane leg north at x_ch, the leg west through Gate B at y_g, and the leg south at x_s (east of A1). The
    descent then passes under A1's south end and comes onto the pad from pad_from_south_in south of it, as
    the as-built window route does. The window route's head is reused as far as the tube turn.
    """
    d = json.load(open(course_path))
    head, zones = window_route(course_path, z_maze, maze_top_north_m=maze_top_north_m, tip_radius_m=tip_radius_m,
                               tube_turn_y_in=tube_turn_y_in)
    head = head[:[w.name for w in head].index("tube_turn") + 1]
    tube = d["tunnel"]["footprint_in"]
    tube_x = (tube["x_min"] + tube["x_max"]) / 2 - 3                   # the window route's tube line
    y_w, x_ch, y_g, x_s = turns_in
    px, py = d["landing_pad_in"]["center"]
    W = lambda x, y, z: (*map_to_world(x, y), z)
    wps = head + [
        Waypoint("tube_south", W(tube_x, y_w, z_maze), r, yaw=0.0),
        Waypoint("c1_south", W(x_ch, y_w, z_maze), r),
        Waypoint("gate_b", W(x_ch, y_g, z_maze), r),
        Waypoint("room_east", W(x_s, y_g, z_maze), r),
        Waypoint("a1_south", W(x_s, py - pad_from_south_in, 0.75), r),
        Waypoint("pad_south", W(px, py - pad_from_south_in, 0.60), 0.35),
        Waypoint("pad_above", W(px, py, 0.35), 0.25),
        Waypoint("pad", W(px, py, GROUND_Z + 0.05), 0.0),
    ]
    zones = dict(zones, window=(W(x_ch, y_g, z_maze), 0.55))        # cap speed at the Gate B turn
    return wps, zones


def gate_b_route(course_path=COURSE_JSON, z_maze=0.94, rho_c1=0.485, rho_a1=0.475, r_tube=None, r_room=None):
    """Launch -> corridor -> maze -> Gate A -> tube -> round C1's south end -> chicane -> Gate B -> pad.

    The window route as far as the tube. After it, the path turns round three wall ends on arcs centred on
    them, and every straight leg is tangent to those circles:
      - C1's south end, radius rho_c1, which also puts the chicane leg rho_c1 west of the C1 line;
      - C2b's north end, radius = the rest of the 38 in between the C1 and C2 lines. The leg out of that turn
        runs west through Gate B;
      - A1's south end, radius rho_a1. The leg into it runs south, rho_a1 east of A1; the leg out of it heads
        for the pad.
    r_tube is the turn from the tube heading west, and r_room the turn from Gate B heading south. None (the
    default) means the largest radius that fits beside the neighbouring fillets, so no arc gets shrunk.
    The default radii balance nose-north clearance: 17.4 cm to C1, 17.4 cm to C2b and 17.7 cm to A1 (2 cm grid).
    """
    d = json.load(open(course_path))
    head, zones = window_route(course_path, z_maze)
    head = head[:[w.name for w in head].index("tube_turn") + 1]
    walls = {w["name"]: w for w in d["walls"]}
    M = lambda q: np.array(map_to_world(*q))
    b = math.radians(walls["C1"]["bearing_deg"])
    u = np.array([math.sin(b), math.cos(b)])                  # along the chicane walls (NNE)
    nrm = np.array([math.cos(b), -math.sin(b)])               # across them, towards C1
    t1, t2, t3 = M(walls["C1"]["start_in"]), M(walls["C2b"]["end_in"]), M(walls["A1"]["end_in"])
    rho_c2 = float(nrm @ (t1 - t2)) - rho_c1                  # C2b's end gets what is left of the chicane width
    tube = d["tunnel"]["footprint_in"]
    x_tube = M(((tube["x_min"] + tube["x_max"]) / 2, 0))[0]
    pad = M(d["landing_pad_in"]["center"])

    def meet(p, dp, q, dq):                                   # intersection of two lines (point, direction)
        k = np.linalg.solve(np.array([dp, -dq]).T, q - p)
        return p + dp * k[0]

    dv = t3 - pad                                             # tangent from the pad, passing south of A1's end
    ang = math.atan2(dv[1], dv[0]) - math.asin(rho_a1 / np.linalg.norm(dv))
    east, north = np.array([1.0, 0.0]), np.array([0.0, 1.0])
    y_w, y_g, x_s = t1[1] - rho_c1, t2[1] + rho_c2, t3[0] + rho_a1
    chicane = t1 - nrm * rho_c1                               # a point on the chicane leg
    corners = [
        ("tube_south", np.array([x_tube, y_w]), r_tube),
        ("c1_end", meet(np.array([0.0, y_w]), east, chicane, u), rho_c1),
        ("gate_b", meet(chicane, u, np.array([0.0, y_g]), east), rho_c2),
        ("room_south", np.array([x_s, y_g]), r_room),
        ("a1_end", meet(np.array([x_s, 0.0]), north, pad, np.array([math.cos(ang), math.sin(ang)])), rho_a1),
    ]
    # free radii: the largest whose tangent lengths fit on both neighbouring legs
    pts = [np.array(head[-2].p[:2]), np.array(head[-1].p[:2])] + [q for _, q, _ in corners] + [pad]
    rad = [0.0, head[-1].r] + [r for *_, r in corners] + [0.0]
    lens = [float(np.linalg.norm(b_ - a_)) for a_, b_ in zip(pts[:-1], pts[1:])]
    dirs = [(b_ - a_) / ln for a_, b_, ln in zip(pts[:-1], pts[1:], lens)]
    tans = [0.0] + [math.tan(math.acos(float(np.clip(dirs[i - 1] @ dirs[i], -1, 1))) / 2)
                    for i in range(1, len(pts) - 1)] + [0.0]
    for i in range(2, len(pts) - 1):
        if rad[i] is None:
            rad[i] = 0.99 * min(lens[i - 1] - (rad[i - 1] or 0.0) * tans[i - 1],
                                lens[i] - (rad[i + 1] or 0.0) * tans[i + 1]) / tans[i]
    corners = [(n, q, rad[i + 2]) for i, (n, q, _) in enumerate(corners)]
    wps = head + [Waypoint(n, (float(q[0]), float(q[1]), z_maze), r, yaw=0.0 if n == "tube_south" else None)
                  for n, q, r in corners] + [
        Waypoint("pad_above", (float(pad[0]), float(pad[1]), 0.35), 0.25),
        Waypoint("pad", (float(pad[0]), float(pad[1]), GROUND_Z + 0.05), 0.0),
    ]
    # build_trajectory caps speed in zones["window"]: here that is the Gate B opening
    zones = dict(zones, window=((float(t2[0]), float(y_g), z_maze), 0.55))
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
