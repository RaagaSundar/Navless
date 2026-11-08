#!/usr/bin/env python3
"""Render the README's animations from the course model and a recorded Gazebo lap.

  python docs/media/make_readme_media.py                 # all three GIFs
  python docs/media/make_readme_media.py hero            # just one: hero | asbuilt | tube

Inputs (all in the repo):
  gazebo_sim/course/course_asbuilt.json, course_v3.json          the as-built course and the organisers' plan
  gazebo_sim/course/route_centreline_window*.json                 the routes planned on each
  docs/media/data/gate_lap/{log.csv,summary.json}                 one lap of gazebo_sim/tools/gate_asbuilt.sh
Outputs: docs/media/hero_lap.gif, plan_vs_asbuilt.gif, tube_tof.gif.

The hero lap's position, height, speed, wall clearance and checkpoint times are the recorded Gazebo SITL lap.
The camera cone, the side-ToF beams (2D raycast against the course model) and the dark tube are drawn on top.
Needs numpy, matplotlib and Pillow.
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, Ellipse, FancyBboxPatch, Polygon, Rectangle, Wedge  # noqa: E402
from PIL import Image  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
COURSE = os.path.join(REPO, "gazebo_sim", "course")
IN = 0.0254

# Palette (dark, reads well on GitHub's light and dark themes alike).
BG = "#0b0f14"
PANEL = "#111821"
GRID = "#16212c"
NET = "#3a4656"
WALL = "#cdd6e0"
TEXT = "#e6edf3"
MUTED = "#8593a3"
DIM = "#4d5a69"
CYAN = "#38bdf8"
GREEN = "#3ddc97"
AMBER = "#f5a524"
ORANGE = "#ff8a3d"
RED = "#ff5d6c"
VIOLET = "#a78bfa"

plt.rcParams.update({
    "font.family": ["Segoe UI", "DejaVu Sans"],
    "font.size": 11,
    "text.color": TEXT,
    "axes.facecolor": BG,
    "figure.facecolor": BG,
    "savefig.facecolor": BG,
})
MONO = {"family": ["Consolas", "DejaVu Sans Mono"]}


# --------------------------------------------------------------------------- geometry

def to_m(p):
    """course_v3 inches (origin net SW corner, x east, y north) -> world metres (origin floor centre)."""
    return np.array([(p[0] - 120.0) * IN, (p[1] - 120.0) * IN])


def wall_poly(start, end, thick):
    a, b = to_m(start), to_m(end)
    d = (b - a) / np.linalg.norm(b - a)
    n = np.array([-d[1], d[0]]) * thick * IN / 2
    return np.array([a + n, b + n, b - n, a - n])


def load_course(name):
    c = json.load(open(os.path.join(COURSE, name)))
    walls = {w["name"]: dict(w, poly=wall_poly(w["start_in"], w["end_in"], w["thickness_in"])) for w in c["walls"]}
    t = c["tunnel"]["footprint_in"]
    tube = (to_m((t["x_min"], t["y_min"])), to_m((t["x_max"], t["y_max"])))
    pad = c["landing_pad_in"]
    return {
        "raw": c,
        "walls": walls,
        "tube": tube,
        "pad": (to_m(pad["center"]), [r * IN for r in (pad["outer_ring_radius"], *pad["inner_ring_radii"])]),
        "launch": to_m(c["launch_mark_in"]),
        "boulders": [(to_m(b["center"]), b["diameter"] / 2 * IN) for b in c["boulders_in"]],
        "half": 120 * IN,
    }


def load_route(name):
    r = json.load(open(os.path.join(COURSE, name)))
    return r, np.array([w[:2] for w in r["waypoints_enu"]])


def obstacle_segments(course):
    """2D line segments the side-ToF beams can hit: wall faces, the tube's side walls and the net."""
    segs = []
    for w in course["walls"].values():
        p = w["poly"]
        segs += [(p[i], p[(i + 1) % 4]) for i in range(4)]
    (x0, y0), (x1, y1) = course["tube"]
    segs += [(np.array([x0, y0]), np.array([x0, y1])), (np.array([x1, y0]), np.array([x1, y1]))]
    h = course["half"]
    corners = [np.array(v) for v in ((-h, -h), (h, -h), (h, h), (-h, h))]
    segs += [(corners[i], corners[(i + 1) % 4]) for i in range(4)]
    return segs


def raycast(origin, direction, segs, max_r=4.0):
    best = max_r
    for a, b in segs:
        e = b - a
        den = direction[0] * e[1] - direction[1] * e[0]
        if abs(den) < 1e-12:
            continue
        w = a - origin
        t = (w[0] * e[1] - w[1] * e[0]) / den
        u = (w[0] * direction[1] - w[1] * direction[0]) / den
        if 0 <= u <= 1 and 1e-6 < t < best:
            best = t
    return best


def seg_point_dist(p, a, b):
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0, 1)
    return float(np.linalg.norm(p - (a + t * ab)))


def poly_contains(poly, p):
    s = None
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        c = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        if s is None:
            s = c > 0
        elif (c > 0) != s:
            return False
    return True


def footprint_clearance(center, poly, half_x=0.2375, half_y=0.183):
    """Gap between the drone's nose-north footprint (0.475 m east-west, 0.366 m north-south) and a wall."""
    fp = np.array([center + np.array(v) for v in ((-half_x, -half_y), (half_x, -half_y), (half_x, half_y), (-half_x, half_y))])
    if any(poly_contains(poly, v) for v in fp) or any(poly_contains(fp, v) for v in poly):
        return 0.0
    d = min(seg_point_dist(v, poly[i], poly[(i + 1) % 4]) for v in fp for i in range(4))
    return min(d, min(seg_point_dist(v, fp[i], fp[(i + 1) % 4]) for v in poly for i in range(4)))


# --------------------------------------------------------------------------- drawing helpers

def setup_map_axes(fig, rect, course, pad=0.1):
    ax = fig.add_axes(rect)
    h = course["half"] + pad
    ax.set_xlim(-h, h)
    ax.set_ylim(-h, h)
    ax.set_aspect("equal")
    ax.axis("off")
    return ax


def draw_floor(ax, course):
    h = course["half"]
    ax.add_patch(Rectangle((-h, -h), 2 * h, 2 * h, facecolor="#0e141b", edgecolor="none", zorder=0))
    for v in np.arange(-3, 3.01, 0.5):
        if abs(v) < h:
            ax.plot([v, v], [-h, h], color=GRID, lw=0.6, zorder=0.5)
            ax.plot([-h, h], [v, v], color=GRID, lw=0.6, zorder=0.5)
    ax.add_patch(Rectangle((-h, -h), 2 * h, 2 * h, facecolor="none", edgecolor=NET, lw=1.6, zorder=1))


def draw_static(ax, course, window_hi=False, wall_color=WALL, skip=()):
    draw_floor(ax, course)
    for b, r in course["boulders"]:
        ax.add_patch(Circle(b, r, facecolor="#1a232e", edgecolor="#2c3846", lw=1, zorder=1.5))
    c, rings = course["pad"]
    for i, r in enumerate(rings):
        ax.add_patch(Circle(c, r, facecolor="none", edgecolor=AMBER, lw=1.4 if i == 0 else 1.0, alpha=0.75, zorder=1.6))
    ax.add_patch(Circle(c, rings[-1], facecolor=AMBER, alpha=0.25, edgecolor="none", zorder=1.6))
    lm = course["launch"]
    ax.plot([lm[0] - 0.07, lm[0] + 0.07], [lm[1], lm[1]], color=MUTED, lw=1.2, zorder=1.6)
    ax.plot([lm[0], lm[0]], [lm[1] - 0.07, lm[1] + 0.07], color=MUTED, lw=1.2, zorder=1.6)
    for name, w in course["walls"].items():
        if name in skip:
            continue
        ax.add_patch(Polygon(w["poly"], closed=True, facecolor=wall_color, edgecolor="#f3f6f9", lw=0.6, zorder=3))
    win = course["walls"]["C2a"].get("window")
    if win:
        wc = to_m(win["center_in"])
        a, b = to_m(course["walls"]["C2a"]["start_in"]), to_m(course["walls"]["C2a"]["end_in"])
        d = (b - a) / np.linalg.norm(b - a) * win["width_in"] * IN / 2
        ax.plot([wc[0] - d[0], wc[0] + d[0]], [wc[1] - d[1], wc[1] + d[1]], color=CYAN, lw=4.2 if window_hi else 3.2,
                solid_capstyle="butt", zorder=3.5, alpha=1.0 if window_hi else 0.9)
    (x0, y0), (x1, y1) = course["tube"]
    ax.add_patch(FancyBboxPatch((x0, y0), x1 - x0, y1 - y0, boxstyle="round,pad=0,rounding_size=0.08",
                                facecolor="#04060a", edgecolor="#6b7c90", lw=1.4, alpha=0.92, zorder=4))
    for yy in np.linspace(y0 + 0.12, y1 - 0.12, 6):          # the tube's ribs
        ax.plot([x0 + 0.03, x1 - 0.03], [yy, yy], color="#1f2a36", lw=0.8, zorder=4.1)


def draw_drone(ax, p, yaw, z=6, scale=1.0, color="#ffffff"):
    """skybeta-19 at true size: motors from course_asbuilt drone_m, 6.1 in props, nose = body +x."""
    R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    motors = [(0.08, 0.16), (0.08, -0.16), (-0.15, 0.14), (-0.15, -0.14)]
    arts = []
    for mx, my in motors:
        q = p + R @ (np.array([mx, my]) * scale)
        arts.append(ax.plot([p[0], q[0]], [p[1], q[1]], color=color, lw=2.2, alpha=0.9, zorder=z, solid_capstyle="round")[0])
        arts.append(ax.add_patch(Circle(q, 0.0775 * scale, facecolor=color, alpha=0.18, edgecolor=color, lw=1.0, zorder=z)))
    body = np.array([(0.11, 0.05), (0.11, -0.05), (-0.12, -0.05), (-0.12, 0.05)]) * scale
    arts.append(ax.add_patch(Polygon(p + body @ R.T, closed=True, facecolor="#1b2633", edgecolor=color, lw=1.2, zorder=z + 0.1)))
    nose = np.array([(0.16, 0.0), (0.10, 0.035), (0.10, -0.035)]) * scale
    arts.append(ax.add_patch(Polygon(p + nose @ R.T, closed=True, facecolor=CYAN, edgecolor="none", zorder=z + 0.2)))
    return arts


def chip(fig, x, y, w, h, text, sub, on, accent=GREEN):
    """A checkpoint chip: outlined while pending, filled once passed."""
    fig.patches.append(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=0.008", transform=fig.transFigure,
                                      facecolor=accent if on else PANEL, edgecolor=accent if on else "#273241", lw=1.2,
                                      figure=fig))
    fig.text(x + 0.011, y + h * 0.5, text, ha="left", va="center", fontsize=10, fontweight="bold",
             color=BG if on else TEXT)
    fig.text(x + w - 0.010, y + h * 0.5, sub, ha="right", va="center", fontsize=8.5, color=BG if on else MUTED)


def pill(fig, x, y, text, color, fontsize=10.5):
    t = fig.text(x, y, text, ha="left", va="center", fontsize=fontsize, fontweight="bold", color=color,
                 bbox=dict(boxstyle="round,pad=0.45,rounding_size=0.8", facecolor=PANEL, edgecolor=color, lw=1.2))
    return t


def ease(t):
    t = min(max(t, 0.0), 1.0)
    return t * t * (3 - 2 * t)


# --------------------------------------------------------------------------- GIF writer

def frames_to_gif(frames, path, fps, hold_last=0.0, colors=160):
    """Quantize every frame to one shared palette (no dithering: flat colours and small deltas)."""
    imgs = [Image.fromarray(f[:, :, :3]) for f in frames]
    sample_idx = np.linspace(0, len(imgs) - 1, min(8, len(imgs))).astype(int)
    strip = Image.new("RGB", (imgs[0].width, imgs[0].height * len(sample_idx)))
    for k, i in enumerate(sample_idx):
        strip.paste(imgs[i], (0, k * imgs[0].height))
    pal = strip.quantize(colors=colors, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE)
    q = [im.quantize(palette=pal, dither=Image.Dither.NONE) for im in imgs]
    dur = [int(round(1000 / fps))] * len(q)
    if hold_last:
        dur[-1] = int(hold_last * 1000)
    q[0].save(path, save_all=True, append_images=q[1:], duration=dur, loop=0, optimize=True, disposal=1)
    print(f"{os.path.relpath(path, REPO)}: {len(q)} frames, {os.path.getsize(path) / 1e6:.2f} MB")


def grab(fig):
    fig.canvas.draw()
    return np.asarray(fig.canvas.buffer_rgba()).copy()


# --------------------------------------------------------------------------- 1. hero lap

def load_lap(lap_dir):
    rows = list(csv.DictReader(open(os.path.join(lap_dir, "log.csv"))))
    s = json.load(open(os.path.join(lap_dir, "summary.json")))
    t = np.array([float(r["t"]) for r in rows])
    xyz = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows])
    yaw = np.array([float(r["yaw_enu"]) for r in rows])
    clr = np.array([float(r["clearance"]) if r["clearance"] not in ("", "inf") else np.inf for r in rows])
    state = [r["state"] for r in rows]
    armed = np.array([r["armed"] == "1" for r in rows])
    return dict(t=t, xyz=xyz, yaw=yaw, clr=clr, state=state, armed=armed, summary=s)


def hero(lap_dir, out, fps=15, target_s=11.0, code_label=""):
    course = load_course("course_asbuilt.json")
    _, route = load_route("route_centreline_window_asbuilt.json")
    lap = load_lap(lap_dir)
    s = lap["summary"]
    cp = s["checkpoints_s"]
    t_land = s["touchdown_time_from_arm_s"]
    i0 = int(np.argmax(lap["armed"]))
    t, xyz, yaw, clr = lap["t"], lap["xyz"], lap["yaw"], lap["clr"]
    t_end = t_land + 1.5
    speedup = (t_end - t[i0]) / target_s
    segs = obstacle_segments(course)
    # horizontal speed from the armed part of the log (t is 0 for every row before arming), ~0.5 s smoothing
    spd = np.zeros(len(t))
    d = np.linalg.norm(np.diff(xyz[i0:, :2], axis=0), axis=1)
    dt = np.maximum(np.diff(t[i0:]), 1e-3)
    spd[i0 + 1:] = np.convolve(d / dt, np.ones(5) / 5, mode="same")
    walls_min = s.get("min_clearance_cm")
    walls_at = (s.get("min_clearance_at") or "").replace("_", " ").replace("net north", "north net")

    stages = [("1", "CORRIDOR", 4, cp["corridor_done"]), ("2", "MAZE", 7, cp["gate_a"]),
              ("3", "TUNNEL", 10, cp["tube_out"]), ("4", "WINDOW", 7, cp["window"]), ("5", "LANDING", 7, t_land)]
    tube_in, tube_out = cp["tube_in"], cp["tube_out"]
    marks = {"1": to_m((21, 214)), "2": to_m((130, 226)), "3": to_m((211, 186)), "4": to_m((80, 40)), "5": to_m((76, 88.8))}

    W, H = 8.0, 9.2
    times = np.arange(t[i0], t_end, speedup / fps)
    frames = []
    for k, now in enumerate(times):
        fig = plt.figure(figsize=(W, H), dpi=100)
        ax = setup_map_axes(fig, [0.025, 0.075, 0.95, 0.95 * W / H], course)
        win_hi = abs(now - cp["window"]) < 1.2
        draw_static(ax, course, window_hi=win_hi)
        ax.plot(route[:, 0], route[:, 1], color=MUTED, lw=1.1, ls=(0, (4, 4)), alpha=0.55, zorder=2)
        j = int(np.searchsorted(t, now))
        j = min(max(j, i0 + 1), len(t) - 1)
        P = xyz[i0:j + 1, :2]
        if len(P) > 1:
            ax.plot(P[:, 0], P[:, 1], color=CYAN, lw=7, alpha=0.10, zorder=5, solid_capstyle="round")
            ax.plot(P[:, 0], P[:, 1], color=CYAN, lw=2.2, alpha=0.95, zorder=5.1, solid_capstyle="round")
        p, th = xyz[j, :2], yaw[j]
        (tx0, ty0), (tx1, ty1) = course["tube"]
        # inside the tube's footprint (the log's tube_in/tube_out checkpoints bracket the same stretch)
        in_tube = tx0 <= p[0] <= tx1 and ty0 <= p[1] <= ty1 and tube_in - 1.0 <= now <= tube_out + 1.0
        if not in_tube:
            ax.text((tx0 + tx1) / 2, ty0 + 0.1, "DARK TUBE", ha="center", va="bottom", fontsize=8.5, color=DIM,
                    fontweight="bold", zorder=4.2)
        if in_tube:
            h = course["half"]
            ax.add_patch(Rectangle((-h, -h), 2 * h, 2 * h, facecolor="#000000", alpha=0.5, edgecolor="none", zorder=4.5))
            Rm = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
            for side, sy in (("L", 0.075), ("R", -0.075)):
                o = p + Rm @ np.array([-0.019, sy])
                d = Rm @ np.array([0.0, 1.0 if sy > 0 else -1.0])
                r = raycast(o, d, segs)
                q = o + d * r
                ax.plot([o[0], q[0]], [o[1], q[1]], color=ORANGE, lw=2.4, zorder=7, solid_capstyle="round")
                ax.add_patch(Circle(q, 0.03, facecolor=ORANGE, edgecolor="none", zorder=7))
                mid = (o + q) / 2
                ax.text(mid[0], mid[1] + 0.09, f"{r:.2f} m", ha="center", va="bottom", fontsize=10, color=ORANGE,
                        fontweight="bold", zorder=8, **{"fontfamily": MONO["family"]})
        else:
            fov = 80.0
            ax.add_patch(Wedge(p, 0.95, math.degrees(th) - fov / 2, math.degrees(th) + fov / 2,
                               facecolor=CYAN, alpha=0.10, edgecolor=CYAN, lw=0.8, zorder=5.5))
        draw_drone(ax, p, th, z=6)
        for key, (mx, my) in marks.items():
            st = next(x for x in stages if x[0] == key)
            done = now >= st[3]
            ax.add_patch(Circle((mx, my), 0.12, facecolor=GREEN if done else PANEL, edgecolor=GREEN if done else DIM,
                                lw=1.4, zorder=8))
            ax.text(mx, my, "✓" if done else key, ha="center", va="center", fontsize=10.5, fontweight="bold",
                    color=BG if done else TEXT, zorder=9)

        # header
        fig.text(0.03, 0.962, "NAVLESS", fontsize=24, fontweight="bold", color=TEXT, va="center")
        fig.text(0.255, 0.966, "autonomous lap of the DroneHacks course, no GPS", fontsize=12, color=TEXT, va="center")
        fig.text(0.255, 0.945, "recorded in Gazebo + PX4 SITL on the as-built course, real flight code", fontsize=9.5,
                 color=MUTED, va="center")
        x = 0.03
        cw = (0.94 - 4 * 0.01) / 5
        for key, name, pts, tdone in stages:
            chip(fig, x, 0.893, cw, 0.03, f"{key} {name}", f"{pts} pts", now >= tdone)
            x += cw + 0.01

        # footer: stage + live numbers
        if now >= t_land:
            mode, mc = f"LANDED {s['touchdown_offset_from_pad_cm']:.1f} cm from the pad", GREEN
        elif in_tube:
            mode, mc = "TUNNEL: dark, side ToF centring", ORANGE
        elif now < cp["corridor_done"]:
            mode, mc = "CORRIDOR: stereo SLAM, nose north", CYAN
        elif now < cp["gate_a"]:
            mode, mc = "MAZE: following the as-built route", CYAN
        elif now < tube_in:
            mode, mc = "TO THE TUNNEL", CYAN
        elif now < cp["window"] + 0.5:
            mode, mc = "CHICANE: 36 in window", CYAN
        else:
            mode, mc = "LANDING", AMBER
        pill(fig, 0.035, 0.037, mode, mc)
        if now >= t_land:
            hud = f"lap {t_land:.1f} s  closest {walls_min:.1f} cm ({walls_at})  {len(s.get('contacts', []))} contacts"
            hud_col = GREEN
        else:
            c = f"{clr[j] * 100:4.1f} cm" if np.isfinite(clr[j]) else "  —  "
            hud = f"t {now - t[i0]:5.1f} s  {spd[j]:4.2f} m/s  z {xyz[j, 2]:4.2f} m  wall {c}"
            hud_col = TEXT
        fig.text(0.965, 0.037, hud, ha="right", va="center", fontsize=11, color=hud_col, fontfamily=MONO["family"])
        fig.text(0.965, 0.012, f"{speedup:.0f}x speed  {code_label}", ha="right", va="center", fontsize=8.5,
                 color=DIM)
        frames.append(grab(fig))
        plt.close(fig)
    frames_to_gif(frames, out, fps, hold_last=2.5)


# --------------------------------------------------------------------------- 2. plan vs as-built

def plan_vs_asbuilt(out, fps=12.5):
    plan = load_course("course_v3.json")
    asb = load_course("course_asbuilt.json")
    r_old, old = load_route("route_centreline_window.json")
    r_new, new = load_route("route_centreline_window_asbuilt.json")
    moved = [n for n in asb["walls"] if not np.allclose(asb["walls"][n]["poly"], plan["walls"][n]["poly"])]
    # where the plan's route (its centre line, i.e. the drone's centre) comes closest to a real, as-built wall face
    worst = (np.inf, None, None)
    for i, p in enumerate(old):
        for n in moved:
            poly = asb["walls"][n]["poly"]
            g = 0.0 if poly_contains(poly, p) else min(seg_point_dist(p, poly[q], poly[(q + 1) % 4]) for q in range(4))
            if g < worst[0]:
                worst = (g, i, n)
    gap_cm, wi, wname = worst[0] * 100, worst[1], worst[2]
    fp_cm = footprint_clearance(old[wi], asb["walls"][wname]["poly"]) * 100
    print(f"plan route vs as-built walls: centre line {gap_cm:.1f} cm from {wname} (waypoint {wi}); "
          f"drone footprint gap there {fp_cm:.1f} cm")
    shift = {n: np.linalg.norm(asb["walls"][n]["poly"].mean(0) - plan["walls"][n]["poly"].mean(0)) for n in moved}
    big = max(shift, key=shift.get)

    seq = [("plan", 1.6), ("morph", 1.6), ("clash", 2.4), ("new", 2.2), ("hold", 2.4)]
    frames = []
    W, H = 7.2, 8.0
    for phase, dur in seq:
        n = int(round(dur * fps))
        for k in range(n):
            f = (k + 1) / n
            fig = plt.figure(figsize=(W, H), dpi=100)
            ax = setup_map_axes(fig, [0.03, 0.03, 0.94, 0.94 * W / H], asb)
            m = 0.0 if phase == "plan" else (ease(f) if phase == "morph" else 1.0)
            course = dict(asb)
            course["walls"] = {}
            for name, w in asb["walls"].items():
                pw = plan["walls"][name]["poly"]
                course["walls"][name] = dict(w, poly=pw + (w["poly"] - pw) * m)
            draw_static(ax, course, wall_color=WALL)
            if m > 0:
                for name in moved:
                    ax.add_patch(Polygon(plan["walls"][name]["poly"], closed=True, facecolor="none", edgecolor=RED,
                                         lw=1.2, ls=(0, (3, 2)), alpha=0.85 * min(1, m * 1.5), zorder=2.5))
                    ax.add_patch(Polygon(course["walls"][name]["poly"], closed=True, facecolor=AMBER, edgecolor=AMBER,
                                         lw=0.8, alpha=0.9, zorder=3.2))
            old_col = GREEN if phase in ("plan", "morph") else RED
            old_alpha = 0.9 if phase in ("plan", "morph", "clash") else 0.35
            ax.plot(old[:, 0], old[:, 1], color=old_col, lw=2.2, alpha=old_alpha, zorder=5, solid_capstyle="round")
            if phase in ("clash", "new", "hold"):
                p = old[wi]
                pulse = 0.16 + 0.05 * math.sin(k / n * 2 * math.pi * 2) if phase == "clash" else 0.16
                ax.add_patch(Circle(p, pulse, facecolor="none", edgecolor=RED, lw=2.2, zorder=9))
                ax.text(p[0] + 0.24, p[1] + 0.02, f"{gap_cm:.1f} cm from {wname}", ha="left", va="center",
                        fontsize=11, color=RED, fontweight="bold", zorder=9,
                        bbox=dict(boxstyle="round,pad=0.3", facecolor=BG, edgecolor=RED, lw=1.0, alpha=0.92))
            if phase in ("new", "hold"):
                upto = len(new) if phase == "hold" else max(2, int(len(new) * ease(f)))
                ax.plot(new[:upto, 0], new[:upto, 1], color=CYAN, lw=7, alpha=0.12, zorder=6)
                ax.plot(new[:upto, 0], new[:upto, 1], color=CYAN, lw=2.4, zorder=6.1, solid_capstyle="round")
            if phase != "plan":
                a = plan["walls"][big]["poly"].mean(0)
                b = course["walls"][big]["poly"].mean(0)
                if np.linalg.norm(b - a) > 0.08:
                    ax.annotate("", xy=b, xytext=a, arrowprops=dict(arrowstyle="-|>", color=AMBER, lw=1.8), zorder=9)
                ax.text(b[0] + 0.14, b[1] + 0.55, f"{big}: {shift[big] * 100:.0f} cm east", color=AMBER, fontsize=10.5,
                        fontweight="bold", zorder=9, ha="left")
            # header
            if phase == "plan":
                title, sub, col = "The official map", "our first route, planned on it", GREEN
            elif phase == "morph":
                title, sub, col = "Phone LiDAR scan of the real course", f"{len(moved)} walls are somewhere else", AMBER
            elif phase == "clash":
                title, sub, col = "That route hits the real course", \
                    f"its centre line passes {gap_cm:.1f} cm from {wname}; the drone is 47.5 cm wide", RED
            else:
                title, sub, col = "Rebuilt route on the as-built course", \
                    f"closest wall {r_new['min_clearance_m_nose_north'] * 100:.1f} cm (planned)", CYAN
            fig.text(0.04, 0.955, title, fontsize=17, fontweight="bold", color=col, va="center")
            fig.text(0.04, 0.918, sub, fontsize=11, color=MUTED, va="center")
            frames.append(grab(fig))
            plt.close(fig)
    frames_to_gif(frames, out, fps, hold_last=1.5)


# --------------------------------------------------------------------------- 3. tube ToF centring

def tube_tof(out, fps=15):
    tube_w = 58 * IN                       # interior width at flight height (the oval's widest point)
    tube_h = 72 * IN
    zc = 37 * IN                           # oval centre height = cruise height in the tube (0.94 m)
    sensor_y = 0.075
    width_ok = (1.45 - 0.15, 1.45 + 0.15)
    offs = []
    # 12 cm left of centre, held, pulled back to within 2 cm, held
    for k in range(int(1.0 * fps)):
        offs.append(0.12)
    n = int(2.6 * fps)
    for k in range(n):
        offs.append(0.018 + 0.102 * math.exp(-4.5 * (k + 1) / n))
    for k in range(int(1.8 * fps)):
        offs.append(offs[-1])
    frames = []
    W, H = 8.0, 4.6
    for k, off in enumerate(offs):
        fig = plt.figure(figsize=(W, H), dpi=100)
        ax = fig.add_axes([0.02, 0.06, 0.52, 0.8])
        ax.set_xlim(-0.95, 0.95)
        ax.set_ylim(0, 2.0)
        ax.set_aspect("equal")
        ax.axis("off")
        ax.add_patch(Ellipse((0, zc), tube_w + 0.12, tube_h + 0.12, facecolor="#1a2430", edgecolor="#6b7c90", lw=1.5))
        ax.add_patch(Ellipse((0, zc), tube_w, tube_h, facecolor="#04060a", edgecolor="#2c3846", lw=1.0))
        ax.plot([-0.9, 0.9], [0.0, 0.0], color=DIM, lw=1.2)
        ax.plot([0, 0], [zc - 0.75, zc + 0.75], color=DIM, lw=1.0, ls=(0, (3, 3)))
        ax.text(0, zc + 0.8, "tube centre", ha="center", fontsize=9, color=MUTED)
        x = -off                            # looking along the flight direction: left is -x
        L = (tube_w / 2 + x) - sensor_y     # left sensor to left wall
        R = (tube_w / 2 - x) - sensor_y
        # drone seen from behind: arms + props + body
        ax.plot([x - 0.2375 + 0.0775, x + 0.2375 - 0.0775], [zc, zc], color="#ffffff", lw=2.5)
        for sx in (-1, 1):
            ax.add_patch(Ellipse((x + sx * (0.2375 - 0.0775), zc + 0.03), 0.155, 0.025, facecolor="#ffffff",
                                 alpha=0.35, edgecolor="#ffffff", lw=0.8))
        ax.add_patch(FancyBboxPatch((x - 0.07, zc - 0.06), 0.14, 0.07, boxstyle="round,pad=0,rounding_size=0.015",
                                    facecolor="#1b2633", edgecolor="#ffffff", lw=1.0))
        for sx, dist in ((-1, L), (1, R)):
            o = np.array([x + sx * sensor_y, zc - 0.06])
            q = o + np.array([sx * dist, 0])
            ax.plot([o[0], q[0]], [o[1], q[1]], color=ORANGE, lw=2.6, solid_capstyle="round")
            ax.add_patch(Circle(q, 0.022, facecolor=ORANGE, edgecolor="none"))
            ax.add_patch(Circle(o, 0.018, facecolor=ORANGE, edgecolor="none"))
            ax.text((o[0] + q[0]) / 2, o[1] - 0.13, f"{'L' if sx < 0 else 'R'} {dist:.2f} m", ha="center", fontsize=11,
                    color=ORANGE, fontweight="bold", fontfamily=MONO["family"])
        if off > 0.03:
            ax.annotate("", xy=(x + 0.12, zc + 0.22), xytext=(x, zc + 0.22),
                        arrowprops=dict(arrowstyle="-|>", color=GREEN, lw=2.0))
        # right panel
        fig.text(0.56, 0.90, "Dark tube: two side ToF sensors", fontsize=15, fontweight="bold", color=TEXT, va="center")
        fig.text(0.56, 0.835, "VL53L1X: 940 nm infrared, they bring their own light", fontsize=10, color=MUTED,
                 va="center")
        est = (L - R) / 2
        fig.text(0.56, 0.72, "offset = (L − R) / 2", fontsize=13, color=TEXT, va="center", fontfamily=MONO["family"])
        side = "centred" if abs(est) < 0.005 else f"{abs(est) * 100:4.1f} cm {'left' if est < 0 else 'right'}"
        fig.text(0.56, 0.645, f"       = {side}", fontsize=13, color=GREEN if abs(est) < 0.03 else ORANGE,
                 va="center", fontfamily=MONO["family"], fontweight="bold")
        s = L + R + 2 * sensor_y
        ok = width_ok[0] <= s <= width_ok[1]
        rules = [
            ("used only inside the tube", True),
            (f"L + R must add up to the tube: {s:.2f} m", ok),
            ("readings older than 0.3 s are dropped", True),
            ("moves the drone at most 15 cm off the route", True),
            ("anything else: fly the planned route", True),
        ]
        yy = 0.54
        for text, good in rules:
            fig.text(0.56, yy, "✓" if good else "✗", fontsize=11, color=GREEN if good else RED, va="center",
                     fontweight="bold")
            fig.text(0.585, yy, text, fontsize=10, color=TEXT, va="center")
            yy -= 0.075
        fig.text(0.56, 0.12, "Schematic at real scale: 58 in tube, 47.5 cm drone.", fontsize=9, color=DIM, va="center")
        fig.text(0.56, 0.065, "Closed-loop sim: 12 cm off centre → within 2 cm.", fontsize=9, color=DIM,
                 va="center")
        frames.append(grab(fig))
        plt.close(fig)
    frames_to_gif(frames, out, fps, hold_last=1.0, colors=128)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("which", nargs="*", default=["hero", "asbuilt", "tube"])
    ap.add_argument("--lap", default=os.path.join(HERE, "data", "gate_lap"), help="gate lap dir (log.csv, summary.json)")
    ap.add_argument("--label", default="", help="code version to print under the hero lap")
    ap.add_argument("--out", default=HERE)
    a = ap.parse_args()
    if "hero" in a.which:
        hero(a.lap, os.path.join(a.out, "hero_lap.gif"), code_label=a.label)
    if "asbuilt" in a.which:
        plan_vs_asbuilt(os.path.join(a.out, "plan_vs_asbuilt.gif"))
    if "tube" in a.which:
        tube_tof(os.path.join(a.out, "tube_tof.gif"))


if __name__ == "__main__":
    sys.exit(main())
