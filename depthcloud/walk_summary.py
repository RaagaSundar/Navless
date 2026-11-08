#!/usr/bin/env python3
"""Summarise one walk session: how well did the localizer know where the drone was?

    python3 walk_summary.py <session dir> [course_asbuilt.json]

Reads <session>/track.csv (walk_recorder.py) and, if present, <session>/checkpoints/results.json
(walk_test.py: Enter pressed at known spots). Writes:
  summary.txt  status shares, gaps, jumps, odometry stalls, how far plain odometry drifted from the
               localizer, what the drift correction did, checkpoint errors
  plan.svg     top view of the course (walls, tube, boulders, pad, launch mark) with
               the localizer's trail coloured by status, plain odometry from the launch mark (grey dashes:
               what navless_nav would believe without correction), odometry + odom_correction (purple),
               checkpoints (truth = black ring, estimate = red dot)
Pure Python (no numpy), so it runs on a laptop as well as in the container.
"""
import csv
import json
import math
import os
import sys
from statistics import median

IN = 0.0254
HALF = 3.048                       # the net is 240 in = 6.096 m square, origin at its centre
LAUNCH_YAW_DEG = 90.0              # start: on the launch mark facing north (what cloud_localize assumes)
STATUS_COLOURS = {"tracking": "#1a9850", "weak": "#f39c12", "odometry": "#2c7bb6"}
JUMP_M, JUMP_DT = 0.15, 0.25       # estimate moving >15 cm in <0.25 s while walking slowly = a jump
STALL_S = 0.4                      # cuVSLAM silent this long = PX4 would drop vision


def m(p):
    return ((p[0] - 120) * IN, (p[1] - 120) * IN)


def num(v):
    try:
        f = float(v)
        return f if math.isfinite(f) else None
    except (TypeError, ValueError):
        return None


def compose(a, b):                  # SE(2) poses (x, y, yaw rad): a * b
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def load_rows(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    out = []
    for r in rows:
        d = {k: num(v) for k, v in r.items() if k not in ("loc_status", "odom_source")}
        d["loc_status"], d["odom_source"] = r.get("loc_status", ""), r.get("odom_source", "")
        if d.get("t") is not None:
            out.append(d)
    return out


def find_course(arg, session):
    here = os.path.dirname(os.path.abspath(__file__))
    for p in ([arg] if arg else []) + [os.path.join(here, "course_asbuilt.json"),
                                       os.path.join(here, "..", "gazebo_sim", "course", "course_asbuilt.json")]:
        if p and os.path.isfile(p):
            return p
    return None


def fresh(r, name, max_age=1.0):
    a = r.get(f"{name}_age")
    return a is not None and a <= max_age and r.get(f"{name}_x") is not None


def analyse(rows, launch, sess=None):
    t0 = rows[0]["t"]
    dur = rows[-1]["t"] - t0
    lines = [f"walk: {dur:.0f} s, {len(rows)} rows"]
    # --- localizer status ---------------------------------------------------------------------
    counts = {}
    for r in rows:
        counts[r["loc_status"] or "none"] = counts.get(r["loc_status"] or "none", 0) + 1
    lines.append("localizer status: " + ", ".join(f"{k} {100 * v / len(rows):.0f}%"
                                                 for k, v in sorted(counts.items(), key=lambda kv: -kv[1])))
    gap, worst, start = 0.0, (0.0, None), None
    for r in rows:
        if r["loc_status"] == "tracking":
            start = None
        else:
            start = r["t"] if start is None else start
            if r["t"] - start > worst[0]:
                worst = (r["t"] - start, start - t0)
    if worst[1] is not None:
        lines.append(f"longest stretch without 'tracking': {worst[0]:.1f} s (from t={worst[1]:.0f} s)")
    ages = [r["loc_age"] for r in rows if r.get("loc_age") is not None]
    if ages:
        lines.append(f"estimate age: median {median(ages):.2f} s, worst {max(ages):.1f} s")
    # --- jumps in the estimate ----------------------------------------------------------------
    jumps, prev = [], None
    for r in rows:
        if not fresh(r, "loc"):
            continue
        p = (r["loc_x"], r["loc_y"], r["t"])
        if prev and p[:2] != prev[:2] and p[2] - prev[2] < JUMP_DT:
            d = math.hypot(p[0] - prev[0], p[1] - prev[1])
            if d > JUMP_M:
                jumps.append((p[2] - t0, d, r["loc_status"]))
        prev = p
    lines.append(f"estimate jumps (>{JUMP_M * 100:.0f} cm in <{JUMP_DT} s): {len(jumps)}"
                 + ("" if not jumps else "; biggest " + ", ".join(
                     f"{d * 100:.0f} cm at t={t:.0f} s ({st})" for t, d, st in sorted(jumps, key=lambda j: -j[1])[:5])))
    # --- cuVSLAM / stereo / PX4 health ----------------------------------------------------------
    stalls, last_ok = [], None
    for r in rows:
        a = r.get("odom_age")
        if a is not None and a > STALL_S:
            if last_ok is None or r["t"] - last_ok > 1.0:
                stalls.append(r["t"] - t0)
            last_ok = r["t"]
    lines.append(f"cuVSLAM stalls >{STALL_S} s: {len(stalls)}" +
                 (f" (at t= {', '.join(f'{s:.0f}' for s in stalls[:8])} s)" if stalls else ""))
    vo = {}
    for r in rows:
        if r.get("vo_state") is not None:
            vo[int(r["vo_state"])] = vo.get(int(r["vo_state"]), 0) + 1
    if vo:
        lines.append("cuVSLAM vo_state: " + ", ".join(f"{k}: {100 * v / len(rows):.0f}%" for k, v in sorted(vo.items()))
                     + "  (1 = tracking)")
    cps = [r["clouds_per_s"] for r in rows if r.get("clouds_per_s") is not None]
    if cps:
        lines.append(f"stereo clouds/s: median {median(cps):.1f}, min {min(cps):.0f}")
    px4 = sum(1 for r in rows if fresh(r, "px4"))
    lines.append(f"PX4 estimate present in {100 * px4 / len(rows):.0f}% of rows"
                 + ("" if px4 else " (PX4 not linked, or not on this domain)"))

    def resets(col):                 # how many times a PX4 reset counter went up during the walk
        vals = [int(r[col]) for r in rows if r.get(col) is not None]
        return sum((b - a) % 256 for a, b in zip(vals, vals[1:]) if b != a)
    if px4:
        lines.append(f"PX4 EKF2 resets during the walk: heading {resets('px4_heading_resets')}, "
                     f"xy {resets('px4_xy_resets')}, z {resets('px4_z_resets')}  (a heading reset = vision lost "
                     f">0.4 s, EKF2 fell back to the compass)")
        dr = sum(1 for r in rows if r.get("px4_dead_reckoning") == 1)
        eph = [r["px4_eph"] for r in rows if r.get("px4_eph") is not None]
        lines.append(f"PX4 dead reckoning {100 * dr / len(rows):.0f}% of rows"
                     + (f"; eph median {median(eph) * 100:.0f} cm, worst {max(eph) * 100:.0f} cm" if eph else ""))
    vin = [r["vision_in_age"] for r in rows if r.get("vision_in_age") is not None]
    if vin:
        gaps = sum(1 for a, b in zip(vin, vin[1:]) if b > STALL_S >= a)
        lines.append(f"vision feed into PX4: {gaps} gaps >{STALL_S} s, worst {max(vin):.2f} s")
    imu = [r["imu_age"] for r in rows if r.get("imu_age") is not None]
    if imu:
        lines.append(f"camera IMU: worst gap {max(imu):.2f} s")
    raw = os.path.join(sess or "", "raw")
    if os.path.isdir(raw):
        lines.append("raw full-rate logs: " + ", ".join(
            f"{f[:-4]} {max(0, sum(1 for _ in open(os.path.join(raw, f))) - 1)} rows"
            for f in sorted(os.listdir(raw)) if f.endswith(".csv")))
    # --- trails: localizer, plain odometry from the launch mark, odometry + correction ----------
    T_nom = (launch[0], launch[1], math.radians(LAUNCH_YAW_DEG))
    trail_loc, trail_nom, trail_cor = [], [], []
    d_nom, d_cor, corr_valid, corr_max = [], [], 0, (0.0, 0.0)
    for r in rows:
        if fresh(r, "loc"):
            trail_loc.append((r["loc_x"], r["loc_y"], r["loc_status"]))
        if not fresh(r, "odom"):
            continue
        o = (r["odom_x"], r["odom_y"], math.radians(r["odom_yaw_deg"] or 0.0))
        nom = compose(T_nom, o)
        trail_nom.append(nom[:2])
        cor = None
        if r.get("corr_valid") == 1 and r.get("corr_dx") is not None:
            D = (r["corr_dx"], r["corr_dy"], math.radians(r["corr_dyaw_deg"] or 0.0))
            cor = compose(T_nom, compose(inverse(D), o))
            trail_cor.append(cor[:2])
            corr_valid += 1
            corr_max = (max(corr_max[0], math.hypot(D[0], D[1])), max(corr_max[1], abs(math.degrees(D[2]))))
        if r["loc_status"] == "tracking" and fresh(r, "loc", 0.5):
            d_nom.append(math.hypot(nom[0] - r["loc_x"], nom[1] - r["loc_y"]))
            if cor:
                d_cor.append(math.hypot(cor[0] - r["loc_x"], cor[1] - r["loc_y"]))
    if d_nom:
        lines.append(f"plain odometry vs localizer (while tracking): median {median(d_nom) * 100:.0f} cm, "
                     f"worst {max(d_nom) * 100:.0f} cm, at the end {d_nom[-1] * 100:.0f} cm")
    if d_cor:
        lines.append(f"odometry + odom_correction vs localizer: median {median(d_cor) * 100:.0f} cm, "
                     f"worst {max(d_cor) * 100:.0f} cm")
    lines.append(f"odom_correction valid in {100 * corr_valid / len(rows):.0f}% of rows; largest "
                 f"{corr_max[0] * 100:.0f} cm / {corr_max[1]:.1f} deg")
    lines.append("Caveat: 'vs localizer' compares two estimates, not the truth. Checkpoints (below) are the truth.")
    return lines, trail_loc, trail_nom, trail_cor


def checkpoint_lines(res):
    lines, pts = [], []
    for c in res.get("checkpoints", []):
        e, o = c.get("error_m"), c.get("odom_only_error_m")
        lines.append(f"  {c['name']:<18} localizer {'-' if e is None else f'{e * 100:.0f} cm':>7}"
                     f"   odom only {'-' if o is None else f'{o * 100:.0f} cm':>7}   {', '.join(c.get('statuses') or [])}")
        pts.append((c["truth"], c.get("estimate"), c["name"]))
    errs = [c["error_m"] for c in res.get("checkpoints", []) if c.get("error_m") is not None and c.get("n", 1) > 0]
    if errs:
        lines.append(f"  localizer error: median {median(errs) * 100:.1f} cm, worst {max(errs) * 100:.1f} cm "
                     f"over {len(errs)} checkpoints (hand placement is good to about +-3 cm)")
    return lines, pts


def svg(course, trail_loc, trail_nom, trail_cor, cps, title):
    S, PAD = 120.0, 30
    W = int(2 * HALF * S + 2 * PAD)
    H = W + 70

    def X(x):
        return PAD + (x + HALF) * S

    def Y(y):
        return PAD + (HALF - y) * S

    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" '
         f'font-family="sans-serif" font-size="12">',
         f'<rect width="{W}" height="{H}" fill="white"/>',
         f'<rect x="{X(-HALF)}" y="{Y(HALF)}" width="{2 * HALF * S}" height="{2 * HALF * S}" fill="#f7f7f7" stroke="#999"/>']
    if course:
        tb = course.get("tunnel", {}).get("footprint_in")
        if tb:
            (x0, y0), (x1, y1) = m((tb["x_min"], tb["y_min"])), m((tb["x_max"], tb["y_max"]))
            o.append(f'<rect x="{X(x0):.1f}" y="{Y(y1):.1f}" width="{(x1 - x0) * S:.1f}" height="{(y1 - y0) * S:.1f}" '
                     f'fill="#e3e3e3" stroke="#aaa"/><text x="{X(x0) + 4:.1f}" y="{Y(y1) + 14:.1f}" fill="#777">tube</text>')
        for b in course.get("boulders_in", []):
            (cx, cy) = m(b["center"])
            o.append(f'<circle cx="{X(cx):.1f}" cy="{Y(cy):.1f}" r="{b["diameter"] / 2 * IN * S:.1f}" fill="#ddd" stroke="#aaa"/>')
        pad = course.get("landing_pad_in")
        if pad:
            (cx, cy) = m(pad["center"])
            o.append(f'<circle cx="{X(cx):.1f}" cy="{Y(cy):.1f}" r="{pad["outer_ring_radius"] * IN * S:.1f}" '
                     f'fill="none" stroke="#bbb" stroke-dasharray="4 3"/>')
        tape = course.get("floor_tape_in")
        if tape:
            pts = " ".join(f"{X(m(p)[0]):.1f},{Y(m(p)[1]):.1f}" for p in tape)
            o.append(f'<polyline points="{pts}" fill="none" stroke="#c9b458" stroke-width="2" stroke-dasharray="2 5"/>')
        for w in course.get("walls", []):
            (x0, y0), (x1, y1) = m(w["start_in"]), m(w["end_in"])
            o.append(f'<line x1="{X(x0):.1f}" y1="{Y(y0):.1f}" x2="{X(x1):.1f}" y2="{Y(y1):.1f}" stroke="#333" '
                     f'stroke-width="{max(2.0, w.get("thickness_in", 4) * IN * S):.1f}" stroke-linecap="square"/>')
        lm = course.get("launch_mark_in")
        if lm:
            (lx, ly) = m(lm)
            o.append(f'<path d="M{X(lx) - 7:.1f},{Y(ly):.1f}h14M{X(lx):.1f},{Y(ly) - 7:.1f}v14" stroke="#000" stroke-width="2"/>')
    if trail_nom:
        o.append('<polyline points="' + " ".join(f"{X(x):.1f},{Y(y):.1f}" for x, y in trail_nom[::2]) +
                 '" fill="none" stroke="#888" stroke-width="1.5" stroke-dasharray="5 4"/>')
    if trail_cor:
        o.append('<polyline points="' + " ".join(f"{X(x):.1f},{Y(y):.1f}" for x, y in trail_cor[::2]) +
                 '" fill="none" stroke="#8e44ad" stroke-width="1.5" opacity="0.8"/>')
    last = None
    for x, y, st in trail_loc:
        if last and abs(x - last[0]) < 0.01 and abs(y - last[1]) < 0.01:
            continue
        o.append(f'<circle cx="{X(x):.1f}" cy="{Y(y):.1f}" r="2.2" fill="{STATUS_COLOURS.get(st, "#d7191c")}"/>')
        last = (x, y)
    for truth, est, name in cps:
        o.append(f'<circle cx="{X(truth[0]):.1f}" cy="{Y(truth[1]):.1f}" r="7" fill="none" stroke="#000" stroke-width="2"/>')
        if est:
            o.append(f'<line x1="{X(truth[0]):.1f}" y1="{Y(truth[1]):.1f}" x2="{X(est[0]):.1f}" y2="{Y(est[1]):.1f}" stroke="#d7191c"/>'
                     f'<circle cx="{X(est[0]):.1f}" cy="{Y(est[1]):.1f}" r="3.5" fill="#d7191c"/>')
        o.append(f'<text x="{X(truth[0]) + 9:.1f}" y="{Y(truth[1]) - 6:.1f}">{name}</text>')
    ly = W + 8
    items = [("dot", c, s) for s, c in STATUS_COLOURS.items()] + [("dot", "#d7191c", "lost / other"),
             ("dash", "#888", "plain odometry"), ("line", "#8e44ad", "odometry + correction"),
             ("dash", "#c9b458", "old floor tape (do not follow)")]
    x = PAD
    for kind, c, label in items:
        if kind == "dot":
            o.append(f'<circle cx="{x + 5}" cy="{ly + 6}" r="4" fill="{c}"/>')
        else:
            o.append(f'<line x1="{x}" y1="{ly + 6}" x2="{x + 18}" y2="{ly + 6}" stroke="{c}" stroke-width="2"'
                     + (' stroke-dasharray="4 3"/>' if kind == "dash" else "/>"))
        o.append(f'<text x="{x + 22}" y="{ly + 10}">{label}</text>')
        x += 30 + 7 * len(label)
        if x > W - 160:
            x, ly = PAD, ly + 20
    o.append(f'<text x="{PAD}" y="{H - 8}" fill="#555">{title} (north up; the net is 6.1 m square)</text></svg>')
    return "\n".join(o)


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    sess = sys.argv[1]
    track = os.path.join(sess, "track.csv")
    if not os.path.isfile(track):
        sys.exit(f"no {track}")
    rows = load_rows(track)
    if len(rows) < 2:
        sys.exit("track.csv has no data (did the recorder run?)")
    cpath = find_course(sys.argv[2] if len(sys.argv) > 2 else None, sess)
    course = json.load(open(cpath)) if cpath else None
    launch = m(course["launch_mark_in"]) if course else (-2.5146, 0.9144)
    lines, tl, tn, tc = analyse(rows, launch, sess)
    cps = []
    rpath = os.path.join(sess, "checkpoints", "results.json")
    if os.path.isfile(rpath):
        cl, cps = checkpoint_lines(json.load(open(rpath)))
        lines += ["checkpoints (truth = where the drone was placed):"] + cl
    else:
        lines.append("no checkpoints recorded (run `walk_session.sh checkpoints` during the walk for ground truth)")
    text = "\n".join(lines)
    with open(os.path.join(sess, "summary.txt"), "w") as f:
        f.write(text + "\n")
    with open(os.path.join(sess, "plan.svg"), "w") as f:
        f.write(svg(course, tl, tn, tc, cps, os.path.basename(os.path.normpath(sess))))
    print(text)
    print(f"wrote {os.path.join(sess, 'summary.txt')} and plan.svg")


if __name__ == "__main__":
    main()
