#!/usr/bin/env python3
"""Record one lap of cloud_localize's simulated flight as an animated GIF (top view).

  python3 sim_gif.py dronehacks-course-asbuilt.obj route_centreline_window_asbuilt.json out.gif
  options: --every 2 (draw every 2nd frame)  --ms 100 (per GIF frame)  --px 640  --speed 0.3  --seed 11
           --yaw travel|fixed

Same simulation as `cloud_localize.py --simflight` (generated camera points and odometry, the real
localizer code), run as fast as the CPU allows instead of in real time. Each GIF frame shows:
  grey     the course model from above (walls darker)
  pink     the route
  white    the TRUE pose (dot + heading line) and its trail
  green / blue   the localizer's estimate (green: matched to the course, blue: odometry only) + trail
  cloud    the simulated camera's points placed at the ESTIMATE: green = on the model, orange = not
Needs numpy, scipy and Pillow (the stack container has them).
"""
import argparse
import math

import numpy as np
from PIL import Image, ImageDraw, ImageFont

import cloud_localize as cl

BG, GRID = (18, 19, 24), (38, 40, 48)
COL = {"model": (90, 94, 108), "wall": (150, 155, 170), "route": (214, 90, 150), "truth": (245, 245, 245),
       "est": (64, 210, 110), "odom": (80, 150, 255), "other": (150, 150, 150),
       "hit": (64, 210, 110), "miss": (240, 140, 60), "text": (230, 232, 238), "dim": (150, 154, 166)}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("obj")
    ap.add_argument("route")
    ap.add_argument("out")
    ap.add_argument("--every", type=int, default=2)
    ap.add_argument("--ms", type=int, default=100)
    ap.add_argument("--px", type=int, default=640)
    ap.add_argument("--speed", type=float, default=0.3)
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--yaw", default="travel")
    ap.add_argument("--title", default="as-built course, simulated camera + odometry")
    a = ap.parse_args()

    model = cl.CourseModel(a.obj)
    half, top = 3.25, 54                                   # metres shown either side of centre; HUD strip
    S = a.px / (2 * half)
    X = lambda x: (x + half) * S
    Y = lambda y: top + (half - y) * S
    W, H = a.px, a.px + top
    font = ImageFont.load_default(size=15)
    small = ImageFont.load_default(size=12)

    # static background: grid, model dots (walls brighter), route
    base = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(base)
    for k in range(-3, 4):
        d.line([(X(k), Y(-half)), (X(k), Y(half))], fill=GRID)
        d.line([(X(-half), Y(k)), (X(half), Y(k))], fill=GRID)
    vp = model.view_pts
    walls = vp[:, 2] > 0.25
    for p, c in ((vp[~walls], COL["model"]), (vp[walls], COL["wall"])):
        for x, y in p[:: max(1, len(p) // 12000), :2]:
            d.point((X(x), Y(y)), fill=c)
    import json
    R = np.asarray(json.load(open(a.route))["waypoints_enu"], float)
    d.line([(X(x), Y(y)) for x, y in R[:, :2]], fill=COL["route"], width=2)

    frames, ttrail, etrail = [], [], []
    rate = 3.0

    def fwd(T):
        f = (T[:3, :3] @ cl.R_OPT_TO_FLU.T)[:, 0]
        return f[:2] / max(np.linalg.norm(f[:2]), 1e-9)

    def on_frame(i, T_true, T_est, status, cloud, err, herr):
        ttrail.append(T_true[:3, 3][:2].copy())
        etrail.append((T_est[:3, 3][:2].copy(), status))
        if i % a.every:
            return
        im = base.copy()
        g = ImageDraw.Draw(im)
        # cloud placed at the estimate, coloured by whether it lands on the model
        pw = cl.transform(T_est, cloud)
        _, dist = model.nn.query(pw, 0.06)
        hit = dist < 0.06
        for p, c in ((pw[~hit], COL["miss"]), (pw[hit], COL["hit"])):
            for x, y in p[:: max(1, len(p) // 900), :2]:
                g.rectangle([X(x) - 1, Y(y) - 1, X(x) + 1, Y(y) + 1], fill=c)
        if len(ttrail) > 1:
            g.line([(X(x), Y(y)) for x, y in ttrail], fill=COL["truth"], width=1)
        for (p0, s0), (p1, _) in zip(etrail[:-1], etrail[1:]):
            c = COL["odom"] if s0 == "odometry" else COL["est"]
            g.line([(X(p0[0]), Y(p0[1])), (X(p1[0]), Y(p1[1]))], fill=c, width=2)
        # estimate: arrow; truth: white dot + heading
        c = COL["odom"] if status == "odometry" else COL["est"] if status in ("tracking", "weak") else COL["other"]
        e, fe = T_est[:3, 3][:2], fwd(T_est)
        side = np.array([-fe[1], fe[0]])
        tip, l, r = e + fe * 0.22, e - fe * 0.08 + side * 0.10, e - fe * 0.08 - side * 0.10
        g.polygon([(X(p[0]), Y(p[1])) for p in (tip, l, r)], fill=c)
        t, ft = T_true[:3, 3][:2], fwd(T_true)
        g.ellipse([X(t[0]) - 4, Y(t[1]) - 4, X(t[0]) + 4, Y(t[1]) + 4], fill=COL["truth"])
        g.line([(X(t[0]), Y(t[1])), (X(t[0] + ft[0] * 0.3), Y(t[1] + ft[1] * 0.3))], fill=COL["truth"], width=2)
        # HUD
        g.rectangle([0, 0, W, top], fill=(10, 11, 14))
        g.text((10, 6), f"t {i / rate:5.1f} s   error {err * 100:4.1f} cm   heading {herr:3.1f}°   {status}",
               font=font, fill=COL["text"])
        g.text((10, 30), a.title + "   white = true pose, green = estimate (blue = odometry only)",
               font=small, fill=COL["dim"])
        frames.append(im.quantize(colors=64, method=Image.Quantize.MEDIANCUT))

    perr, herr = cl.simflight(a.obj, a.route, speed=a.speed, rate=rate, yawmode=a.yaw, seed=a.seed,
                              every=10 ** 9, on_frame=on_frame)
    frames[0].save(a.out, save_all=True, append_images=frames[1:], duration=a.ms, loop=0, optimize=True)
    print(f"{len(frames)} frames -> {a.out}; position error median {np.median(perr) * 100:.0f} cm, "
          f"max {np.max(perr) * 100:.0f} cm; heading median {np.median(herr):.1f} deg")


if __name__ == "__main__":
    main()
