#!/usr/bin/env python3
"""Add the net frame's poles to the course .obj, so cloud_localize can match against them.

The organisers' dronehacks-course-v3.obj models each net side as a flat see-through sheet, but the
real net hangs on a frame of solid poles (about 1.5 in across). Stereo sees solid poles well, and a
vertical pole at a known spot pins down position in both horizontal directions. Measured by Shining
on the real course (2026-11-07):

  each 20 ft side: 5 vertical poles, evenly spaced (0, 5, 10, 15, 20 ft), floor to the 10 ft top;
                   1 horizontal pole along the side at mid-height (5 ft).
  Corner poles are shared by two sides, so 16 vertical poles in all, plus 4 horizontal ones.

The original file is copied unchanged and the poles are appended as new groups named "Frame pole ..."
and "Frame rail ...". The names deliberately don't contain "net": cloud_localize treats "net" groups
as see-through, and these poles are solid. Poles are open tubes (no end caps), so nothing new is
horizontal at the model's top, where cloud_localize looks for the ceiling.

  python3 add_net_poles.py dronehacks-course-v3.obj dronehacks-course-v3-poles.obj
  options: --diameter-in 1.5  --pole-spacing-ft 5  --rail-height-ft 5  --inset-in 0 (+ = inside the net)
           --top-rails (also a horizontal pole along each top edge; not in the measurement above)

Coordinates as in the .obj: metres, x east, y north, z up, origin at the floor centre. The net
sheets sit at x, y = +-3.048 m and are 3.048 m tall; the script reads both from the file.
"""
import argparse
import math
import sys

FT, IN = 0.3048, 0.0254


def read_net(path):
    """Extent of the 'Net side' groups: (half width in x, half width in y, height) and the vertex count."""
    V, group, nv = [], "", 0
    xs, ys, zs = [], [], []
    with open(path) as fh:
        for line in fh:
            t = line.split()
            if not t:
                continue
            if t[0] == "v":
                V.append((float(t[1]), float(t[2]), float(t[3])))
            elif t[0] == "g":
                group = " ".join(t[1:])
            elif t[0] == "f" and group.lower().startswith("net side"):
                for a in t[1:]:
                    i = int(a.split("/")[0])
                    x, y, z = V[i - 1 if i > 0 else len(V) + i]
                    xs.append(x), ys.append(y), zs.append(z)
    if not xs:
        raise SystemExit(f"{path}: no 'Net side' groups found")
    return max(abs(min(xs)), abs(max(xs))), max(abs(min(ys)), abs(max(ys))), max(zs), len(V)


class Mesh:
    def __init__(self, first_index):
        self.next = first_index          # 1-based index the next appended vertex gets
        self.lines = []

    def tube(self, name, a, b, r, sides):
        """Open cylinder from point a to point b, radius r, as a prism with `sides` faces."""
        ax = [b[i] - a[i] for i in range(3)]
        ln = math.sqrt(sum(c * c for c in ax))
        ax = [c / ln for c in ax]
        ref = (0.0, 0.0, 1.0) if abs(ax[2]) < 0.9 else (1.0, 0.0, 0.0)
        u = [ax[1] * ref[2] - ax[2] * ref[1], ax[2] * ref[0] - ax[0] * ref[2], ax[0] * ref[1] - ax[1] * ref[0]]
        lu = math.sqrt(sum(c * c for c in u))
        u = [c / lu for c in u]
        w = [ax[1] * u[2] - ax[2] * u[1], ax[2] * u[0] - ax[0] * u[2], ax[0] * u[1] - ax[1] * u[0]]
        self.lines.append(f"g {name}")
        base = self.next
        for end in (a, b):
            for k in range(sides):
                t = 2 * math.pi * k / sides
                p = [end[i] + r * (math.cos(t) * u[i] + math.sin(t) * w[i]) for i in range(3)]
                self.lines.append(f"v {p[0]:.5f} {p[1]:.5f} {p[2]:.5f}")
        self.next += 2 * sides
        for k in range(sides):
            k2 = (k + 1) % sides
            a0, a1, b0, b1 = base + k, base + k2, base + sides + k, base + sides + k2
            self.lines.append(f"f {a0} {a1} {b1}")
            self.lines.append(f"f {b1} {b0} {a0}")


def build(hx, hy, h, first_index, diameter_in=1.5, spacing_ft=5.0, rail_ft=5.0, inset_in=0.0,
          top_rails=False, sides=12):
    r, inset = diameter_in * IN / 2, inset_in * IN
    px, py = hx - inset, hy - inset                          # pole centre lines
    m = Mesh(first_index)
    n = max(1, round(2 * hx / (spacing_ft * FT)))            # bays per side (4 for 20 ft at 5 ft)
    xs = [-px + 2 * px * k / n for k in range(n + 1)]
    ys = [-py + 2 * py * k / max(1, round(2 * hy / (spacing_ft * FT))) for k in
          range(max(1, round(2 * hy / (spacing_ft * FT))) + 1)]
    seen = set()

    def pole(x, y, name):
        key = (round(x, 4), round(y, 4))
        if key not in seen:
            seen.add(key)
            m.tube(name, (x, y, 0.0), (x, y, h), r, sides)

    for k, x in enumerate(xs):
        pole(x, py, f"Frame pole north {k + 1}")
        pole(x, -py, f"Frame pole south {k + 1}")
    for k, y in enumerate(ys):
        pole(-px, y, f"Frame pole west {k + 1}")
        pole(px, y, f"Frame pole east {k + 1}")
    heights = [("middle", rail_ft * FT)] + ([("top", h - r)] if top_rails else [])
    for label, z in heights:
        m.tube(f"Frame rail north {label}", (-px, py, z), (px, py, z), r, sides)
        m.tube(f"Frame rail south {label}", (-px, -py, z), (px, -py, z), r, sides)
        m.tube(f"Frame rail west {label}", (-px, -py, z), (-px, py, z), r, sides)
        m.tube(f"Frame rail east {label}", (px, -py, z), (px, py, z), r, sides)
    return m, len(seen)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--diameter-in", type=float, default=1.5)
    ap.add_argument("--pole-spacing-ft", type=float, default=5.0)
    ap.add_argument("--rail-height-ft", type=float, default=5.0)
    ap.add_argument("--inset-in", type=float, default=0.0, help="pole centres this far inside the net sheets")
    ap.add_argument("--top-rails", action="store_true")
    ap.add_argument("--sides", type=int, default=12, help="faces per pole")
    a = ap.parse_args()
    hx, hy, h, nv = read_net(a.src)
    m, nposts = build(hx, hy, h, nv + 1, a.diameter_in, a.pole_spacing_ft, a.rail_height_ft, a.inset_in,
                      a.top_rails, a.sides)
    with open(a.src) as fin, open(a.dst, "w") as fout:
        for line in fin:
            fout.write(line)
        fout.write("\n# --- net frame poles, added by depthcloud/add_net_poles.py "
                   f"(diameter {a.diameter_in} in, spacing {a.pole_spacing_ft} ft, "
                   f"rail at {a.rail_height_ft} ft, inset {a.inset_in} in) ---\n")
        fout.write("\n".join(m.lines) + "\n")
    print(f"net {2 * hx:.3f} x {2 * hy:.3f} m, {h:.3f} m tall; added {nposts} vertical poles and "
          f"{4 * (2 if a.top_rails else 1)} horizontal rails ({m.next - nv - 1} vertices) -> {a.dst}")


if __name__ == "__main__":
    sys.exit(main())
