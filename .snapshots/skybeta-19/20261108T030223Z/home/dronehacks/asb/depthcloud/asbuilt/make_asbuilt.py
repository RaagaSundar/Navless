#!/usr/bin/env python3
"""Build the AS-BUILT course (json + .obj) from the plan plus the walls measured in the phone scan.

The organisers' course_v3.json / dronehacks-course-v3.obj are a temporary plan. The real course (phone LiDAR
scan, 2026-11-06) matches it for the net, the barrier line, the window wall C2a, the tube and the boulders
(within ~2-6 cm), but five walls are elsewhere: M1, M2, M3 (maze), A1 and C1 (chicane), by 9-76 cm and
leaning up to 13 deg. Their scan fits are in scan_walls.json (how they were made: scan/).

  python3 make_asbuilt.py                                  # -> gazebo_sim/course/course_asbuilt.json
  python3 make_asbuilt.py --obj dronehacks-course-v3.obj   # ... and dronehacks-course-asbuilt.obj next to it

Pure Python 3, no numpy, so it runs on any laptop. What it changes in the plan json:
  walls      M1 M2 M3 A1 C1 replaced (same names, 4 in thick, 68 in tall), "source" says where from
  floor_line recomputed the way the plan's line was laid out: every leg midway between the two walls
             (or wall and net/barrier) either side of it. Checked: on the plan walls this reproduces the
             plan's floor_line_in exactly. NOT observed in the scan (the tape doesn't show in it).
  gates_in   gate A from M3's north end to the north net
  checkpoints the wall-bounded zones follow the moved walls
Everything else (launch mark, tube, C2a + window, C2b, barriers, boulders, pad, net) is the plan's.
In the .obj, the five walls' plan faces are dropped and boxes at the measured positions are added under the
same group names, so cloud_localize and other tools find them as before.

Shining confirmed the as-built walls against the real course on 2026-11-07, so this is the course to plan and
fly on. To correct a wall later, edit scan_walls.json and re-run.
"""
import argparse
import copy
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
PLAN = os.path.join(REPO, "gazebo_sim", "course", "course_v3.json")
OUT = os.path.join(REPO, "gazebo_sim", "course", "course_asbuilt.json")
IN = 0.0254
NET = 240.0                                   # net is 240 in square, origin at its south-west corner
BAR_N = 140.0                                 # barrier line's north face (centre 138, 4 in thick)
OBJ_GROUP = {"M1": "M1 maze divider", "M2": "M2 maze divider", "M3": "M3 maze divider",
             "A1": "A1 approach wall", "C1": "C1 chicane wall"}


def x_at(w, y, side=0):
    """x of wall w's centre line at height y (inches), or of its west (side=-1) / east (+1) face."""
    (x0, y0), (x1, y1) = w["start_in"], w["end_in"]
    x = x0 + (y - y0) * (x1 - x0) / (y1 - y0)
    lean = math.atan2(x1 - x0, y1 - y0)
    return x + side * (w["thickness_in"] / 2) / abs(math.cos(lean))


def ends(w):
    """(south end, north end) of a wall that runs roughly north-south."""
    a, b = w["start_in"], w["end_in"]
    return (a, b) if a[1] <= b[1] else (b, a)


def floor_line(c):
    """The guideline: launch mark -> first corridor -> over M1 -> down between M1/M2 -> under M2 -> up between
    M2/M3 -> over M3 -> gate A. Each leg midway between what bounds it (same rule as the plan's line)."""
    W = {w["name"]: w for w in c["walls"]}
    m1, m2, m3 = W["M1"], W["M2"], W["M3"]
    lx, ly = c["launch_mark_in"]
    top1 = (ends(m1)[1][1] + NET) / 2                      # passage over M1's north end
    bot = (BAR_N + ends(m2)[0][1]) / 2                      # passage under M2's south end
    top2 = (ends(m3)[1][1] + NET) / 2                       # passage over M3's north end
    leg_a = lambda y: (0.0 + x_at(m1, y, -1)) / 2           # net .. M1 west face
    leg_b = lambda y: (x_at(m1, y, +1) + x_at(m2, y, -1)) / 2
    leg_c = lambda y: (x_at(m2, y, +1) + x_at(m3, y, -1)) / 2
    pts = [[lx, ly], [leg_a(top1), top1], [leg_b(top1), top1], [leg_b(bot), bot], [leg_c(bot), bot],
           [leg_c(top2), top2], [ends(m3)[1][0], top2]]
    return [[round(x, 2), round(y, 2)] for x, y in pts]


def build(plan, scan):
    c = copy.deepcopy(plan)
    fits = {w["name"]: w for w in scan["walls"]}
    for w in c["walls"]:
        g = OBJ_GROUP.get(w["name"])
        if g and g in fits:
            f = fits[g]
            w["start_in"], w["end_in"] = [round(v, 1) for v in f["start_in"]], [round(v, 1) for v in f["end_in"]]
            dx, dy = w["end_in"][0] - w["start_in"][0], w["end_in"][1] - w["start_in"][1]
            w["length_in"] = round(math.hypot(dx, dy), 1)
            w["bearing_deg"] = round(math.degrees(math.atan2(dx, dy)) % 360, 1)
            w["source"] = "phone scan 2026-11-06 (depthcloud/asbuilt), confirmed against the real course 2026-11-07"
    W = {w["name"]: w for w in c["walls"]}
    c["floor_line_in"] = floor_line(c)
    c["gates_in"]["gate_A"]["between"] = [ends(W["M3"])[1], [ends(W["M3"])[1][0], NET]]
    m1s, m1n = ends(W["M1"])
    m3s = ends(W["M3"])[0]
    top1 = c["floor_line_in"][1][1]
    for cp in c["checkpoints"]:
        if cp["name"] == "First corridor":
            cp["area_in"] = [[0, BAR_N], [round(x_at(W["M1"], BAR_N), 1), BAR_N],
                             [round(x_at(W["M1"], top1), 1), top1], [0, top1]]
        elif cp["name"] == "Maze":
            cp["area_in"] = [[0, top1], [round(x_at(W["M1"], top1), 1), top1], [round(x_at(W["M1"], BAR_N), 1), BAR_N],
                             [round(x_at(W["M3"], BAR_N), 1), BAR_N], [round(x_at(W["M3"], NET), 1), NET], [0, NET]]
        elif cp["name"] == "Tunnel":
            cp["area_in"][0] = [round(x_at(W["M3"], BAR_N), 1), BAR_N]
            cp["area_in"][-1] = [round(x_at(W["M3"], NET), 1), NET]
    c["_notes"] = ("AS-BUILT course: course_v3.json (the organisers' temporary plan) with M1, M2, M3, A1 and C1 moved "
                   "to where the 2026-11-06 phone LiDAR scan puts them, and the floor line, gate A and the "
                   "wall-bounded checkpoint zones recomputed from them. Made by depthcloud/asbuilt/make_asbuilt.py; "
                   "don't edit by hand, edit scan_walls.json and re-run. Walls confirmed against the real course "
                   "2026-11-07; the floor line is computed, not observed. clearances_in below are "
                   "still the PLAN's. Same frame as course_v3.json: inches, origin = net's south-west corner, "
                   "x east, y north (OBJ/world metres = (inches - 120) * 0.0254). " + plan["_notes"])
    return c


def to_in(p):
    return [round(p[0] / IN + 120, 2), round(p[1] / IN + 120, 2)]


def apply_changes(c, ch):
    """Later course changes on top of the scanned as-built course (e.g. maze_2026-11-07.json): walls removed,
    walls added (with their own .obj group), the observed floor tape (NOT a path to follow) and the maze exit."""
    gone = set(ch.get("remove_walls", []))
    c["removed_walls"] = [{"name": w["name"], "obj_group": OBJ_GROUP.get(w["name"], w["name"]),
                           "removed": "organisers' maze change 2026-11-07"} for w in c["walls"] if w["name"] in gone]
    c["walls"] = [w for w in c["walls"] if w["name"] not in gone]
    for a in ch.get("add_walls", []):
        s, e = to_in(a["start_m"]), to_in(a["end_m"])
        dx, dy = e[0] - s[0], e[1] - s[1]
        c["walls"].append({"name": a["name"], "kind": a["kind"], "start_in": s, "end_in": e,
                           "length_in": round(math.hypot(dx, dy), 1),
                           "bearing_deg": round(math.degrees(math.atan2(dx, dy)) % 360, 1),
                           "thickness_in": a["thickness_in"], "height_in": a["height_in"],
                           "obj_group": a["obj_group"], "note": a.get("note", ""),
                           "source": "organisers' maze change 2026-11-07 (Shining's sketch, 1 m gap measured)"})
    if "floor_tape_m" in ch:
        # The taped line on the floor is the OLD maze's and was not re-laid. Keep it for reference (a camera
        # sees it) but remove floor_line_in, so nothing plans a route along it by mistake.
        c.pop("floor_line_in", None)
        c["floor_tape_in"] = [to_in(p) for p in ch["floor_tape_m"]]
        c["floor_tape_note"] = ("The old maze's tape, still on the floor after the 2026-11-07 maze change. "
                                "DO NOT FOLLOW: it runs under the new long diagonal (MD) at about (-0.85, 1.47) m "
                                "and within ~14 cm of the new north wall (MN). Plan the maze path from the walls.")
    if "maze_exit_m" in ch:
        ex = [to_in(p) for p in ch["maze_exit_m"]]
        c["gates_in"]["gate_A"] = {"between": ex, "note": "maze exit since 2026-11-07: the 1 m gap between the "
                                   "long diagonal's (MD) tip and the north net"}
        tip_x, tip_y = ex[0]
        md = next(w for w in c["walls"] if w["name"] == "MD")
        for cp in c["checkpoints"]:
            if cp["name"] == "First corridor":
                cp["area_in"] = [[0, BAR_N], [md["start_in"][0], BAR_N], [tip_x, tip_y], [0, tip_y]]
            elif cp["name"] == "Maze":
                cp["area_in"] = [[0, tip_y], [tip_x, tip_y], [tip_x, NET], [0, NET]]
            elif cp["name"] == "Tunnel":
                cp["area_in"][0], cp["area_in"][-1] = [tip_x, BAR_N], [tip_x, NET]
        c["checkpoint_zones_note"] = "First corridor / Maze / Tunnel zones redrawn roughly around the 2026-11-07 maze"
    c["_notes"] = ("MAZE CHANGED 2026-11-07: M1, M2, M3 removed; walls MN, MD, MT added (depthcloud/asbuilt/"
                   "maze_2026-11-07.json); floor_line_in removed (the tape is the old maze's: see floor_tape_note); "
                   "gate A = the maze exit gap. " + c["_notes"])
    return c


def write_obj(src, dst, course):
    """Plan .obj minus the faces of every moved or removed wall, plus boxes for the as-built and added walls."""
    moved = {w.get("obj_group") or OBJ_GROUP[w["name"]]: w for w in course["walls"] if "source" in w}
    drop = set(moved) | {r["obj_group"] for r in course.get("removed_walls", [])}
    nv, group = 0, ""
    with open(src) as fin, open(dst, "w") as fout:
        for line in fin:
            t = line.split()
            if t and t[0] == "v":
                nv += 1
            elif t and t[0] == "g":
                group = " ".join(t[1:])
            elif t and t[0] == "f" and group in drop:
                continue
            fout.write(line)
        fout.write("\n# --- AS-BUILT walls from depthcloud/asbuilt/make_asbuilt.py (phone scan 2026-11-06, "
                   "confirmed against the real course 2026-11-07) ---\n")
        i = nv + 1
        for g, w in moved.items():
            (x0, y0), (x1, y1) = [((p[0] - 120) * IN, (p[1] - 120) * IN) for p in (w["start_in"], w["end_in"])]
            L = math.hypot(x1 - x0, y1 - y0)
            nx, ny = -(y1 - y0) / L * w["thickness_in"] * IN / 2, (x1 - x0) / L * w["thickness_in"] * IN / 2
            corners = [(x0 - nx, y0 - ny), (x1 - nx, y1 - ny), (x1 + nx, y1 + ny), (x0 + nx, y0 + ny)]
            fout.write(f"g {g}\n")
            for z in (0.0, w["height_in"] * IN):
                for x, y in corners:
                    fout.write(f"v {x:.5f} {y:.5f} {z:.5f}\n")
            b, t_ = i, i + 4
            for q in ((b, b + 1, b + 2, b + 3), (t_ + 3, t_ + 2, t_ + 1, t_), (b, b + 3, t_ + 3, t_),
                      (b + 1, t_ + 1, t_ + 2, b + 2), (b, t_, t_ + 1, b + 1), (b + 3, b + 2, t_ + 2, t_ + 3)):
                fout.write("f " + " ".join(map(str, q)) + "\n")
            i += 8


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--plan", default=PLAN)
    ap.add_argument("--scan-walls", default=os.path.join(HERE, "scan_walls.json"))
    ap.add_argument("--out", default=OUT)
    ap.add_argument("--obj", help="the organisers' dronehacks-course-v3.obj: also write the as-built .obj")
    ap.add_argument("--obj-out", help="default: dronehacks-course-asbuilt.obj next to --obj")
    ap.add_argument("--changes", default=os.path.join(HERE, "maze_2026-11-07.json"),
                    help="later course changes applied after the scan walls ('' = none)")
    a = ap.parse_args()
    plan = json.load(open(a.plan))
    # the floor-line rule must reproduce the plan's own line from the plan's walls
    assert all(abs(p - q) < 0.01 for P, Q in zip(floor_line(plan), plan["floor_line_in"]) for p, q in zip(P, Q)), \
        (floor_line(plan), plan["floor_line_in"])
    c = build(plan, json.load(open(a.scan_walls)))
    if a.changes:
        c = apply_changes(c, json.load(open(a.changes)))
        print(f"applied {a.changes}")
    with open(a.out, "w") as f:
        json.dump(c, f, indent=2)
        f.write("\n")
    print(f"wrote {a.out}")
    for w in c["walls"]:
        if "source" in w:
            print(f"  {w['name']:<3} {w['start_in']} -> {w['end_in']}  bearing {w['bearing_deg']:5.1f}")
    print("  removed:", [r["name"] for r in c.get("removed_walls", [])])
    print("  floor line:", c.get("floor_line_in", "(none)"), "| floor tape (do not follow):", c.get("floor_tape_in"))
    print("  gate A:", c["gates_in"]["gate_A"]["between"])
    if a.obj:
        dst = a.obj_out or os.path.join(os.path.dirname(os.path.abspath(a.obj)), "dronehacks-course-asbuilt.obj")
        write_obj(a.obj, dst, c)
        print(f"wrote {dst}")


if __name__ == "__main__":
    main()
