#!/usr/bin/env python3
"""Walk test for cloud_localize.py: carry the drone through the course and check where it thinks it is.

Props OFF. Only if walking the drone through the course is allowed.

How it works: you put the drone down on known spots on the course (the launch mark, the corners of the
taped floor line, the tube mouths, the landing pad centre), nose pointing along the direction of travel,
and press Enter. The script takes the median of the last 2 s of the localizer's estimate
(/team/localize/base_pose) and compares it with the spot's real position. It does the same for camera
odometry alone (/visual_slam/tracking/odometry, anchored on the launch mark), so the result shows
whether matching against the course model beats odometry drift.

Needs stereo_cloud.py and cloud_localize.py already running (see depthcloud/README.md). Writes, into
--out (default /root/team_ws/walk_tests/<time>, i.e. ~/team_ws/walk_tests on the drone):
  results.json   one entry per checkpoint: truth, estimate, error, odometry-only error, statuses
  track.csv      every pose, status and odometry sample received
  plan.svg       top view: course walls, checkpoints, localizer track (blue), odometry-only (grey)

Keys while it runs: Enter = record the next checkpoint (the drone is sitting on it), s = skip it,
b = go back one, q = finish and write the results.

  python3 walk_test.py                       on the drone, in the stack container (docker exec -it)
  python3 walk_test.py --list                print the checkpoints and exit
  python3 walk_test.py --selftest            synthetic walk, no ROS: checks the maths and the outputs

Course coordinates: metres, x east, y north, origin at the floor centre (same as the .obj and the
routes). Checkpoints come from the course json, the AS-BUILT one by default
(gazebo_sim/course/course_asbuilt.json; --course plan for course_v3.json): inches from the net's
south-west corner, converted as (inches - 120) * 0.0254. Run cloud_localize on the same course.
On the as-built course the floor-line corners are computed from the scanned walls, not observed: if the
real tape corner is elsewhere, put the drone on the computed spot (measure from the west net), not the tape.
"""
import argparse
import csv
import json
import math
import os
import sys
import threading
import time

IN = 0.0254


def course_m(x_in, y_in):
    return round((x_in - 120) * IN, 4), round((y_in - 120) * IN, 4)


COURSE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "gazebo_sim", "course")
COURSES = {"asbuilt": os.path.join(COURSE_DIR, "course_asbuilt.json"),
           "plan": os.path.join(COURSE_DIR, "course_v3.json")}
LINE_HINTS = ["first corner, north end of the first corridor", "east end of that top stretch (over M1)",
              "south end, between M1 and M2", "east end of that bottom stretch (under M2)",
              "north end, between M2 and M3"]


def checkpoints_from(course_json):
    """(name, x, y, what to look for) in the window route's order, from a course json: launch mark, the
    floor line's corners and end (gate A), the tube's mouths and middle, the landing pad centre."""
    c = json.load(open(course_json))
    line = c["floor_line_in"]
    tube = c["tunnel"]["footprint_in"]
    tx = (tube["x_min"] + tube["x_max"]) / 2
    cps = [("launch mark", *course_m(*c["launch_mark_in"]), "start here, nose north, BEFORE starting cloud_localize")]
    for k, p in enumerate(line[1:-1]):
        hint = LINE_HINTS[k] if k < len(LINE_HINTS) else "floor line corner"
        cps.append((f"line corner {k + 1}", *course_m(*p), f"floor line: {hint}"))
    cps.append(("line end", *course_m(*line[-1]), "floor line: its end, at gate A (maze exit)"))
    cps += [("tube north mouth", *course_m(tx, tube["y_max"]), "centre of the tube's north opening, on the floor"),
            ("tube middle", *course_m(tx, (tube["y_min"] + tube["y_max"]) / 2), "halfway through the tube (dark: the hard one)"),
            ("tube south mouth", *course_m(tx, tube["y_min"]), "centre of the tube's south opening"),
            ("landing pad", *course_m(*c["landing_pad_in"]["center"]), "centre of the landing pad rings")]
    return cps


CHECKPOINTS = checkpoints_from(COURSES["asbuilt"] if os.path.exists(COURSES["asbuilt"]) else COURSES["plan"])
LAUNCH_YAW_DEG = 90.0                      # facing north, as cloud_localize and navless_nav assume
WINDOW_S = 2.0                             # seconds of samples averaged at each checkpoint
GOOD = ("tracking", "weak")                # statuses where the pose was checked against the model


def median(v):
    v = sorted(v)
    n = len(v)
    return None if n == 0 else (v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2]))


def yaw_of(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


class WalkLog:
    """Everything that isn't ROS: samples in, checkpoint results and files out."""

    def __init__(self, checkpoints=CHECKPOINTS):
        self.cps = list(checkpoints)
        self.lock = threading.Lock()
        self.est = []        # (t, x, y, z, yaw_deg, status)
        self.odom = []       # (t, x, y, yaw_deg) raw, odom frame
        self.status = None   # latest status dict
        self.status_log = []  # (t, status, fitness)
        self.anchor = None   # (odom x, y, yaw at launch, launch x, y)
        self.results = []

    # --- inputs ------------------------------------------------------------------------------
    def on_status(self, t, s):
        with self.lock:
            self.status = s
            self.status_log.append((t, s.get("status", "?"), s.get("fitness")))

    def on_pose(self, t, x, y, z, yaw_deg):
        with self.lock:
            st = self.status.get("status", "?") if self.status else "?"
            self.est.append((t, x, y, z, yaw_deg, st))

    def on_odom(self, t, x, y, yaw_deg):
        with self.lock:
            self.odom.append((t, x, y, yaw_deg))

    # --- odometry only, placed on the course from the launch mark ------------------------------
    def set_anchor(self, launch_x, launch_y):
        """Call with the drone sitting on the launch mark, nose north: odom's pose there = launch."""
        with self.lock:
            if not self.odom:
                return False
            _, ox, oy, oyaw = self.odom[-1]
            self.anchor = (ox, oy, oyaw, launch_x, launch_y)
            return True

    def odom_to_course(self, x, y, yaw_deg):
        ox, oy, oyaw, lx, ly = self.anchor
        r = math.radians(LAUNCH_YAW_DEG - oyaw)
        dx, dy = x - ox, y - oy
        return (lx + math.cos(r) * dx - math.sin(r) * dy, ly + math.sin(r) * dx + math.cos(r) * dy,
                wrap_deg(yaw_deg + LAUNCH_YAW_DEG - oyaw))

    # --- checkpoints -------------------------------------------------------------------------
    def record(self, i, now):
        name, tx, ty, _ = self.cps[i]
        with self.lock:
            if i == 0 and self.anchor is None and self.odom:
                _, ox, oy, oyaw = self.odom[-1]
                self.anchor = (ox, oy, oyaw, tx, ty)
            win = [e for e in self.est if now - e[0] <= WINDOW_S]
            good = [e for e in win if e[5] in GOOD]
            use = good or win
            stats = {}
            for t, st, _ in self.status_log:
                if now - t <= WINDOW_S:
                    stats[st] = stats.get(st, 0) + 1
            fits = [f for t, _, f in self.status_log if now - t <= WINDOW_S and f is not None]
            odo = [o for o in self.odom if now - o[0] <= WINDOW_S]
            anchor = self.anchor
        r = {"n": i, "name": name, "truth": [tx, ty], "time": round(now, 2), "statuses": stats,
             "fitness": None if not fits else round(median(fits), 3), "samples": len(use),
             "checked_against_model": bool(good)}
        if use:
            ex, ey = median([e[1] for e in use]), median([e[2] for e in use])
            r["estimate"] = [round(ex, 3), round(ey, 3)]
            r["yaw_deg"] = round(median([e[4] for e in use]), 1)
            r["error_m"] = round(math.hypot(ex - tx, ey - ty), 3)
        else:
            r["estimate"] = r["error_m"] = r["yaw_deg"] = None
        if anchor and odo:
            pts = [self.odom_to_course(o[1], o[2], o[3]) for o in odo]
            ox, oy = median([p[0] for p in pts]), median([p[1] for p in pts])
            r["odom_only"] = [round(ox, 3), round(oy, 3)]
            r["odom_only_error_m"] = round(math.hypot(ox - tx, oy - ty), 3)
        else:
            r["odom_only"] = r["odom_only_error_m"] = None
        self.results = [x for x in self.results if x["n"] != i] + [r]
        self.results.sort(key=lambda x: x["n"])
        return r

    # --- outputs -----------------------------------------------------------------------------
    def summary(self):
        lines = [f"{'checkpoint':<18}{'localizer':>11}{'odom only':>11}  {'checked':<8}statuses"]
        for r in self.results:
            e = "   --" if r["error_m"] is None else f"{r['error_m'] * 100:6.1f} cm"
            o = "   --" if r["odom_only_error_m"] is None else f"{r['odom_only_error_m'] * 100:6.1f} cm"
            st = ", ".join(f"{k} {v}" for k, v in sorted(r["statuses"].items()))
            lines.append(f"{r['name']:<18}{e:>11}{o:>11}  {'yes' if r['checked_against_model'] else 'NO':<8}{st}")
        errs = [r["error_m"] for r in self.results if r["error_m"] is not None and r["n"] > 0]
        oerrs = [r["odom_only_error_m"] for r in self.results if r["odom_only_error_m"] is not None and r["n"] > 0]
        if errs:
            lines.append(f"localizer: median {median(errs) * 100:.1f} cm, worst {max(errs) * 100:.1f} cm "
                         f"over {len(errs)} checkpoints after the start")
        if oerrs:
            lines.append(f"odom only: median {median(oerrs) * 100:.1f} cm, worst {max(oerrs) * 100:.1f} cm")
        with self.lock:
            n = len(self.status_log)
            counts = {}
            for _, st, _ in self.status_log:
                counts[st] = counts.get(st, 0) + 1
        if n:
            lines.append("whole walk: " + ", ".join(f"{k} {100.0 * v / n:.0f}%" for k, v in
                                                   sorted(counts.items(), key=lambda kv: -kv[1])))
        lines.append("Placement by hand is good to about +-3 cm, so errors below ~5 cm are within it.")
        return "\n".join(lines)

    def write(self, out, course_json=None):
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, "results.json"), "w") as f:
            json.dump({"checkpoints": self.results, "window_s": WINDOW_S,
                       "summary": self.summary().splitlines()}, f, indent=1)
        with self.lock:
            est, odom, stl = list(self.est), list(self.odom), list(self.status_log)
        with open(os.path.join(out, "track.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "source", "x", "y", "z", "yaw_deg", "status", "fitness"])
            for t, x, y, z, yaw, st in est:
                w.writerow([f"{t:.3f}", "localizer", f"{x:.4f}", f"{y:.4f}", f"{z:.4f}", f"{yaw:.1f}", st, ""])
            for t, x, y, yaw in odom:
                if self.anchor:
                    x, y, yaw = self.odom_to_course(x, y, yaw)
                w.writerow([f"{t:.3f}", "odom_only" if self.anchor else "odom_raw",
                            f"{x:.4f}", f"{y:.4f}", "", f"{yaw:.1f}", "", ""])
            for t, st, fit in stl:
                w.writerow([f"{t:.3f}", "status", "", "", "", "", st, "" if fit is None else fit])
        with open(os.path.join(out, "plan.svg"), "w") as f:
            f.write(self.svg(course_json, est, odom))
        return out

    def svg(self, course_json, est, odom):
        S, PAD, HALF = 120.0, 30, 3.048                      # px per metre; the net is 6.1 m square
        size = int(2 * HALF * S + 2 * PAD)
        X = lambda x: PAD + (x + HALF) * S
        Y = lambda y: PAD + (HALF - y) * S
        o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{size}" height="{size + 40}" '
             f'font-family="sans-serif" font-size="11">',
             f'<rect width="100%" height="100%" fill="white"/>',
             f'<rect x="{X(-HALF)}" y="{Y(HALF)}" width="{2 * HALF * S}" height="{2 * HALF * S}" '
             f'fill="none" stroke="#999" stroke-dasharray="4 3"/>']
        course = None
        if course_json and os.path.exists(course_json):
            with open(course_json) as f:
                course = json.load(f)
        if course:
            for w in course.get("walls", []):
                (x0, y0), (x1, y1) = course_m(*w["start_in"]), course_m(*w["end_in"])
                o.append(f'<line x1="{X(x0):.1f}" y1="{Y(y0):.1f}" x2="{X(x1):.1f}" y2="{Y(y1):.1f}" '
                         f'stroke="#555" stroke-width="{max(2, w.get("thickness_in", 4) * IN * S):.1f}"/>')
            fp = course.get("tunnel", {}).get("footprint_in")
            if fp:
                (ax, ay), (bx, by) = course_m(fp["x_min"], fp["y_max"]), course_m(fp["x_max"], fp["y_min"])
                o.append(f'<rect x="{X(ax):.1f}" y="{Y(ay):.1f}" width="{(bx - ax) * S:.1f}" '
                         f'height="{(ay - by) * S:.1f}" fill="#eee" stroke="#888"/>')
            pad = course.get("landing_pad_in")
            if pad:
                cx, cy = course_m(*pad["center"])
                o.append(f'<circle cx="{X(cx):.1f}" cy="{Y(cy):.1f}" r="{pad["outer_ring_radius"] * IN * S:.1f}" '
                         f'fill="none" stroke="#bbb"/>')
            line = course.get("floor_line_in")
            if line:
                pts = " ".join(f"{X(a):.1f},{Y(b):.1f}" for a, b in (course_m(*p) for p in line))
                o.append(f'<polyline points="{pts}" fill="none" stroke="#e0a000" stroke-width="2"/>')
        if self.anchor and odom:
            pts = " ".join(f"{X(a):.1f},{Y(b):.1f}" for a, b, _ in
                           (self.odom_to_course(x, y, yw) for _, x, y, yw in odom[::3]))
            o.append(f'<polyline points="{pts}" fill="none" stroke="#999" stroke-width="1.5" '
                     f'stroke-dasharray="5 3"/>')
        if est:
            pts = " ".join(f"{X(e[1]):.1f},{Y(e[2]):.1f}" for e in est)
            o.append(f'<polyline points="{pts}" fill="none" stroke="#1f6fd1" stroke-width="2"/>')
        done = {r["n"]: r for r in self.results}
        for i, (name, tx, ty, _) in enumerate(self.cps):
            r = done.get(i)
            o.append(f'<circle cx="{X(tx):.1f}" cy="{Y(ty):.1f}" r="5" fill="{"#2a9d4b" if r else "#ccc"}"/>')
            label = name if not r or r["error_m"] is None else f"{name} {r['error_m'] * 100:.0f} cm"
            o.append(f'<text x="{X(tx) + 7:.1f}" y="{Y(ty) - 6:.1f}">{label}</text>')
            if r and r["estimate"]:
                ex, ey = r["estimate"]
                o.append(f'<line x1="{X(tx):.1f}" y1="{Y(ty):.1f}" x2="{X(ex):.1f}" y2="{Y(ey):.1f}" '
                         f'stroke="#d33" stroke-width="1.5"/>')
                o.append(f'<path d="M{X(ex) - 4:.1f},{Y(ey) - 4:.1f}l8,8m0,-8l-8,8" stroke="#d33" stroke-width="2"/>')
        o.append(f'<text x="{PAD}" y="{size + 20}">blue: localizer   grey dashed: odometry only   '
                 f'green dot: checkpoint   red x: localizer estimate there   yellow: floor line</text>')
        o.append("</svg>")
        return "\n".join(o)


# ---------------------------------------------------------------------------------------------
def selftest(course_json=COURSES["asbuilt"]):
    """Synthetic walk: the localizer sits 3 cm off the truth, odometry drifts 5% and 2 deg."""
    log = WalkLog(CHECKPOINTS)
    t = 100.0
    odom_yaw0 = 30.0                              # odom frame starts rotated, like real cuVSLAM
    for i, (name, tx, ty, _) in enumerate(CHECKPOINTS):
        dx, dy = tx - CHECKPOINTS[0][1], ty - CHECKPOINTS[0][2]
        # odometry: course offset rotated into odom (minus 2 deg of drift), scaled 1.05
        r = math.radians(odom_yaw0 - LAUNCH_YAW_DEG - 2.0)
        ox, oy = 1.05 * (math.cos(r) * dx - math.sin(r) * dy), 1.05 * (math.sin(r) * dx + math.cos(r) * dy)
        for k in range(30):                       # 3 s at 10 Hz
            t += 0.1
            st = "lost" if name == "tube middle" else "tracking"
            log.on_status(t, {"status": st, "fitness": 0.7})
            if st != "lost":
                log.on_pose(t, tx + 0.03, ty, 0.07, 90.0)
            log.on_odom(t, ox, oy, odom_yaw0)
        r = log.record(i, t)
        if i == 0:
            assert r["error_m"] == 0.03 and r["odom_only_error_m"] == 0.0, r
    res = {r["name"]: r for r in log.results}
    assert all(abs(r["error_m"] - 0.03) < 1e-9 for r in log.results if r["error_m"] is not None)
    assert res["tube middle"]["estimate"] is None and list(res["tube middle"]["statuses"]) == ["lost"]
    far = res["landing pad"]
    want = 0.05 * math.hypot(far["truth"][0] - CHECKPOINTS[0][1], far["truth"][1] - CHECKPOINTS[0][2])
    assert far["odom_only_error_m"] > want, far           # 5% scale + 2 deg drift shows up
    here = os.path.dirname(os.path.abspath(__file__))
    out = log.write(os.path.join(here, "walk_test_selftest"), course_json)
    print(log.summary())
    print(f"selftest OK, outputs in {out}")


def run(out, course_json):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String

    log = WalkLog(CHECKPOINTS)
    rclpy.init()
    node = rclpy.create_node("walk_test")
    deg = math.degrees

    def on_pose(m):
        q, p = m.pose.orientation, m.pose.position
        log.on_pose(time.monotonic(), p.x, p.y, p.z, deg(yaw_of(q.x, q.y, q.z, q.w)))

    def on_status(m):
        try:
            log.on_status(time.monotonic(), json.loads(m.data))
        except ValueError:
            pass

    def on_odom(m):
        q, p = m.pose.pose.orientation, m.pose.pose.position
        log.on_odom(time.monotonic(), p.x, p.y, deg(yaw_of(q.x, q.y, q.z, q.w)))

    node.create_subscription(PoseStamped, "/team/localize/base_pose", on_pose, 10)
    node.create_subscription(String, "/team/localize/status", on_status, 10)
    node.create_subscription(Odometry, "/visual_slam/tracking/odometry", on_odom, qos_profile_sensor_data)
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    stop = threading.Event()

    def ticker():                                  # one live line every 2 s
        while not stop.wait(2.0):
            with log.lock:
                s = log.status or {}
                e = log.est[-1] if log.est else None
            nxt = CHECKPOINTS[i_next[0]] if i_next[0] < len(CHECKPOINTS) else None
            where = "no estimate yet" if e is None else f"est ({e[1]:+.2f}, {e[2]:+.2f}) yaw {e[4]:.0f}"
            dist = "" if (e is None or nxt is None) else f" | {math.hypot(e[1] - nxt[1], e[2] - nxt[2]):.2f} m to next"
            fit = s.get("fitness")
            print(f"  [{s.get('status', 'no status')}{'' if fit is None else f' fit {fit:.2f}'}] {where}{dist}",
                  flush=True)

    i_next = [0]
    threading.Thread(target=ticker, daemon=True).start()
    print(__doc__.split("Needs")[0].strip(), "\n", flush=True)
    try:
        while i_next[0] < len(CHECKPOINTS):
            name, tx, ty, hint = CHECKPOINTS[i_next[0]]
            cmd = input(f"\n>> {i_next[0]}: {name} ({tx:+.3f}, {ty:+.3f}): {hint}. "
                        f"Put the drone on it, wait 3 s, Enter (s skip, b back, q finish): ").strip().lower()
            if cmd == "q":
                break
            if cmd == "s":
                i_next[0] += 1
                continue
            if cmd == "b":
                i_next[0] = max(0, i_next[0] - 1)
                continue
            r = log.record(i_next[0], time.monotonic())
            if r["estimate"] is None:
                print(f"   no localizer pose in the last {WINDOW_S:.0f} s (statuses {r['statuses']}). "
                      "Recorded as a miss.")
            else:
                oe = r["odom_only_error_m"]
                odo = "--" if oe is None else f"{oe * 100:.1f} cm"
                note = "" if r["checked_against_model"] else " (NOT checked against the model)"
                print(f"   localizer {r['estimate']} -> error {r['error_m'] * 100:.1f} cm{note}; "
                      f"odometry only: {odo}")
            i_next[0] += 1
    except (EOFError, KeyboardInterrupt):
        print()
    stop.set()
    print("\n" + log.summary())
    print(f"\nwritten to {log.write(out, course_json)}")
    executor.shutdown()                            # stop spinning before tearing the node down
    spinner.join(timeout=2.0)
    node.destroy_node()
    rclpy.try_shutdown()


def main():
    global CHECKPOINTS
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", default=os.path.join("/root/team_ws/walk_tests", time.strftime("%Y%m%d-%H%M%S")))
    ap.add_argument("--course", default="asbuilt",
                    help="asbuilt (the real course, default), plan (course_v3.json, FlyX Lab), or a course json "
                         "path: the checkpoints and the walls on plan.svg. Run cloud_localize on the same course.")
    ap.add_argument("--list", action="store_true", help="print the checkpoints and exit")
    ap.add_argument("--selftest", action="store_true", help="synthetic walk, no ROS")
    a = ap.parse_args()
    course_json = COURSES.get(a.course, a.course)
    CHECKPOINTS = checkpoints_from(course_json)
    print(f"course: {a.course} ({os.path.normpath(course_json)})")
    if a.list:
        for i, (n, x, y, h) in enumerate(CHECKPOINTS):
            print(f"{i:2d}  {n:<18} ({x:+.3f}, {y:+.3f})  {h}")
    elif a.selftest:
        selftest(course_json)
    else:
        run(a.out, course_json)


if __name__ == "__main__":
    sys.exit(main())
