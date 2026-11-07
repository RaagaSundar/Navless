#!/usr/bin/env python3
"""Export a centreline route as a navless_nav route file (waypoints_enu in WORLD metres).

  python3 tools/export_route.py [out.json] [--spacing 0.05]                 # window route (default)
  python3 tools/export_route.py --route gate_b                              # chicane + Gate B instead

The route starts above the launch mark at navless_nav's takeoff height (0.85 m) and ends above the pad centre
at 0.40 m; navless_nav takes off and descends on its own. Corners are the same circular fillets fly_course.py
flies (no time smoothing: navless_nav limits speed from curvature itself).
window: tube -> through the C2a window -> pad. gate_b: tube -> round C1's south end -> up the chicane -> west
through Gate B -> round A1's south end -> pad (dh/route.py gate_b_route).
Frame: x east, y north, z up above the floor, origin = floor centre (course OBJ frame).
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from dh.geom import COURSE_SDF, Obstacles, clearance           # noqa: E402
from dh.route import COURSE_JSON, Waypoint, _fillet_path, gate_b_route, window_route  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("out", nargs="?", default=None,
                help="default: course/route_centreline_<route>.json (_<course> appended for another course map)")
ap.add_argument("--course", default=COURSE_JSON,
                help="course map: the organisers' plan by default; course/course_asbuilt.json for the real course")
ap.add_argument("--sdf", default=COURSE_SDF,
                help="collision model the clearances are measured against (default models/dh_course, built from "
                     "the plan); for another course, build one with tools/build_course_model.py")
ap.add_argument("--route", choices=("window", "gate_b"), default="window")
ap.add_argument("--spacing", type=float, default=0.05)
ap.add_argument("--takeoff-z", type=float, default=0.85)
ap.add_argument("--end-z", type=float, default=0.40, help="last point above the pad (stays clear of a 0.3 m floor after a 7 cm start-height offset)")
# navless_nav's carrot cuts the turn before the window and crosses off-centre towards C2a_a: -5.5 cm at the
# course launch's 0.7 m/s, -2.8 cm at 0.5 m/s. Aiming 4 cm the other way keeps >= 10.9 cm to every wall (the
# maze; the window >= 11.5 cm) at both speeds for a +-1 deg placement error (Gazebo, drone profile, 6 laps).
# Unshifted, 1 deg east left 7.4 cm at the window. The aim matches that cut, so re-check it if the course
# launch's speeds or max_turn_sagitta_m change. A smaller cut turns the 4 cm into a bias towards C2a_b,
# which has room (21.0 cm planned).
ap.add_argument("--window-shift-m", type=float, default=0.04,
                help="window route: move the crossing along the wall, + towards C2a_b (dh/route.py window_route); "
                     "0 = the geometric centreline")
# As-built course: C1's and A1's south ends sit in the plan route's last two legs (4.5 and 4.1 cm). Turning
# towards the window 42 in below the tube keeps 30.9 cm to C1; coming onto the pad from 26 in south keeps
# 19.6 cm to A1 (nose north). The defaults are the plan's route.
ap.add_argument("--tube-exit-drop-in", type=float, default=26.0,
                help="window route: turn towards the window this far below the tube's south end, inches")
ap.add_argument("--pad-from-south-in", type=float, default=0.0,
                help="window route: > 0 adds a point this far south of the pad and descends from there, inches")
ap.add_argument("--maze-top-north-m", type=float, default=0.0,
                help="window route: move the crossings over M1's and M3's north ends this far towards the north "
                     "net (the drone reaches further behind its centre than ahead of it), m")
a = ap.parse_args()
course = os.path.basename(a.course)
plan = os.path.samefile(a.course, COURSE_JSON)
if not plan and os.path.samefile(a.sdf, COURSE_SDF):
    sys.exit(f"{course}: the clearances would be measured against the plan's walls; pass --sdf built from it")
tag = "" if plan else "_" + course[:-len(".json")].replace("course_", "")
a.out = a.out or os.path.join(HERE, "..", "course", f"route_centreline_{a.route}{tag}.json")

wps, _ = (window_route(a.course, window_shift_m=a.window_shift_m, tube_exit_drop_in=a.tube_exit_drop_in,
                       pad_from_south_in=a.pad_from_south_in, maze_top_north_m=a.maze_top_north_m)
          if a.route == "window" else gate_b_route(a.course))
launch = wps[0].p
# navless_nav climbs vertically to its takeoff height first, so start the route there; stop above the pad.
wps = [Waypoint("launch_air", (launch[0], launch[1], a.takeoff_z), 0.0)] + wps[2:-2] + [Waypoint("pad_above", (*wps[-2].p[:2], a.end_z), 0.0)]
s, p, _, _, _ = _fillet_path(wps, ds=0.005)
keep = np.searchsorted(s, np.arange(0, s[-1], a.spacing))
pts = np.vstack([p[keep], p[-1:]])

obs = Obstacles(a.sdf)
flying = pts[:, 2] > 0.25
c0, n0 = clearance(obs, pts, np.zeros(len(pts)))                       # nose north the whole way
yaw_win = np.where(np.arange(len(pts)) >= np.argmin(np.abs(pts[:, 1] + 1.0) + (pts[:, 0] < 1.5) * 9), np.radians(20), 0.0)
c20, n20 = clearance(obs, pts, yaw_win)                                 # nose 20 deg (along the window wall) after the tube
k0, k20 = int(np.argmin(np.where(flying, c0, 9))), int(np.argmin(np.where(flying, c20, 9)))

out = {
    "route_type": "WINDOW_CENTRELINE" if a.route == "window" else "GATE_B_CENTRELINE",
    "frame_convention": "world_enu (x east, y north, z up above floor, origin floor centre)",
    "source": f"gazebo_sim/tools/export_route.py (corridor centrelines from {course}, filleted corners)"
              + ("" if a.route == "window" else "; past the tube, arcs centred on the C1, C2b and A1 wall ends"),
    "launch_origin_enu": [round(launch[0], 4), round(launch[1], 4), 0.0],
    "spacing_m": a.spacing,
    "length_m": round(float(s[-1]), 3),
    "min_clearance_m_nose_north": round(float(c0[k0]), 3),
    "min_clearance_at_nose_north": n0[k0],
}
if a.route == "window" and a.window_shift_m:
    out["window_shift_m"] = a.window_shift_m
if a.route == "window" and (a.tube_exit_drop_in != 26.0 or a.pad_from_south_in or a.maze_top_north_m):
    out.update({"tube_exit_drop_in": a.tube_exit_drop_in, "pad_from_south_in": a.pad_from_south_in,
                "maze_top_north_m": a.maze_top_north_m})
if not plan:
    out["course_file"] = f"gazebo_sim/course/{course}"
if a.route == "window":
    out.update({
        "min_clearance_m_nose_20deg_after_tube": round(float(c20[k20]), 3),
        "min_clearance_at_nose_20deg_after_tube": n20[k20],
        "notes": "window crossing at z 0.85 m (opening 0.305-1.219 m)"
                 + (f", aimed {a.window_shift_m * 100:.0f} cm along the wall towards C2a_b so navless_nav's "
                    "flown crossing is centred" if a.window_shift_m else "")
                 + "; maze and tube at 0.94 m; ends above the pad at 0.40 m",
    })
else:
    tight = {}
    for ci, ni, f in zip(c0, n0, flying):
        if f:
            tight[ni] = min(tight.get(ni, 9.0), float(ci))
    out.update({
        "tight_spots_m_nose_north": {k: round(v, 3) for k, v in sorted(tight.items(), key=lambda kv: kv[1])[:5]},
        "notes": "maze, tube, chicane and Gate B at 0.94 m (walls 1.727 m, barrier at Gate B's north side); "
                 "crosses Gate B heading west; descends towards the pad after A1's south end; ends above the pad at 0.40 m",
    })
out["waypoints_enu"] = [[round(float(v), 4) for v in q] for q in pts]
with open(a.out, "w") as f:
    json.dump(out, f, indent=1)
print(f"{len(pts)} waypoints, {s[-1]:.2f} m -> {os.path.normpath(a.out)}")
print(f"min clearance nose north: {c0[k0] * 100:.1f} cm ({n0[k0]})"
      + (f"; nose 20 deg after tube: {c20[k20] * 100:.1f} cm ({n20[k20]})" if a.route == "window" else
         "; tight spots: " + ", ".join(f"{k} {v * 100:.1f}" for k, v in out["tight_spots_m_nose_north"].items())))
