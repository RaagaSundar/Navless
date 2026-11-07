#!/usr/bin/env python3
"""Export the centreline window route as a navless_nav route file (waypoints_enu in WORLD metres).

  python3 tools/export_route.py [out.json] [--spacing 0.05]

The route starts above the launch mark at navless_nav's takeoff height (0.85 m) and ends above the pad centre
at 0.40 m; navless_nav takes off and descends on its own. Corners are the same circular fillets fly_course.py
flies (no time smoothing: navless_nav limits speed from curvature itself).
Frame: x east, y north, z up above the floor, origin = floor centre (course OBJ frame).
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from dh.geom import Obstacles, clearance           # noqa: E402
from dh.route import Waypoint, _fillet_path, window_route  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("out", nargs="?", default=os.path.join(HERE, "..", "course", "route_centreline_window.json"))
ap.add_argument("--spacing", type=float, default=0.05)
ap.add_argument("--takeoff-z", type=float, default=0.85)
ap.add_argument("--end-z", type=float, default=0.40, help="last point above the pad (stays clear of a 0.3 m floor after a 7 cm start-height offset)")
a = ap.parse_args()

wps, _ = window_route()
launch = wps[0].p
# navless_nav climbs vertically to its takeoff height first, so start the route there; stop above the pad.
wps = [Waypoint("launch_air", (launch[0], launch[1], a.takeoff_z), 0.0)] + wps[2:-2] + [Waypoint("pad_above", (*wps[-2].p[:2], a.end_z), 0.0)]
s, p, _, _, _ = _fillet_path(wps, ds=0.005)
keep = np.searchsorted(s, np.arange(0, s[-1], a.spacing))
pts = np.vstack([p[keep], p[-1:]])

obs = Obstacles()
flying = pts[:, 2] > 0.25
c0, n0 = clearance(obs, pts, np.zeros(len(pts)))                       # nose north the whole way
yaw_win = np.where(np.arange(len(pts)) >= np.argmin(np.abs(pts[:, 1] + 1.0) + (pts[:, 0] < 1.5) * 9), np.radians(20), 0.0)
c20, n20 = clearance(obs, pts, yaw_win)                                 # nose 20 deg (along the window wall) after the tube
k0, k20 = int(np.argmin(np.where(flying, c0, 9))), int(np.argmin(np.where(flying, c20, 9)))

out = {
    "route_type": "WINDOW_CENTRELINE",
    "frame_convention": "world_enu (x east, y north, z up above floor, origin floor centre)",
    "source": "gazebo_sim/tools/export_route.py (corridor centrelines from course_v3.json, filleted corners)",
    "launch_origin_enu": [round(launch[0], 4), round(launch[1], 4), 0.0],
    "spacing_m": a.spacing,
    "length_m": round(float(s[-1]), 3),
    "min_clearance_m_nose_north": round(float(c0[k0]), 3),
    "min_clearance_at_nose_north": n0[k0],
    "min_clearance_m_nose_20deg_after_tube": round(float(c20[k20]), 3),
    "min_clearance_at_nose_20deg_after_tube": n20[k20],
    "notes": "window crossing at z 0.85 m (opening 0.305-1.219 m); maze and tube at 0.94 m; ends above the pad at 0.40 m",
    "waypoints_enu": [[round(float(v), 4) for v in q] for q in pts],
}
with open(a.out, "w") as f:
    json.dump(out, f, indent=1)
print(f"{len(pts)} waypoints, {s[-1]:.2f} m -> {os.path.normpath(a.out)}")
print(f"min clearance nose north: {c0[k0] * 100:.1f} cm ({n0[k0]}); nose 20 deg after tube: {c20[k20] * 100:.1f} cm ({n20[k20]})")
