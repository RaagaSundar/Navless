#!/usr/bin/env python3
"""Add a per-leg heading profile to an exported route: course_mission then holds each leg's heading and turns on
the spot between legs (route_headings). Prototype for comparing against nose-north flight.

    python3 tools/add_leg_headings.py course/route_centreline_window_asbuilt.json OUT.json \
        --sdf <as-built model.sdf> --leg 69:99 --leg 106:0

`--leg I:BEARING` gives waypoint I and every later one BEARING (deg clockwise from course north) until the next
--leg; waypoints before the first --leg keep 0 (nose north). headings_deg[i] is the heading of the leg that starts
at waypoint i, so the drone turns at waypoint I. Clearances are measured with dh.geom.clearance at each waypoint's
heading, plus the room to turn on the spot (every 5 deg) at each turn point, the same way export_route.py measures
nose north.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from dh.geom import COURSE_SDF, Obstacles, clearance           # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("route")
ap.add_argument("out")
ap.add_argument("--sdf", default=COURSE_SDF, help="collision model the clearances are measured against")
ap.add_argument("--leg", action="append", default=[], metavar="I:BEARING")
a = ap.parse_args()

data = json.load(open(a.route))
pts = np.array(data["waypoints_enu"], float)
legs = sorted((int(i), float(b)) for i, b in (s.split(":") for s in a.leg))
heading = np.zeros(len(pts))
for i, b in legs:
    heading[i:] = b % 360.0
obs = Obstacles(a.sdf)
c, names = clearance(obs, pts, np.radians(heading))
c0, n0 = clearance(obs, pts, np.zeros(len(pts)))
turns = [i for i in range(1, len(pts)) if abs((heading[i] - heading[i - 1] + 180.0) % 360.0 - 180.0) > 1e-6]
room = {}
for i in turns:
    room[i] = min(clearance(obs, pts[i:i + 1], np.array([math.radians(d)]))[0][0] for d in range(0, 360, 5))
s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
k, k0 = int(np.argmin(c)), int(np.argmin(c0))
data["headings_deg"] = [round(float(h), 1) for h in heading]
data["heading_profile"] = {
    "legs": [{"from_waypoint": i, "bearing_deg": b, "from_m": round(float(s[i]), 2)} for i, b in legs],
    "turn_on_the_spot_at": [{"waypoint": i, "from_m": round(float(s[i]), 2),
                             "from_deg": float(heading[i - 1]), "to_deg": float(heading[i]),
                             "room_cm": round(room[i] * 100, 1)} for i in turns],
    "min_clearance_m": round(float(c[k]), 3), "min_clearance_at": names[k], "min_clearance_waypoint": k,
    "min_clearance_m_nose_north": round(float(c0[k0]), 3), "min_clearance_at_nose_north": n0[k0],
    "note": "prototype: hold each leg's heading, turn on the spot between legs (course_mission route_headings)",
}
json.dump(data, open(a.out, "w"), indent=1)
print(f"{a.out}: {len(pts)} waypoints; turns at " + ", ".join(
    f"{i} ({s[i]:.2f} m, {heading[i - 1]:.0f}->{heading[i]:.0f} deg, room {room[i] * 100:.1f} cm)" for i in turns)
      + f"; min clearance {c[k] * 100:.1f} cm at {names[k]} (nose north: {c0[k0] * 100:.1f} cm at {n0[k0]})")
