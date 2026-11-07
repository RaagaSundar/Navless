#!/usr/bin/env python3
"""Stand-in for the drone: replay a recorded sim flight through navless_nav's mirror_relay protocol.

  python3 gazebo_sim/mirror/replay_relay.py gazebo_sim/runs/<fly_navless run>/ [--speed 1 --loop]
  python3 gazebo_sim/mirror/mirror.py --drone 127.0.0.1

It serves the run's ground-truth path as cuVSLAM-style odometry (power-up frame, from the launch mark
and start bearing), using the relay's own FrameServer and make_frame, so the whole path from relay to
mirror is exercised without a drone.
"""
import argparse
import csv
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, "navless_nav"))
from navless_nav.mirror_relay import FrameServer, make_frame  # noqa: E402
from navless_nav.route import course_to_odom                   # noqa: E402

LAUNCH = (-2.5146, 0.9144)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", help="run directory (or its log.csv) from fly_navless.py")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed factor")
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--start-bearing", type=float, default=0.0)
    ap.add_argument("--start-height", type=float, default=0.07)
    a = ap.parse_args()

    path = a.run if a.run.endswith(".csv") else os.path.join(a.run, "log.csv")
    rows = list(csv.DictReader(open(path)))
    psi0 = math.radians(90.0 - a.start_bearing)
    frames = []
    for r in rows:
        w = (float(r["x"]), float(r["y"]), float(r["z"]))
        p = course_to_odom(w, LAUNCH, a.start_bearing, a.start_height)
        yaw = float(r["yaw_enu"]) - psi0 if "yaw_enu" in r else 0.0
        q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
        sp = None
        if r.get("sp_kind") == "position" and r.get("sp_x") not in (None, "", "nan"):
            sp = course_to_odom((float(r["sp_x"]), float(r["sp_y"]), float(r["sp_z"])),
                                LAUNCH, a.start_bearing, a.start_height)
        armed = r.get("armed") == "1"
        state = {"armed": armed, "offboard_active": armed and r.get("mode") == "6",
                 "nav_state_name": "OFFBOARD" if r.get("mode") == "6" else r.get("state")}
        frames.append((float(r["sim_t"]), p, q, sp, state))
    server = FrameServer(a.port, on_connect=lambda addr: print(f"[replay] client {addr[0]}:{addr[1]}", flush=True))
    print(f"[replay] {len(frames)} frames from {path} on TCP {server.port}; waiting for a client", flush=True)
    while True:
        while not server.n_clients:
            time.sleep(0.2)
        t0, w0 = frames[0][0], time.time()
        for t, p, q, sp, state in frames:
            time.sleep(max(0.0, (t - t0) / a.speed - (time.time() - w0)))
            # No camera TF in a sim log: the mirror then uses its typical (unmeasured) mount.
            server.send(make_frame(time.time(), 0.03, (p, q), 1, state, sp, None))
        print("[replay] end of run", flush=True)
        if not a.loop:
            time.sleep(2)
            server.close()
            return


if __name__ == "__main__":
    main()
