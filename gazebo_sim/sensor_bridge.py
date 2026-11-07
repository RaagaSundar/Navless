#!/usr/bin/env python3
"""Feed PX4 SITL the real drone's sensor path. run_sim.sh starts this with DH_PX4_PROFILE=drone.

Visual odometry
  Gazebo pose -> MAVLink ODOMETRY at 30 Hz. The odom frame is the body's FLU axes at power-up (origin = start
  base_link), sent to PX4 as the drone's odometry bridge does: (odom.y, odom.x, -odom.z), FRD pose frame. Each sample is delivered after a random transport latency (lognormal,
  p50 60 ms / p95 106 ms by default, the measured image -> flight controller chain), with small noise. PX4 stamps
  it on arrival and EKF2_EV_DELAY takes the latency out; EKF2_EV_CTRL=9 fuses horizontal position + yaw only.
  --tube-blackout drops vision inside the dark tube and bumps reset_counter when tracking comes back.
Rangefinder
  A downward laser (2.7 cm left of / 3.4 cm below the body origin) ray-cast against the course collision model:
  MAVLink DISTANCE_SENSOR at 40 Hz, 0.02-4 m, 1 cm noise. The EKF uses it as the height reference while low and slow
  (EKF2_HGT_REF=2, EKF2_RNG_CTRL=1), with the barometer behind it.

  python3 sensor_bridge.py [--instance N] [--tube-blackout] [--ev-lat-p50 0.06 --ev-lat-p95 0.106]
"""
import argparse
import heapq
import math
import os
import sys
import threading
import time

import numpy as np

os.environ.setdefault("MAVLINK20", "1")    # ODOMETRY / DISTANCE_SENSOR extension fields need MAVLink 2
from pymavlink import mavutil              # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dh.geom import Obstacles  # noqa: E402

MODEL = "dh_quad_0"
RNG_OFFSET_FLU = np.array([0.0, 0.027432, -0.03445])     # measured mount: FRD y -0.027, z +0.034
TUBE = ((1.5748, 3.048), (-0.3048, 1.2192))              # x, y extent of the tube footprint (world)


def quat_to_mat(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]


ENU_TO_NED = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class RayCaster:
    """Ray vs. the course's collision primitives (boxes, ellipsoids) and the floor plane z = 0."""

    def __init__(self):
        o = Obstacles()
        self.c, self.R, self.h, self.box = o.c, o.R, o.size, o.is_box

    def cast(self, origin, direction):
        best = (-origin[2] / direction[2]) if direction[2] < -1e-6 else np.inf
        ol = np.einsum("mji,mj->mi", self.R, origin[None, :] - self.c)       # R^T (o - c)
        dl = np.einsum("mji,j->mi", self.R, direction)
        dl = np.where(np.abs(dl) < 1e-9, 1e-9, dl)
        # boxes: slab test
        t1, t2 = (-self.h - ol) / dl, (self.h - ol) / dl
        tmin, tmax = np.minimum(t1, t2).max(1), np.maximum(t1, t2).min(1)
        hit = self.box & (tmax >= np.maximum(tmin, 0)) & (tmin > 0)
        if hit.any():
            best = min(best, float(tmin[hit].min()))
        # ellipsoids: unit sphere after scaling
        os_, ds = ol / self.h, dl / self.h
        a = (ds * ds).sum(1)
        b = 2 * (os_ * ds).sum(1)
        cc = (os_ * os_).sum(1) - 1
        disc = b * b - 4 * a * cc
        ok = ~self.box & (disc >= 0)
        if ok.any():
            t = (-b[ok] - np.sqrt(disc[ok])) / (2 * a[ok])
            t = t[t > 0]
            if t.size:
                best = min(best, float(t.min()))
        return best


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--instance", type=int, default=int(os.environ.get("DH_INSTANCE", 0)))
    ap.add_argument("--ev-rate", type=float, default=30.0)
    ap.add_argument("--ev-lat-p50", type=float, default=0.060)
    ap.add_argument("--ev-lat-p95", type=float, default=0.106)
    ap.add_argument("--ev-noise", type=float, default=0.005, help="position noise (m, 1 sigma)")
    ap.add_argument("--ev-yaw-noise-deg", type=float, default=0.3)
    ap.add_argument("--tube-blackout", action="store_true", default=os.environ.get("DH_TUBE_BLACKOUT") == "1")
    ap.add_argument("--rng-rate", type=float, default=40.0)
    ap.add_argument("--rng-noise", type=float, default=0.01)
    ap.add_argument("--rng-min", type=float, default=0.02)
    ap.add_argument("--rng-max", type=float, default=4.0)
    a = ap.parse_args()

    os.environ["GZ_PARTITION"] = f"dh{a.instance}"
    os.environ.setdefault("GZ_IP", "127.0.0.1")
    from gz.transport13 import Node
    from gz.msgs10.odometry_pb2 import Odometry

    lock = threading.Lock()
    state = {"t": None, "p": None, "R": None}

    def on_odom(m):
        with lock:
            state["t"] = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
            state["p"] = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
            o = m.pose.orientation
            state["R"] = quat_to_mat(o.w, o.x, o.y, o.z)            # body FLU -> world ENU

    node = Node()
    node.subscribe(Odometry, f"/model/{MODEL}/odometry", on_odom)
    mav = mavutil.mavlink_connection(f"udpout:127.0.0.1:{14280 + a.instance}", source_system=2, source_component=197)
    caster = RayCaster()
    rng = np.random.default_rng()
    sigma = math.log(a.ev_lat_p95 / a.ev_lat_p50) / 1.645           # lognormal latency
    print(f"sensor bridge: instance {a.instance}, EV {a.ev_rate:.0f} Hz latency p50 {a.ev_lat_p50 * 1e3:.0f} / "
          f"p95 {a.ev_lat_p95 * 1e3:.0f} ms, rangefinder {a.rng_rate:.0f} Hz, tube blackout {a.tube_blackout}", flush=True)

    anchor = None                 # (p0 NED, R_frd_ned) of the start pose
    queue = []                    # (deliver_at_sim_t, seq, payload)
    seq = 0
    next_ev = next_rng = None
    reset_counter = 0
    blind = False
    while True:
        with lock:
            t, p, R = state["t"], state["p"], state["R"]
        if t is None:
            time.sleep(0.01)
            continue
        if anchor is None:
            # odom frame = the body's FLU axes at power-up (yaw only), origin at the start base_link, like a VIO
            # frame that starts where the drone booted. PX4 then gets it the way the drone's odometry bridge sends
            # it: ENU->NED as (odom.y, odom.x, -odom.z) in an FRD (arbitrary heading) pose frame.
            psi0 = math.atan2(R[1, 0], R[0, 0])
            anchor = (p.copy(), rz(-psi0))
            next_ev = next_rng = t
            print(f"anchored VIO frame at world ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}), ENU heading {math.degrees(psi0):.1f} deg", flush=True)

        if t >= next_ev:
            next_ev += 1.0 / a.ev_rate
            in_tube = TUBE[0][0] < p[0] < TUBE[0][1] and TUBE[1][0] < p[1] < TUBE[1][1]
            if a.tube_blackout and in_tube:
                if not blind:
                    print(f"t={t:.2f} vision lost in the tube", flush=True)
                blind = True
            else:
                if blind:
                    reset_counter = (reset_counter + 1) % 256
                    print(f"t={t:.2f} vision back, reset_counter {reset_counter}", flush=True)
                blind = False
                p0, Ra = anchor
                odom_p = Ra @ (p - p0)                       # start-FLU position
                odom_R = Ra @ R                              # body FLU -> start-FLU
                pos = ENU_TO_NED @ odom_p + rng.normal(0, a.ev_noise, 3)
                Rb = ENU_TO_NED @ odom_R @ FLU_TO_FRD
                Rb = rz(math.radians(rng.normal(0, a.ev_yaw_noise_deg))) @ Rb
                lat = a.ev_lat_p50 * math.exp(sigma * rng.normal())
                heapq.heappush(queue, (t + lat, seq, (pos, mat_to_quat(Rb), reset_counter)))
                seq += 1

        while queue and queue[0][0] <= t:
            _, _, (pos, q, rc) = heapq.heappop(queue)
            nan = float("nan")
            pc = [0.0] * 21
            pc[0] = pc[6] = pc[11] = a.ev_noise ** 2
            pc[15] = pc[18] = pc[20] = math.radians(a.ev_yaw_noise_deg) ** 2
            mav.mav.odometry_send(
                int(t * 1e6), mavutil.mavlink.MAV_FRAME_LOCAL_FRD, mavutil.mavlink.MAV_FRAME_BODY_FRD,
                float(pos[0]), float(pos[1]), float(pos[2]), [float(v) for v in q],
                nan, nan, nan, nan, nan, nan, pc, [nan] * 21, rc, mavutil.mavlink.MAV_ESTIMATOR_TYPE_VIO, 100)

        if t >= next_rng:
            next_rng += 1.0 / a.rng_rate
            origin = p + R @ RNG_OFFSET_FLU
            d = caster.cast(origin, R @ np.array([0.0, 0.0, -1.0]))
            d = d + rng.normal(0, a.rng_noise) if np.isfinite(d) else np.inf
            cm = int(round(min(max(d, a.rng_min), a.rng_max + 0.05) * 100))
            mav.mav.distance_sensor_send(
                int(t * 1e3) & 0xFFFFFFFF, int(a.rng_min * 100), int(a.rng_max * 100), cm,
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, 0, mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, 0,
                0, 0, [0, 0, 0, 0], 100)
        time.sleep(0.002)


if __name__ == "__main__":
    main()
