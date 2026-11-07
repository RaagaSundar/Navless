#!/usr/bin/env python3
"""Feed PX4 SITL the real drone's sensor path. run_sim.sh starts this with DH_PX4_PROFILE=drone.

Visual odometry
  Gazebo pose -> MAVLink ODOMETRY at 30 Hz. The odom frame is the body's FLU axes at power-up (origin = start
  base_link), sent to PX4 as the drone's odometry bridge does: (odom.y, odom.x, -odom.z), FRD pose frame. Each sample is delivered after a random transport latency (lognormal,
  p50 60 ms / p95 106 ms by default, the measured image -> flight controller chain), with small noise. PX4 stamps
  it on arrival and EKF2_EV_DELAY takes the latency out; EKF2_EV_CTRL=9 fuses horizontal position + yaw only.
  --tube-blackout stop|freeze (default DH_TUBE_BLACKOUT): inside the dark tube no vision reaches PX4 (stop) or
  the last tracked pose repeats (freeze), as the drone's bridge forwards whatever cuVSLAM publishes; reset_counter
  goes up when tracking comes back. DH_TUBE_RECOVER_DELAY / DH_TUBE_RESET_JUMP add a late recovery and a pose
  jump (dh/tube.py). fly_navless.py reads the same variables, so the node's odometry does exactly the same.
Rangefinder
  A downward laser (2.7 cm left of / 3.4 cm below the body origin) ray-cast against the course collision model:
  MAVLink DISTANCE_SENSOR at 40 Hz, 0.02-4 m, 1 cm noise. The EKF uses it as the height reference while low and slow
  (EKF2_HGT_REF=2, EKF2_RNG_CTRL=1), with the barometer behind it.

  python3 sensor_bridge.py [--instance N] [--tube-blackout [stop|freeze]] [--ev-lat-p50 0.06 --ev-lat-p95 0.106]
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
from dh.geom import RayCaster  # noqa: E402
from dh.tube import MODES, TubeBlackout, env_settings  # noqa: E402

MODEL = "dh_quad_0"
RNG_OFFSET_FLU = np.array([0.0, 0.027432, -0.03445])     # measured mount: FRD y -0.027, z +0.034


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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--instance", type=int, default=int(os.environ.get("DH_INSTANCE", 0)))
    ap.add_argument("--ev-rate", type=float, default=30.0)
    ap.add_argument("--ev-lat-p50", type=float, default=0.060)
    ap.add_argument("--ev-lat-p95", type=float, default=0.106)
    ap.add_argument("--ev-noise", type=float, default=0.005, help="position noise (m, 1 sigma)")
    ap.add_argument("--ev-yaw-noise-deg", type=float, default=0.3)
    env_mode, env_delay, env_jump = env_settings()
    ap.add_argument("--tube-blackout", choices=MODES, nargs="?", const="stop", default=env_mode,
                    help="camera blind in the tube (default DH_TUBE_BLACKOUT; bare flag = stop)")
    ap.add_argument("--tube-recover-delay", type=float, default=env_delay,
                    help="s after leaving the tube before tracking returns (default DH_TUBE_RECOVER_DELAY)")
    ap.add_argument("--tube-reset-jump", default=env_jump, metavar="DX,DY,DZ|resume",
                    help="odom-frame pose jump when tracking returns (default DH_TUBE_RESET_JUMP)")
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
          f"p95 {a.ev_lat_p95 * 1e3:.0f} ms, rangefinder {a.rng_rate:.0f} Hz", flush=True)

    anchor = None                 # (p0 NED, R_frd_ned) of the start pose
    queue = []                    # (deliver_at_sim_t, seq, payload)
    seq = 0
    next_ev = next_rng = None
    reset_counter = 0
    tube = R_last = None
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
            tube = TubeBlackout(a.tube_blackout, psi0, a.tube_recover_delay, a.tube_reset_jump,
                                log=lambda text: print(text, flush=True))
            print(f"tube blackout: {tube.describe()}", flush=True)

        if t >= next_ev:
            next_ev += 1.0 / a.ev_rate
            was_tracking = tube.tracking
            tracking, pr = tube.step(t, p)               # pr: the world position cuVSLAM reports, None = nothing
            if tracking:
                R_last = R
                if not was_tracking:                     # the drone's bridge: every lost -> tracking transition
                    reset_counter = (reset_counter + 1) % 256
                    print(f"t={t:.2f} reset_counter {reset_counter}", flush=True)
            if pr is not None:
                p0, Ra = anchor
                odom_p = Ra @ (pr - p0)                      # start-FLU position
                odom_R = Ra @ R_last                         # body FLU -> start-FLU (frozen with the pose)
                pos = ENU_TO_NED @ odom_p + (rng.normal(0, a.ev_noise, 3) if tracking else 0.0)
                Rb = ENU_TO_NED @ odom_R @ FLU_TO_FRD
                if tracking:
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
