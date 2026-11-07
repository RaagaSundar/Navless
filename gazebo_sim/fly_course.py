#!/usr/bin/env python3
"""Fly the DroneHacks course autonomously in Gazebo + PX4 SITL (start the sim first with ./run_sim.sh).

  python3 fly_course.py                      # default limits (conservative)
  python3 fly_course.py --v-max 3 --a-lat 5  # faster
  python3 fly_course.py --interface pos10    # what the real drone's flight_interface allows (start sim with DH_PX4_PROFILE=stock)

One start, then: arm -> offboard -> corridor -> maze -> Gate A -> tube -> window -> pad -> PX4 land.
--interface ff    (default) position + velocity + acceleration setpoints at 50 Hz over MAVLink
                  (SET_POSITION_TARGET_LOCAL_NED), timed by Gazebo's simulation clock: the sim-ideal follower.
--interface pos10 position-only setpoints at 10 Hz: a carrot along the planned path, its lead set by the
                  planned speed (lead = speed * --carrot-time, at least --carrot-min), poses floored at 0.3 m.
--interface vel10 velocity-only setpoints at 10 Hz: planned velocity along the path + a pull back onto it.
pos10/vel10 mirror the real drone's flight interface, which forwards only position-only or velocity-only at 10 Hz,
and steer on PX4's own position estimate, never on ground truth.
Ground truth (Gazebo odometry) and contact sensors are only used for logging and scoring, never for control:
PX4 flies on its own EKF (vision stand-in, no GPS).

Outputs runs/<timestamp>/: log.csv, summary.json, run.png
"""
import argparse
import csv
import json
import math
import os
import sys
import threading
import time

import numpy as np
from pymavlink import mavutil

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dh.geom import IN, HALF, Obstacles, clearance, map_to_world, world_to_ned  # noqa: E402
from dh.route import GROUND_Z, build_trajectory, window_route                # noqa: E402
from plan import add_limit_args, limits_from_args, plot                      # noqa: E402

PX4_OFFBOARD = 6
MODEL = "dh_quad_0"


# ------------------------------------------------------------------------------------ ground truth
class GroundTruth:
    """Gazebo odometry + contact sensors, read on gz-transport threads."""

    def __init__(self, model=MODEL):
        from gz.transport13 import Node
        from gz.msgs10.odometry_pb2 import Odometry
        from gz.msgs10.contacts_pb2 import Contacts
        self.node = Node()
        self.lock = threading.Lock()
        self.t = self.p = self.q = None
        self.contacts = []
        self._seen = set()
        self.node.subscribe(Odometry, f"/model/{model}/odometry", self._odom)
        for topic in ["/dh/contact/base"] + [f"/dh/contact/prop{i}" for i in range(4)]:
            self.node.subscribe(Contacts, topic, self._contact)

    def _odom(self, m):
        with self.lock:
            self.t = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
            self.p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
            o = m.pose.orientation
            self.q = (o.w, o.x, o.y, o.z)

    def _contact(self, m):
        with self.lock:
            st = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
            for c in m.contact:
                a, b = c.collision1.name, c.collision2.name
                other = b if MODEL in a else a
                if other.startswith("ground") or MODEL in other:
                    continue
                self.contacts.append((st, other))
                if other not in self._seen:
                    self._seen.add(other)
                    print(f"  !! CONTACT with {other} at sim t={st:.2f}", flush=True)

    def snapshot(self):
        with self.lock:
            return self.t, (None if self.p is None else self.p.copy()), self.q


def tilt_deg(q):
    w, x, y, z = q
    # angle between body z and world z
    bz = 1 - 2 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, bz))))


# ------------------------------------------------------------------------------------ PX4 link
class PX4:
    def __init__(self, url):
        self.m = mavutil.mavlink_connection(url, source_system=255, source_component=190)
        self.armed = False
        self.main_mode = None
        self.landed_state = None
        self.local = None        # NED, PX4's local frame
        self.yaw = None          # PX4 yaw in its local frame (rad)
        self.boot_ms = 0
        self.texts = []
        self._stop = False
        # world NED -> PX4 local: L = L0 + Rz(-dpsi) (W - W0). Identity until align() is called.
        self.W0, self.L0, self.dpsi = np.zeros(3), np.zeros(3), 0.0

    def align(self, world_pos, world_yaw_ned, local, yaw):
        """Tie PX4's local frame to the world from the known launch pose (drone sitting on the launch mark).
        local / yaw: PX4's position and heading averaged while it sits there. If PX4's frame already is the world
        frame (ideal estimator: EKF origin = world origin) use the exact identity; a start-anchored vision frame
        is rotated and offset, and the heading offset is snapped to 0.1 deg."""
        W0 = world_to_ned(world_pos)
        dpsi = math.atan2(math.sin(world_yaw_ned - yaw), math.cos(world_yaw_ned - yaw))
        if np.linalg.norm(local[:2] - W0[:2]) < 0.1 and abs(math.degrees(dpsi)) < 5:
            self.W0, self.L0, self.dpsi = np.zeros(3), np.zeros(3), 0.0
            return
        self.W0, self.L0, self.dpsi = W0, np.array(local, float), math.radians(round(math.degrees(dpsi), 1))

    def _rot(self, v, a):
        c, s = math.cos(a), math.sin(a)
        return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])

    def to_local(self, p_w):
        return self.L0 + self._rot(world_to_ned(p_w) - self.W0, -self.dpsi)

    def vec_to_local(self, v_w):
        return self._rot(world_to_ned(v_w), -self.dpsi)

    def connect(self, timeout=30):
        hb = self.m.wait_heartbeat(timeout=timeout)
        if hb is None:
            raise SystemExit("no heartbeat from PX4 (is ./run_sim.sh running?)")
        threading.Thread(target=self._heartbeats, daemon=True).start()
        # HEARTBEAT, LOCAL_POSITION_NED, EXTENDED_SYS_STATE, ATTITUDE
        for msg_id, hz in ((0, 10), (32, 50), (245, 10), (30, 50)):
            self.cmd(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)

    def _heartbeats(self):
        while not self._stop:
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            time.sleep(0.5)

    def cmd(self, command, *params):
        p = list(params) + [0] * (7 - len(params))
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, command, 0, *p)

    def poll(self):
        while True:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                return
            t = msg.get_type()
            if t == "HEARTBEAT" and msg.get_srcSystem() == self.m.target_system and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.main_mode = (msg.custom_mode >> 16) & 0xFF
            elif t == "LOCAL_POSITION_NED":
                self.local = np.array([msg.x, msg.y, msg.z])
                self.boot_ms = msg.time_boot_ms
            elif t == "EXTENDED_SYS_STATE":
                self.landed_state = msg.landed_state
            elif t == "ATTITUDE":
                self.yaw = msg.yaw
            elif t == "COMMAND_ACK" and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                # acks arrive immediately; heartbeats can be up to a period late
                if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and not self.armed:
                    self.armed = True
                elif msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE and self.main_mode != PX4_OFFBOARD:
                    self.main_mode = PX4_OFFBOARD
            elif t == "STATUSTEXT":
                self.texts.append(msg.text)
                print(f"  PX4: {msg.text}", flush=True)

    MASK_PVA = 0b0000100000000000   # position, velocity, acceleration, yaw (ignore yaw rate)
    MASK_POS = 0b0000100111111000   # position + yaw only
    MASK_VEL = 0b0000100111000111   # velocity + yaw only

    def setpoint(self, p_w, v_w, a_w, yaw, mask=MASK_PVA):
        """World-frame setpoint (and world NED yaw), converted into PX4's local frame."""
        z3 = np.zeros(3)
        n = self.to_local(p_w) if p_w is not None else z3
        v = self.vec_to_local(v_w) if v_w is not None else z3
        a = self.vec_to_local(a_w) if a_w is not None else z3
        self.m.mav.set_position_target_local_ned_send(
            self.boot_ms, self.m.target_system, self.m.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mask, *n, *v, *a, yaw - self.dpsi, 0.0)

    def local_world(self):
        """PX4's own position estimate in the world frame."""
        w = self.W0 + self._rot(self.local - self.L0, self.dpsi)
        return np.array([w[1], w[0], -w[2]])

    def set_param(self, name, value, timeout=3.0):
        """Set a float PX4 parameter and wait for PX4 to echo it back."""
        for _ in range(3):
            self.m.mav.param_set_send(self.m.target_system, self.m.target_component, name.encode(), float(value),
                                      mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            t_end = time.time() + timeout / 3
            while time.time() < t_end:
                msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
                if msg is not None and msg.param_id == name and abs(msg.param_value - value) < 1e-3:
                    return True
        return False

    def close(self):
        self._stop = True


# ------------------------------------------------------------------------------------ 10 Hz path following
class PathFollower:
    """Projects a position estimate onto the planned path and produces a carrot or a velocity command."""

    def __init__(self, traj, s_end):
        k = np.searchsorted(traj.s, s_end)
        self.s, self.p, self.yaw = traj.s[:k + 1], traj.p[:k + 1], traj.yaw[:k + 1]
        self.speed = traj.speed[:k + 1]
        tv = traj.v[:k + 1]
        self.tan = tv / np.maximum(np.linalg.norm(tv, axis=1), 1e-6)[:, None]
        self.s_proj = 0.0

    def at(self, s):
        i = int(np.clip(np.searchsorted(self.s, s), 0, len(self.s) - 1))
        return i

    def project(self, pos):
        lo, hi = np.searchsorted(self.s, [self.s_proj - 0.1, self.s_proj + 1.0])
        hi = max(hi, lo + 1)
        d = np.linalg.norm(self.p[lo:hi] - pos, axis=1)
        j = lo + int(np.argmin(d))
        self.s_proj = max(self.s_proj, float(self.s[j]))     # never move backwards along the route
        return self.s_proj, float(d.min())

    def carrot(self, pos, carrot_time, carrot_min, z_floor, max_dev=0.04):
        s, dev = self.project(pos)
        i = self.at(s)
        lead = max(carrot_min, self.speed[i] * carrot_time)
        # shrink the lead until the straight line to the carrot stays within max_dev of the path (no corner cutting)
        while lead > carrot_min:
            j = self.at(s + lead)
            seg = self.p[i:j + 1:max(1, (j - i) // 12)]
            a, b = self.p[i], self.p[j]
            ab = b - a
            L2 = float(ab @ ab)
            if L2 < 1e-9:
                break
            u = np.clip((seg - a) @ ab / L2, 0, 1)
            if np.max(np.linalg.norm(seg - (a + u[:, None] * ab), axis=1)) <= max_dev:
                break
            lead *= 0.8
        lead = max(lead, carrot_min)
        j = self.at(s + lead)
        c = self.p[j].copy()
        c[2] = max(c[2], z_floor)
        return c, float(self.yaw[j]), self.p[i], dev, s

    def velocity(self, pos, gain, v_cap):
        s, dev = self.project(pos)
        i = self.at(s + 0.05)
        v = self.speed[i] * self.tan[i] + gain * (self.p[i] - pos)
        n = np.linalg.norm(v)
        if n > v_cap:
            v *= v_cap / n
        return v, float(self.yaw[i]), self.p[i], dev, s

    def done(self, pos, tol=0.12):
        return self.s_proj >= self.s[-1] - 0.05 and np.linalg.norm(pos[:2] - self.p[-1][:2]) < tol


# ------------------------------------------------------------------------------------ scoring
def crossing(P, T, axis, value, lo, hi, other_axis):
    """First time the path crosses coordinate[axis] == value with the other coordinate in [lo, hi]."""
    s = P[:, axis] - value
    for i in range(len(s) - 1):
        if s[i] == 0 or s[i] * s[i + 1] < 0:
            f = s[i] / (s[i] - s[i + 1]) if s[i] != s[i + 1] else 0
            o = P[i, other_axis] + f * (P[i + 1, other_axis] - P[i, other_axis])
            if lo <= o <= hi:
                return float(T[i] + f * (T[i + 1] - T[i]))
    return None


def score(P, T, course):
    W = lambda v: v * IN - HALF
    ev = {}
    ev["corridor_done"] = crossing(P, T, 1, W(203.5), W(0), W(42), 0)          # past the end of M1 in corridor 1
    ev["gate_a"] = crossing(P, T, 0, W(132), W(203.5), W(240), 1)              # maze exit
    ev["tube_in"] = crossing(P, T, 1, W(168), W(182), W(240), 0)
    ev["tube_out"] = crossing(P, T, 1, W(108), W(182), W(240), 0)
    gb = course["gates_in"]["gate_B"]["between"]                               # C2b north end -> barrier
    t_gb = crossing(P, T, 0, W(gb[0][0]), W(gb[0][1]), W(gb[1][1]), 1)
    if t_gb is not None:
        ev["gate_b"] = t_gb
    # window: rotate into the wall frame and test the crossing inside the opening
    ww = next(w for w in course["walls"] if "window" in w)
    a, b = np.array(map_to_world(*ww["start_in"])), np.array(map_to_world(*ww["end_in"]))
    u = (b - a) / np.linalg.norm(b - a)
    nrm = np.array([-u[1], u[0]])
    c = np.array(map_to_world(*ww["window"]["center_in"]))
    along, across = (P[:, :2] - c) @ u, (P[:, :2] - c) @ nrm
    Q = np.stack([along, across, P[:, 2]], 1)
    hw = ww["window"]["width_in"] * IN / 2
    t_win = crossing(Q, T, 1, 0.0, -hw, hw, 0)
    if t_win is not None:
        i = int(np.searchsorted(T, t_win))
        ev["window"] = t_win
        ev["window_offset_along_cm"] = round(float(along[i]) * 100, 1)
        ev["window_height_m"] = round(float(P[i, 2]), 3)
    return ev


# ------------------------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_limit_args(ap)
    ap.add_argument("--instance", type=int, default=int(os.environ.get("DH_INSTANCE", 0)),
                    help="sim instance started with DH_INSTANCE=N ./run_sim.sh (sets GZ_PARTITION and MAVLink port)")
    ap.add_argument("--url", default=None, help="MAVLink url (default udpin:0.0.0.0:<14540 + instance>)")
    ap.add_argument("--interface", choices=("ff", "pos10", "vel10"), default="ff")
    ap.add_argument("--carrot-time", type=float, default=1.0, help="pos10: carrot lead = planned speed x this (s); ~1/MPC_XY_P")
    ap.add_argument("--carrot-min", type=float, default=0.15, help="pos10: minimum carrot lead (m)")
    ap.add_argument("--carrot-dev", type=float, default=0.04, help="pos10: max distance between the carrot line and the path (m)")
    ap.add_argument("--vel-gain", type=float, default=1.5, help="vel10: pull-back gain onto the path (1/s)")
    ap.add_argument("--z-floor", type=float, default=0.3, help="pos10: position setpoints are floored here (flight_interface)")
    ap.add_argument("--lead", type=float, default=0.0, help="position/yaw setpoint time lead (s)")
    ap.add_argument("--vel-lead", type=float, default=0.05, help="velocity feedforward lead (s)")
    ap.add_argument("--acc-lead", type=float, default=0.12, help="acceleration feedforward lead (s): covers PX4's tilt response delay")
    ap.add_argument("--abort-error", type=float, default=0.45, help="land if tracking error exceeds this (m)")
    ap.add_argument("--spoolup", type=float, default=1.1, help="hold after arming before the trajectory starts (s)")
    ap.add_argument("--tilt-max", type=float, default=None,
                    help="set PX4 MPC_TILTMAX_AIR/LND (deg) before arming; presets race and max set 55")
    ap.add_argument("--tag", default="")
    a = ap.parse_args()
    from plan import PRESETS
    tilt = a.tilt_max if a.tilt_max is not None else PRESETS.get(a.preset or "", {}).get("tilt")

    course = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "course", "course_v3.json")))
    lim = limits_from_args(a)
    wps, zones = window_route()
    traj = build_trajectory(wps, zones, lim)
    print(f"plan: {traj.length:.2f} m, {traj.duration:.2f} s, peak {traj.speed.max():.2f} m/s  ({lim})", flush=True)

    stamp = time.strftime("%Y%m%d-%H%M%S") + (f"-{a.tag}" if a.tag else "")
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs", stamp)
    os.makedirs(out, exist_ok=True)

    os.environ["GZ_PARTITION"] = f"dh{a.instance}"      # must be set before the gz Node is created
    os.environ.setdefault("GZ_IP", "127.0.0.1")
    gt = GroundTruth()
    px4 = PX4(a.url or f"udpin:0.0.0.0:{14540 + a.instance}")
    drone_like = a.interface != "ff"
    follower = PathFollower(traj, traj.wp_s["pad_above"]) if drone_like else None
    send_every = 5 if drone_like else 1                  # 10 Hz vs 50 Hz
    px4.connect()
    if tilt is not None:
        ok = px4.set_param("MPC_TILTMAX_AIR", tilt) and px4.set_param("MPC_TILTMAX_LND", tilt)
        print(f"PX4 tilt limit set to {tilt:.0f} deg" + ("" if ok else " (NOT confirmed)"), flush=True)
    print("connected to PX4; waiting for EKF position and ground truth", flush=True)
    t_end = time.time() + 30
    while time.time() < t_end:
        px4.poll()
        st, p, q = gt.snapshot()
        if px4.local is not None and px4.yaw is not None and p is not None and px4.landed_state is not None:
            break
        time.sleep(0.05)
    else:
        raise SystemExit("timed out waiting for PX4 local position / Gazebo odometry")

    st, p, q = gt.snapshot()
    start_err = np.linalg.norm(p[:2] - np.array(wps[0].p[:2]))
    print(f"vehicle at world ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}), {start_err * 100:.1f} cm from the launch mark", flush=True)
    # Known launch pose -> PX4 local frame (the real drone gets this from the launch mark and its start bearing).
    # Average PX4's position and heading for a second while it sits still.
    w, x, y, z = q
    yaw_ned = math.pi / 2 - math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    locs, yaws = [], []
    t_end = time.time() + 1.0
    while time.time() < t_end:
        px4.poll()
        locs.append(px4.local.copy())
        yaws.append(px4.yaw)
        time.sleep(0.02)
    yaw_avg = math.atan2(np.mean(np.sin(yaws)), np.mean(np.cos(yaws)))
    px4.align(p, yaw_ned, np.mean(locs, axis=0), yaw_avg)
    print(f"PX4 local frame: origin offset {np.round(px4.L0 - px4.W0, 3)} m, heading offset {math.degrees(px4.dpsi):.1f} deg", flush=True)

    # ---- the one start: stream a hold setpoint, switch to offboard, arm
    # Hold = launch point 5 cm below the vehicle with no feedforward, so PX4 cannot lift off before the clock starts.
    y0 = traj.sample(0.0)[3]
    ground_z = p[2]
    p_hold = np.array([p[0], p[1], p[2] - 0.05])
    zero = np.zeros(3)
    rows = []
    phase = "arming"
    t0 = None
    t_arm_wall = None
    abort_reason = None
    t_land_cmd = None
    period = 0.02
    next_tick = t_start = time.time()
    sent_mode = sent_arm = 0
    tick = 0
    while True:
        px4.poll()
        st, p, q = gt.snapshot()
        now = time.time()
        tick += 1
        send = tick % send_every == 0

        def follow():
            """pos10 / vel10 command from PX4's own estimate; returns (path point, deviation, yaw)."""
            est = px4.local_world()
            if a.interface == "pos10":
                c, yw, pp, dev, _ = follower.carrot(est, a.carrot_time, a.carrot_min, a.z_floor, a.carrot_dev)
                if send:
                    px4.setpoint(c, None, None, yw, PX4.MASK_POS)
            else:
                v, yw, pp, dev, _ = follower.velocity(est, a.vel_gain, max(lim.v_max, 0.5))
                if send:
                    px4.setpoint(None, v, None, yw, PX4.MASK_VEL)
            return pp, yw

        if phase == "arming":
            if send:
                px4.setpoint(p_hold, zero, zero, y0, PX4.MASK_POS if drone_like else PX4.MASK_PVA)
            if now - t_start > 20:
                raise SystemExit("could not arm / enter offboard in 20 s; PX4 said: " + " | ".join(px4.texts[-5:]))
            if now - t_start > 0.5 and sent_mode < now - 1.0 and px4.main_mode != PX4_OFFBOARD:
                px4.cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, PX4_OFFBOARD)
                sent_mode = now
            if px4.main_mode == PX4_OFFBOARD and not px4.armed and sent_arm < now - 1.0:
                px4.cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
                sent_arm = now
            if px4.armed and px4.main_mode == PX4_OFFBOARD:
                t_arm_sim = st
                t_arm_wall = now
                phase = "spoolup"
                print(f"armed in offboard at sim t={st:.2f}: motors spooling up", flush=True)

        elif phase == "spoolup":
            # PX4 holds the motors at idle for COM_SPOOLUP_TIME after arming
            if send:
                px4.setpoint(p_hold, zero, zero, y0, PX4.MASK_POS if drone_like else PX4.MASK_PVA)
            if st - t_arm_sim >= a.spoolup:
                phase = "liftoff"
                t_lift_cmd = st

        elif phase == "liftoff":
            # give PX4 the first climb setpoint and start the clock when the feet leave the ground
            if drone_like:
                follow()
            else:
                sp_p, _, _, sp_y = traj.sample(0.0)
                px4.setpoint(sp_p + np.array([0, 0, 0.1]), traj.sample(a.vel_lead)[1], traj.sample(a.acc_lead)[2], sp_y)
            if p[2] > ground_z + 0.01 or st - t_lift_cmd > (12.0 if drone_like else 2.0):
                t0 = st
                phase = "fly"
                print(f"liftoff {st - t_arm_sim:.2f} s after arming: flying", flush=True)

        elif phase == "brake":
            # abort: hold where the EKF says we are, zero velocity, then land
            if send:
                px4.setpoint(p_brake, zero, zero, yaw_brake, PX4.MASK_POS if drone_like else PX4.MASK_PVA)
            tt = st - t0
            rows.append([st, tt, *traj.sample(tt)[0], *p, tilt_deg(q), yaw_brake, *(px4.local if px4.local is not None else [np.nan] * 3),
                         float("nan"), int(px4.armed), px4.main_mode, px4.landed_state])
            if st - t_brake > 1.5:
                px4.cmd(mavutil.mavlink.MAV_CMD_NAV_LAND)
                phase, t_land_cmd = "landing", st

        elif phase == "fly":
            tt = st - t0
            if drone_like:
                ref, sp_y = follow()
            else:
                sp_p, _, _, sp_y = traj.sample(tt + a.lead)
                sp_v = traj.sample(tt + a.vel_lead)[1]
                sp_a = traj.sample(tt + a.acc_lead)[2]
                px4.setpoint(sp_p, sp_v, sp_a, sp_y)
                ref = traj.sample(tt)[0]
            err = float(np.linalg.norm(p - ref))
            rows.append([st, tt, *ref, *p, tilt_deg(q), sp_y, *(px4.local if px4.local is not None else [np.nan] * 3),
                         err, int(px4.armed), px4.main_mode, px4.landed_state])
            if err > a.abort_error and tt > 0.5:
                abort_reason = f"tracking error {err:.2f} m at t={tt:.2f}s"
            elif tilt_deg(q) > 70:
                abort_reason = f"tilt {tilt_deg(q):.0f} deg at t={tt:.2f}s"
            if abort_reason:
                print("ABORT:", abort_reason, "-> braking to a hover, then land", flush=True)
                p_brake, yaw_brake, t_brake = px4.local_world(), sp_y, st
                phase = "brake"
            elif (follower.done(px4.local_world()) if drone_like else tt >= traj.duration):
                px4.cmd(mavutil.mavlink.MAV_CMD_NAV_LAND)
                phase, t_land_cmd = "landing", st
                print(f"route done at t={tt:.2f}s: PX4 land", flush=True)

        elif phase == "landing":
            tt = st - t0
            rows.append([st, tt, *traj.sample(tt)[0], *p, tilt_deg(q), float("nan"), *(px4.local if px4.local is not None else [np.nan] * 3),
                         float("nan"), int(px4.armed), px4.main_mode, px4.landed_state])
            if not px4.armed:
                print(f"disarmed at t={tt:.2f}s", flush=True)
                break
            if st - t_land_cmd > 15:
                print("still armed 15 s after land command", flush=True)
                break

        next_tick += period
        time.sleep(max(0.0, next_tick - time.time()))

    # ---- settle and summarise
    time.sleep(0.5)
    px4.close()
    cols = ["sim_t", "t", "sp_x", "sp_y", "sp_z", "x", "y", "z", "tilt_deg", "sp_yaw", "px4_n", "px4_e", "px4_d",
            "err", "armed", "mode", "landed_state"]
    R = np.array(rows, float)
    T, P = R[:, 1], R[:, 5:8]
    yaw = np.where(np.isfinite(R[:, 9]), R[:, 9], 0.3491)
    obs = Obstacles()
    flying = P[:, 2] > 0.2
    clr, names = clearance(obs, P, yaw)
    clr = np.where(flying, clr, np.inf)
    ev = score(P, T, course)
    pad = np.array(map_to_world(*course["landing_pad_in"]["center"]))
    # touchdown: first sample after the descent began where base_link is back at resting height
    td = None
    desc = T > ((t_land_cmd - t0 - 1.0) if t_land_cmd is not None else np.inf)
    for i in np.where(desc & (P[:, 2] < GROUND_Z + 0.012))[0]:
        td = i
        break
    contacts = sorted({n for (_, n) in gt.contacts})
    fly_err = R[:, 13][np.isfinite(R[:, 13])]
    summary = {
        "result": "ABORTED: " + abort_reason if abort_reason else ("COLLISION" if contacts else "CLEAN RUN"),
        "interface": a.interface,
        "px4_profile": os.environ.get("DH_PX4_PROFILE", "sim"),
        "limits": lim.__dict__,
        "planned_time_s": round(traj.duration, 2),
        "planned_length_m": round(traj.length, 2),
        "touchdown_time_s": None if td is None else round(float(T[td]), 2),
        "touchdown_time_from_arm_s": None if td is None else round(float(T[td]) + (t0 - t_arm_sim), 2),
        "touchdown_offset_from_pad_cm": None if td is None else round(float(np.linalg.norm(P[td, :2] - pad)) * 100, 1),
        "checkpoints_s": {k: (round(v, 2) if isinstance(v, float) and k in ("corridor_done", "gate_a", "tube_in", "tube_out", "gate_b", "window") else v)
                          for k, v in ev.items()},
        "min_clearance_cm": round(float(clr.min()) * 100, 1),
        "min_clearance_at": names[int(np.argmin(clr))],
        "tracking_error_cm": {"mean": round(float(fly_err.mean()) * 100, 1), "p95": round(float(np.percentile(fly_err, 95)) * 100, 1),
                              "max": round(float(fly_err.max()) * 100, 1)},
        "max_tilt_deg": round(float(np.nanmax(R[:, 8])), 1),
        "max_speed_mps": round(float(np.max(np.linalg.norm(np.diff(P, axis=0), axis=1) / np.maximum(np.diff(T), 1e-3))), 2),
        "contacts": contacts,
    }
    with open(os.path.join(out, "log.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(cols + ["clearance"])
        for r, c in zip(rows, clr):
            wr.writerow([f"{v:.4f}" if isinstance(v, float) else v for v in r] + [f"{c:.4f}"])
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    plot(traj, obs, os.path.join(out, "run.png"), run=P)
    print(json.dumps(summary, indent=2))
    print(f"run saved to {out}")


if __name__ == "__main__":
    main()
