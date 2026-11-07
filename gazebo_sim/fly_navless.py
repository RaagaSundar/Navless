#!/usr/bin/env python3
"""Fly the real competition node (navless_nav) through the DroneHacks course in Gazebo + PX4 SITL.

  ./run_sim.sh                                   # start the sim first (DH_INSTANCE=N for a parallel one)
  python3 fly_navless.py                         # route_gate_b.json with navless_nav's own defaults
  python3 fly_navless.py --instance 1 --tag fast -p cruise_speed_mps=0.8 -p max_lead_m=0.8
  python3 fly_navless.py --route route_window.json --noise 0.01 --vio-dropout 20:23

navless_nav is a ROS 2 node and this machine has no ROS, so the node runs UNMODIFIED on a small
in-process rclpy stand-in. FlightInterface below stands in for the stack's flight_interface. It
sends PX4 exactly what the drone's interface sends:
  - position-only setpoints (no yaw), or velocity-only ones, at 10 Hz
  - altitude clamped to [min_alt, max_alt] above the odometry origin, twist norm capped
  - a twist expires after 1 s and falls back to the last pose
  - one takeover per run: a fresh setpoint, landed, local position valid, then Offboard. On the
    drone the pilot arms, then switches to Position; in SITL nobody does, so this sends offboard + arm.
Odometry is Gazebo ground truth in the power-up frame: origin and heading at start, like cuVSLAM.
Noise and latency are optional. Scoring reuses fly_course.py's (read-only import).

Outputs runs/<timestamp>-navless[-tag]/: log.csv, summary.json
"""
import argparse
import collections
import csv
import json
import math
import os
import random
import sys
import time
import types

import numpy as np
from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "navless_nav"))

ODOM_TOPIC = "/visual_slam/tracking/odometry"
STATUS_TOPIC = "/visual_slam/status"
PX4_OFFBOARD = 6
LANDED_ON_GROUND = 1


# ------------------------------------------------------------------------------------ rclpy stand-in
class _Clock:
    t = 0.0          # simulation time, s (Gazebo's clock)


CLOCK = _Clock()
BUS = collections.defaultdict(list)      # topic -> callbacks
TIMERS = []                              # [period, callback, last_fired]
OVERRIDES = {}                           # ROS parameter overrides for the node
LOGS = []                                # (t, level, message) from the node


def _ns(**kw):
    return types.SimpleNamespace(**kw)


def _vec():
    return _ns(x=0.0, y=0.0, z=0.0)


class PoseStamped:
    def __init__(self):
        self.header = _ns(stamp=None, frame_id="")
        self.pose = _ns(position=_vec(), orientation=_ns(x=0.0, y=0.0, z=0.0, w=0.0))


class TwistStamped:
    def __init__(self):
        self.header = _ns(stamp=None, frame_id="")
        self.twist = _ns(linear=_vec(), angular=_vec())


class Odometry:
    def __init__(self):
        self.header = _ns(stamp=None, frame_id="odom")
        self.pose = _ns(pose=_ns(position=_vec(), orientation=_ns(x=0.0, y=0.0, z=0.0, w=1.0)))


class String:
    def __init__(self, data=""):
        self.data = data


class VisualSlamStatus:
    def __init__(self, vo_state=1):
        self.vo_state = vo_state


class _Time:
    def __init__(self, t):
        self.nanoseconds = int(round(t * 1e9))
        self._t = t

    def to_msg(self):
        return self._t


class _Logger:
    def __init__(self, name):
        self.name = name

    def _log(self, level, msg):
        LOGS.append((CLOCK.t, level, msg))
        print(f"  [{CLOCK.t:7.2f}] {self.name} {level}: {msg}", flush=True)

    def info(self, msg):
        self._log("INFO", msg)

    def warn(self, msg):
        self._log("WARN", msg)

    warning = warn

    def error(self, msg):
        self._log("ERROR", msg)


class _Publisher:
    def __init__(self, topic):
        self.topic = topic

    def publish(self, msg):
        publish(self.topic, msg)


def publish(topic, msg):
    for cb in list(BUS[topic]):
        cb(msg)


class Node:
    def __init__(self, name, **kwargs):
        self._name = name
        self._params = {}

    def declare_parameter(self, name, value=None, descriptor=None, ignore_override=False):
        self._params[name] = _ns(value=OVERRIDES.get(name, value))
        return self._params[name]

    def get_parameter(self, name):
        return self._params[name]

    def create_publisher(self, msg_type, topic, qos):
        return _Publisher(topic)

    def create_subscription(self, msg_type, topic, callback, qos):
        BUS[topic].append(callback)

    def create_timer(self, period, callback):
        TIMERS.append([float(period), callback, -1e9])

    def get_clock(self):
        return _ns(now=lambda: _Time(CLOCK.t))

    def get_logger(self):
        return _Logger(self._name)

    def destroy_node(self):
        pass


def install_ros_standin():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    mod("rclpy", init=lambda args=None: None, ok=lambda: False, spin=lambda n: None, shutdown=lambda: None)
    sys.modules["rclpy"].node = mod("rclpy.node", Node=Node)
    for pkg, attrs in (("geometry_msgs", dict(PoseStamped=PoseStamped, TwistStamped=TwistStamped)),
                       ("nav_msgs", dict(Odometry=Odometry)),
                       ("std_msgs", dict(String=String)),
                       ("rcl_interfaces", dict(ParameterDescriptor=lambda **kw: kw)),
                       ("isaac_ros_visual_slam_interfaces", dict(VisualSlamStatus=VisualSlamStatus))):
        sys.modules[pkg] = mod(pkg)
        sys.modules[pkg].msg = mod(pkg + ".msg", **attrs)


# ------------------------------------------------------------------------------------ flight interface
class FlightInterface:
    """The drone's flight_interface, re-implemented against PX4 SITL over MAVLink."""

    # SET_POSITION_TARGET_LOCAL_NED type_mask: a set bit means "ignore that field"
    MASK_POS = 0b110111111000    # x y z only: yaw and yaw rate ignored (yaw = NaN on the drone)
    MASK_VEL = 0b010111000111    # vx vy vz + yaw rate

    def __init__(self, px4, p0, yaw0, pose_topic, twist_topic, min_alt=0.3, max_alt=2.5,
                 max_speed=1.0, twist_timeout=1.0, px4_frame="world"):
        self.px4, self.p0, self.yaw0 = px4, np.asarray(p0, float), yaw0
        self.px4_frame = px4_frame
        self.min_alt, self.max_alt, self.max_speed = min_alt, max_alt, max_speed
        self.twist_timeout = twist_timeout
        self.mode = "hold"
        self.target = self.hold = self.vel = None     # world ENU
        self.yawspeed_ned = 0.0
        self.last_setpoint = self.last_twist = None
        self.clamped = []
        self.saw_disarmed = False
        self.takeover_at = None       # sim time the takeover started (stand-in for the pilot's arm)
        self.height_settled_since = None
        self.height_err = None
        self.armed_at = None
        self._last_cmd = -1e9
        BUS[pose_topic].append(self.on_pose)
        BUS[twist_topic].append(self.on_twist)

    # odom (power-up frame: x = nose at start) -> world ENU
    def rot(self, v):
        c, s = math.cos(self.yaw0), math.sin(self.yaw0)
        return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])

    def unrot(self, v):
        c, s = math.cos(self.yaw0), math.sin(self.yaw0)
        return np.array([c * v[0] + s * v[1], -s * v[0] + c * v[1], v[2]])

    def world_to_px4(self, v, velocity=False):
        """World ENU -> PX4 local NED. 'world': EKF origin = world origin (sim/stock
        profiles, vision = Gazebo pose). 'start': drone profile, where sensor_bridge.py
        sends the odom frame (FLU at power-up) as (odom.y, odom.x, -odom.z), like the
        drone's odometry bridge. The flight interface's plain ENU->NED map of an odom
        setpoint is then exactly right, which is what this does."""
        if self.px4_frame == "world":
            return [v[1], v[0], -v[2]]
        q = self.unrot(v if velocity else np.asarray(v) - self.p0)    # world -> odom
        return [q[1], q[0], -q[2]]

    def px4_to_world(self, n):
        if self.px4_frame == "world":
            return np.array([n[1], n[0], -n[2]])
        return self.p0 + self.rot([n[1], n[0], -n[2]])

    def on_pose(self, msg):
        p = msg.pose.position
        q = [float(p.x), float(p.y), float(p.z)]
        if not all(math.isfinite(v) for v in q):
            return
        self.clamped = []
        if q[2] > self.max_alt:
            q[2], self.clamped = self.max_alt, ["max_altitude"]
        elif q[2] < self.min_alt:
            q[2], self.clamped = self.min_alt, ["min_altitude"]
        self.target = self.hold = self.p0 + self.rot(q)
        self.mode, self.last_setpoint = "position", CLOCK.t

    def on_twist(self, msg):
        v = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z], float)
        if not np.all(np.isfinite(v)):
            return
        n = float(np.linalg.norm(v))
        self.clamped = []
        if n > self.max_speed:
            v, self.clamped = v * (self.max_speed / n), ["max_speed"]
        self.vel = self.rot(v)
        wz = float(msg.twist.angular.z)
        self.yawspeed_ned = -wz if math.isfinite(wz) else 0.0
        self.mode, self.last_setpoint = "velocity", CLOCK.t
        self.last_twist = CLOCK.t

    @property
    def offboard_active(self):
        return bool(self.px4.armed and self.px4.main_mode == PX4_OFFBOARD)

    def local_world(self):
        loc = self.px4.local
        return None if loc is None else self.px4_to_world(loc)

    def commanded(self):
        """(mode, world ENU position or velocity) that this tick sends."""
        if self.mode == "velocity" and CLOCK.t - self.last_twist >= self.twist_timeout:
            self.mode = "hold"
        if self.mode == "velocity":
            return "velocity", self.vel
        if self.mode == "position" and self.target is not None:
            return "position", self.target
        where = self.hold if self.hold is not None else self.local_world()
        return ("position", where) if where is not None else ("velocity", np.zeros(3))

    def tick(self):
        kind, v = self.commanded()
        m, px4 = self.px4.m, self.px4
        ned = self.world_to_px4(v, velocity=(kind == "velocity"))
        if kind == "position":
            m.mav.set_position_target_local_ned_send(
                px4.boot_ms, m.target_system, m.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                self.MASK_POS, *ned, 0, 0, 0, 0, 0, 0, 0, 0)
        else:
            m.mav.set_position_target_local_ned_send(
                px4.boot_ms, m.target_system, m.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                self.MASK_VEL, 0, 0, 0, *ned, 0, 0, 0, 0, self.yawspeed_ned)
        self.step_takeover()
        return kind, v

    def step_takeover(self):
        px4 = self.px4
        if not px4.armed and self.takeover_at is None:
            self.saw_disarmed = True
        if self.takeover_at is None:
            fresh = self.last_setpoint is not None and CLOCK.t - self.last_setpoint <= 1.0
            # PX4's height can boot ~0.6 m off and park the error in its terrain estimate; with
            # EV_CTRL 9 nothing pulls it back, and every odom-z setpoint is then flown that far low
            # (lap 2 of the live demo). Hand over only once PX4 agrees the drone is on the floor.
            if px4.local is not None:
                dz = abs(self.px4_to_world(px4.local)[2] - self.p0[2])
                self.height_settled_since = ((self.height_settled_since or CLOCK.t) if dz < 0.05 else None)
                self.height_err = dz
            settled = (self.height_settled_since is not None
                       and CLOCK.t - self.height_settled_since >= 2.0)
            if (self.saw_disarmed and fresh and settled and px4.landed_state == LANDED_ON_GROUND
                    and px4.local is not None):
                self.takeover_at = CLOCK.t
                print(f"  [{CLOCK.t:7.2f}] flight_interface: takeover (stand-in for arm -> Position)", flush=True)
            return
        if self.armed_at is None and px4.armed and px4.main_mode == PX4_OFFBOARD:
            self.armed_at = CLOCK.t
        if self.armed_at is not None or CLOCK.t - self.takeover_at > 10.0:
            return
        if CLOCK.t - self._last_cmd < 1.0:
            return
        self._last_cmd = CLOCK.t
        if px4.main_mode != PX4_OFFBOARD:
            px4.cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, PX4_OFFBOARD)
        elif not px4.armed:
            px4.cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)

    def state_json(self):
        loc = self.local_world()
        return json.dumps({
            "stamp": round(CLOCK.t, 3), "px4_link": True, "armed": self.px4.armed,
            "landed": self.px4.landed_state == LANDED_ON_GROUND,
            "nav_state_name": "OFFBOARD" if self.px4.main_mode == PX4_OFFBOARD else str(self.px4.main_mode),
            "offboard_active": self.offboard_active,
            # The real flight_interface reports ned_to_enu(PX4 local), which on the drone is the odom frame.
            # Report PX4's estimate in odom too (exactly ned_to_enu in the drone profile), not in world
            # coordinates, which carried a constant +0.07 m (the base_link height) into height checks.
            "local_enu": None if loc is None else [float(x) for x in self.unrot(loc - self.p0)],
            "setpoint_mode": self.mode, "clamped": self.clamped, "frozen": False,
            "envelope": {"min_altitude_m": self.min_alt, "max_altitude_m": self.max_alt,
                         "max_speed_mps": self.max_speed},
        })


# ------------------------------------------------------------------------------------ helpers
def yaw_enu(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def parse_override(text):
    name, _, raw = text.partition("=")
    if not name or not _:
        raise SystemExit(f"-p wants name=value, got {text!r}")
    try:
        return name, json.loads(raw)
    except ValueError:
        return name, raw


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--instance", type=int, default=int(os.environ.get("DH_INSTANCE", 0)),
                    help="sim instance from DH_INSTANCE=N ./run_sim.sh (gz partition dhN, MAVLink 14540+N)")
    ap.add_argument("--route", default="route_gate_b.json", help="planner route file (repo root or absolute)")
    ap.add_argument("-p", dest="params", action="append", default=[], metavar="NAME=VALUE",
                    help="navless_nav ROS parameter override, repeatable")
    ap.add_argument("--min-alt", type=float, default=0.3, help="flight_interface floor (launch file default)")
    ap.add_argument("--max-alt", type=float, default=2.5, help="flight_interface ceiling (navless.launch.py)")
    ap.add_argument("--max-speed", type=float, default=1.0, help="twist cap (navless.launch.py)")
    ap.add_argument("--px4-frame", choices=("world", "start"),
                    default="start" if os.environ.get("DH_PX4_PROFILE") == "drone" else "world",
                    help="PX4's local frame: 'world' for the sim/stock profiles, 'start' (start-anchored FRD "
                         "vision from sensor_bridge.py) for DH_PX4_PROFILE=drone. Default follows DH_PX4_PROFILE")
    ap.add_argument("--noise", type=float, default=0.0, help="odometry position noise sigma, m")
    ap.add_argument("--odom-delay", type=float, default=0.05, help="odometry latency, s (cuVSLAM output)")
    ap.add_argument("--vio-dropout", default="", metavar="T0:T1",
                    help="report vo_state=2 between these seconds after takeover")
    ap.add_argument("--timeout", type=float, default=240.0, help="give up after this much sim time, s")
    ap.add_argument("--tag", default="")
    ap.add_argument("--relay", type=int, default=0, metavar="PORT",
                    help="stream the run live for gazebo_sim/mirror/mirror.py --run 127.0.0.1:PORT (0 = off)")
    a = ap.parse_args()

    os.environ["GZ_PARTITION"] = f"dh{a.instance}"     # before any gz Node exists
    os.environ.setdefault("GZ_IP", "127.0.0.1")
    from fly_course import PX4, GroundTruth, score, tilt_deg          # noqa: E402  (read-only reuse)
    from dh.geom import Obstacles, clearance, map_to_world             # noqa: E402
    from dh.route import GROUND_Z                                      # noqa: E402

    OVERRIDES.update(dict(parse_override(t) for t in a.params))
    OVERRIDES.setdefault("route_file", a.route)
    install_ros_standin()
    from navless_nav import course_mission                             # noqa: E402
    from navless_nav.mirror_relay import FrameServer, make_frame       # noqa: E402
    relay = FrameServer(a.relay, host="127.0.0.1") if a.relay else None
    if relay is not None:
        print(f"live stream for the dashboard on 127.0.0.1:{relay.port}", flush=True)

    course = json.load(open(os.path.join(HERE, "course", "course_v3.json")))
    stamp = time.strftime("%Y%m%d-%H%M%S") + "-navless" + (f"-{a.tag}" if a.tag else "")
    out = os.path.join(HERE, "runs", stamp)
    os.makedirs(out, exist_ok=True)

    gt = GroundTruth()
    px4 = PX4(f"udpin:0.0.0.0:{14540 + a.instance}")
    px4.connect()
    print(f"connected to PX4 (instance {a.instance}); waiting for EKF position and ground truth", flush=True)
    t_end = time.time() + 30
    while time.time() < t_end:
        px4.poll()
        st, p, q = gt.snapshot()
        if px4.local is not None and p is not None and px4.landed_state is not None:
            break
        time.sleep(0.05)
    else:
        raise SystemExit("timed out waiting for PX4 local position / Gazebo odometry")

    st, p0, q0 = gt.snapshot()
    yaw0 = yaw_enu(q0)
    CLOCK.t = st
    print(f"power-up pose: world ({p0[0]:.3f}, {p0[1]:.3f}, {p0[2]:.3f}), nose {math.degrees(yaw0):.1f} deg "
          f"ENU (bearing {(90 - math.degrees(yaw0)) % 360:.1f} deg from north)", flush=True)

    node = course_mission.CourseMission()
    fi = FlightInterface(px4, p0, yaw0, course_mission.POSE_TOPIC, course_mission.TWIST_TOPIC,
                         a.min_alt, a.max_alt, a.max_speed, px4_frame=a.px4_frame)
    print(f"PX4 local frame: {a.px4_frame}", flush=True)
    dropout = tuple(float(x) for x in a.vio_dropout.split(":")) if a.vio_dropout else None

    odom_buf = collections.deque()
    rows = []
    t_odom = t_fi = -1e9
    t_start = st
    t_land_cmd = t_done = None
    held_since = None
    result = None
    while True:
        px4.poll()
        st, p, q = gt.snapshot()
        if st is None or p is None:
            time.sleep(0.002)
            continue
        CLOCK.t = max(CLOCK.t, st)

        # cuVSLAM stand-in: ground truth in the power-up frame at 30 Hz, delayed and noisy
        if st - t_odom >= 1 / 30:
            t_odom = st
            c, s = math.cos(-yaw0), math.sin(-yaw0)
            d = p - p0
            qv = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], d[2]])
            if a.noise:
                qv = qv + np.array([random.gauss(0, a.noise) for _ in range(3)])
            odom_buf.append((st, qv))
            while len(odom_buf) > 1 and odom_buf[1][0] <= st - a.odom_delay:
                odom_buf.popleft()
            if odom_buf[0][0] <= st - a.odom_delay + 1e-9:
                msg = Odometry()
                msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, odom_buf[0][1])
                publish(ODOM_TOPIC, msg)
            since = st - (fi.armed_at or 1e18)
            lost = bool(dropout and dropout[0] <= since < dropout[1])
            publish(STATUS_TOPIC, VisualSlamStatus(2 if lost else 1))
            if relay is not None and relay.n_clients:
                # The live dashboard (gazebo_sim/mirror/mirror.py --run): the drone's true pose in the
                # power-up frame, what the node commands, and which state it is in.
                w, x, y, z = q                                         # Gazebo body FLU -> world
                hs, hc = math.sin(-yaw0 / 2), math.cos(-yaw0 / 2)
                q_odom = (hc * x - hs * y, hc * y + hs * x,          # qz(-yaw0) * q_world, (x, y, z, w)
                          hc * z + hs * w, hc * w - hs * z)
                sp = None
                if fi.mode == "position" and fi.target is not None:
                    sp = fi.unrot(np.asarray(fi.target) - p0)
                relay.send(make_frame(st, a.odom_delay, (qv, q_odom), 2 if lost else 1,
                                      {"armed": px4.armed, "offboard_active": fi.offboard_active,
                                       "nav_state_name": "OFFBOARD" if fi.offboard_active else "sim"},
                                      sp, None, None, node.state))

        # 10 Hz: flight_interface state, the node's timer, then the interface's setpoint
        if st - t_fi >= 0.1:
            t_fi = st
            publish(course_mission.STATE_TOPIC, String(fi.state_json()))
            for timer in TIMERS:
                if st - timer[2] >= timer[0] - 1e-6:
                    timer[2] = st
                    timer[1]()
            kind, v = fi.tick()
            sp = v if kind == "position" else np.full(3, np.nan)
            rows.append([st, st - (fi.armed_at or st), node.state, kind, *sp, *p, tilt_deg(q), yaw_enu(q),
                         int(px4.armed), px4.main_mode, px4.landed_state])

            # end conditions
            if node.state == "DONE":
                t_done = t_done or st
                if st - t_done > 1.0:
                    result = "LANDED"
                    break
            if node.state in ("HOLD", "PAUSED") and fi.armed_at is not None:
                held_since = held_since or st
                if st - held_since > 5.0 and t_land_cmd is None:
                    print("  node is holding: PX4 land (stand-in for the pilot landing it)", flush=True)
                    px4.cmd(mavutil.mavlink.MAV_CMD_NAV_LAND)
                    t_land_cmd = st
            else:
                held_since = None
            if t_land_cmd is not None and not px4.armed:
                result = "HELD then pilot-landed"
                break
            if fi.takeover_at is None and st - t_start > 30:
                raise SystemExit("no takeover in 30 s: the node never published, PX4 is not landed/ready, "
                                 f"or PX4's height never settled (off by {fi.height_err} m on the ground)")
            if fi.takeover_at is not None and fi.armed_at is None and st - fi.takeover_at > 10:
                raise SystemExit("PX4 refused offboard/arm; PX4 said: " + " | ".join(px4.texts[-5:]))
            if st - t_start > a.timeout:
                result = "TIMEOUT"
                px4.cmd(mavutil.mavlink.MAV_CMD_NAV_LAND)
                break
        time.sleep(0.002)

    time.sleep(0.5)
    px4.close()
    if relay is not None:
        relay.close()

    # ---- summarise
    cols = ["sim_t", "t", "state", "sp_kind", "sp_x", "sp_y", "sp_z", "x", "y", "z", "tilt_deg", "yaw_enu",
            "armed", "mode", "landed_state"]
    T = np.array([r[1] for r in rows], float)
    P = np.array([r[7:10] for r in rows], float)
    SP = np.array([r[4:7] for r in rows], float)
    states = [r[2] for r in rows]
    yaw_ned = math.pi / 2 - np.array([r[11] for r in rows], float)
    flying = P[:, 2] > GROUND_Z + 0.1
    obs = Obstacles()
    clr, names = clearance(obs, P, yaw_ned)
    clr = np.where(flying, clr, np.inf)
    ev = score(P, T, course)                     # includes gate_b
    pad =np.array(map_to_world(*course["landing_pad_in"]["center"]))
    liftoff = next((T[i] for i in range(len(T)) if fi.armed_at and T[i] >= 0 and P[i, 2] > p0[2] + 0.05), None)
    land_i = next((i for i, s in enumerate(states) if s == "LAND"), None)
    td = None
    if land_i is not None:
        td = next((i for i in range(land_i, len(T)) if P[i, 2] < GROUND_Z + 0.012), None)
    route_rows = [i for i, s in enumerate(states) if s == "ROUTE"]
    lead = [float(np.linalg.norm(SP[i] - P[i])) for i in route_rows if np.all(np.isfinite(SP[i]))]
    contacts = sorted({n for (_, n) in gt.contacts})
    errors = [m for (_, lvl, m) in LOGS if lvl == "ERROR"]
    summary = {
        "result": ("COLLISION + " if contacts else "") + (result or "?"),
        "route": OVERRIDES["route_file"],
        "overrides": {k: v for k, v in OVERRIDES.items() if k != "route_file"},
        "odometry": {"noise_m": a.noise, "delay_s": a.odom_delay, "vio_dropout": a.vio_dropout or None},
        "px4_frame": a.px4_frame,
        "px4_profile": os.environ.get("DH_PX4_PROFILE"),
        "liftoff_after_arm_s": None if liftoff is None else round(float(liftoff), 2),
        "touchdown_time_from_arm_s": None if td is None else round(float(T[td]), 2),
        "touchdown_offset_from_pad_cm": None if td is None else round(float(np.linalg.norm(P[td, :2] - pad)) * 100, 1),
        "checkpoints_s": {k: (round(v, 2) if isinstance(v, float) else v) for k, v in ev.items()},
        "min_clearance_cm": round(float(clr.min()) * 100, 1) if np.isfinite(clr.min()) else None,
        "min_clearance_at": names[int(np.argmin(clr))] if np.isfinite(clr.min()) else None,
        "setpoint_lead_m": {"mean": round(float(np.mean(lead)), 3), "max": round(float(np.max(lead)), 3)} if lead else None,
        "max_tilt_deg": round(float(np.nanmax([r[10] for r in rows])), 1),
        "max_speed_mps": round(float(np.max(np.linalg.norm(np.diff(P, axis=0), axis=1) / np.maximum(np.diff(T), 1e-3))), 2),
        "contacts": contacts,
        "node_errors": errors,
    }
    with open(os.path.join(out, "log.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(cols + ["clearance"])
        for r, c in zip(rows, clr):
            wr.writerow([f"{v:.4f}" if isinstance(v, float) else v for v in r] + [f"{c:.4f}"])
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))
    print(f"run saved to {out}")


if __name__ == "__main__":
    main()
