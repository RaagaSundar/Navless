#!/usr/bin/env python3
"""The drone stack's side of the master sim, as one real ROS 2 node over Gazebo + PX4 SITL.

Stands in for what runs on skybeta and is not the team's code:
    cuVSLAM          /visual_slam/tracking/odometry (power-up frame, 30 Hz, delayed), /visual_slam/status
                     (vo_state), TF odom -> base_link. Blind in the dark tube with DH_TUBE_BLACKOUT, through
                     gazebo_sim/dh/tube.py, the same model sensor_bridge.py applies to PX4's vision.
    camera TF        static base_link -> oak_{left,right}_camera_optical_frame (the images come from
                     Gazebo through ros_gz_bridge)
    flight_interface /skybeta/setpoint/{pose,twist} -> PX4 over MAVLink, /skybeta/state JSON, and the
                     interface's takeover rules: gazebo_sim/fly_navless.py's FlightInterface, reused.
    the pilot        fly_navless.py's Pilot. --pilot modes (default) is the drone's handover (virtual sticks,
                     Altitude, arm, then Position), once PX4's height has settled and the brain is streaming.
                     --rearm N re-arms after a landing (the fly-home leg). If the vehicle holds still in the
                     air for --hold-land-s, the pilot lands it.
--estimator-health also publishes /estimator/healthy, which nothing on the drone publishes. It is off by
default, so a brain that waits for it shows that here too.

It scores the run like fly_navless.py once the vehicle is disarmed with nothing more to do, or at
--timeout, and writes gazebo_sim/runs/<time>-master-<brain>/{summary.json,log.csv}.
"""
import argparse
import collections
import csv
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
GZ_SIM = os.path.join(os.path.dirname(HERE), "gazebo_sim")
sys.path.insert(0, GZ_SIM)

CAMERAS = {   # base_link -> optical frame, matching make_assets.py
    "oak_left_camera_optical_frame": (0.115, 0.0375, 0.01),
    "oak_right_camera_optical_frame": (0.115, -0.0375, 0.01),
}
OPTICAL_Q = (-0.5, 0.5, -0.5, 0.5)     # x, y, z, w: z forward, x right, y down


def qmul(a, b):
    """Hamilton product of (w, x, y, z) quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def stamp(t):
    from builtin_interfaces.msg import Time
    return Time(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)) % 1_000_000_000)


def parse_args():
    from dh.tube import MODES as TUBE_MODES, env_settings as tube_env
    env_tube = tube_env()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--instance", type=int, default=int(os.environ.get("DH_INSTANCE", 0)))
    ap.add_argument("--brain", default="?", help="label for the run folder")
    ap.add_argument("--odom-delay", type=float, default=0.05, help="cuVSLAM output latency, s")
    ap.add_argument("--odom-noise", type=float, default=0.0, help="odometry position noise sigma, m")
    ap.add_argument("--vio-dropout", default="", metavar="T0:T1",
                    help="vo_state 2 and no odometry between these seconds after the first takeover")
    ap.add_argument("--pilot", choices=("modes", "direct"), default="modes",
                    help="'modes': the drone's handover and the interface's takeover rules; 'direct': SITL shortcut")
    ap.add_argument("--rearm", type=int, default=0, help="re-arms after a landing (1 for the fly-home leg)")
    ap.add_argument("--rearm-delay", type=float, default=3.0, help="s on the ground before a re-arm")
    ap.add_argument("--switch-delay", type=float, default=1.0, help="s between arming and the switch to Position")
    ap.add_argument("--settle-tol", type=float, default=0.15, help="arm only once PX4's height is this close, m")
    ap.add_argument("--estimator-health", action="store_true", help="also publish /estimator/healthy")
    ap.add_argument("--hold-land-s", type=float, default=8.0)
    ap.add_argument("--settle-s", type=float, default=6.0, help="s disarmed with nothing to do before scoring")
    ap.add_argument("--timeout", type=float, default=300.0, help="sim seconds after the first takeover")
    ap.add_argument("--wait-takeover-s", type=float, default=60.0, help="give up if nobody arms by then")
    ap.add_argument("--min-alt", type=float, default=0.3)
    ap.add_argument("--max-alt", type=float, default=2.5)
    ap.add_argument("--max-speed", type=float, default=1.0)
    ap.add_argument("--tube-blackout", choices=TUBE_MODES, default=env_tube[0],
                    help="camera blind in the dark tube (default DH_TUBE_BLACKOUT, which sensor_bridge.py reads)")
    ap.add_argument("--tube-recover-delay", type=float, default=env_tube[1])
    ap.add_argument("--tube-reset-jump", default=env_tube[2], metavar="DX,DY,DZ|resume")
    a = ap.parse_args()
    a.env_tube = env_tube
    return a


def main():
    a = parse_args()
    os.environ["GZ_PARTITION"] = f"dh{a.instance}"
    os.environ.setdefault("GZ_IP", "127.0.0.1")
    from pymavlink import mavutil
    import fly_navless
    from dh.tube import TubeBlackout
    from fly_course import PX4, GroundTruth, score, tilt_deg
    from dh.geom import Obstacles, clearance, map_to_world
    from dh.route import GROUND_Z

    import rclpy
    from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Bool, String
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
    try:
        from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
    except ImportError:
        VisualSlamStatus = None
        print("[sim_core] WARNING: no isaac_ros_visual_slam_interfaces, so /visual_slam/status (vo_state) is not "
              "published and the brains cannot see a tracking loss. See master_sim/README.md.", flush=True)
    CLOCK = fly_navless.CLOCK

    gt = GroundTruth()
    px4 = PX4(f"udpin:0.0.0.0:{14540 + a.instance}")
    px4.connect()
    print(f"[sim_core] connected to PX4 {a.instance}; waiting for EKF position and Gazebo odometry", flush=True)
    t_end = time.time() + 60
    while time.time() < t_end:
        px4.poll()
        st, p0, q0 = gt.snapshot()
        if px4.local is not None and p0 is not None and px4.landed_state is not None:
            break
        time.sleep(0.05)
    else:
        raise SystemExit("[sim_core] timed out waiting for PX4 local position / Gazebo odometry")
    yaw0 = fly_navless.yaw_enu(q0)
    q_unyaw = (math.cos(-yaw0 / 2), 0.0, 0.0, math.sin(-yaw0 / 2))
    CLOCK.t = st
    print(f"[sim_core] power-up pose ({p0[0]:.3f}, {p0[1]:.3f}, {p0[2]:.3f}), nose "
          f"{math.degrees(yaw0):.1f} deg ENU", flush=True)
    tube = None
    if a.tube_blackout != "off" or os.environ.get("DH_PAD_DRIFT"):     # pad drift lives there too
        if (a.tube_blackout, a.tube_recover_delay, a.tube_reset_jump) != a.env_tube:
            print("[sim_core] WARNING: tube settings differ from DH_TUBE_*, which sensor_bridge.py uses for PX4's "
                  "vision: the brain and PX4 will not lose tracking together", flush=True)
        tube = TubeBlackout(a.tube_blackout, yaw0, a.tube_recover_delay, a.tube_reset_jump,
                            log=lambda text: print(f"  [tube] {text}", flush=True))
        print(f"[sim_core] tube blackout: {tube.describe()}", flush=True)

    fi = fly_navless.FlightInterface(px4, p0, yaw0, "/skybeta/setpoint/pose", "/skybeta/setpoint/twist",
                                     a.min_alt, a.max_alt, a.max_speed, px4_frame="start",
                                     rules=(a.pilot == "modes"), settle_tol=a.settle_tol)
    pilot = fly_navless.Pilot(px4, a.pilot, a.switch_delay)
    if a.pilot == "modes":
        got = fly_navless.set_int_param(px4, "COM_RC_IN_MODE", 1)      # the virtual sticks are a joystick
        if got != 1:
            raise SystemExit(f"[sim_core] could not set COM_RC_IN_MODE=1 (PX4 says {got})")

    rclpy.init()
    node = rclpy.create_node("master_sim_core")
    node.create_subscription(PoseStamped, "/skybeta/setpoint/pose", fi.on_pose, 10)
    node.create_subscription(TwistStamped, "/skybeta/setpoint/twist", fi.on_twist, 10)
    odom_pub = node.create_publisher(Odometry, "/visual_slam/tracking/odometry", 10)
    status_pub = node.create_publisher(VisualSlamStatus, "/visual_slam/status", 10) if VisualSlamStatus else None
    state_pub = node.create_publisher(String, "/skybeta/state", 10)
    health_pub = node.create_publisher(Bool, "/estimator/healthy", 10) if a.estimator_health else None
    tf_pub = TransformBroadcaster(node)
    static = []
    for frame, (x, y, z) in CAMERAS.items():
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp(st), "base_link", frame
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = OPTICAL_Q
        static.append(t)
    StaticTransformBroadcaster(node).sendTransform(static)

    # Perception scorecard: point-cloud rate and size, and /team/localize/pose (the left camera's
    # pose in the course frame, at the cloud's capture time) against Gazebo's true left-camera position
    # at that time.
    perception = {"clouds": [], "loc_err": []}
    left_cam = np.array(CAMERAS["oak_left_camera_optical_frame"])
    truth_cam = collections.deque(maxlen=300)          # (sim t, true left-camera xy), 30 Hz, 10 s

    def on_cloud(m):
        perception["clouds"].append((CLOCK.t, m.width * m.height))

    def on_localize(m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if not truth_cam or t < truth_cam[0][0]:
            return
        _, xy = min(truth_cam, key=lambda s: abs(s[0] - t))
        est = np.array([m.pose.position.x, m.pose.position.y])
        perception["loc_err"].append((t, float(np.linalg.norm(est - xy))))

    node.create_subscription(PointCloud2, "/team/stereo/points", on_cloud, 2)
    node.create_subscription(PoseStamped, "/team/localize/pose", on_localize, 10)

    def to_odom(pw):
        c, s = math.cos(-yaw0), math.sin(-yaw0)
        d = pw - p0
        return np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], d[2]])

    course = json.load(open(os.path.join(GZ_SIM, "course", "course_v3.json")))
    out = os.path.join(GZ_SIM, "runs", time.strftime("%Y%m%d-%H%M%S") + f"-master-{a.brain}")
    os.makedirs(out, exist_ok=True)

    dropout = tuple(float(v) for v in a.vio_dropout.split(":")) if a.vio_dropout else None
    buf = collections.deque()
    rows = []
    t_odom = t_fi = -1e9
    prev = None
    vel = np.zeros(3)
    rearms_left = a.rearm
    t_flight = t_disarmed = None
    was_armed = False
    landings = 0
    quiet_since = still_since = None
    t_land_cmd = None
    result = None
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)
        px4.poll()
        st, p, q = gt.snapshot()
        if st is None or p is None:
            time.sleep(0.002)
            continue
        CLOCK.t = max(CLOCK.t, st)
        pilot.sticks()

        if st - t_odom >= 1 / 30:                       # cuVSLAM stand-in
            t_odom = st
            w, x, y, z = q
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            c, s = math.cos(yaw), math.sin(yaw)       # level enough at <3 deg tilt
            truth_cam.append((st, p[:2] + np.array([c * left_cam[0] - s * left_cam[1],
                                                     s * left_cam[0] + c * left_cam[1]])))
            tracking, pr = tube.step(st, p) if tube is not None else (True, p)
            since = st - fi.armed_at if fi.armed_at is not None else -1.0
            if dropout and dropout[0] <= since < dropout[1]:
                tracking, pr = False, None
            pos = None
            if pr is not None:                        # what cuVSLAM publishes, if anything
                pos = to_odom(pr)
                if prev is not None and st > prev[0]:
                    vel = 0.5 * vel + 0.5 * (pos - prev[1]) / (st - prev[0])
                prev = (st, pos)
                if a.odom_noise and tracking:
                    pos = pos + np.random.normal(0.0, a.odom_noise, 3)
            buf.append((st, pos, qmul(q_unyaw, q), vel.copy(), tracking))
            while len(buf) > 1 and buf[1][0] <= st - a.odom_delay:
                buf.popleft()
            ts, pos, rot, v, trk = buf[0]
            if ts <= st - a.odom_delay + 1e-9:
                if status_pub is not None:
                    status_pub.publish(VisualSlamStatus(vo_state=1 if trk else 2))
                if pos is not None:
                    m = Odometry()
                    m.header.stamp, m.header.frame_id, m.child_frame_id = stamp(ts), "odom", "base_link"
                    m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = map(float, pos)
                    (m.pose.pose.orientation.w, m.pose.pose.orientation.x,
                     m.pose.pose.orientation.y, m.pose.pose.orientation.z) = map(float, rot)
                    m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = map(float, v)
                    odom_pub.publish(m)
                    t = TransformStamped()
                    t.header.stamp, t.header.frame_id, t.child_frame_id = m.header.stamp, "odom", "base_link"
                    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, pos)
                    t.transform.rotation = m.pose.pose.orientation
                    tf_pub.sendTransform(t)

        if st - t_fi < 0.1:                              # 10 Hz: flight_interface + the pilot
            time.sleep(0.002)
            continue
        t_fi = st
        state_pub.publish(String(data=fi.state_json()))
        if health_pub is not None:
            health_pub.publish(Bool(data=px4.local is not None))
        n_takeovers = len(fi.takeovers)
        kind, v = fi.tick()
        pilot.step()
        if len(fi.takeovers) > n_takeovers:
            print(f"[sim_core] takeover {len(fi.takeovers)} at {st:.1f} s", flush=True)
        sp = v if kind == "position" else np.full(3, np.nan)
        rows.append([st, st - (fi.armed_at or st), len(fi.takeovers), kind, *sp, *p, tilt_deg(q),
                     fly_navless.yaw_enu(q), int(px4.armed), px4.main_mode, px4.landed_state])

        # The pilot: first flight once PX4's height has settled and the brain streams; re-arms after landings.
        if was_armed and not px4.armed and fi.takeovers:
            t_disarmed, landings = st, landings + 1
            print(f"[sim_core] disarmed after landing {landings}", flush=True)
        was_armed = px4.armed
        if not pilot.busy and fi.ready_on_ground():
            if pilot.flights == 0:
                pilot.start()
                t_flight = st
            elif rearms_left > 0 and t_disarmed is not None and st - t_disarmed >= a.rearm_delay:
                rearms_left -= 1
                pilot.start()
                t_flight = st
        if t_flight is not None and len(fi.takeovers) < pilot.flights and st - t_flight > 15.0:
            result = (f"NO-TAKEOVER on flight {pilot.flights} (rules state {fi.ao}, PX4 mode {px4.main_mode}); "
                      "PX4: " + " | ".join(px4.texts[-3:]))
            if px4.armed:
                px4.cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
            break

        idle = (fi.takeovers and not px4.armed and not pilot.busy
                and px4.landed_state == fly_navless.LANDED_ON_GROUND)
        done = idle and (rearms_left == 0 or not fi.setpoint_fresh())
        quiet_since = (quiet_since or st) if done else None
        if quiet_since is not None and st - quiet_since > a.settle_s:
            result = ("PILOT-LANDED" if t_land_cmd else "LANDED") + (f" ({landings} landings)" if landings > 1 else "")
            break
        moving = len(rows) > 10 and np.linalg.norm(np.subtract(rows[-1][7:10], rows[-11][7:10])) > 0.05
        if px4.armed and not moving and p[2] > GROUND_Z + 0.2:
            still_since = still_since or st
            if st - still_since > a.hold_land_s and t_land_cmd is None:
                print("[sim_core] holding in the air: the pilot lands it", flush=True)
                px4.cmd(mavutil.mavlink.MAV_CMD_NAV_LAND)
                t_land_cmd = st
        else:
            still_since = None
        if fi.armed_at is not None and st - fi.armed_at > a.timeout:
            result = "TIMEOUT"
            px4.cmd(mavutil.mavlink.MAV_CMD_NAV_LAND)
            break
        if not fi.takeovers and pilot.flights == 0 and len(rows) > 10 * a.wait_takeover_s:
            result = ("NO-TAKEOVER (bench run, the brain never streamed, or PX4's height never settled: "
                      f"{fi.height_err} m off)")
            break

    px4.close()
    a.tube = tube.passes if tube is not None else None
    write_summary(out, rows, course, gt, a, result, score, clearance, Obstacles, map_to_world, GROUND_Z,
                  perception, fi.armed_at)
    node.destroy_node()
    rclpy.shutdown()


def write_summary(out, rows, course, gt, a, result, score, clearance, Obstacles, map_to_world, ground_z,
                  perception, first_arm):
    cols = ["sim_t", "t", "takeovers", "sp_kind", "sp_x", "sp_y", "sp_z", "x", "y", "z", "tilt_deg",
            "yaw_enu", "armed", "mode", "landed_state"]
    T = np.array([r[1] for r in rows], float)
    P = np.array([r[7:10] for r in rows], float)
    flying = P[:, 2] > ground_z + 0.1
    clr, names = clearance(Obstacles(), P, math.pi / 2 - np.array([r[11] for r in rows], float))
    clr = np.where(flying, clr, np.inf)
    pad = np.array(map_to_world(*course["landing_pad_in"]["center"]))
    shift = [float(v) for v in os.environ.get("DH_PAD_SHIFT", "").split(",") if v.strip()]
    if shift:                                   # make_assets.py moved the pad in the world
        pad = pad + np.array(shift[:2])
    home = P[0, :2]
    lows = [i for i in range(1, len(P)) if flying[i - 1] and not flying[i]]   # each touchdown
    contacts = sorted({n for (_, n) in gt.contacts})
    summary = {
        # Gazebo contacts, or the airframe (props included) inside a wall by Raaga's clearance model.
        "result": ("COLLISION + " if contacts or clr.min() < 0 else "") + (result or "STOPPED"),
        "brain": a.brain,
        "pilot": a.pilot,
        "odometry": {"delay_s": a.odom_delay, "noise_m": a.odom_noise, "vio_dropout": a.vio_dropout or None},
        "tube_blackout": None if a.tube is None else {"mode": a.tube_blackout, "passes": a.tube},
        "takeovers": int(rows[-1][2]) if rows else 0,
        "touchdowns": [{"t": round(float(T[i]), 1),
                        "from_pad_cm": round(float(np.linalg.norm(P[i, :2] - pad)) * 100, 1),
                        "from_start_cm": round(float(np.linalg.norm(P[i, :2] - home)) * 100, 1)} for i in lows],
        "checkpoints_s": {k: (round(v, 2) if isinstance(v, float) else v) for k, v in score(P, T, course).items()},
        "min_clearance_cm": round(float(clr.min()) * 100, 1) if np.isfinite(clr.min()) else None,
        "min_clearance_at": names[int(np.argmin(clr))] if np.isfinite(clr.min()) else None,
        "max_alt_m": round(float(P[:, 2].max()), 2),
        "max_tilt_deg": round(float(np.nanmax([r[10] for r in rows])), 1),
        "contacts": contacts,
        "perception": perception_summary(perception, first_arm, rows[-1][0] if rows else None),
    }
    with open(os.path.join(out, "log.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols + ["clearance"])
        for r, c in zip(rows, clr):
            w.writerow([f"{v:.4f}" if isinstance(v, float) else v for v in r] + [f"{c:.4f}"])
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))
    print(f"[sim_core] run saved to {out}", flush=True)


def perception_summary(perception, t0, t1):
    """Cloud rate and localization error over the flight (first takeover to the end)."""
    if t0 is None and perception["clouds"]:           # bench run: score from the first cloud
        t0 = perception["clouds"][0][0]
    if t0 is None or t1 is None or t1 <= t0:
        return None
    clouds = [n for t, n in perception["clouds"] if t >= t0]
    errs = sorted(e for t, e in perception["loc_err"] if t >= t0)
    return {
        "cloud_hz": round(len(clouds) / (t1 - t0), 2),
        "cloud_points_mean": int(np.mean(clouds)) if clouds else 0,
        "localize_hz": round(len(errs) / (t1 - t0), 2),
        "localize_err_xy_cm": {"median": round(100 * errs[len(errs) // 2], 1),
                               "p95": round(100 * errs[int(0.95 * (len(errs) - 1))], 1),
                               "max": round(100 * errs[-1], 1)} if errs else None,
    }


if __name__ == "__main__":
    main()
