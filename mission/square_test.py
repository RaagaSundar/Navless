#!/usr/bin/env python3
"""Movement test: take off, fly a small square with the camera facing the direction of travel, land.

Real drone (Skybeta flight_interface). On the Jetson host, drone on the launch mark, disarmed:
  bash ~/team_ws/src/Navless/planner/real_flight_start.sh mission/square_test.py
  # pilot: Altitude, arm, then Position -> flight_interface hands Offboard to this script
  bash ~/team_ws/src/Navless/planner/real_flight_kill.sh      # any time: lands where it is, then exits
FlyX lab / plain PX4 (ARMS AND FLIES BY ITSELF: team approval first):
  python3 mission/square_test.py --px4 --ros-args -p use_sim_time:=true
Offline check (no ROS):  python3 mission/square_test.py --selftest

The square (--side, default 0.305 m = 1 ft) starts at the takeoff point and goes forward, right, back, left
(clockwise from above), since the launch mark has only ~0.47 m to the west wall; --left mirrors it. At
every corner the drone stops and turns in place to face the next side, then flies it nose-first. Then it
lands where it took off. Slow on purpose (0.15 m/s, 0.5 rad/s) so cuVSLAM keeps tracking.

flight_interface ignores yaw in pose setpoints, so in Skybeta mode this flies velocity setpoints (with yaw
rate) that track the moving setpoint from cuVSLAM odometry. --px4 sends position + yaw to PX4 directly.

Lands (and exits) on: SIGINT/SIGTERM, VIO not tracking or telemetry older than 0.5 s, leaving a box 0.5 m
around the square, 120 s in the air. A second signal stops publishing at once. Skybeta mode never arms;
any pilot mode change takes control back and the script exits.
"""
import argparse
import json
import math
import signal
import sys

SPEED, YAW_RATE, CLIMB, DESCENT = 0.15, 0.5, 0.3, 0.3   # m/s, rad/s, m/s, m/s
POS_TOL, YAW_TOL, HOLD_S = 0.06, math.radians(6.0), 1.5
KP, KYAW, VXY_MAX, VZ_MAX, WZ_MAX = 1.5, 1.5, 0.3, 0.4, 0.8   # Skybeta velocity tracking
STALE_S, STATUS_STALE_S, MAX_AIR_S, FENCE_M = 0.5, 1.5, 120.0, 0.5


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def toward(cur, target, step):
    """Move scalar `cur` toward `target` by at most `step`."""
    return target if abs(target - cur) <= step else cur + math.copysign(step, target - cur)


class Square:
    """Setpoint generator. Frame: x, y, z right-handed with z up (Skybeta odom, or PX4 local NED as ENU);
    yaw counter-clockwise. The square is laid out from the takeoff pose `origin`/`yaw0`."""

    def __init__(self, origin, yaw0, side=0.305, alt=0.7, left=False):
        c, s, w = math.cos(yaw0), math.sin(yaw0), (side if left else -side)
        local = [(0.0, 0.0), (side, 0.0), (side, w), (0.0, w), (0.0, 0.0)]
        self.corners = [(origin[0] + c * x - s * y, origin[1] + s * x + c * y) for x, y in local]
        self.ground_z, self.z = origin[2], origin[2] + alt
        self.sp = [origin[0], origin[1], origin[2]]
        self.sp_yaw = yaw0
        self.phase, self.leg, self.hold_left = "TAKEOFF", 0, HOLD_S
        xs, ys = [p[0] for p in self.corners], [p[1] for p in self.corners]
        self.box = (min(xs) - FENCE_M, max(xs) + FENCE_M, min(ys) - FENCE_M, max(ys) + FENCE_M)

    def heading(self):
        (x0, y0), (x1, y1) = self.corners[self.leg], self.corners[self.leg + 1]
        return math.atan2(y1 - y0, x1 - x0)

    def inside(self, pos):
        x0, x1, y0, y1 = self.box
        return x0 <= pos[0] <= x1 and y0 <= pos[1] <= y1 and pos[2] <= self.z + FENCE_M

    def step(self, pos, yaw, dt):
        """Advance the setpoint; returns the phase: TAKEOFF, HOLD, TURN, MOVE, then LAND."""
        if self.phase == "TAKEOFF":
            self.sp[2] = toward(self.sp[2], self.z, CLIMB * dt)
            if self.sp[2] == self.z and abs(pos[2] - self.z) < 0.08:
                self.phase = "HOLD"
        elif self.phase == "HOLD":
            self.hold_left -= dt
            if self.hold_left <= 0.0:
                self.phase = "TURN" if self.leg < len(self.corners) - 1 else "LAND"
        elif self.phase == "TURN":  # in place, until the nose points along the next side
            h = self.heading()
            self.sp_yaw = wrap(self.sp_yaw + max(-YAW_RATE * dt, min(YAW_RATE * dt, wrap(h - self.sp_yaw))))
            if abs(wrap(h - self.sp_yaw)) < 1e-6 and abs(wrap(yaw - h)) < YAW_TOL:
                self.sp_yaw, self.phase = h, "MOVE"
        elif self.phase == "MOVE":  # nose-first along the side
            ex, ey = self.corners[self.leg + 1]
            dx, dy = ex - self.sp[0], ey - self.sp[1]
            d = math.hypot(dx, dy)
            f = 1.0 if d <= SPEED * dt else SPEED * dt / d
            self.sp[0], self.sp[1] = self.sp[0] + f * dx, self.sp[1] + f * dy
            if f == 1.0 and math.hypot(pos[0] - ex, pos[1] - ey) < POS_TOL:
                self.leg, self.phase, self.hold_left = self.leg + 1, "HOLD", 0.5
        return self.phase


def track_twist(sq, pos, yaw):
    """Skybeta: velocity + yaw rate that pull the drone onto the setpoint (odom frame)."""
    v = [KP * (sq.sp[i] - pos[i]) for i in range(3)]
    n = math.hypot(v[0], v[1])
    if n > VXY_MAX:
        v[0], v[1] = v[0] * VXY_MAX / n, v[1] * VXY_MAX / n
    v[2] = max(-VZ_MAX, min(VZ_MAX, v[2]))
    return v, max(-WZ_MAX, min(WZ_MAX, KYAW * wrap(sq.sp_yaw - yaw)))


def land_twist(sq, pos, land_xy):
    """Skybeta landing: hold xy, descend (slower near the floor) until PX4 disarms."""
    vx, vy = KP * (land_xy[0] - pos[0]), KP * (land_xy[1] - pos[1])
    n = math.hypot(vx, vy)
    if n > 0.2:
        vx, vy = vx * 0.2 / n, vy * 0.2 / n
    return [vx, vy, -(DESCENT if pos[2] - sq.ground_z > 0.25 else 0.15)], 0.0


def selftest():
    """Fly the square against a first-order drone (velocity lag) with the Skybeta tracker."""
    for yaw0, left in ((0.0, False), (1.0, True), (-2.5, False)):
        origin = (0.4, -0.2, 0.05)
        sq, pos, yaw, vel, dt, t = Square(origin, yaw0, left=left), list(origin), yaw0, [0.0, 0.0, 0.0], 1 / 30, 0.0
        visited, worst_face = set(), 0.0
        while sq.step(pos, yaw, dt) != "LAND":
            t += dt
            assert t < 90.0, f"stuck in {sq.phase} (yaw0={yaw0})"
            cmd_v, wz = track_twist(sq, pos, yaw)
            vel = [v + (c - v) * min(1.0, dt / 0.3) for v, c in zip(vel, cmd_v)]   # 0.3 s velocity lag
            pos = [p + v * dt for p, v in zip(pos, vel)]
            yaw = wrap(yaw + wz * dt)
            sp_xy = math.hypot(vel[0], vel[1])
            if sq.phase == "MOVE" and sp_xy > 0.05:  # the camera faces the way it is moving
                worst_face = max(worst_face, abs(wrap(math.atan2(vel[1], vel[0]) - yaw)))
            for i, c in enumerate(sq.corners):
                if math.hypot(pos[0] - c[0], pos[1] - c[1]) < 0.08 and abs(pos[2] - sq.z) < 0.1:
                    visited.add(i)
            assert sq.inside(pos), f"left the box at {pos}"
        assert visited == {0, 1, 2, 3, 4}, visited
        assert math.degrees(worst_face) < 15.0, f"camera {math.degrees(worst_face):.1f} deg off the direction of travel"
        assert math.hypot(pos[0] - origin[0], pos[1] - origin[1]) < POS_TOL + 0.02
        side_dir = math.atan2(sq.corners[2][1] - sq.corners[1][1], sq.corners[2][0] - sq.corners[1][0])
        assert abs(wrap(side_dir - yaw0 - (math.pi / 2 if left else -math.pi / 2))) < 1e-6  # right by default
        print(f"ok  yaw0={yaw0:+.1f} left={left}: square in {t:.1f} s, camera within {math.degrees(worst_face):.1f} deg of travel")
    print("selftest passed")


def run(a):
    import rclpy
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=a.ros_args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=a.ros_args)
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    stop = []

    def on_signal(signum, _frame):  # 1st: land then exit; 2nd: stop publishing now
        stop.append(signum)
        if len(stop) >= 2:
            raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    node = Node("square_test")
    log = node.get_logger()
    st = {"pos": None, "yaw": 0.0, "pos_t": None, "status_t": None, "vo": None, "vo_bad_since": None,
          "armed": False, "offboard": False, "landed": True}
    now = lambda: node.get_clock().now().nanoseconds * 1e-9

    def on_vo(m):
        st["vo"] = int(getattr(m, "vo_state", 1))
    try:
        from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
        node.create_subscription(VisualSlamStatus, "/visual_slam/status", on_vo, 10)
    except ImportError:
        log.warning("VisualSlamStatus not importable: VIO tracking state not checked")

    if a.px4:
        from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLandDetected,
                                  VehicleLocalPosition, VehicleStatus)

        def on_pos(m):  # NED -> ENU (x east, y north, z up), heading -> CCW yaw
            if m.xy_valid and m.z_valid:
                st["pos"], st["yaw"], st["pos_t"] = [m.y, m.x, -m.z], wrap(math.pi / 2 - m.heading), now()

        def on_status(m):
            st["armed"] = m.arming_state == VehicleStatus.ARMING_STATE_ARMED
            st["offboard"] = m.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            st["auto_land"] = m.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND
            st["status_t"] = now()
        ver = int(getattr(VehicleLandDetected, "MESSAGE_VERSION", 0))
        node.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", on_pos, qos_profile_sensor_data)
        node.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", on_status, qos_profile_sensor_data)
        node.create_subscription(VehicleLandDetected, "/fmu/out/vehicle_land_detected" + (f"_v{ver}" if ver else ""),
                                 lambda m: st.update(landed=bool(m.landed)), qos_profile_sensor_data)
        p_mode = node.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        p_sp = node.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        p_cmd = node.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        def send_sp(xyz, yaw):
            m = OffboardControlMode()
            m.position, m.timestamp = True, int(now() * 1e6)
            p_mode.publish(m)
            sp = TrajectorySetpoint()
            sp.position = [float(xyz[1]), float(xyz[0]), float(-xyz[2])]
            sp.velocity = sp.acceleration = [math.nan] * 3
            sp.yaw, sp.timestamp = float(wrap(math.pi / 2 - yaw)), int(now() * 1e6)
            p_sp.publish(sp)

        def send_cmd(command, p1=0.0, p2=0.0):
            m = VehicleCommand()
            m.command, m.param1, m.param2 = int(command), float(p1), float(p2)
            m.target_system = m.target_component = m.source_system = m.source_component = 1
            m.from_external, m.timestamp = True, int(now() * 1e6)
            p_cmd.publish(m)
        LAND_CMD = VehicleCommand.VEHICLE_CMD_NAV_LAND
    else:
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String

        def on_odom(m):
            p, q = m.pose.pose.position, m.pose.pose.orientation
            st["pos"], st["pos_t"] = [p.x, p.y, p.z], now()
            st["yaw"] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        def on_state(m):
            try:
                s = json.loads(m.data)
            except ValueError:
                return
            st.update(armed=bool(s.get("armed")), offboard=bool(s.get("offboard_active")),
                      landed=bool(s.get("landed")), status_t=now())
        node.create_subscription(Odometry, a.odom_topic, on_odom, 10)
        node.create_subscription(String, "/skybeta/state", on_state, 10)
        p_pose = node.create_publisher(PoseStamped, "/skybeta/setpoint/pose", 10)
        p_twist = node.create_publisher(TwistStamped, "/skybeta/setpoint/twist", 10)

        def send_pose(xyz):  # flight_interface ignores the orientation
            m = PoseStamped()
            m.header.stamp, m.header.frame_id = node.get_clock().now().to_msg(), "odom"
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in xyz)
            p_pose.publish(m)

        def send_twist(v, wz):
            m = TwistStamped()
            m.header.stamp, m.header.frame_id = node.get_clock().now().to_msg(), "odom"
            m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = (float(x) for x in v)
            m.twist.angular.z = float(wz)
            p_twist.publish(m)

    # Runtime state: WAIT (for ownership) -> FLY (Square phases) -> LAND -> DONE
    run_st = {"mode": "WAIT", "sq": None, "t_last": None, "t_air": None, "settled_t": None, "anchor": None,
              "prime_t": None, "cmd_t": -1e9, "land_xy": None, "code": 0, "phase": None}

    def healthy(t):
        if st["pos_t"] is None or st["status_t"] is None:
            return False, "no telemetry yet"
        if t - st["pos_t"] > STALE_S:
            return False, "position stale"
        if t - st["status_t"] > STATUS_STALE_S:  # vehicle_status is ~1.6 Hz in the FlyX lab
            return False, "vehicle status stale"
        if st["vo"] is not None and st["vo"] != 1:
            st["vo_bad_since"] = st["vo_bad_since"] or t
            if t - st["vo_bad_since"] > STALE_S:
                return False, f"VIO not tracking (vo_state={st['vo']})"
        else:
            st["vo_bad_since"] = None
        return True, ""

    def finish(code, why):
        log.info(f"DONE ({code}): {why}")
        run_st["code"], run_st["mode"] = code, "DONE"

    def start_land(why, code):
        if run_st["mode"] == "LAND":
            return
        log.warning(f"LAND: {why}")
        run_st["mode"], run_st["code"], run_st["cmd_t"] = "LAND", code, -1e9
        run_st["land_xy"] = (st["pos"][0], st["pos"][1]) if st["pos"] else None

    def tick():
        t = now()
        if t <= 0.0:
            return
        dt = 0.0 if run_st["t_last"] is None else max(0.0, min(0.1, t - run_st["t_last"]))
        run_st["t_last"] = t
        mode, ok, why = run_st["mode"], *healthy(t)
        if mode == "DONE":
            return
        owned = st["armed"] and st["offboard"]

        if stop and mode == "WAIT" and not (a.px4 and st["armed"]):
            return finish(0, "stopped before takeoff")
        if mode == "FLY" and not owned:
            return finish(1, "lost Offboard: pilot or failsafe has control; stopped publishing")
        if mode == "FLY":
            if stop:
                start_land("stop requested", 0)
            elif not ok:
                start_land(why, 2)
            elif not run_st["sq"].inside(st["pos"]):
                start_land(f"left the test box at {[round(v, 2) for v in st['pos']]}", 2)
            elif t - run_st["t_air"] > MAX_AIR_S:
                start_land("time limit", 2)
            mode = run_st["mode"]

        if mode == "WAIT":
            if not ok:
                return
            if a.px4:  # arm ourselves: 2 s settled on the ground, 1 s of setpoints, then Offboard + arm
                if not st["armed"]:
                    if not st["landed"]:
                        return
                    if run_st["anchor"] is None or math.dist(st["pos"], run_st["anchor"]) > 0.05:
                        run_st["anchor"], run_st["settled_t"], run_st["prime_t"] = list(st["pos"]), t, None
                        return
                    if t - run_st["settled_t"] < 2.0:
                        return
                    run_st["prime_t"] = run_st["prime_t"] or t
                    send_sp(run_st["anchor"], st["yaw"])
                    if t - run_st["prime_t"] > 1.0 and t - run_st["cmd_t"] > 1.0:
                        run_st["cmd_t"] = t
                        send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                        send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                    if t - run_st["prime_t"] > 9.0:
                        return finish(1, "PX4 refused Offboard/arm")
                    return
                if not owned:
                    if run_st["prime_t"] is None:
                        return finish(1, "already armed by someone else: not taking over")
                    send_sp(run_st["anchor"], st["yaw"])
                    return
            else:  # Skybeta: stream where we are until the pilot's Position switch hands us Offboard
                if not owned:
                    send_pose(st["pos"])
                    return
            run_st["sq"] = Square(st["pos"], st["yaw"], side=a.side, alt=a.alt, left=a.left)
            run_st["mode"], run_st["t_air"] = "FLY", t
            log.info(f"FLY: square {a.side:.3f} m at {a.alt:.2f} m from {[round(v, 2) for v in st['pos']]}, "
                     f"yaw {math.degrees(st['yaw']):.0f} deg, {'left' if a.left else 'right'} turns")
            mode = "FLY"

        if mode == "FLY":
            sq = run_st["sq"]
            phase = sq.step(st["pos"], st["yaw"], dt)
            if phase != run_st["phase"]:
                run_st["phase"] = phase
                leg = f" side {sq.leg + 1}/4 heading {math.degrees(sq.heading()):.0f} deg" if phase in ("TURN", "MOVE") else ""
                log.info(f"[{phase}]{leg} at {[round(v, 2) for v in st['pos']]}")
            if phase == "LAND":
                start_land("square complete", 0)
            elif a.px4:
                send_sp(sq.sp, sq.sp_yaw)
            else:
                send_twist(*track_twist(sq, st["pos"], st["yaw"]))
            return

        if mode == "LAND":
            if a.px4:
                if not st["armed"] and st["landed"]:
                    return finish(run_st["code"], "landed and disarmed")
                if not st.get("auto_land") and t - run_st["cmd_t"] > 1.0:
                    run_st["cmd_t"] = t
                    send_cmd(LAND_CMD)
                if st["offboard"] and st["pos"] is not None:  # keep the stream alive until PX4 takes Land
                    send_sp(st["pos"], st["yaw"])
            else:
                if not owned:  # disarmed on landing (COM_DISARM_LAND), or the pilot took over
                    return finish(run_st["code"], "landed and disarmed" if not st["armed"] else "pilot has control")
                sq = run_st["sq"] or Square(st["pos"], st["yaw"])
                land_xy = run_st["land_xy"] or (st["pos"][0], st["pos"][1])
                send_twist(*land_twist(sq, st["pos"], land_xy))

    node.create_timer(1.0 / 30.0, tick)
    log.info(f"square_test ready ({'plain PX4, arms itself' if a.px4 else 'Skybeta: pilot arms, then Position'})")
    try:
        while rclpy.ok() and run_st["mode"] != "DONE":
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        if a.px4 and st["armed"]:
            for _ in range(5):
                send_cmd(LAND_CMD)
            log.warning("second stop signal: NAV_LAND sent, exiting")
        elif st["armed"]:
            log.warning("second stop signal: exiting; flight_interface holds the last setpoint, pilot takes over")
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return run_st["code"]


def main():
    argv = sys.argv[1:]
    ros_args = []
    if "--ros-args" in argv:
        i = argv.index("--ros-args")
        argv, ros_args = argv[:i], argv[i:]
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--px4", action="store_true", help="plain PX4 /fmu/in (FlyX lab): arms and flies by itself")
    ap.add_argument("--side", type=float, default=0.305, help="square side, m (default 1 ft)")
    ap.add_argument("--alt", type=float, default=0.7, help="height above the takeoff point, m")
    ap.add_argument("--left", action="store_true", help="turn left at the corners instead of right")
    ap.add_argument("--odom-topic", default="/visual_slam/tracking/odometry", help="Skybeta mode odometry")
    ap.add_argument("--selftest", action="store_true", help="offline check, no ROS")
    a = ap.parse_args(argv)
    if not 0.1 <= a.side <= 1.0 or not 0.4 <= a.alt <= 1.5:
        ap.error("--side must be 0.1-1.0 m and --alt 0.4-1.5 m")
    if a.selftest:
        return selftest()
    a.ros_args = [sys.argv[0]] + ros_args
    return run(a)


if __name__ == "__main__":
    sys.exit(main())
