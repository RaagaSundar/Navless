#!/usr/bin/env python3
"""Fly a planner route.json through PX4 Offboard. Sole owner of /fmu/in/vehicle_command.

  python3 mission/mission.py --check route_gate_b.json        # offline: no ROS, nothing flies
  python3 mission/mission.py route_gate_b.json --ros-args -p use_sim_time:=true

Do NOT run planner/px4_bridge_node.py at the same time: both publish /fmu/in/trajectory_setpoint.

BEFORE THE FIRST REAL RUN: diff the topic names, QoS and VehicleCommand fields below
against onboard/examples/takeoff_land.py on the live image. Written while the sim was down.

Frames: route waypoints_enu are course ENU (x east, y north, z up; floor-centre origin).
PX4 local NED has its origin wherever the estimator started, so we subtract --launch and
rotate by --local-heading (course heading of the PX4 local x axis, deg, clockwise from
course north). Both knobs are measured on the day, never assumed.
"""
import argparse
import json
import math
import sys


# ---------- pure logic (no ROS; tested by test_mission.py) ----------

def course_to_local(p, launch, heading_deg):
    n, e, d = p[1] - launch[1], p[0] - launch[0], -(p[2] - launch[2])
    c, s = math.cos(math.radians(heading_deg)), math.sin(math.radians(heading_deg))
    return (n * c + e * s, -n * s + e * c, d)


def dist(a, b):
    return math.dist(a, b)


def fence_for(path, margin):
    lo = [min(p[i] for p in path) - margin for i in range(3)]
    hi = [max(p[i] for p in path) + margin for i in range(3)]
    return lo, hi


def inside(p, fence):
    lo, hi = fence
    return all(lo[i] <= p[i] <= hi[i] for i in range(3))


def turn_toward(current, target, max_step):
    d = math.atan2(math.sin(target - current), math.cos(target - current))
    if abs(d) <= max_step:
        return target
    return math.atan2(math.sin(current + math.copysign(max_step, d)),
                      math.cos(current + math.copysign(max_step, d)))


class Carrot:
    """Setpoint slides along the path at `speed`, but never more than `max_lead` ahead of the drone."""

    def __init__(self, path, speed, max_lead, turn_slowdown=False):
        self.path, self.speed, self.max_lead = path, speed, max_lead
        self.turn_slowdown = bool(turn_slowdown)
        self.i, self.p = 0, tuple(path[0])

    def _turn_factor(self):
        if not self.turn_slowdown or len(self.path) < 3:
            return 1.0
        i0 = max(0, self.i - 1)
        i1 = min(len(self.path) - 1, self.i + 1)
        i2 = min(len(self.path) - 1, self.i + 3)
        if i1 == i0 or i2 == i1:
            return 1.0
        dx1, dy1 = self.path[i1][0] - self.path[i0][0], self.path[i1][1] - self.path[i0][1]
        dx2, dy2 = self.path[i2][0] - self.path[i1][0], self.path[i2][1] - self.path[i1][1]
        n1, n2 = math.hypot(dx1, dy1), math.hypot(dx2, dy2)
        if n1 < 1e-4 or n2 < 1e-4:
            return 1.0
        cos_a = max(-1.0, min(1.0, (dx1 * dx2 + dy1 * dy2) / (n1 * n2)))
        ang = math.acos(cos_a)
        return max(0.50, 1.0 - 0.50 * min(1.0, max(0.0, (ang - 0.20) / 0.80)))

    def step(self, drone, dt, vel=None):
        tf = self._turn_factor()
        eff_lead = self.max_lead * tf
        if dist(drone, self.p) <= eff_lead:
            budget = self.speed * tf * dt
            while budget > 0 and self.i < len(self.path) - 1:
                nxt = self.path[self.i + 1]
                d = dist(self.p, nxt)
                if d <= budget:
                    self.p, self.i, budget = tuple(nxt), self.i + 1, budget - d
                else:
                    f = budget / d
                    self.p = tuple(a + (b - a) * f for a, b in zip(self.p, nxt))
                    budget = 0
        return self.p

    def done(self, drone, accept):
        return self.i == len(self.path) - 1 and dist(drone, self.p) < accept

    def heading(self, fallback=0.0):
        """Local NED yaw (atan2(east, north) = atan2(dy, dx)) of the current/next horizontal segment."""
        for j in range(self.i, len(self.path) - 1):
            dx = self.path[j + 1][0] - self.path[j][0]
            dy = self.path[j + 1][1] - self.path[j][1]
            if math.hypot(dx, dy) > 1e-3:
                return math.atan2(dy, dx)
        return fallback


def local_to_course(ned, launch, heading_deg):
    c, s = math.cos(math.radians(heading_deg)), math.sin(math.radians(heading_deg))
    n = ned[0] * c - ned[1] * s
    e = ned[0] * s + ned[1] * c
    return (launch[0] + e, launch[1] + n, launch[2] - ned[2])


def load_route(path, launch, heading):
    with open(path) as f:
        r = json.load(f)
    enu = r["waypoints_enu"]
    if launch is None:  # planner's launch mark, else its first point on the floor
        launch = tuple(r.get("launch_origin_enu") or (enu[0][0], enu[0][1], 0.0))
    return [course_to_local(p, launch, heading) for p in enu], launch


def _import_live_planner():
    import os
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    nav_pkg = os.path.join(repo_root, "navless_nav")
    for p in (repo_root, nav_pkg):
        if p not in sys.path:
            sys.path.insert(0, p)
    from navless_nav.route import create_live_course_planner, plan_dynamic_course_route
    return create_live_course_planner, plan_dynamic_course_route


def plan_route(route_arg, launch, heading, router=None):
    """Plan a 3D A* + EGO B-spline route dynamically from the 3D occupancy grid (never relying on static route.json)."""
    _, plan_dynamic_course_route = _import_live_planner()
    branch = "gate_b" if "gate_b" in str(route_arg).lower() else "window"
    start_enu = (float(launch[0]), float(launch[1]), 0.85) if launch is not None else None
    enu, launch_xy, _ = plan_dynamic_course_route(branch=branch, start_enu=start_enu, router=router)
    if launch is None:
        launch = (float(launch_xy[0]), float(launch_xy[1]), 0.0)
    return [course_to_local(p, launch, heading) for p in enu], launch


def check(route, fence_margin):
    steps = [dist(a, b) for a, b in zip(route, route[1:])]
    lo, hi = fence_for(route, fence_margin)
    print(f"{len(route)} waypoints, {sum(steps):.2f} m, max step {max(steps):.2f} m")
    print(f"first local NED {tuple(round(v, 3) for v in route[0])}, last {tuple(round(v, 3) for v in route[-1])}")
    print(f"fence NED lo {[round(v, 2) for v in lo]} hi {[round(v, 2) for v in hi]}")
    assert max(steps) < 1.5, "gap in route > 1.5 m"
    assert all(p[2] < 0 for p in route), "route touches/below floor"


# ---------- ROS node ----------

STALE_S = {"pos": 0.5, "status": 1.5, "land": 2.5}
RATE_HZ = 30.0
PREARM_SETTLE_S = 2.0


def run(route, a):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                              VehicleStatus, VehicleLocalPosition, VehicleLandDetected)

    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)

    class Mission(Node):
        def __init__(self):
            super().__init__("navless_mission")
            self.p_mode = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
            self.p_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos)
            self.p_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos)
            self.msg, self.t = {}, {}
            for k, typ, topic in (("status", VehicleStatus, "/fmu/out/vehicle_status_v1"),
                                  ("pos", VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1"),
                                  ("land", VehicleLandDetected, "/fmu/out/vehicle_land_detected")):
                self.create_subscription(typ, topic, lambda m, k=k: self._rx(k, m), qos)
            self.vo_state = 1
            try:
                from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
                self.create_subscription(VisualSlamStatus, "/visual_slam/status",
                                         lambda m: setattr(self, "vo_state", int(m.vo_state)), 10)
            except ImportError:
                pass
            create_live_course_planner, self._plan_dynamic = _import_live_planner()
            self.planner_cfg, self.planner_grid, self.planner_router = create_live_course_planner()
            self.launch_enu = a.launch_enu
            self.branch = "gate_b" if "gate_b" in str(a.route).lower() else "window"
            self.live_grid_updated = False
            self.last_replan_ns = 0
            try:
                from sensor_msgs.msg import PointCloud2
                self.create_subscription(PointCloud2, "/team/stereo/points", self._pointcloud_cb, 10)
            except ImportError:
                pass
            self.state, self.ticks, self.timer = "PREFLIGHT", 0, None
            self.pos_zero = (0.0, 0.0, 0.0)
            self.prearm_anchor = None
            self.prearm_stable_since_ns = None
            self.prev_pos = None
            self.unhealthy_ticks = 0
            self.exit_code = 0

        def _rx(self, k, m):
            self.msg[k], self.t[k] = m, self.get_clock().now()

        def now_us(self):
            return self.get_clock().now().nanoseconds // 1000

        def fresh(self):
            now = self.get_clock().now()
            return all(k in self.t and (now - self.t[k]).nanoseconds < STALE_S[k] * 1e9 for k in STALE_S)

        def raw_pos(self):
            m = self.msg["pos"]
            return (float(m.x), float(m.y), float(m.z))

        def pos(self):
            rx, ry, rz = self.raw_pos()
            return (rx - self.pos_zero[0], ry - self.pos_zero[1], rz - self.pos_zero[2])

        def est_ok(self):
            m = self.msg["pos"]
            return bool(m.xy_valid and m.z_valid and self.vo_state == 1)

        def armed(self):
            return self.msg["status"].arming_state == VehicleStatus.ARMING_STATE_ARMED

        def offboard(self):
            return self.msg["status"].nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        def preflight_ok(self):
            if not (self.fresh() and self.est_ok() and self.msg["land"].landed and not self.armed()):
                self.prearm_anchor = None
                self.prearm_stable_since_ns = None
                return False
            rp = self.raw_pos()
            m = self.msg["pos"]
            vel = math.hypot(float(getattr(m, "vx", 0.0)), float(getattr(m, "vy", 0.0)), float(getattr(m, "vz", 0.0)))
            now_ns = self.get_clock().now().nanoseconds
            if self.prearm_anchor is None or dist(rp, self.prearm_anchor) > 0.05 or vel > 0.15:
                self.prearm_anchor = rp
                self.prearm_stable_since_ns = now_ns
                return False
            if (now_ns - self.prearm_stable_since_ns) < int(PREARM_SETTLE_S * 1e9):
                return False
            # Zero-tare the settled ground reading so local (0,0,0) is anchored on the launch pad
            self.pos_zero = rp
            return True

        def cmd(self, command, p1=0.0, p2=0.0):
            m = VehicleCommand()
            m.command, m.param1, m.param2 = command, float(p1), float(p2)
            m.target_system = m.target_component = m.source_system = m.source_component = 1
            m.from_external, m.timestamp = True, self.now_us()
            self.p_cmd.publish(m)

        def publish_sp(self, p):
            m = OffboardControlMode()
            m.position, m.timestamp = True, self.now_us()
            self.p_mode.publish(m)
            sp = TrajectorySetpoint()
            sp.position = [float(v + z0) for v, z0 in zip(p, self.pos_zero)]
            sp.velocity = sp.acceleration = [math.nan] * 3
            sp.yaw, sp.timestamp = float(self.yaw), self.now_us()
            self.p_sp.publish(sp)

        def _pointcloud_cb(self, msg):
            if self.state != "FLY" or "pos" not in self.msg:
                return
            from planner.px4_bridge_node import decode_ros_pointcloud2_optical
            cur_enu = local_to_course(self.pos(), self.launch_enu, a.local_heading)
            yaw_enu = 0.5 * math.pi - float(self.msg["pos"].heading) - math.radians(a.local_heading)
            pts_world = decode_ros_pointcloud2_optical(msg, cur_enu, yaw_enu, max_depth_m=2.80)
            if pts_world.size > 0:
                if self.planner_grid.insert_point_cloud(pts_world, sensor_origin_m=cur_enu, min_live_hits=2) > 0:
                    self.live_grid_updated = True

        def _maybe_replan(self, here):
            now_ns = self.get_clock().now().nanoseconds
            if not self.live_grid_updated or (now_ns - self.last_replan_ns) < int(0.5e9):
                return
            self.live_grid_updated = False
            rem = self.carrot.path[self.carrot.i:]
            if not any(self.planner_grid.is_occupied(local_to_course(q, self.launch_enu, a.local_heading), inflated=True) for q in rem):
                return
            cur_enu = local_to_course(here, self.launch_enu, a.local_heading)
            try:
                wps_enu, _, kind = self._plan_dynamic(branch=self.branch, start_enu=cur_enu, router=self.planner_router)
                new_path = [course_to_local(p, self.launch_enu, a.local_heading) for p in wps_enu]
                self.carrot = Carrot(new_path, a.speed, a.max_lead, turn_slowdown=True)
                self.last_replan_ns = now_ns
                self.get_logger().info(f"Online 3D A* + B-spline replan ({kind}): {len(new_path)} pts")
            except (ValueError, RuntimeError) as exc:
                self.get_logger().warn(f"Online replan skipped ({exc})")

        def start(self):
            self.pos_zero = self.raw_pos()
            here = self.pos()
            self.prev_pos = here
            path = [here] + route  # climb from zeroed launch pad origin
            self.fence = fence_for(path, a.fence_margin)
            self.carrot = Carrot(path, a.speed, a.max_lead, turn_slowdown=True)
            self.yaw = self.msg["pos"].heading  # ponytail: fixed heading; face-travel if depth needs it
            self.state = "STREAM"
            self.timer = self.create_timer(1.0 / RATE_HZ, self.tick)

        def land(self, why):
            self.get_logger().warn(f"LAND: {why}")
            self.cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.state, self.ticks = "LANDING", 0

        def finish(self, code, why):
            self.get_logger().info(f"DONE ({code}): {why}")
            self.exit_code, self.state = code, "DONE"
            self.timer.cancel()

        def tick(self):
            self.ticks += 1
            s = self.state
            if s == "LANDING":  # PX4 Land mode owns the vehicle; we only watch
                if self.fresh() and self.msg["land"].landed and not self.armed():
                    self.finish(self.exit_code, "landed and disarmed")
                elif self.ticks % 30 == 0:
                    self.cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)  # re-send once a second until landed
                return
            if not self.fresh() or not self.est_ok():
                self.unhealthy_ticks += 1
                if self.unhealthy_ticks <= int(RATE_HZ) and s in ("TAKEOFF", "FLY", "HOVER"):
                    self.publish_sp(route[0] if s == "TAKEOFF" else self.carrot.p)
                    return
                self.exit_code = 2
                return self.land("telemetry stale or estimator invalid")
            self.unhealthy_ticks = 0
            here = self.pos()
            if self.prev_pos is not None and self.armed() and dist(here, self.prev_pos) > 0.85:
                self.exit_code = 6
                return self.land(f"impossible position jump {self.prev_pos} -> {here}")
            self.prev_pos = here
            if s == "STREAM":  # PX4 needs a setpoint stream before it accepts Offboard
                if not self.armed():
                    self.pos_zero = self.raw_pos()
                self.publish_sp((0.0, 0.0, 0.0) if not self.armed() else route[0])
                if self.ticks == int(RATE_HZ):
                    self.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
                    self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1)
                if self.armed() and self.offboard():
                    self.prev_pos = self.pos()
                    self.state = "TAKEOFF"
                elif self.ticks > 4 * RATE_HZ:
                    self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0)
                    self.finish(3, "PX4 refused offboard/arm; disarm sent")
                return
            if not self.offboard():  # pilot/failsafe took over: never fight it
                return self.finish(4, f"left Offboard (nav_state={self.msg['status'].nav_state}); stopped publishing")
            if not inside(here, self.fence):
                self.exit_code = 5
                return self.land(f"geofence breach at {here}")
            if s == "TAKEOFF":
                self.publish_sp(route[0])
                if here[2] <= route[0][2] + 0.20:
                    self.state = "FLY"
            elif s == "FLY":
                self._maybe_replan(here)
                sp_next = self.carrot.step(here, 1.0 / RATE_HZ)
                if not a.no_face_travel:
                    self.yaw = turn_toward(self.yaw, self.carrot.heading(self.yaw), a.max_yaw_rate / RATE_HZ)
                self.publish_sp(sp_next)
                if self.carrot.done(here, a.accept):
                    self.state, self.ticks = "HOVER", 0
            elif s == "HOVER":
                self.publish_sp(self.carrot.p)
                if self.ticks > 2 * RATE_HZ:
                    self.land("route complete")

    rclpy.init(args=a.ros_args)
    node = Mission()
    try:
        deadline = None
        while not node.preflight_ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now_ns = node.get_clock().now().nanoseconds
            if now_ns > 0 and deadline is None:
                deadline = now_ns + int(30e9)
            if deadline is not None and now_ns > deadline:
                print("Preflight failed: need 2.0 s of fresh status/position/land-detected, valid stationary estimator, landed, disarmed.")
                print({k: (k in node.msg) for k in ("status", "pos", "land")})
                return 1
        print(f"Ready (2.0 s settled & zeroed). Zeroed local position {tuple(round(v, 2) for v in node.pos())}, raw {tuple(round(v, 2) for v in node.raw_pos())}, heading {node.msg['pos'].heading:.2f} rad.")
        print(f"Planned 3D A* + B-spline Route: {len(route)} pts, first {tuple(round(v, 2) for v in route[0])}, speed {a.speed} m/s.")
        if input("Team-approved? Type ARM to fly: ").strip() != "ARM":
            print("Not armed.")
            return 1
        for _ in range(25):  # input() blocked callbacks; re-verify 2.0 s settled state before arming
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.preflight_ok():
            print("Preflight no longer OK after confirmation. Not armed.")
            return 1
        node.start()
        while node.state != "DONE":
            rclpy.spin_once(node, timeout_sec=0.1)
        return node.exit_code
    except KeyboardInterrupt:  # Ctrl-C means land, not "stop streaming and fall into failsafe"
        if node.state not in ("PREFLIGHT", "DONE"):
            for _ in range(5):
                node.cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            print("Ctrl-C: Land commanded. Watch it touch down.")
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("route", nargs="?", default="dynamic",
                    help="branch ('window'/'gate_b'/'dynamic'); plans dynamically via 3D A* + B-spline")
    ap.add_argument("--check", action="store_true", help="validate offline, no ROS")
    ap.add_argument("--launch", type=float, nargs=3, metavar=("X", "Y", "Z"),
                    help="course ENU of PX4 local origin (default: launch mark xy, z=0)")
    ap.add_argument("--local-heading", type=float, default=0.0)
    ap.add_argument("--speed", type=float, default=0.5, help="m/s along path")
    ap.add_argument("--max-lead", type=float, default=0.6, help="m setpoint may lead the drone")
    ap.add_argument("--accept", type=float, default=0.15, help="m to call the final point reached")
    ap.add_argument("--fence-margin", type=float, default=0.5, help="m beyond route bbox before abort")
    ap.add_argument("--max-yaw-rate", type=float, default=1.40, help="rad/s yaw rate limit when facing travel")
    ap.add_argument("--no-face-travel", action="store_true", help="keep fixed takeoff yaw instead of facing travel")
    argv = sys.argv[1:]
    ros_args = []
    if "--ros-args" in argv:
        i = argv.index("--ros-args")
        argv, ros_args = argv[:i], argv[i:]
    a = ap.parse_args(argv)
    a.ros_args = [sys.argv[0]] + ros_args
    route, launch_enu = plan_route(a.route, a.launch, a.local_heading)
    a.launch_enu = launch_enu
    check(route, a.fence_margin)
    if not a.check:
        sys.exit(run(route, a))


if __name__ == "__main__":
    main()
