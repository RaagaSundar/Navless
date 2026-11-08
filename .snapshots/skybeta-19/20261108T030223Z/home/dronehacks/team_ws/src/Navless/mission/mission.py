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


class Carrot:
    """Setpoint slides along the path at `speed`, but never more than `max_lead` ahead of the drone."""

    def __init__(self, path, speed, max_lead):
        self.path, self.speed, self.max_lead = path, speed, max_lead
        self.i, self.p = 0, tuple(path[0])

    def step(self, drone, dt):
        if dist(drone, self.p) > self.max_lead:
            return self.p  # drone is lagging (wind, contact, bad estimate): wait for it
        budget = self.speed * dt
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


def load_route(path, launch, heading):
    with open(path) as f:
        r = json.load(f)
    enu = r["waypoints_enu"]
    if launch is None:  # planner's launch mark, else its first point on the floor
        launch = tuple(r.get("launch_origin_enu") or (enu[0][0], enu[0][1], 0.0))
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

STALE_S = 0.5
RATE_HZ = 30.0


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
            self.state, self.ticks, self.timer = "PREFLIGHT", 0, None
            self.exit_code = 0

        def _rx(self, k, m):
            self.msg[k], self.t[k] = m, self.get_clock().now()

        def now_us(self):
            return self.get_clock().now().nanoseconds // 1000

        def fresh(self):
            now = self.get_clock().now()
            return all(k in self.t and (now - self.t[k]).nanoseconds < STALE_S * 1e9 for k in ("status", "pos", "land"))

        def pos(self):
            m = self.msg["pos"]
            return (m.x, m.y, m.z)

        def est_ok(self):
            m = self.msg["pos"]
            return m.xy_valid and m.z_valid

        def armed(self):
            return self.msg["status"].arming_state == VehicleStatus.ARMING_STATE_ARMED

        def offboard(self):
            return self.msg["status"].nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        def preflight_ok(self):
            return self.fresh() and self.est_ok() and self.msg["land"].landed and not self.armed()

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
            sp.position = [float(v) for v in p]
            sp.velocity = sp.acceleration = [math.nan] * 3
            sp.yaw, sp.timestamp = float(self.yaw), self.now_us()
            self.p_sp.publish(sp)

        def start(self):
            here = self.pos()
            path = [here] + route  # climb from wherever we actually sit
            self.fence = fence_for(path, a.fence_margin)
            self.carrot = Carrot(path, a.speed, a.max_lead)
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
                self.exit_code = 2
                return self.land("telemetry stale or estimator invalid")
            here = self.pos()
            if s == "STREAM":  # PX4 needs a setpoint stream before it accepts Offboard
                self.publish_sp(self.carrot.p)
                if self.ticks == int(RATE_HZ):
                    self.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
                    self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1)
                if self.armed() and self.offboard():
                    self.state = "FLY"
                elif self.ticks > 4 * RATE_HZ:
                    self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0)
                    self.finish(3, "PX4 refused offboard/arm; disarm sent")
                return
            if not self.offboard():  # pilot/failsafe took over: never fight it
                return self.finish(4, f"left Offboard (nav_state={self.msg['status'].nav_state}); stopped publishing")
            if not inside(here, self.fence):
                self.exit_code = 5
                return self.land(f"geofence breach at {here}")
            if s == "FLY":
                self.publish_sp(self.carrot.step(here, 1.0 / RATE_HZ))
                if self.carrot.done(here, a.accept):
                    self.state, self.ticks = "HOVER", 0
            elif s == "HOVER":
                self.publish_sp(self.carrot.p)
                if self.ticks > 2 * RATE_HZ:
                    self.land("route complete")

    rclpy.init(args=a.ros_args)
    node = Mission()
    try:
        deadline = node.get_clock().now().nanoseconds + 30e9
        while not node.preflight_ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.get_clock().now().nanoseconds > deadline:
                print("Preflight failed: need fresh status/position/land-detected, valid estimator, landed, disarmed.")
                print({k: (k in node.msg) for k in ("status", "pos", "land")})
                return 1
        print(f"Ready. Local position {tuple(round(v, 2) for v in node.pos())}, heading {node.msg['pos'].heading:.2f} rad.")
        print(f"Route: {len(route)} pts, first {tuple(round(v, 2) for v in route[0])}, speed {a.speed} m/s.")
        if input("Team-approved? Type ARM to fly: ").strip() != "ARM":
            print("Not armed.")
            return 1
        for _ in range(20):  # input() blocked callbacks; refresh before re-checking
            rclpy.spin_once(node, timeout_sec=0.05)
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
    ap.add_argument("route")
    ap.add_argument("--check", action="store_true", help="validate offline, no ROS")
    ap.add_argument("--launch", type=float, nargs=3, metavar=("X", "Y", "Z"),
                    help="course ENU of PX4 local origin (default: first waypoint xy, z=0)")
    ap.add_argument("--local-heading", type=float, default=0.0)
    ap.add_argument("--speed", type=float, default=0.5, help="m/s along path")
    ap.add_argument("--max-lead", type=float, default=0.6, help="m setpoint may lead the drone")
    ap.add_argument("--accept", type=float, default=0.15, help="m to call the final point reached")
    ap.add_argument("--fence-margin", type=float, default=0.5, help="m beyond route bbox before abort")
    argv = sys.argv[1:]
    ros_args = []
    if "--ros-args" in argv:
        i = argv.index("--ros-args")
        argv, ros_args = argv[:i], argv[i:]
    a = ap.parse_args(argv)
    a.ros_args = [sys.argv[0]] + ros_args
    route, _ = load_route(a.route, a.launch, a.local_heading)
    check(route, a.fence_margin)
    if not a.check:
        sys.exit(run(route, a))


if __name__ == "__main__":
    main()
