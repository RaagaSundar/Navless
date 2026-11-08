#!/usr/bin/env python3
"""FlyX lab only: record sim truth vs PX4 estimate vs cuVSLAM state at 10 Hz. Commands nothing.

  python3 tools/sim_truth_recorder.py > /tmp/truth.csv   # Ctrl-C to stop

Columns: sim_t, truth N/E/U relative to the PX4 local origin, PX4 local x/y/up, error_xy (m),
vo_state (1 tracking, 2 lost), PX4 flags ev_pos/ev_yaw/mag_hdg/mag_3d, nav_state, arming_state,
true and PX4 heading (deg, cw from north), and the running commander's setpoint (local x/y/up, yaw deg).
Truth comes from dronehacks_sim (simulation oracle) and must never feed flight code.
"""
import math
import sys

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import EstimatorStatusFlags, TrajectorySetpoint, VehicleLocalPosition, VehicleStatus
from dronehacks_sim import Simulation, Unavailable, WorldChanged

# Course ENU of PX4's local origin, measured in the lab on 2026-11-07 (spawn after Reset world).
ORIGIN_ENU = (-2.515, 1.114, 0.126)

qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 history=HistoryPolicy.KEEP_LAST, depth=1)
rclpy.init()
node = rclpy.create_node("est_recorder")
node.set_parameters([Parameter("use_sim_time", value=True)])
last = {}
node.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", lambda m: last.update(pos=m), qos)
node.create_subscription(EstimatorStatusFlags, "/fmu/out/estimator_status_flags", lambda m: last.update(flags=m), qos)
node.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", lambda m: last.update(status=m), qos)
# whatever commander is running: its latest setpoint (local NED position + yaw)
node.create_subscription(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", lambda m: last.update(sp=m),
                         QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1))
try:
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
    node.create_subscription(VisualSlamStatus, "/visual_slam/status", lambda m: last.update(vo=m.vo_state), 10)
except ImportError:
    print("# VisualSlamStatus not importable; vo_state column stays blank", file=sys.stderr)


def flag(name):
    return int(bool(getattr(last["flags"], name, False))) if "flags" in last else ""


def deg(rad):
    return f"{math.degrees(rad):.1f}" if rad is not None and math.isfinite(rad) else ""


print("sim_t,truth_n,truth_e,truth_u,px4_x,px4_y,px4_up,err_xy,vo_state,ev_pos,ev_yaw,mag_hdg,mag_3d,nav_state,arming_state,"
      "truth_hdg_deg,px4_hdg_deg,sp_x,sp_y,sp_up,sp_yaw_deg", flush=True)
with Simulation() as sim:
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                truth = sim.pose()
            except WorldChanged:
                print("# world reset; stopping", file=sys.stderr)
                break
            except Unavailable:
                continue
            e, n, u = truth.position_enu_m
            tn, te, tu = n - ORIGIN_ENU[1], e - ORIGIN_ENU[0], u - ORIGIN_ENU[2]
            p, st, sp = last.get("pos"), last.get("status"), last.get("sp")
            est = f"{p.x:.3f},{p.y:.3f},{-p.z:.3f},{math.hypot(p.x - tn, p.y - te):.3f}" if p else ",,,"
            qx, qy, qz, qw = truth.orientation_xyzw  # body FLU -> world ENU; heading = cw from north
            yaw_enu = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            truth_hdg = math.atan2(math.sin(math.pi / 2 - yaw_enu), math.cos(math.pi / 2 - yaw_enu))
            spc = (f"{sp.position[0]:.3f},{sp.position[1]:.3f},{-sp.position[2]:.3f},{deg(sp.yaw)}" if sp else ",,,")
            print(f"{truth.sim_time_ns / 1e9:.2f},{tn:.3f},{te:.3f},{tu:.3f},{est},{last.get('vo', '')},"
                  f"{flag('cs_ev_pos')},{flag('cs_ev_yaw')},{flag('cs_mag_hdg')},{flag('cs_mag_3d')},"
                  f"{getattr(st, 'nav_state', '')},{getattr(st, 'arming_state', '')},"
                  f"{deg(truth_hdg)},{deg(p.heading) if p else ''},{spc}", flush=True)
    except KeyboardInterrupt:
        pass
node.destroy_node()
rclpy.try_shutdown()
