#!/usr/bin/env python3
"""Read-only 10 Hz estimator vs. sim-truth CSV recorder for FlyX Lab / SITL diagnostics.

  python3 navless/tools/est_recorder.py > /tmp/est.csv

Commands nothing; never imported by flight code.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition, VehicleStatus


class EstRecorder(Node):
    def __init__(self) -> None:
        super().__init__("est_recorder")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pos = None
        self.status = None
        self.sp = None
        self.truth = None
        self.vo_state = -1
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._on_pos, qos)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self._on_status, qos)
        self.create_subscription(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", self._on_sp, qos)
        self.create_subscription(Odometry, "/visual_slam/tracking/odometry", self._on_truth, 10)
        try:
            from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
            self.create_subscription(VisualSlamStatus, "/visual_slam/status", self._on_vslam, 10)
        except ImportError:
            pass
        print("t_s,px4_x,px4_y,px4_z,px4_yaw,xy_valid,z_valid,vo_state,sp_x,sp_y,sp_z,odom_x,odom_y,odom_z", flush=True)
        self.create_timer(0.1, self._tick)

    def _on_pos(self, m: VehicleLocalPosition) -> None:
        self.pos = m

    def _on_status(self, m: VehicleStatus) -> None:
        self.status = m

    def _on_sp(self, m: TrajectorySetpoint) -> None:
        self.sp = m

    def _on_truth(self, m: Odometry) -> None:
        self.truth = m

    def _on_vslam(self, m: object) -> None:
        self.vo_state = int(getattr(m, "vo_state", -1))

    def _tick(self) -> None:
        if self.pos is None:
            return
        t_s = self.get_clock().now().nanoseconds * 1e-9
        px, py, pz = float(self.pos.x), float(self.pos.y), float(self.pos.z)
        yaw = float(getattr(self.pos, "heading", math.nan))
        xy_v = int(bool(self.pos.xy_valid))
        z_v = int(bool(self.pos.z_valid))
        if self.sp is not None and len(self.sp.position) >= 3:
            sx, sy, sz = float(self.sp.position[0]), float(self.sp.position[1]), float(self.sp.position[2])
        else:
            sx = sy = sz = math.nan
        if self.truth is not None:
            p = self.truth.pose.pose.position
            ox, oy, oz = float(p.x), float(p.y), float(p.z)
        else:
            ox = oy = oz = math.nan
        print(
            f"{t_s:.3f},{px:.3f},{py:.3f},{pz:.3f},{yaw:.3f},{xy_v},{z_v},{self.vo_state},"
            f"{sx:.3f},{sy:.3f},{sz:.3f},{ox:.3f},{oy:.3f},{oz:.3f}",
            flush=True,
        )


def main() -> int:
    rclpy.init(args=sys.argv)
    node = EstRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
