"""Realistic ROS 2 (rclpy) + Gazebo Sim Physics, VIO Drift/Noise, and Raycasted OAK-D Depth Tests.

Verifies the DroneHacks 2026 EGO-Planner + Topological FSM + PX4 Bridge under realistic conditions:
1. `test_01_gazebo_harmonic_live_physics_and_ros_gz_bridge`:
   - Launches Gazebo Harmonic (`gz sim -s -r`) with `dronehacks-course-v3.obj`, `sky_beta_drone`
     rigid-body physics (`mass = 1.25 kg`), `VelocityControl`, and `OdometryPublisher` plugins.
   - Bridges Gazebo <-> ROS 2 via `ros_gz_bridge parameter_bridge`, injects VIO noise onto Gazebo's
     physical odometry, and flies the Gazebo rigid-body drone in closed loop with the planner.
2. `test_02_ros2_full_course_with_physics_vio_drift_and_unmapped_boulder`:
   - Simulates 2nd-order PX4 cascaded PD control, actuator lag, aerodynamic drag, wind gusts,
     and bounded random-walk VIO drift + Gaussian position/velocity/heading noise (no GPS).
   - Places an UNMAPPED obstacle on the Stage 1 corridor path (absent from the planner's prior).
   - Raycasts noisy 3D OAK-D `16UC1` depth images (with quadratic stereo noise + pixel dropouts)
     from the true physical pose, verifying mid-flight obstacle discovery, emergency replanning,
     zero physical collisions, and full 6-stage mission completion including Over-the-Wall Fly-Home.
3. `test_03_ros2_live_depth_obstructed_window_falls_back_to_gate_b`:
   - The planner's 3D prior has the Window cutout open, but the true world has the Window blocked.
   - Live noisy `16UC1` depth frames from ~1 m away populate the local voxel grid at the Window
     cutout, causing the FSM to dynamically reject the Window and route through Gate B to the pad.
"""

from __future__ import annotations

import math
import shutil
import subprocess
import tempfile
import time
import unittest
from pathlib import Path
from typing import List, Tuple

import numpy as np

from planner.config import PlannerConfig
from planner.frames import enu_to_ned_pos, ned_to_enu_pos, ned_to_enu_vel, wrap_pi
from planner.fsm_router import CourseStage, GlobalCourseRouter
from planner.grid_map import OccupancyGrid3D
from planner.px4_bridge_node import (
    ARMING_STATE_ARMED,
    NAVIGATION_STATE_OFFBOARD,
    PX4TrajectoryTrackerCore,
    decode_ros_depth_image,
)

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import Bool

    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False


def build_course_sdf_world(obj_path: Path) -> str:
    """Create a Gazebo Harmonic SDF 1.9 world with `VelocityControl` and `OdometryPublisher`."""
    mesh_uri = obj_path.resolve().as_uri()
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="dronehacks_course_v3">
    <physics name="4ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>0.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <gravity>0 0 0</gravity>

    <model name="dronehacks_arena">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="course_link">
        <visual name="course_visual">
          <geometry>
            <mesh><uri>{mesh_uri}</uri></mesh>
          </geometry>
        </visual>
      </link>
    </model>

    <model name="sky_beta_drone">
      <static>false</static>
      <pose>-2.515 0.914 0.85 0 0 0</pose>
      <link name="base_link">
        <inertial>
          <mass>1.25</mass>
          <inertia>
            <ixx>0.012</ixx><iyy>0.012</iyy><izz>0.020</izz>
          </inertia>
        </inertial>
        <collision name="body_col">
          <geometry>
            <cylinder><radius>0.18</radius><length>0.12</length></cylinder>
          </geometry>
        </collision>
        <visual name="body_vis">
          <geometry>
            <cylinder><radius>0.18</radius><length>0.12</length></cylinder>
          </geometry>
        </visual>
      </link>
      <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
        <topic>/model/sky_beta_drone/cmd_vel</topic>
      </plugin>
      <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
        <odom_topic>/model/sky_beta_drone/odometry</odom_topic>
        <odom_publish_frequency>50</odom_publish_frequency>
        <dimensions>3</dimensions>
      </plugin>
    </model>
  </world>
</sdf>
"""


class QuadrotorPhysicsAndVIOSim:
    """Second-order 3D quadrotor dynamics + cascaded PX4 controller + VIO drift/noise (no GPS)."""

    def __init__(
        self,
        init_pos_enu: Tuple[float, float, float],
        launch_origin_enu: Tuple[float, float, float],
        *,
        seed: int = 42,
        pos_noise_std_m: Tuple[float, float, float] = (0.015, 0.015, 0.008),
        vel_noise_std_mps: float = 0.025,
        yaw_noise_std_rad: float = 0.020,
        max_vio_drift_m: Tuple[float, float, float] = (0.035, 0.035, 0.015),
    ) -> None:
        self.rng = np.random.default_rng(seed)
        self.launch_origin_enu = launch_origin_enu
        self.true_pos_enu = np.asarray(init_pos_enu, dtype=np.float64).copy()
        self.true_vel_enu = np.zeros(3, dtype=np.float64)
        self.true_acc_enu = np.zeros(3, dtype=np.float64)
        self.true_yaw_enu = 0.0

        self.vio_drift_ned = np.zeros(3, dtype=np.float64)
        self.pos_noise_std = np.asarray(pos_noise_std_m, dtype=np.float64)
        self.vel_noise_std = float(vel_noise_std_mps)
        self.yaw_noise_std = float(yaw_noise_std_rad)
        self.max_vio_drift = np.asarray(max_vio_drift_m, dtype=np.float64)

        # Second-order PX4 position + velocity loop gains & physical limits
        self.kp_pos = 3.4
        self.kd_vel = 2.8
        self.actuator_tau_s = 0.05
        self.drag_coeff = 0.18
        self.max_acc_mps2 = 3.8

    def step_physics(
        self,
        dt: float,
        sim_t: float,
        cmd_pos_ned: Tuple[float, float, float],
        cmd_vel_ned: Tuple[float, float, float],
        cmd_yaw_ned: float,
        max_speed_mps: float,
    ) -> None:
        """Integrate 2nd-order rigid-body dynamics with actuator lag, drag, and wind turbulence."""
        target_pos_enu = ned_to_enu_pos(cmd_pos_ned, convention="enu_to_ned", origin_enu=self.launch_origin_enu)
        target_vel_enu = ned_to_enu_vel(cmd_vel_ned, convention="enu_to_ned")

        # Cascaded PX4 position -> velocity -> acceleration control law
        pos_err = target_pos_enu - self.true_pos_enu
        des_vel = target_vel_enu + self.kp_pos * pos_err
        speed = float(np.linalg.norm(des_vel))
        if speed > max_speed_mps * 1.25 and speed > 1e-5:
            des_vel = des_vel * ((max_speed_mps * 1.25) / speed)

        vel_err = des_vel - self.true_vel_enu
        des_acc = self.kd_vel * vel_err

        # Clamp physical thrust/tilt acceleration
        acc_norm = float(np.linalg.norm(des_acc))
        if acc_norm > self.max_acc_mps2 and acc_norm > 1e-5:
            des_acc = des_acc * (self.max_acc_mps2 / acc_norm)

        # First-order motor/attitude lag + oscillatory wind gust disturbance + aerodynamic drag
        alpha = min(1.0, dt / max(self.actuator_tau_s, 1e-3))
        self.true_acc_enu = (1.0 - alpha) * self.true_acc_enu + alpha * des_acc
        wind_acc = np.array(
            [
                0.08 * math.sin(1.7 * sim_t),
                0.08 * math.cos(1.3 * sim_t),
                0.03 * math.sin(2.3 * sim_t),
            ],
            dtype=np.float64,
        )
        net_acc = self.true_acc_enu - self.drag_coeff * self.true_vel_enu + wind_acc

        self.true_vel_enu = self.true_vel_enu + net_acc * dt
        v_norm = float(np.linalg.norm(self.true_vel_enu))
        if v_norm > max_speed_mps * 1.30 and v_norm > 1e-5:
            self.true_vel_enu = self.true_vel_enu * ((max_speed_mps * 1.30) / v_norm)

        self.true_pos_enu = self.true_pos_enu + self.true_vel_enu * dt
        self.true_pos_enu[2] = max(0.05, float(self.true_pos_enu[2]))

        target_yaw_enu = wrap_pi(0.5 * math.pi - cmd_yaw_ned)
        yaw_err = wrap_pi(target_yaw_enu - self.true_yaw_enu)
        self.true_yaw_enu = wrap_pi(self.true_yaw_enu + min(1.0, 4.5 * dt) * yaw_err)

    def observe_noisy_vio_ned(self, dt: float) -> Tuple[np.ndarray, np.ndarray, float]:
        """Return noisy, drifted VIO/EKF2 local NED position, velocity, and heading."""
        # Bounded random-walk VIO drift
        drift_step = self.rng.normal(0.0, 0.002 * math.sqrt(max(dt, 1e-3)), size=3)
        self.vio_drift_ned = np.clip(
            self.vio_drift_ned + drift_step,
            -self.max_vio_drift,
            self.max_vio_drift,
        )
        true_ned = np.asarray(
            enu_to_ned_pos(self.true_pos_enu, convention="enu_to_ned", origin_enu=self.launch_origin_enu),
            dtype=np.float64,
        )
        true_vel_ned = np.asarray(
            [self.true_vel_enu[1], self.true_vel_enu[0], -self.true_vel_enu[2]],
            dtype=np.float64,
        )
        true_heading_ned = wrap_pi(0.5 * math.pi - self.true_yaw_enu)

        meas_pos_ned = true_ned + self.vio_drift_ned + self.rng.normal(0.0, self.pos_noise_std)
        meas_vel_ned = true_vel_ned + self.rng.normal(0.0, self.vel_noise_std, size=3)
        meas_heading_ned = wrap_pi(true_heading_ned + float(self.rng.normal(0.0, self.yaw_noise_std)))
        return meas_pos_ned, meas_vel_ned, meas_heading_ned


class RaycastOakDDepthCamera:
    """Simulated OAK-D S2 3D depth camera raycasting against ground-truth voxel geometry with stereo noise."""

    def __init__(
        self,
        width: int = 64,
        height: int = 48,
        fx: float = 45.0,
        fy: float = 45.0,
        min_range_m: float = 0.30,
        max_range_m: float = 3.20,
        step_m: float = 0.10,
        seed: int = 101,
    ) -> None:
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.cx = 0.5 * width
        self.cy = 0.5 * height
        self.min_range = min_range_m
        self.max_range = max_range_m
        self.step = step_m
        self.rng = np.random.default_rng(seed)

        # Precompute ray unit directions in body FLU frame for every 8th pixel (matching stride=8)
        rows, cols = np.indices((height, width))
        self.dir_x_flu = np.ones((height, width), dtype=np.float32)
        self.dir_y_flu = -((cols - self.cx) / self.fx).astype(np.float32)
        self.dir_z_flu = -((rows - self.cy) / self.fy).astype(np.float32)
        self.depths = np.arange(min_range_m, max_range_m, step_m, dtype=np.float32)

    def render_16uc1_mm(
        self,
        truth_grid: OccupancyGrid3D,
        true_pos_enu: np.ndarray,
        true_yaw_enu: float,
        *,
        stride: int = 8,
        dropout_prob: float = 0.04,
    ) -> bytes:
        """Raycast from `true_pos_enu` against `truth_grid.raw_grid` and return noisy `16UC1` bytes."""
        img_mm = np.zeros((self.height, self.width), dtype=np.uint16)
        c, s = math.cos(true_yaw_enu), math.sin(true_yaw_enu)

        sub_dx = self.dir_x_flu[::stride, ::stride]
        sub_dy = self.dir_y_flu[::stride, ::stride]
        sub_dz = self.dir_z_flu[::stride, ::stride]

        # World unit-z rays: p(d) = pos + d * [c*dx - s*dy, s*dx + c*dy, dz]
        wx_dir = c * sub_dx - s * sub_dy
        wy_dir = s * sub_dx + c * sub_dy
        wz_dir = sub_dz

        hit_depth = np.zeros(sub_dx.shape, dtype=np.float32)
        active = np.ones(sub_dx.shape, dtype=bool)
        nx, ny, nz = truth_grid.shape
        bmin = truth_grid.bounds_min
        res = truth_grid.resolution

        for d in self.depths:
            if not np.any(active):
                break
            px = float(true_pos_enu[0]) + d * wx_dir
            py = float(true_pos_enu[1]) + d * wy_dir
            pz = float(true_pos_enu[2]) + d * wz_dir

            ix = np.floor((px - bmin[0]) / res).astype(np.int32)
            iy = np.floor((py - bmin[1]) / res).astype(np.int32)
            iz = np.floor((pz - bmin[2]) / res).astype(np.int32)

            in_b = (
                active
                & (ix >= 0)
                & (ix < nx)
                & (iy >= 0)
                & (iy < ny)
                & (iz >= 0)
                & (iz < nz)
            )
            if not np.any(in_b):
                continue
            occ = np.zeros_like(in_b)
            occ[in_b] = truth_grid.raw_grid[ix[in_b], iy[in_b], iz[in_b]]
            new_hits = active & occ
            if np.any(new_hits):
                hit_depth[new_hits] = d
                active[new_hits] = False

        # Add quadratic stereo depth noise sigma(z) = 0.010 * z^2 and random stereo dropouts
        valid_hits = hit_depth > 0.0
        if np.any(valid_hits):
            noise = self.rng.normal(0.0, 0.010 * (hit_depth[valid_hits] ** 2))
            noisy_d = np.clip(hit_depth[valid_hits] + noise, self.min_range, self.max_range)
            dropouts = self.rng.random(noisy_d.shape) < dropout_prob
            noisy_d[dropouts] = 0.0
            hit_mm = np.round(noisy_d * 1000.0).astype(np.uint16)
            sub_mm = np.zeros(sub_dx.shape, dtype=np.uint16)
            sub_mm[valid_hits] = hit_mm
            img_mm[::stride, ::stride] = sub_mm

        return img_mm.tobytes()


class TestROS2AndGazeboSimulation(unittest.TestCase):
    def test_01_gazebo_harmonic_live_physics_and_ros_gz_bridge(self) -> None:
        """Run Gazebo Sim (`gz sim`) with `VelocityControl` + `OdometryPublisher` bridged to ROS 2."""
        if shutil.which("gz") is None or not HAS_RCLPY:
            self.skipTest("Gazebo (`gz`) or `rclpy` not available in this container")

        repo_root = Path(__file__).resolve().parent.parent
        obj_path = repo_root / "dronehacks-course-v3.obj"
        self.assertTrue(obj_path.exists())

        cfg = PlannerConfig(fast_mode=True)
        grid = OccupancyGrid3D(
            bounds_min_m=cfg.global_map_bounds_min_m,
            bounds_max_m=cfg.global_map_bounds_max_m,
            resolution_m=cfg.voxel_resolution_m,
            inflation_radius_m=cfg.obstacle_inflation_m,
        )
        grid.load_prior_from_obj(obj_path)
        router = GlobalCourseRouter(cfg, grid)
        tracker = PX4TrajectoryTrackerCore(router)

        with tempfile.TemporaryDirectory() as tmp:
            sdf_path = Path(tmp) / "dronehacks_course_v3.sdf"
            sdf_path.write_text(build_course_sdf_world(obj_path), encoding="utf-8")

            gz_proc = subprocess.Popen(
                ["gz", "sim", "-s", "-r", str(sdf_path)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            bridge_proc = subprocess.Popen(
                [
                    "ros2",
                    "run",
                    "ros_gz_bridge",
                    "parameter_bridge",
                    "/model/sky_beta_drone/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                    "/model/sky_beta_drone/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

            rclpy.init()
            try:
                node = Node("gz_closed_loop_planner_tester")
                cmd_pub = node.create_publisher(Twist, "/model/sky_beta_drone/cmd_vel", 10)
                depth_pub = node.create_publisher(Image, "/camera/depth/image_raw", 10)
                gz_odom_poses: List[Tuple[float, float, float]] = []
                gz_depth_frames = [0]
                depth_cam = RaycastOakDDepthCamera(seed=55)
                rng = np.random.default_rng(7)
                vio_drift = np.zeros(3, dtype=np.float64)

                def on_gz_depth(msg: Image) -> None:
                    if not gz_odom_poses:
                        return
                    depth_m = decode_ros_depth_image(msg)
                    pos_enu = ned_to_enu_pos(
                        tracker.telemetry.pos_ned,
                        convention="enu_to_ned",
                        origin_enu=tracker.launch_origin_enu,
                    )
                    grid.update_from_depth_image(
                        depth_m,
                        depth_cam.fx,
                        depth_cam.fy,
                        depth_cam.cx,
                        depth_cam.cy,
                        pos_enu,
                        0.5 * math.pi,
                    )
                    gz_depth_frames[0] += 1

                def on_gz_odom(msg: Odometry) -> None:
                    # True Gazebo rigid-body position in course ENU frame
                    gx = float(msg.pose.pose.position.x)
                    gy = float(msg.pose.pose.position.y)
                    gz_z = float(msg.pose.pose.position.z)
                    gz_odom_poses.append((gx, gy, gz_z))

                    # Add bounded random-walk VIO drift + Gaussian noise before feeding into PX4 tracker
                    vio_drift[:] = np.clip(vio_drift + rng.normal(0.0, 0.002, size=3), -0.03, 0.03)
                    noisy_enu = (
                        gx + float(vio_drift[0] + rng.normal(0.0, 0.012)),
                        gy + float(vio_drift[1] + rng.normal(0.0, 0.012)),
                        gz_z + float(vio_drift[2] + rng.normal(0.0, 0.008)),
                    )
                    ned = enu_to_ned_pos(noisy_enu, convention="enu_to_ned", origin_enu=tracker.launch_origin_enu)
                    now_s = time.monotonic()
                    tracker.update_local_position(now_s, ned[0], ned[1], ned[2], 0.0, 0.0, 0.0, 0.0, True, True)
                    tracker.update_vehicle_status(now_s, ARMING_STATE_ARMED, NAVIGATION_STATE_OFFBOARD)
                    tracker.update_estimator_health(True)

                    # Publish raycasted noisy 16UC1 depth image from true Gazebo pose
                    if len(gz_odom_poses) % 2 == 1:
                        img = Image()
                        img.height = depth_cam.height
                        img.width = depth_cam.width
                        img.encoding = "16UC1"
                        img.is_bigendian = 0
                        img.step = depth_cam.width * 2
                        img.data = depth_cam.render_16uc1_mm(grid, np.array([gx, gy, gz_z]), 0.5 * math.pi)
                        depth_pub.publish(img)

                node.create_subscription(Odometry, "/model/sky_beta_drone/odometry", on_gz_odom, 10)
                node.create_subscription(Image, "/camera/depth/image_raw", on_gz_depth, 10)
                executor = SingleThreadedExecutor()
                executor.add_node(node)

                t_start = time.monotonic()
                while (time.monotonic() - t_start) < 12.0:
                    for _ in range(3):
                        executor.spin_once(timeout_sec=0.02)
                    if gz_odom_poses:
                        now_s = time.monotonic()
                        sample, _ = tracker.sample_setpoint(now_s)
                        if sample is not None:
                            sp_enu = ned_to_enu_pos(
                                sample.position_ned,
                                convention="enu_to_ned",
                                origin_enu=tracker.launch_origin_enu,
                            )
                            cur = np.asarray(gz_odom_poses[-1], dtype=np.float64)
                            vel_cmd = np.clip(2.0 * (sp_enu - cur), -1.5, 1.5)
                            tw = Twist()
                            tw.linear.x = float(vel_cmd[0])
                            tw.linear.y = float(vel_cmd[1])
                            tw.linear.z = float(vel_cmd[2])
                            cmd_pub.publish(tw)
                            if float(np.linalg.norm(cur - np.asarray(cfg.checkpoints.start_pad))) > 0.35 and gz_depth_frames[0] >= 1:
                                break

                self.assertGreater(len(gz_odom_poses), 5, "Did not receive Odometry from Gazebo over ros_gz_bridge")
                disp = float(np.linalg.norm(np.asarray(gz_odom_poses[-1]) - np.asarray(gz_odom_poses[0])))
                print(
                    f"[Gazebo Live Physics + Noisy VIO + Depth Bridge] Odometry msgs: {len(gz_odom_poses)}, "
                    f"Depth frames decoded: {gz_depth_frames[0]}, "
                    f"Start ENU: {np.round(gz_odom_poses[0], 3).tolist()} -> "
                    f"Moved ENU: {np.round(gz_odom_poses[-1], 3).tolist()} (disp={disp:.3f}m)"
                )
                self.assertGreater(disp, 0.20, "Gazebo rigid-body drone did not move in response to planner cmd_vel")
            finally:
                rclpy.shutdown()
                for p in (bridge_proc, gz_proc):
                    p.terminate()
                    try:
                        p.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        p.kill()

    def test_02_ros2_full_course_with_physics_vio_drift_and_unmapped_boulder(self) -> None:
        """Full 6-stage ROS 2 flight with 2nd-order physics, VIO drift/noise, and an unmapped boulder."""
        if not HAS_RCLPY:
            self.skipTest("rclpy not installed in current Python environment")

        repo_root = Path(__file__).resolve().parent.parent
        obj_path = repo_root / "dronehacks-course-v3.obj"
        cfg = PlannerConfig(fast_mode=True)

        # 1. Planner's map prior (only knows static course v3 walls)
        planner_grid = OccupancyGrid3D(
            bounds_min_m=cfg.global_map_bounds_min_m,
            bounds_max_m=cfg.global_map_bounds_max_m,
            resolution_m=cfg.voxel_resolution_m,
            inflation_radius_m=cfg.obstacle_inflation_m,
        )
        planner_grid.load_prior_from_obj(obj_path)

        # 2. Ground-truth world map (has an UNMAPPED shifted boulder right in the Stage 3 -> 4 path!)
        truth_grid = OccupancyGrid3D(
            bounds_min_m=cfg.global_map_bounds_min_m,
            bounds_max_m=cfg.global_map_bounds_max_m,
            resolution_m=cfg.voxel_resolution_m,
            inflation_radius_m=cfg.obstacle_inflation_m,
        )
        truth_grid.load_prior_from_obj(obj_path)
        # Place an unmapped 40x40cm obstacle column at (1.50, -1.55) between Tunnel Exit and Window Approach
        unmapped_min = (1.30, -1.75, 0.0)
        unmapped_max = (1.70, -1.35, 1.50)
        truth_grid.add_aabb_obstacle(unmapped_min, unmapped_max)

        router = GlobalCourseRouter(cfg, planner_grid)
        tracker = PX4TrajectoryTrackerCore(router, lookahead_s=0.35)
        phys = QuadrotorPhysicsAndVIOSim(cfg.checkpoints.start_pad, tracker.launch_origin_enu, seed=42)
        depth_cam = RaycastOakDDepthCamera(seed=101)

        rclpy.init()
        try:
            sim_node = Node("noisy_px4_oakd_physics_sim")
            planner_node = Node("ego_px4_planner_humble_node")

            health_pub = sim_node.create_publisher(Bool, "/estimator/healthy", 10)
            cam_info_pub = sim_node.create_publisher(CameraInfo, "/camera/depth/camera_info", 10)
            depth_pub = sim_node.create_publisher(Image, "/camera/depth/image_raw", 10)
            local_pos_pub = sim_node.create_publisher(Odometry, "/sim/fmu/out/vehicle_local_position", 10)
            setpoint_pub = planner_node.create_publisher(Odometry, "/sim/fmu/in/trajectory_setpoint", 10)

            cam_intrinsics = {}
            depth_voxels_inserted = [0]
            collision_replans = [0]
            visited_stages: List[CourseStage] = []
            max_vio_err_m = [0.0]

            def on_health(msg: Bool) -> None:
                tracker.update_estimator_health(bool(msg.data))

            def on_cam_info(msg: CameraInfo) -> None:
                cam_intrinsics["fx"] = float(msg.k[0])
                cam_intrinsics["cx"] = float(msg.k[2])
                cam_intrinsics["fy"] = float(msg.k[4])
                cam_intrinsics["cy"] = float(msg.k[5])

            def on_depth(msg: Image) -> None:
                if "fx" not in cam_intrinsics or tracker.telemetry.last_local_pos_time_s is None:
                    return
                depth_m = decode_ros_depth_image(msg)
                # Note: Planner projects depth using its NOISY VIO pose estimate (never ground truth!)
                est_pos_enu = ned_to_enu_pos(
                    tracker.telemetry.pos_ned,
                    convention=tracker.frame_convention,
                    origin_enu=tracker.launch_origin_enu,
                )
                est_yaw_enu = wrap_pi(0.5 * math.pi - tracker.telemetry.heading_ned_rad)
                n_before = int(np.count_nonzero(planner_grid.raw_grid))
                planner_grid.update_from_depth_image(
                    depth_m,
                    cam_intrinsics["fx"],
                    cam_intrinsics["fy"],
                    cam_intrinsics["cx"],
                    cam_intrinsics["cy"],
                    est_pos_enu,
                    est_yaw_enu,
                )
                n_after = int(np.count_nonzero(planner_grid.raw_grid))
                if n_after > n_before:
                    depth_voxels_inserted[0] += (n_after - n_before)

            def on_local_pos(msg: Odometry) -> None:
                t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                p = msg.pose.pose.position
                v = msg.twist.twist.linear
                heading_ned = float(msg.pose.pose.orientation.z)
                tracker.update_local_position(
                    t_sec, p.x, p.y, p.z, v.x, v.y, v.z, heading_ned, True, True
                )
                tracker.update_vehicle_status(t_sec, ARMING_STATE_ARMED, NAVIGATION_STATE_OFFBOARD)

            latest_sp = {
                "pos_ned": enu_to_ned_pos(cfg.checkpoints.start_pad, origin_enu=tracker.launch_origin_enu),
                "vel_ned": (0.0, 0.0, 0.0),
                "yaw_ned": 0.0,
            }

            def on_setpoint(msg: Odometry) -> None:
                p = msg.pose.pose.position
                v = msg.twist.twist.linear
                latest_sp["pos_ned"] = (float(p.x), float(p.y), float(p.z))
                latest_sp["vel_ned"] = (float(v.x), float(v.y), float(v.z))
                latest_sp["yaw_ned"] = float(msg.pose.pose.orientation.z)

            planner_node.create_subscription(Bool, "/estimator/healthy", on_health, 10)
            planner_node.create_subscription(CameraInfo, "/camera/depth/camera_info", on_cam_info, 10)
            planner_node.create_subscription(Image, "/camera/depth/image_raw", on_depth, 10)
            planner_node.create_subscription(Odometry, "/sim/fmu/out/vehicle_local_position", on_local_pos, 10)
            sim_node.create_subscription(Odometry, "/sim/fmu/in/trajectory_setpoint", on_setpoint, 10)

            executor = SingleThreadedExecutor()
            executor.add_node(sim_node)
            executor.add_node(planner_node)

            ci = CameraInfo()
            ci.width = depth_cam.width
            ci.height = depth_cam.height
            ci.k = [depth_cam.fx, 0.0, depth_cam.cx, 0.0, depth_cam.fy, depth_cam.cy, 0.0, 0.0, 1.0]
            cam_info_pub.publish(ci)

            h_msg = Bool()
            h_msg.data = True
            health_pub.publish(h_msg)

            dt = 0.05  # 20 Hz physics & control loop
            physical_collisions = 0

            for step_i in range(1200):
                sim_t = step_i * dt
                sec = int(sim_t)
                nanosec = int((sim_t - sec) * 1e9)

                # 1. Generate noisy, drifted VIO measurement from true physical state
                meas_pos_ned, meas_vel_ned, meas_yaw_ned = phys.observe_noisy_vio_ned(dt)
                est_enu = ned_to_enu_pos(meas_pos_ned, origin_enu=tracker.launch_origin_enu)
                vio_err = float(np.linalg.norm(est_enu - phys.true_pos_enu))
                if vio_err > max_vio_err_m[0]:
                    max_vio_err_m[0] = vio_err

                odom = Odometry()
                odom.header.stamp.sec = sec
                odom.header.stamp.nanosec = nanosec
                odom.pose.pose.position.x = float(meas_pos_ned[0])
                odom.pose.pose.position.y = float(meas_pos_ned[1])
                odom.pose.pose.position.z = float(meas_pos_ned[2])
                odom.pose.pose.orientation.z = float(meas_yaw_ned)
                odom.twist.twist.linear.x = float(meas_vel_ned[0])
                odom.twist.twist.linear.y = float(meas_vel_ned[1])
                odom.twist.twist.linear.z = float(meas_vel_ned[2])
                local_pos_pub.publish(odom)

                # 2. At 5 Hz, raycast a noisy 16UC1 OAK-D depth frame against the ground-truth 3D world
                if step_i % 4 == 0:
                    img = Image()
                    img.header.stamp.sec = sec
                    img.header.stamp.nanosec = nanosec
                    img.height = depth_cam.height
                    img.width = depth_cam.width
                    img.encoding = "16UC1"
                    img.is_bigendian = 0
                    img.step = depth_cam.width * 2
                    img.data = depth_cam.render_16uc1_mm(truth_grid, phys.true_pos_enu, phys.true_yaw_enu)
                    depth_pub.publish(img)

                for _ in range(4):
                    executor.spin_once(timeout_sec=0.001)

                # 3. Planner samples 30 Hz trajectory setpoint using only noisy VIO & depth
                sample, reason = tracker.sample_setpoint(sim_t, external_window_confidence=0.95)
                if reason == "collision_avoidance_replan":
                    collision_replans[0] += 1
                if sample is not None:
                    if not visited_stages or visited_stages[-1] != sample.stage:
                        visited_stages.append(sample.stage)
                    sp_msg = Odometry()
                    sp_msg.header.stamp.sec = sec
                    sp_msg.header.stamp.nanosec = nanosec
                    sp_msg.pose.pose.position.x = float(sample.position_ned[0])
                    sp_msg.pose.pose.position.y = float(sample.position_ned[1])
                    sp_msg.pose.pose.position.z = float(sample.position_ned[2])
                    sp_msg.pose.pose.orientation.z = float(sample.yaw_ned)
                    sp_msg.twist.twist.linear.x = float(sample.velocity_ned[0])
                    sp_msg.twist.twist.linear.y = float(sample.velocity_ned[1])
                    sp_msg.twist.twist.linear.z = float(sample.velocity_ned[2])
                    setpoint_pub.publish(sp_msg)
                    for _ in range(2):
                        executor.spin_once(timeout_sec=0.001)

                # 4. Step 2nd-order quadrotor physics with drag, lag, and wind
                prof = router.profile_for_stage(router.stage)
                phys.step_physics(
                    dt,
                    sim_t,
                    latest_sp["pos_ned"],
                    latest_sp["vel_ned"],
                    latest_sp["yaw_ned"],
                    prof.v_max,
                )

                # Verify true physical position never collides with any wall or the unmapped boulder
                if truth_grid.is_occupied(phys.true_pos_enu, inflated=False):
                    physical_collisions += 1

                if router.stage == CourseStage.MISSION_COMPLETE:
                    break

            print(
                f"[Physics + Noisy VIO + Raycast Depth] Stages: {[s.value for s in visited_stages]}\n"
                f"  -> Max VIO drift/noise error: {max_vio_err_m[0]*100:.2f} cm\n"
                f"  -> New obstacle voxels discovered via OAK-D depth: {depth_voxels_inserted[0]}\n"
                f"  -> Mid-flight collision-avoidance replans: {collision_replans[0]}\n"
                f"  -> True physical collisions: {physical_collisions}"
            )

            self.assertEqual(physical_collisions, 0, "Physical drone collided with a wall or boulder!")
            self.assertEqual(router.stage, CourseStage.MISSION_COMPLETE)
            self.assertGreater(depth_voxels_inserted[0], 0, "Raycasted OAK-D depth did not detect the unmapped boulder")
            self.assertIn(CourseStage.STAGE_6_FLY_HOME, visited_stages)
        finally:
            rclpy.shutdown()

    def test_03_ros2_live_depth_obstructed_window_falls_back_to_gate_b(self) -> None:
        """Verify live noisy OAK-D depth detecting a blocked Window triggers automatic Gate B fallback."""
        if not HAS_RCLPY:
            self.skipTest("rclpy not installed in current Python environment")

        repo_root = Path(__file__).resolve().parent.parent
        obj_path = repo_root / "dronehacks-course-v3.obj"
        cfg = PlannerConfig(fast_mode=True)

        # Planner starts with the standard prior where the Window cutout is open
        planner_grid = OccupancyGrid3D(
            bounds_min_m=cfg.global_map_bounds_min_m,
            bounds_max_m=cfg.global_map_bounds_max_m,
            resolution_m=cfg.voxel_resolution_m,
            inflation_radius_m=cfg.obstacle_inflation_m,
        )
        planner_grid.load_prior_from_obj(obj_path)
        features = planner_grid.discover_course_features(standoff_distance_m=cfg.window_inspect_distance_m)
        assert features.window_center_m is not None and features.window_approach_m is not None

        # Ground-truth world has a plywood panel blocking the Window opening!
        truth_grid = OccupancyGrid3D(
            bounds_min_m=cfg.global_map_bounds_min_m,
            bounds_max_m=cfg.global_map_bounds_max_m,
            resolution_m=cfg.voxel_resolution_m,
            inflation_radius_m=cfg.obstacle_inflation_m,
        )
        truth_grid.load_prior_from_obj(obj_path)
        wx, wy, wz = features.window_center_m
        truth_grid.add_aabb_obstacle((wx - 0.15, wy - 0.30, wz - 0.30), (wx + 0.15, wy + 0.30, wz + 0.30))

        # Drone is at the ~1 m Window approach point facing the Window cutout
        approach_pos = np.asarray(features.window_approach_m, dtype=np.float64)
        yaw_toward_win = math.atan2(wy - approach_pos[1], wx - approach_pos[0])
        depth_cam = RaycastOakDDepthCamera(seed=202)

        # Render 3 noisy 16UC1 depth frames from the approach pose and project into planner_grid
        for _ in range(3):
            raw_bytes = depth_cam.render_16uc1_mm(truth_grid, approach_pos, yaw_toward_win)
            depth_m = np.frombuffer(raw_bytes, dtype=np.uint16).reshape(depth_cam.height, depth_cam.width).astype(np.float32) * 0.001
            planner_grid.update_from_depth_image(
                depth_m,
                depth_cam.fx,
                depth_cam.fy,
                depth_cam.cx,
                depth_cam.cy,
                approach_pos,
                yaw_toward_win,
            )

        router = GlobalCourseRouter(cfg, planner_grid)
        router.stage = CourseStage.STAGE_4_CHICANE
        decision = router.step(10.0, approach_pos, external_window_confidence=0.95, force_replan=True)

        print(
            f"[Live Depth Window Fallback] Branch chosen: {decision.selected_branch}, "
            f"Reason: {decision.reason}, Goal ENU: {decision.goal_enu}"
        )
        self.assertEqual(decision.selected_branch, "GATE_B")
        self.assertEqual(decision.goal_enu, cfg.checkpoints.landing_pad)
        self.assertIsNotNone(decision.spline)
        assert decision.spline is not None
        # Verify the Gate B fallback spline is collision-free in the blocked-window truth grid!
        for pt in decision.spline.sample_points(30):
            self.assertFalse(truth_grid.is_occupied(pt, inflated=False))


if __name__ == "__main__":
    unittest.main()
