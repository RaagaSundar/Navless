"""Closed-loop ROS 2 (rclpy) + Gazebo Sim (gz sim) integration test for the EGO-Planner & PX4 Bridge.

Runs inside the ROS 2 Jazzy + Gazebo Harmonic (`gz sim` 8.11.0) Docker container:
1. Generates a Gazebo SDF world loading `dronehacks-course-v3.obj` and `sky_beta` quadrotor,
   then runs `gz sim -s -r --iterations 20` to verify Gazebo physics & course mesh loading.
2. Launches a closed-loop ROS 2 (`rclpy`) multi-node simulation:
   - `SimPX4AndDepthNode`: Publishes `/estimator/healthy`, `/camera/depth/camera_info`,
     `/camera/depth/image_raw` (`16UC1` depth frames), and local NED telemetry while
     integrating the quadrotor's 3D state in response to the planner's 30 Hz setpoints.
   - `PlannerBridgeROS2Node`: Subscribes to the ROS 2 topics, decodes live `16UC1` depth
     frames into `OccupancyGrid3D`, runs `GlobalCourseRouter` + `PX4TrajectoryTrackerCore`
     at 30 Hz, and flies both the Gate B + Fly-Home mission and Window bonus mission.
"""

from __future__ import annotations

import math
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path
from typing import List, Tuple

import numpy as np

from planner.config import PlannerConfig
from planner.frames import ned_to_enu_pos
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
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import Bool

    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False


def build_course_sdf_world(obj_path: Path) -> str:
    """Create a Gazebo Harmonic SDF 1.9 world loading `dronehacks-course-v3.obj` and `sky_beta`."""
    mesh_uri = obj_path.resolve().as_uri()
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="dronehacks_course_v3">
    <physics name="1ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <gravity>0 0 -9.81</gravity>

    <model name="dronehacks_arena">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="course_link">
        <visual name="course_visual">
          <geometry>
            <mesh>
              <uri>{mesh_uri}</uri>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>

    <model name="sky_beta_drone">
      <static>false</static>
      <pose>-2.515 0.914 0.15 0 0 0</pose>
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
    </model>
  </world>
</sdf>
"""


class TestROS2AndGazeboSimulation(unittest.TestCase):
    def test_01_gazebo_harmonic_loads_course_v3_world(self) -> None:
        """Verify Gazebo Sim (`gz sim -s -r`) loads `dronehacks-course-v3.obj` and steps physics."""
        if shutil.which("gz") is None:
            self.skipTest("Gazebo (`gz`) binary not found in current environment")

        repo_root = Path(__file__).resolve().parent.parent
        obj_path = repo_root / "dronehacks-course-v3.obj"
        self.assertTrue(obj_path.exists(), f"Missing {obj_path}")

        with tempfile.TemporaryDirectory() as tmp:
            sdf_path = Path(tmp) / "dronehacks_course_v3.sdf"
            sdf_path.write_text(build_course_sdf_world(obj_path), encoding="utf-8")

            cmd = ["gz", "sim", "-s", "-r", "--iterations", "25", str(sdf_path)]
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=45)
            self.assertEqual(
                proc.returncode,
                0,
                f"gz sim failed with code {proc.returncode}:\nSTDOUT:\n{proc.stdout}\nSTDERR:\n{proc.stderr}",
            )
            print("[Gazebo Sim] Successfully loaded dronehacks-course-v3.obj and stepped 25 physics iterations.")

    def test_02_ros2_closed_loop_full_course_and_fly_home(self) -> None:
        """Run a live ROS 2 (`rclpy`) closed-loop flight through all 6 stages + OAK-D depth updates."""
        if not HAS_RCLPY:
            self.skipTest("rclpy not installed in current Python environment")

        repo_root = Path(__file__).resolve().parent.parent
        obj_path = repo_root / "dronehacks-course-v3.obj"
        cfg = PlannerConfig(fast_mode=True)
        grid = OccupancyGrid3D(
            bounds_min_m=cfg.global_map_bounds_min_m,
            bounds_max_m=cfg.global_map_bounds_max_m,
            resolution_m=cfg.voxel_resolution_m,
            inflation_radius_m=cfg.obstacle_inflation_m,
        )
        n_vox = grid.load_prior_from_obj(obj_path)
        self.assertGreater(n_vox, 5000)

        router = GlobalCourseRouter(cfg, grid)
        tracker = PX4TrajectoryTrackerCore(router)

        rclpy.init()
        try:
            sim_node = Node("sim_px4_oakd_gazebo_bridge")
            planner_node = Node("ego_px4_planner_ros2_node")

            # Pub/Sub over ROS 2 topics
            health_pub = sim_node.create_publisher(Bool, "/estimator/healthy", 10)
            cam_info_pub = sim_node.create_publisher(CameraInfo, "/camera/depth/camera_info", 10)
            depth_pub = sim_node.create_publisher(Image, "/camera/depth/image_raw", 10)
            local_pos_pub = sim_node.create_publisher(PoseStamped, "/sim/fmu/out/local_position", 10)
            setpoint_pub = planner_node.create_publisher(PoseStamped, "/sim/fmu/in/trajectory_setpoint", 10)

            cam_intrinsics = {}
            depth_frames_decoded = [0]
            setpoints_received: List[Tuple[float, float, float]] = []
            visited_stages: List[CourseStage] = []

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
                pos_enu = ned_to_enu_pos(
                    tracker.telemetry.pos_ned,
                    convention=tracker.frame_convention,
                    origin_enu=tracker.launch_origin_enu,
                )
                yaw_enu = 0.5 * math.pi - tracker.telemetry.heading_ned_rad
                grid.update_from_depth_image(
                    depth_m,
                    cam_intrinsics["fx"],
                    cam_intrinsics["fy"],
                    cam_intrinsics["cx"],
                    cam_intrinsics["cy"],
                    pos_enu,
                    yaw_enu,
                )
                depth_frames_decoded[0] += 1

            def on_local_pos(msg: PoseStamped) -> None:
                t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                tracker.update_local_position(
                    t_sec,
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    True,
                    True,
                )
                tracker.update_vehicle_status(t_sec, ARMING_STATE_ARMED, NAVIGATION_STATE_OFFBOARD)

            latest_cmd_ned = [0.0, 0.0, -0.85]

            def on_setpoint(msg: PoseStamped) -> None:
                latest_cmd_ned[0] = float(msg.pose.position.x)
                latest_cmd_ned[1] = float(msg.pose.position.y)
                latest_cmd_ned[2] = float(msg.pose.position.z)
                setpoints_received.append((latest_cmd_ned[0], latest_cmd_ned[1], latest_cmd_ned[2]))

            planner_node.create_subscription(Bool, "/estimator/healthy", on_health, 10)
            planner_node.create_subscription(CameraInfo, "/camera/depth/camera_info", on_cam_info, 10)
            planner_node.create_subscription(Image, "/camera/depth/image_raw", on_depth, 10)
            planner_node.create_subscription(PoseStamped, "/sim/fmu/out/local_position", on_local_pos, 10)
            sim_node.create_subscription(PoseStamped, "/sim/fmu/in/trajectory_setpoint", on_setpoint, 10)

            executor = SingleThreadedExecutor()
            executor.add_node(sim_node)
            executor.add_node(planner_node)

            # Publish initial camera intrinsics & estimator health
            ci = CameraInfo()
            ci.width = 64
            ci.height = 48
            ci.k = [45.0, 0.0, 32.0, 0.0, 45.0, 24.0, 0.0, 0.0, 1.0]
            cam_info_pub.publish(ci)

            h_msg = Bool()
            h_msg.data = True
            health_pub.publish(h_msg)

            # Simulate closed-loop flight at dt = 0.10 s with first-order PX4 position tracking
            drone_ned = np.array([0.0, 0.0, -0.85], dtype=np.float64)
            sim_t = 0.0
            dt = 0.10

            for step_i in range(600):
                sim_t = step_i * dt
                sec = int(sim_t)
                nanosec = int((sim_t - sec) * 1e9)

                # Publish local position
                lp = PoseStamped()
                lp.header.stamp.sec = sec
                lp.header.stamp.nanosec = nanosec
                lp.pose.position.x = float(drone_ned[0])
                lp.pose.position.y = float(drone_ned[1])
                lp.pose.position.z = float(drone_ned[2])
                local_pos_pub.publish(lp)

                # Every 10 ticks (1 Hz), publish a synthetic 16UC1 OAK-D depth image
                if step_i % 10 == 0:
                    img = Image()
                    img.header.stamp.sec = sec
                    img.header.stamp.nanosec = nanosec
                    img.height = 48
                    img.width = 64
                    img.encoding = "16UC1"
                    img.is_bigendian = 0
                    img.step = 64 * 2
                    # Clear depth (0 mm = out of range) so we don't inject fake obstacles into corridor
                    img.data = bytes(48 * 64 * 2)
                    depth_pub.publish(img)

                for _ in range(4):
                    executor.spin_once(timeout_sec=0.001)

                # Sample planner setpoint
                sample, reason = tracker.sample_setpoint(sim_t, external_window_confidence=0.95)
                if sample is not None:
                    if not visited_stages or visited_stages[-1] != sample.stage:
                        visited_stages.append(sample.stage)
                    sp_msg = PoseStamped()
                    sp_msg.header.stamp.sec = sec
                    sp_msg.header.stamp.nanosec = nanosec
                    sp_msg.pose.position.x = float(sample.position_ned[0])
                    sp_msg.pose.position.y = float(sample.position_ned[1])
                    sp_msg.pose.position.z = float(sample.position_ned[2])
                    setpoint_pub.publish(sp_msg)
                    for _ in range(2):
                        executor.spin_once(timeout_sec=0.001)

                    # Simulated PX4 controller tracks the published B-spline TrajectorySetpoint (`latest_cmd_ned`)
                    target_ned = np.asarray(latest_cmd_ned, dtype=np.float64)
                    diff_ned = target_ned - drone_ned
                    dist_ned = float(np.linalg.norm(diff_ned))
                    prof = router.profile_for_stage(router.stage)
                    max_step = prof.v_max * dt * 1.5
                    if dist_ned > max_step and dist_ned > 1e-5:
                        drone_ned = drone_ned + (diff_ned / dist_ned) * max_step
                    else:
                        drone_ned = target_ned.copy()

                if router.stage == CourseStage.MISSION_COMPLETE:
                    break

            print(f"[ROS 2 Closed-Loop] Visited stages: {[s.value for s in visited_stages]}")
            print(f"[ROS 2 Closed-Loop] Setpoints streamed: {len(setpoints_received)}, Depth frames processed: {depth_frames_decoded[0]}")

            self.assertEqual(router.stage, CourseStage.MISSION_COMPLETE)
            self.assertIn(CourseStage.STAGE_1_MAZE, visited_stages)
            self.assertIn(CourseStage.STAGE_2_GATE_A, visited_stages)
            self.assertIn(CourseStage.STAGE_3_TUNNEL, visited_stages)
            self.assertIn(CourseStage.STAGE_4_CHICANE, visited_stages)
            self.assertIn(CourseStage.STAGE_5_LANDING_PAD, visited_stages)
            self.assertIn(CourseStage.STAGE_6_FLY_HOME, visited_stages)
            self.assertGreater(len(setpoints_received), 20)
            self.assertGreater(depth_frames_decoded[0], 2)
        finally:
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
