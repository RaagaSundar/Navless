#!/usr/bin/env python3
"""Actual ROS2/TF/DDS + adaptive mission, driven by a synthetic sensor model.

Runs ONLY in network-none domain227. No PX4, cuVSLAM, real camera, motor
commands or physical devices. Truth stays in this explicit test sensor/model
producer; portable flight code receives normal ROS messages unchanged.
The response model is first-order kinematics, NOT aircraft dynamics.
"""
import argparse
from collections import deque
from collections import Counter
import copy
import json
import math
import os
from pathlib import Path
import platform
import sys
import time
import traceback

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'navless_nav'), str(ROOT/'gazebo_sim')]
from navless_nav.adaptive_route import surveyed_course, source_identity
from navless_nav.route import odom_to_course
from navless_nav.stereo_tube import rotation


def ray_points(obstacles, origin, camera_rotation, directions, rng, noise):
    # Native camera-optical directions, exact ray/box intersection against the
    # synthetic survey geometry. No navigation envelopes/range columns here.
    keep = np.array([name != 'range_keepout' for name in obstacles.names])
    centers, half, rot = (v[keep] for v in (obstacles.centers, obstacles.half_sizes, obstacles.rotations))
    rays = directions @ camera_rotation.T
    local = np.einsum('mji,mj->mi', rot, np.asarray(origin)-centers)
    dl = np.einsum('mji,kj->kmi', rot, rays)
    dl = np.where(np.abs(dl) < 1e-12, 1e-12, dl)
    a, b = (-half-local)[None]/dl, (half-local)[None]/dl
    entry = np.minimum(a, b).max(axis=2)
    exit_ = np.maximum(a, b).min(axis=2)
    hit = (entry > .3) & (exit_ >= entry)
    distances = np.where(hit, entry, np.inf).min(axis=1)
    valid = np.isfinite(distances) & (distances < 2.8)
    distances = distances[valid]+rng.normal(0, noise, valid.sum())
    return (directions[valid]*distances[:, None]).astype(np.float32)


def run(args):
    if os.environ.get('ROS_DOMAIN_ID') != '227' or os.environ.get('ADAPTIVE_BENCH_ONLY') != '1':
        raise SystemExit('requires explicitly isolated test domain227')
    if any(Path('/dev').glob('ttyACM*')) or any(Path('/dev').glob('video*')):
        raise SystemExit('physical device visible; refusing bench')
    args.output.mkdir(parents=True, exist_ok=False)
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header, String
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from navless_nav.course_mission import CourseMission

    rclpy.init()
    mission, model, executor = None, None, SingleThreadedExecutor()
    report = dict(name=args.case, kind='bench', scope=__doc__.strip(),
                  architecture=platform.machine(), python=platform.python_version(), numpy=np.__version__,
                  sensor_oracle=True, controller='actual ROS CourseMission + AdaptiveCourse; source hashes recorded',
                  camera_images_rendered=False, cuvslam=False, px4=False,
                  simulated_rate=args.sim_rate, model_tau_s=args.tau,
                  depth_hz=2 if args.case == 'depth-2hz' else 10,
                  source_sha256={str(p.relative_to(ROOT)): source_identity(p) for p in
                    (Path(__file__).resolve(), ROOT/'navless_nav/navless_nav/adaptive_course.py',
                     ROOT/'navless_nav/navless_nav/adaptive_route.py', ROOT/'navless_nav/navless_nav/course_mission.py')})
    start = time.monotonic()
    try:
        parameters = dict(use_sim_time=True, adaptive_course=True,
            route_file='gazebo_sim/course/route_centreline_window_asbuilt.json',
            adaptive_survey_file='gazebo_sim/course/course_asbuilt.json',
            odometry_topic='/bench/validated_odometry', cruise_speed_mps=.3, max_lead_m=.1,
            max_lat_accel_mps2=.12, net_margin_m=.25, blind_carry=False,
            fly_home=False, stereo_tube_centering=False, tof_tube_centering=False)
        mission = CourseMission(parameter_overrides=[Parameter(k, value=v) for k, v in parameters.items()])
        report['initialization_wall_s'] = time.monotonic()-start

        class Model(Node):
            def __init__(self):
                super().__init__('explicit_oracle_sensor_bench')
                self.clock = self.create_publisher(Clock, '/clock', 10)
                self.odom = self.create_publisher(Odometry, '/bench/validated_odometry', qos_profile_sensor_data)
                self.status = self.create_publisher(VisualSlamStatus, '/visual_slam/status', qos_profile_sensor_data)
                self.state = self.create_publisher(String, '/skybeta/state', 10)
                self.cloud = self.create_publisher(PointCloud2, '/team/stereo/points', qos_profile_sensor_data)
                self.create_subscription(PoseStamped, '/skybeta/setpoint/pose', self.pose, 10)
                self.create_subscription(TwistStamped, '/skybeta/setpoint/twist', self.twist, 10)
                self.sent, self.last_command, self.goal, self.velocity_command = 0, -math.inf, None, None
                self.s = 1.
                self.true_position, self.velocity = np.zeros(3), np.zeros(3)
                self.history = deque([(0., self.true_position.copy()), (1., self.true_position.copy())], maxlen=500)
                self.rng = np.random.default_rng(args.seed)
                self.depth_last = -math.inf
                self.optical_rotation = rotation((-.5, .5, -.5, .5))
                self.mount = np.array([.1053, .0375, .0304])
                self.course_rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
                xs = np.linspace(-math.tan(math.radians(40)), math.tan(math.radians(40)), 48)
                ys = np.linspace(-math.tan(math.radians(25)), math.tan(math.radians(25)), 30)
                self.directions = np.array([[x, y, 1.] for y in ys for x in xs])
                self.directions /= np.linalg.norm(self.directions, axis=1)[:, None]
                survey = ROOT/'gazebo_sim/course/course_asbuilt.json'
                physical = json.loads(survey.read_text())
                if args.case.startswith('M1-'):
                    cm = int(args.case.split('-')[1][:-2])
                    wall = next(w for w in physical['walls'] if w['name'] == 'M1')
                    for key in ('start_in', 'end_in'):
                        wall[key][0] -= cm/100/.0254
                physical_path = args.output/'synthetic-world.json'
                physical_path.write_text(json.dumps(physical, indent=2)+'\n')
                self.obstacles = surveyed_course(physical_path)
                if args.case != 'missing-tf':
                    self.tf = StaticTransformBroadcaster(self)
                    transform = TransformStamped()
                    transform.header.frame_id, transform.child_frame_id = 'base_link', 'bench_left_optical'
                    transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, self.mount)
                    q = transform.transform.rotation
                    q.x, q.y, q.z, q.w = -.5, .5, -.5, .5
                    self.tf.sendTransform(transform)

            def pose(self, msg):
                self.sent += 1
                self.last_command = self.s
                p = msg.pose.position
                self.goal, self.velocity_command = np.array([p.x, p.y, p.z]), None

            def twist(self, msg):
                self.sent += 1
                self.last_command = self.s
                v = msg.twist.linear
                self.velocity_command = np.array([v.x, v.y, v.z])

            def at(self, t):
                for (ta, pa), (tb, pb) in zip(self.history, list(self.history)[1:]):
                    if ta <= t <= tb and tb > ta:
                        return pa+(pb-pa)*(t-ta)/(tb-ta)
                return self.history[0][1].copy()

            def msg_stamp(self, t):
                from builtin_interfaces.msg import Time
                nanosec = max(0, int(t*1e9))
                return Time(sec=nanosec//1000000000, nanosec=nanosec%1000000000)

            def step(self, dt):
                self.s += dt
                armed = self.s >= 4.
                if armed and self.s-self.last_command < .5:
                    target = (self.velocity_command if self.velocity_command is not None else
                              np.zeros(3) if self.goal is None else (self.goal-self.true_position)/args.tau)
                    target *= min(1., .45/max(np.linalg.norm(target), 1e-9))
                    dv = target-self.velocity
                    self.velocity += dv*min(1., .5*dt/max(np.linalg.norm(dv), 1e-9))
                else:
                    self.velocity *= max(0., 1.-dt/.2)
                self.true_position += self.velocity*dt
                self.true_position[2] = max(0., self.true_position[2])
                self.history.append((self.s, self.true_position.copy()))
                clock = Clock(); clock.clock = self.msg_stamp(self.s); self.clock.publish(clock)
                status = VisualSlamStatus()
                status.vo_state = 2 if args.case == 'tracking-loss' and self.s > 7 else 1
                self.status.publish(status)
                authority = not (args.case == 'authority-loss' and self.s > 7)
                state = String()
                state.data = json.dumps(dict(armed=armed, offboard_active=armed and authority,
                                            landed=not armed, local_enu=self.true_position.tolist()))
                self.state.publish(state)
                # Explicit modeled odometry: 40ms source latency, ideal orientation
                # except the deliberate reset/heading cases. No actual VIO claimed.
                odom_stamp = self.s-.04
                position = self.at(odom_stamp)
                if args.case == 'odom-reset' and self.s > 7:
                    position = position+np.array([1., 0, 0])
                yaw = math.radians(6) if args.case == 'heading-6deg' and self.s > 7 else 0.
                odom = Odometry(); odom.header.stamp = self.msg_stamp(odom_stamp)
                odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
                p = odom.pose.pose.position
                p.x, p.y, p.z = map(float, position)
                odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                self.odom.publish(odom)
                hz = 2 if args.case == 'depth-2hz' else 10
                if self.s-self.depth_last >= 1/hz and not (args.case == 'depth-loss' and self.s > 7):
                    self.depth_last = self.s
                    t = self.s-(.35 if args.case == 'stale-depth' else .10)
                    raw_pos = self.at(t)
                    body_rotation = rotation((0, 0, math.sin(yaw/2), math.cos(yaw/2)))
                    body_course = np.asarray(odom_to_course(raw_pos, mission.frame[0], 0, .07))
                    rot = self.course_rotation @ body_rotation
                    origin = body_course+rot @ self.mount
                    points = ray_points(self.obstacles, origin, rot @ self.optical_rotation,
                                        self.directions, self.rng, args.depth_noise)
                    header = Header(stamp=self.msg_stamp(t), frame_id='bench_left_optical')
                    self.cloud.publish(point_cloud2.create_cloud_xyz32(header, points))

        model = Model()
        executor.add_node(mission); executor.add_node(model)
        start = time.monotonic(); previous = start
        samples, last_sample = [], -math.inf
        while model.s < 1+args.duration and time.monotonic()-start < args.duration/args.sim_rate+10:
            now = time.monotonic(); dt = min(.10, now-previous)*args.sim_rate
            if dt >= .005:
                model.step(dt); previous = now
            executor.spin_once(timeout_sec=.001)
            if model.s-last_sample >= .1:
                last_sample = model.s
                a = mission.adaptive
                samples.append(dict(t=model.s, true_odom=model.true_position.tolist(),
                    state=mission.state, commands=model.sent, fault=a.fault,
                    hold_reason=a.hold_reason, failed_revision=a.failed_revision,
                    confirmed=len(a.depth.confirmed), revision=a.depth.revision,
                    valid_cloud_age_s=None if a.valid_cloud_at is None else mission.now_s()-a.valid_cloud_at,
                    route_index=None if mission.carrot is None else mission.carrot.index))
            if mission.adaptive.fault and model.s > 8:
                break
        (args.output/'samples.json').write_text(json.dumps(samples, indent=2, allow_nan=False)+'\n')
        report.update(outcome='fault_stop' if mission.adaptive.fault else 'bounded_observation_complete',
                      fault=mission.adaptive.fault, elapsed_wall_s=time.monotonic()-start,
                      sim_elapsed_s=model.s-1, commands=model.sent,
                      final_state=mission.state, final_true_odom=model.true_position.tolist(),
                      confirmed_obstacles=len(mission.adaptive.depth.confirmed),
                      route_points=len(mission.route), full_course_completed=mission.state == 'DONE',
                      maximum_observed_cloud_age_s=max([s['valid_cloud_age_s'] for s in samples
                          if s['valid_cloud_age_s'] is not None], default=None),
                      hold_reasons=dict(Counter(s['hold_reason'] for s in samples if s['hold_reason'])),
                      last_command_sim_s=model.last_command,
                      final_command_age_sim_s=model.s-model.last_command)
    except Exception as exc:
        report.update(outcome='initialization_or_runtime_error', error=repr(exc),
                      traceback=traceback.format_exc(), elapsed_wall_s=time.monotonic()-start)
    finally:
        if mission is not None:
            if mission.adaptive is not None: mission.adaptive.close()
            executor.remove_node(mission); mission.destroy_node()
        if model is not None:
            executor.remove_node(model); model.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        (args.output/'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print(json.dumps(report, indent=2, allow_nan=False), flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--case', choices=('nominal', 'M1-3cm', 'M1-5cm', 'M1-10cm', 'depth-2hz',
        'depth-loss', 'tracking-loss', 'authority-loss', 'missing-tf', 'stale-depth', 'odom-reset', 'heading-6deg'), required=True)
    p.add_argument('--seed', type=int, default=1)
    p.add_argument('--duration', type=float, default=10.)
    p.add_argument('--sim-rate', type=float, default=.5)
    p.add_argument('--tau', type=float, default=.5)
    p.add_argument('--depth-noise', type=float, default=.005)
    a = p.parse_args()
    if not (0 < a.duration <= 60 and .1 <= a.sim_rate <= 1 and .1 <= a.tau <= 2):
        p.error('bounded duration/rate/response required')
    run(a)
