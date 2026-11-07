#!/usr/bin/env python3
"""Simulation-only adapter for the existing Navless course controller.

No simulator API/truth input. The only position sources are validated ROS odometry
and PX4 local position. Run in the isolated SITL container/domain only. Arming
requires --approve-flight; the default starts nothing. The upstream controller,
its prearm height gate, route geometry and estimator checks remain unchanged.
"""
import argparse
import hashlib
import re
import json
import math
import os
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
ROOT = Path(__file__).resolve().parents[1]
NAVLESS = Path(__file__).resolve().parent / 'navless'
if not NAVLESS.is_dir():
    NAVLESS = Path('/workspace/Navless')
sys.path.insert(0, str(NAVLESS / 'navless_nav'))


def clock_ready_samples(samples, now_us):
    eligible = [msg for msg in samples if msg.timestamp <= now_us]
    pending = [msg for msg in samples if msg.timestamp > now_us]
    return (max(eligible, key=lambda msg: msg.timestamp) if eligible else None), pending


def fresh(received, now, limit=0.5):
    return received is not None and 0 <= now - received <= limit


from frames import launch_odom_to_ned, ned_to_launch_odom
from startup import RecorderReadiness, prepared_approved


def odom_to_ned(xyz, bearing_deg=0.0):
    return list(launch_odom_to_ned(xyz, bearing_deg))


def ned_to_odom(xyz, bearing_deg=0.0):
    return list(ned_to_launch_odom(xyz, bearing_deg))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    flight_mode = parser.add_mutually_exclusive_group()
    flight_mode.add_argument('--approve-flight', action='store_true')
    flight_mode.add_argument('--prepare-flight', action='store_true', help='observe without authority until matching explicit activation')
    parser.add_argument('--preflight-offboard', action='store_true', help='request Offboard while disarmed, never arm')
    parser.add_argument('--preflight-only', action='store_true', help='disarmed inspect; never publish PX4 commands or setpoints')
    parser.add_argument('--recorder-token', type=Path, help='independent run-scoped recording readiness for split SITL')
    parser.add_argument('--run-id')
    parser.add_argument('--duration', type=float, default=120.0)
    parser.add_argument('--wall-timeout', type=float, default=240.0)
    parser.add_argument('--preflight-observe', type=float, default=30.0)
    parser.add_argument('--route', default=str(NAVLESS / 'gazebo_sim/course/route_centreline_window.json'))
    parser.add_argument('--start-height', type=float, default=0.07)
    parser.add_argument('--start-bearing', type=float, default=None, help='measured launch nose bearing clockwise from course north, degrees; must match EV bridge')
    parser.add_argument('--yaw-policy', choices=('off','first-exterior','map-scheduled'), default='off', help='optional camera look; first-exterior requires calibrated north start, max60deg at10deg/s')
    parser.add_argument('--px4-localization', action=argparse.BooleanOptionalAction, default=True,
                        help='required PX4 fused-position navigation on split SITL')
    parser.add_argument('--qualified-astar', action='store_true', help='checkpoint interior A* settings .40 cruise/.18 lead/.12 lateral acceleration/.015 sagitta')
    parser.add_argument('--height-profile',choices=('range','ev-terrain'),default='range')
    args = parser.parse_args()
    flight_requested = args.approve_flight or args.prepare_flight
    if args.prepare_flight and not re.fullmatch(r'[0-9]{8}T[0-9]{6}Z-split-astar-flight', args.run_id or ''):
        parser.error('prepared mode requires an explicit split run id')
    if not args.px4_localization:
        parser.error('NVIDIA split navigation requires PX4 fused local position; raw VO cannot drive the mission')
    if flight_requested and (not args.recorder_token or not args.run_id or
                                not math.isfinite(args.preflight_observe) or args.preflight_observe < 30):
        parser.error('approved split flight requires recorder token, run id and at least 30 seconds continuous observation')
    if args.prepare_flight:
        manifest_digest = hashlib.sha256(Path(__file__).with_name('sources.sha256').read_bytes()).hexdigest()
        activation_file = args.recorder_token.with_name(args.run_id+'-activation')
        if activation_file.exists():
            parser.error('activation exists before prepared observer startup')
    fixture_path=Path(__file__).resolve().parent / 'fixture.json'
    fixture_bearing=None
    if fixture_path.is_file():
        fixture_bearing=float(json.loads(fixture_path.read_text())['start_bearing_deg'])
    if args.start_bearing is None:
        args.start_bearing=0.0 if fixture_bearing is None else fixture_bearing
    if not math.isfinite(args.start_bearing):
        parser.error('start bearing must be finite')
    if fixture_bearing is not None and abs(math.remainder(args.start_bearing-fixture_bearing,360.))>1e-6:
        parser.error('start bearing disagrees with current launch fixture.json')
    if args.yaw_policy != 'off' and abs(args.start_bearing) > 1e-6:
        parser.error('first-exterior yaw policy requires calibrated north start')
    if os.environ.get('ROS_DOMAIN_ID') != '219':
        parser.error('isolated SITL requires ROS_DOMAIN_ID=219')
    if not flight_requested and not args.preflight_only and not args.preflight_offboard:
        parser.error('explicit team approval required: --approve-flight (SITL only)')
    if not (0 < args.duration <= 180 and 0 < args.wall_timeout <= 600):
        parser.error('duration must be (0,180], wall-timeout (0,600] seconds')
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                             VehicleStatus, VehicleLocalPosition, VehicleLandDetected, VehicleCommandAck, EstimatorStatusFlags)
    from navless_nav.course_mission import CourseMission
    rclpy.init()
    use_sim = Parameter('use_sim_time', value=True)

    class Interface(Node):
        def __init__(self):
            super().__init__('local_course_interface', parameter_overrides=[use_sim])
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.msg, self.rx, self.source, self.wall_rx = {}, {}, {}, {}
            self.pending_px4 = {}
            self.sp, self.sp_at, self.kind = None, None, 'position'
            from split_frame import SplitFrame
            self.split_frame = SplitFrame(args.start_bearing)
            self.control_bearing = None
            self.odom_quaternion = None
            from integrity import IntegrityGate
            from launch_datum import LaunchDatum
            self.launch_datum = LaunchDatum()
            self.integrity = IntegrityGate()
            self.integrity_stamp = None
            self.phase, self.ready_at, self.request_at = 'WAIT', None, None
            self.mode_attempts, self.mode_last_at, self.mode_ack = 0, None, None
            self.start_wall, self.start_sim = time.monotonic(), None
            self.last_report = 0.0
            self.recorder = RecorderReadiness(args.run_id)
            self.last_stream_at = None
            self.observation_reported = False
            self.healthy_wall = None
            self.clock_value, self.clock_advanced_wall = self.now(), time.monotonic()
            self.get_logger().info(f'calibrated launch frame: nose bearing {args.start_bearing:.3f}deg clockwise from north; fixture-calibrated mission transform; existing EV bridge preserves arbitrary heading FRD')
            self.yaw_policy, self.yaw_origin, self.odom_position = None, None, None
            self.map_yaw = None
            self.map_bearing, self.map_last_time = 0., None
            if args.yaw_policy == 'map-scheduled':
                import hashlib
                from map_yaw import MapYaw
                route_file = Path(args.route)
                route = json.loads(route_file.read_text())
                schedule = json.loads((Path(__file__).parent / 'yaw-schedule.json').read_text())
                if schedule['route_sha256'] != hashlib.sha256(route_file.read_bytes()).hexdigest():
                    raise ValueError('camera yaw schedule must match the exact route')
                self.yaw_origin = route['launch_origin_enu']
                self.map_rate = float(schedule['rate_limit_deg_s'])
                self.map_yaw = MapYaw(route['waypoints_enu'], schedule['route_arc_m'], schedule['bearing_unwrapped_deg'], max_rate_deg_s=self.map_rate)
                self.get_logger().info(f'mapped camera bearing enabled: geographic north startup, {self.map_rate:g} deg/s maximum')
            if args.yaw_policy == 'first-exterior':
                from yaw_policy import FirstExteriorLook
                with open(args.route) as route_stream:
                    self.yaw_origin = json.load(route_stream)['launch_origin_enu']
                self.yaw_policy = FirstExteriorLook()
                self.get_logger().info('optional first-exterior camera yaw: max60deg, rate10deg/s, validated odometry only')
            self.nav_guard, self.nav_state, self.brake_position = None, None, None
            from px4_navigation import PoseHistory
            self.pose_history = PoseHistory()
            self.vision_yaw = None
            if args.px4_localization:
                from px4_navigation import FusedNavigationGate
                self.nav_guard = FusedNavigationGate(registered_split_frame=self.split_frame)
                self.nav_pub = self.create_publisher(Odometry,'/split_course/px4_navigation',10)
            self.create_subscription(EstimatorStatusFlags,'/fmu/out/estimator_status_flags',lambda m:self.receive('fusion',m),qos)
            self.exit_code, self.done = 1, False
            input_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                   durability=DurabilityPolicy.VOLATILE)
            self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', input_qos)
            self.sp_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', input_qos)
            self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', input_qos)
            self.state_pub = self.create_publisher(String, '/skybeta/state', 10)
            for key, typ, topic in [('status', VehicleStatus, '/fmu/out/vehicle_status_v1'),
                                    ('pos', VehicleLocalPosition, '/fmu/out/vehicle_local_position'),
                                    ('land', VehicleLandDetected, '/fmu/out/vehicle_land_detected')]:
                self.create_subscription(typ, topic, lambda m, k=key: self.receive(k,m), qos)
            self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.ack, qos)
            self.create_subscription(Odometry, '/mapping/validated_odometry', self.odometry, 10)
            self.create_subscription(PoseStamped, '/skybeta/setpoint/pose', self.pose, 10)
            self.create_subscription(TwistStamped, '/skybeta/setpoint/twist', self.twist, 10)
            self.create_timer(0.05, self.tick)

        def now(self):
            return self.get_clock().now().nanoseconds / 1e9

        def receive(self, key, msg):
            # Repeated source timestamps must not extend telemetry freshness.
            stamp = int(msg.timestamp)
            if stamp <= self.source.get(key, -1):
                return
            now = self.now()
            if stamp > int(now*1e6):
                # DDS and /clock callbacks may arrive in either order. Retain
                # previous independently fresh sample until clock catches up.
                queue = self.pending_px4.setdefault(key, [])
                queue.append(msg)
                del queue[:-16]
                return
            self.msg[key], self.rx[key], self.source[key] = msg, now, stamp
            self.wall_rx[key] = time.monotonic()
            if key == 'pos' and self.pose_history is not None:
                self.pose_history.add(msg)

        def ack(self, msg):
            self.get_logger().info(f'PX4 command ACK command={msg.command} result={msg.result}')
            if msg.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
                self.mode_ack = int(msg.result)

        def odometry(self, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if stamp > self.source.get('odom', -1) and fresh(stamp,self.now()):
                self.source['odom'], self.rx['odom'] = stamp, self.now()
                p = msg.pose.pose.position
                self.odom_position = (float(p.x),float(p.y),float(p.z))
                from frames import launch_quaternion_to_ned
                q = msg.pose.pose.orientation
                self.odom_quaternion = (q.w,q.x,q.y,q.z)
                if self.control_bearing is not None:
                    w,x,y,z=launch_quaternion_to_ned((q.w,q.x,q.y,q.z),self.control_bearing)
                    self.vision_yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
                self.wall_rx['odom'] = time.monotonic()

        def pose(self, msg):
            p = msg.pose.position
            self.setpoint('position', [p.x, p.y, p.z])

        def twist(self, msg):
            v = msg.twist.linear
            self.setpoint('velocity', [v.x, v.y, v.z])

        def setpoint(self, kind, values):
            if self.control_bearing is not None and all(math.isfinite(v) for v in values):
                if kind == 'velocity' and math.sqrt(sum(v*v for v in values)) > 1.0:
                    return
                if kind == 'position' and not -0.15 <= values[2] <= 2.5:
                    return
                transformed = odom_to_ned(values, self.control_bearing)
                if kind == 'position':
                    transformed = self.launch_datum.to_px4(transformed)
                self.kind, self.sp, self.sp_at = kind, transformed, self.now()

        def request_mode(self, now):
            self.mode_attempts += 1
            self.mode_last_at = now
            self.get_logger().info(f'Offboard mode attempt {self.mode_attempts}/3; matched PX4 subscriptions={self.cmd_pub.get_subscription_count()}')
            self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,1,6)

        def command(self, command, p1=0.0, p2=0.0):
            msg = VehicleCommand()
            msg.timestamp = int(self.now()*1e6)
            msg.command, msg.param1, msg.param2 = command, float(p1), float(p2)
            msg.target_system = msg.target_component = msg.source_system = msg.source_component = 1
            msg.from_external = True
            self.cmd_pub.publish(msg)

        def stop(self, reason, code=1):
            self.get_logger().info(reason)
            self.exit_code, self.done = code, True

        def tick(self):
            now = self.now()
            for key, queue in list(self.pending_px4.items()):
                eligible, self.pending_px4[key] = clock_ready_samples(queue, int(now*1e6))
                if eligible is not None:
                    self.receive(key, eligible)
            # PX4 LandDetector.cpp publishes unchanged land state at 1 Hz,
            # and immediately on change. Commander.cpp publishes status at
            # 2 Hz/on change, so allow 250 ms transport/scheduling margin.
            valid = all(fresh(self.source.get(k) / 1e6 if k in self.source and k != 'odom'
                                   else self.source.get(k), now, 1.5 if k == 'land' else 0.75 if k == 'status' else 0.5)
                        for k in (('status','pos','land') if args.px4_localization else ('status','pos','land','odom')))
            if not valid:
                self.ready_at = None
                self.healthy_wall = None
                if self.phase not in ('WAIT', 'MODE_CHECK'):
                    ages = {k: None if k not in self.source else now-(self.source[k] if k == 'odom' else self.source[k]/1e6)
                            for k in ('status','pos','land','odom')}
                    pending = {k: [int(msg.timestamp) for msg in queue] for k,queue in self.pending_px4.items() if queue}
                    self.stop(f'telemetry stale: stopped all control publication sim_now={now:.9f} source_age_sim={ages} source_stamps={self.source} pending_future_us={pending}')
                return
            p, status, land = self.msg['pos'], self.msg['status'], self.msg['land']
            armed = status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            offboard = status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            aligned=self.pose_history.at(self.source.get('odom')) if self.control_bearing is None else None
            frame_state, frame_reason=self.split_frame.observe(now,self.source.get('odom'),
                self.odom_quaternion,p,self.msg.get('fusion'),armed,land.landed,
                None if aligned is None else aligned[3])
            if frame_state != 'ready':
                self.healthy_wall=self.ready_at=None
                if frame_state == 'reject':
                    self.stop(f'control frame rejected: {frame_reason}')
                return
            if self.control_bearing is None:
                self.control_bearing=self.split_frame.bearing
                self.get_logger().info(f'qualified split control bearing {self.control_bearing:.3f}deg; geographic launch bearing {args.start_bearing:.3f}deg; heading_good_for_control={p.heading_good_for_control}')

            if self.source.get('odom') != self.integrity_stamp:
                stamp = self.source['odom']
                aligned_pose = self.pose_history.at(stamp)
                if aligned_pose is None:
                    self.ready_at = self.healthy_wall = None
                    if armed or self.phase not in ('WAIT', 'MODE_CHECK'):
                        self.stop('simulation companion integrity: no PX4 sample aligned to VO acquisition')
                    return
                self.integrity_stamp = stamp
                flags = self.msg.get('fusion')
                stationary = (not armed and land.landed and flags is not None and flags.cs_rng_hgt
                              and not flags.cs_rng_stuck and not flags.cs_rng_fault
                              and math.hypot(p.vx,p.vy,p.vz) <= .15)
                self.launch_datum.observe(stamp, aligned_pose[:3],
                                          odom_to_ned(self.odom_position, self.control_bearing), stationary)
                if not armed and self.launch_datum.offset is not None and abs(aligned_pose[2]+self.odom_position[2])>.15:
                    self.stop('raw PX4/VO heights exceed existing prearm tolerance')
                    return
                local_down = self.launch_datum.from_px4(aligned_pose[:3])[2]
                ok, reason = self.integrity.observe(stamp, self.odom_position, local_down, armed)
                if not ok:
                    self.stop(f'simulation companion integrity: {reason}; stopped all control publication')
                    return
            if args.height_profile == 'ev-terrain':
                flags = self.msg.get('fusion')
                if (flags is None or not flags.cs_ev_hgt or not flags.cs_baro_hgt
                        or not flags.cs_rng_terrain or flags.cs_fake_hgt or flags.cs_baro_fault or flags.fs_bad_acc_vertical):
                    self.healthy_wall = self.ready_at = None
                    if self.phase != 'WAIT':
                        self.stop('qualified EV height/barometer/range terrain fusion lost; stopped publication')
                    return
            if self.phase in ('WAIT', 'MODE_CHECK', 'REQUEST'):
                flags = self.msg.get('fusion')
                if (flags is None or not flags.cs_rng_hgt or flags.cs_rng_stuck or flags.cs_rng_fault):
                    self.ready_at = self.healthy_wall = None
                    if self.phase != 'WAIT':
                        self.stop('PX4 range-height fusion unhealthy during mode/arm sequence')
                    return
            estimator = p.xy_valid and p.z_valid and all(math.isfinite(v) for v in (p.x,p.y,p.z))
            self.state_pub.publish(String(data=json.dumps({
                'offboard_active': offboard and armed, 'armed': armed, 'landed': bool(land.landed),
                'local_enu': ned_to_odom(self.launch_datum.from_px4((p.x,p.y,p.z)),self.control_bearing) if estimator else None, 'clamped': []})))
            if self.phase == 'ACTIVE' and not armed and land.landed:
                self.stop('landed and disarmed',0)
                return
            if self.phase == 'ACTIVE' and (not offboard or not armed):
                self.stop('authority lost: stopped all control publication')
                return
            if self.nav_guard is not None:
                state,reason=self.nav_guard.evaluate(p,now,self.source.get('odom'),
                    None if self.odom_position is None else self.launch_datum.to_px4(odom_to_ned(self.odom_position,self.control_bearing)),
                    self.vision_yaw,self.msg.get('fusion'),
                    None if 'fusion' not in self.source else self.source['fusion']/1e6,
                    aligned_pose=self.pose_history.at(self.source.get('odom')))
                if state != self.nav_state:
                    self.get_logger().info(f'PX4 navigation {self.nav_state}->{state}: {reason}')
                    self.nav_state=state
                if state != 'ready' and self.phase == 'WAIT':
                    self.healthy_wall = self.ready_at = None
                if state in ('waiting','reject'):
                    if state == 'reject' or self.phase not in ('WAIT','MODE_CHECK'):
                        self.stop(f'PX4 navigation integrity: {reason}')
                    return
                if state == 'coast' and self.phase != 'ACTIVE':
                    self.ready_at=None
                    if self.phase == 'REQUEST':
                        self.stop('vision lost during arm request: stopped stream')
                    return  # never request mode/arm using coast-only readiness
                nav=Odometry()
                nav.header.frame_id='odom';nav.child_frame_id='base_link'
                nav.header.stamp.sec=int(p.timestamp)//1000000
                nav.header.stamp.nanosec=(int(p.timestamp)%1000000)*1000
                xyz=ned_to_odom(self.launch_datum.from_px4((p.x,p.y,p.z)),self.control_bearing)
                nav.pose.pose.position.x,nav.pose.pose.position.y,nav.pose.pose.position.z=xyz
                # Controller consumes position only. Do not claim measured
                # roll/pitch, angular velocity or unavailable covariance.
                nav.pose.pose.orientation.x=nav.pose.pose.orientation.y=nav.pose.pose.orientation.z=nav.pose.pose.orientation.w=math.nan
                nav.pose.covariance=[math.nan]*36;nav.twist.covariance=[math.nan]*36
                v=ned_to_odom((p.vx,p.vy,p.vz),self.control_bearing)
                nav.twist.twist.linear.x,nav.twist.twist.linear.y,nav.twist.twist.linear.z=v
                nav.twist.twist.angular.x=nav.twist.twist.angular.y=nav.twist.twist.angular.z=math.nan
                self.nav_pub.publish(nav)
                if state == 'coast':
                    if self.brake_position is None:
                        self.brake_position=[float(p.x),float(p.y),float(p.z)]
                        for timer in mission.timers:timer.cancel()
                    self.kind,self.sp,self.sp_at='position',self.brake_position,now
                elif self.brake_position is not None:
                    self.brake_position=None
                    mission.last_tick=now
                    for timer in mission.timers:timer.reset()
            if not estimator or not fresh(self.sp_at,now):
                self.healthy_wall = None
                if self.phase not in ('WAIT', 'MODE_CHECK'):
                    self.stop('estimator/setpoint stale: stopped all control publication')
                self.ready_at = None
                return
            recorder_current = not flight_requested
            if flight_requested and self.phase in ('WAIT', 'MODE_CHECK'):
                if armed or not land.landed or status.failsafe:
                    self.stop('startup requires landed/disarmed aircraft and no failsafe')
                    return
                wall = time.monotonic()
                if self.healthy_wall is None:
                    self.healthy_wall = wall
                    self.observation_reported = False
                    self.get_logger().info(f'continuous startup observation begins sim={now:.3f}')
                if wall - self.healthy_wall < args.preflight_observe:
                    if self.phase == 'MODE_CHECK':
                        self.stop('startup observation interrupted during mode check')
                    return
                if not self.observation_reported:
                    self.observation_reported = True
                    self.get_logger().info(f'continuous startup observation complete sim={now:.3f}; awaiting independent recorder')
                if args.prepare_flight and not prepared_approved(activation_file, args.run_id, manifest_digest):
                    self.ready_at = None
                    if self.phase != 'WAIT':
                        self.stop('prepared run activation lost during mode check')
                    return
                try:
                    token = json.loads(args.recorder_token.read_text())
                except (OSError, ValueError):
                    token = None
                can_stream, recorder_current = self.recorder.check(token, now)
                if not can_stream or (self.ready_at is None and not recorder_current):
                    self.ready_at = None
                    if self.phase == 'MODE_CHECK':
                        self.stop('independent recording evidence expired during mode check')
                    return
                if self.ready_at is None and self.phase == 'WAIT':
                    self.get_logger().info(f'independent recorder ready; starting PX4 stream sim={now:.3f}')
            if self.launch_datum.offset is None:
                if not self.launch_datum.freeze(now):
                    self.ready_at = self.healthy_wall = None
                    return
                # The pending setpoint predates registration and has not been streamed.
                if self.kind == 'position':
                    self.sp = self.launch_datum.to_px4(self.sp)
                self.get_logger().info(f'froze measured landed PX4 minus VO translation NED {self.launch_datum.offset}')
            if args.preflight_only or (args.preflight_offboard and self.phase == 'MODE_CHECK' and offboard and status.pre_flight_checks_pass):
                if armed or not land.landed:
                    self.stop('preflight rejected: requires landed/disarmed')
                else:
                    self.healthy_wall = time.monotonic() if self.healthy_wall is None else self.healthy_wall
                    if time.monotonic()-self.healthy_wall >= args.preflight_observe:
                        self.stop('preflight ready: continuous fresh estimator, landed/disarmed, Navless setpoint live',0)
                if args.preflight_only or self.done:
                    return
                self.request_at = now
            if self.phase == 'WAIT':
                if armed or not land.landed:
                    self.stop('start requires fresh landed/disarmed aircraft')
                    return
                if self.last_stream_at is None or now - self.last_stream_at > 0.25:
                    self.ready_at = None
                self.ready_at = now if self.ready_at is None else self.ready_at
                if recorder_current and now-self.ready_at >= 2.0 and self.cmd_pub.get_subscription_count() > 0:
                    self.phase, self.request_at = 'MODE_CHECK', now
                    self.get_logger().info('requesting Offboard while disarmed; waiting for mode and preflight checks')
                    self.request_mode(now)
            elif self.phase == 'MODE_CHECK':
                if armed or not land.landed:
                    self.stop('unexpected arming/takeoff during disarmed mode check')
                    return
                if self.mode_ack is not None and self.mode_ack in (2,3,4,6):
                    self.stop(f'PX4 rejected mode request result={self.mode_ack}; no arm request')
                    return
                if recorder_current and not offboard and self.mode_attempts < 3 and now-self.mode_last_at >= 1.0:
                    # Idempotent MODE only, while all fresh estimator/landed/
                    # disarmed checks above still hold. Never retry ARM.
                    self.request_mode(now)
                if recorder_current and offboard and status.pre_flight_checks_pass and not args.preflight_offboard:
                    self.phase, self.request_at = 'REQUEST', now
                    self.get_logger().info('Offboard preflight checks passed, landed/disarmed: single arm request')
                    self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,1)
                elif now-self.request_at > 10.0:
                    self.stop('Offboard mode/preflight check timed out; no arm request')
                    return
            elif self.phase == 'REQUEST':
                if armed and offboard:
                    self.phase, self.start_sim = 'ACTIVE', now
                elif now-self.request_at > 3.0:
                    self.stop('Offboard/arm request timed out')
                    return
            if self.start_sim is not None and now-self.start_sim > args.duration:
                self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.stop('mission duration exceeded: requested land and stopped stream')
                return
            mode = OffboardControlMode()
            mode.timestamp = int(now*1e6)
            mode.position, mode.velocity = self.kind == 'position', self.kind == 'velocity'
            sp = TrajectorySetpoint()
            sp.timestamp = mode.timestamp
            sp.position = self.sp if mode.position else [math.nan]*3
            sp.velocity = self.sp if mode.velocity else [math.nan]*3
            sp.acceleration = [math.nan]*3
            sp.yaw = sp.yawspeed = math.nan
            # During a bounded fused-state brake, hold the measured heading.
            # Camera-look logic must not advance from stale visual samples.
            braking = self.nav_state == 'coast'
            yaw_pose = self.odom_position
            yaw_stamp = self.source.get('odom')
            if self.nav_guard is not None:
                yaw_pose = ned_to_odom(self.launch_datum.from_px4((p.x,p.y,p.z)),self.control_bearing)
                yaw_stamp = p.timestamp_sample/1e6
            if braking:
                sp.yaw = float(p.heading)
            if self.map_yaw is not None and not braking:
                dt = 0. if self.map_last_time is None else now - self.map_last_time
                self.map_last_time = now
                if not armed:
                    self.map_bearing = 0.
                elif yaw_pose[2] > .15 and dt > 0:
                    if dt > .5:
                        self.stop('stale interval in map camera yaw: stopped stream')
                        return
                    target = self.map_yaw.bearing_rad
                    if mission.state == 'ROUTE':
                        from navless_nav.route import odom_to_course
                        xyz = odom_to_course(yaw_pose,self.yaw_origin,args.start_bearing,args.start_height)
                        try:
                            target = self.map_yaw.update(xyz,dt)
                        except ValueError as exc:
                            self.stop(f'map yaw rejected: {exc}; stopped all control publication')
                            return
                    turn = math.remainder(target-self.map_bearing,2*math.pi)
                    bound = math.radians(self.map_rate)*dt
                    self.map_bearing += min(bound,max(-bound,turn))
                sp.yaw = self.split_frame.course_yaw_to_control(self.map_bearing)
            if self.yaw_policy is not None and not braking:
                local = self.split_frame.course_position(yaw_pose)
                yaw = self.yaw_policy.step(self.yaw_origin[0]+local[1],
                                          self.yaw_origin[1]+local[0],
                                          yaw_pose[2],now,yaw_stamp,armed)
                if yaw is None:
                    self.stop('optional yaw policy invalid/stale input: stopped stream')
                    return
                sp.yaw = self.split_frame.course_yaw_to_control(yaw)
            self.mode_pub.publish(mode)
            self.sp_pub.publish(sp)
            self.last_stream_at = now

    mission = CourseMission(parameter_overrides=[use_sim,
        Parameter('route_file',value=str(Path(args.route).resolve())),
        Parameter('odometry_topic',value='/split_course/px4_navigation'),
        Parameter('start_height_m',value=args.start_height),
        Parameter('takeoff_altitude_m',value=float(json.loads(Path(args.route).read_text())['waypoints_enu'][0][2]) if args.qualified_astar else 0.85),
        Parameter('start_bearing_deg',value=args.start_bearing),
        Parameter('cruise_speed_mps',value=0.40 if args.qualified_astar else 0.5), Parameter('max_lead_m',value=0.18 if args.qualified_astar else 0.6),
        Parameter('max_lat_accel_mps2',value=0.12 if args.qualified_astar else 0.25),
        Parameter('max_turn_sagitta_m',value=0.015 if args.qualified_astar else 0.0),
        Parameter('takeoff_lead_m',value=0.25), Parameter('fly_home',value=False),
        Parameter('blind_carry',value=False)])
    interface = Interface()
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    executor.add_node(interface)
    executor.add_node(mission)
    try:
        while rclpy.ok() and not interface.done:
            executor.spin_once(timeout_sec=0.05)
            if (args.preflight_only or args.preflight_offboard or (flight_requested and interface.phase == 'WAIT')) and time.monotonic()-interface.last_report >= 2.0:
                interface.last_report = time.monotonic()
                ages = {k: round(time.monotonic()-v,3) for k,v in interface.wall_rx.items()}
                source_ages = {k: round(interface.now()-(v if k == 'odom' else v/1e6),3) for k,v in interface.source.items()}
                healthy = None if interface.healthy_wall is None else round(time.monotonic()-interface.healthy_wall,2)
                interface.get_logger().info(f'preflight phase={interface.phase} mission={mission.state} receipt_age_wall={ages} source_age_sim={source_ages} healthy_wall={healthy}')
            clock_now = interface.now()
            if clock_now > interface.clock_value:
                interface.clock_value, interface.clock_advanced_wall = clock_now, time.monotonic()
            elif clock_now < interface.clock_value:
                interface.stop('simulation clock moved backward: stopped stream')
            if interface.clock_value > 0 and time.monotonic()-interface.clock_advanced_wall > 2.0:
                interface.stop('simulation clock stopped for 2 wall seconds: stopped stream')
            if flight_requested and interface.phase == 'WAIT' and time.monotonic()-interface.start_wall > 90:
                interface.stop('startup evidence timeout: no mode or arm requested')
            if time.monotonic()-interface.start_wall > args.wall_timeout:
                # No command using potentially stale telemetry; PX4 owns failsafe.
                interface.stop('wall watchdog expired: stopped stream')
    except KeyboardInterrupt:
        interface.stop('interrupted: stopped stream')
    finally:
        executor.shutdown()
        interface.destroy_node()
        mission.destroy_node()
        rclpy.shutdown()
    return interface.exit_code


if __name__ == '__main__':
    sys.exit(main())
