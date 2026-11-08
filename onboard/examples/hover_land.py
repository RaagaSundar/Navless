#!/usr/bin/env python3

"""Take off 1 m, hold position and heading for 10 s, then land.

Requires team flight approval, fresh SLAM/bridge data and PX4 EV position/yaw fusion.
Forward follows the takeoff heading. Times use the configured ROS clock.
"""

import json
import math
import os
from pathlib import Path
import sys

# This is ordinary ROS/PX4 code: no runtime, cloud API or mission wrapper.
# Use the topic versions exported by the installed px4_msgs generation.
def px4_topic(name, message_type):
    version = int(getattr(message_type, "MESSAGE_VERSION", 0))
    return "/fmu/out/" + name + (f"_v{version}" if version else "")

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy,
    DurabilityPolicy, HistoryPolicy
)

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleLandDetected,
    EstimatorStatusFlags,
    VehicleOdometry
)

# Review the plan with the team before running. These scripts request arm.
# After keyboard RC: land, release RC, and Reset world before automation.
# Uses estimated PX4 local NED only; no obstacle avoidance or simulator truth.
MISSION = 'hover'
MISSION_NAME = 'hover_land'

HEIGHT = 1.0       # Takeoff height in meters
HOVER_TIME = 10.0  # Seconds continuously settled at takeoff height before landing
DISTANCE = 2.0     # Meters along the initial heading; return keeps that heading
FORWARD_HOLD = 10.0  # Seconds continuously within tolerance after arrival
MOVE_SPEED = 0.4   # Maximum horizontal setpoint slew, meters/second
YAW_RATE = math.radians(10.0)  # Gradual positive NED yaw (clockwise from above)
POSITION_TOLERANCE = 0.20
HEIGHT_TOLERANCE = 0.15
YAW_TOLERANCE = math.radians(6.0)
SETTLE_TIME = 1.0
MANEUVER_TIMEOUT = 30.0
VISION_MAX_AGE = 0.5  # ROS seconds: capture, receipt and bridge health
# Match the established bridge CaptureGate's bounded 5 ms clock skew budget.
SOURCE_FUTURE_TOLERANCE = 0.005
# PX4 publishes EstimatorStatusFlags at ~1 Hz, plus changes. Two periods
# and 0.5 s transport/scheduling margin avoid treating ordinary cadence as loss.
FUSION_STATUS_MAX_AGE = 2.5
LAND_STATUS_MAX_AGE = 2.5  # Also published at 1 Hz and on state changes.



class TestFlight(Node):
    def __init__(self):
        super().__init__(MISSION_NAME)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos
        )

        self.create_subscription(
            VehicleLocalPosition, px4_topic("vehicle_local_position", VehicleLocalPosition),
            self.on_position, qos
        )
        self.create_subscription(
            VehicleStatus, px4_topic("vehicle_status", VehicleStatus),
            self.on_status, qos
        )
        self.create_subscription(
            VehicleLandDetected, px4_topic("vehicle_land_detected", VehicleLandDetected),
            self.on_landed, qos
        )
        self.create_subscription(
            EstimatorStatusFlags, px4_topic("estimator_status_flags", EstimatorStatusFlags),
            self.on_estimator_flags, qos
        )

        # VOLATILE avoids accepting latched SLAM/bridge data at startup and
        # matches both reliable and best-effort sensor publishers.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10
        )
        self.slam_topic = str(self.declare_parameter(
            "slam_odom_topic", "/visual_slam/tracking/odometry").value)
        self.vision_topic = str(self.declare_parameter(
            "vision_input_topic", "/fmu/in/vehicle_visual_odometry").value)
        self.health_topic = str(self.declare_parameter(
            "vision_health_topic", "/estimator/healthy").value)
        self.create_subscription(Odometry, self.slam_topic, self.on_slam_odometry, sensor_qos)
        self.create_subscription(VehicleOdometry, self.vision_topic, self.on_vision_odometry, sensor_qos)
        self.create_subscription(Bool, self.health_topic, self.on_vision_health, sensor_qos)
        self.slam_valid = self.vision_valid = self.vision_healthy = False
        self.slam_stamp = self.slam_time = self.slam_frames = None
        self.last_slam_observation = None
        self.vision_stamp = self.vision_time = self.vision_reset = None
        self.vision_health_time = self.estimator_flags = None
        self.get_logger().info(
            "Requires SLAM -> vision bridge -> PX4 EV position/yaw fusion; "
            + self.slam_topic + " -> " + self.vision_topic + "; health=" + self.health_topic)

        self.pos = None
        self.status = None
        self.pos_time = 0.0
        self.status_time = 0.0
        self.landed = None
        self.landed_time = 0.0
        self.yaw_aligned = False
        self.yaw_alignment_time = None
        self.yaw_alignment_timestamp = 0
        self.last_wait_log = 0.0
        self.ground_confirmation_started = None
        self.auto_land_observed = False

        self.phase = "WAIT"
        self.phase_start = self.clock_seconds()
        self.last_tick_time = None
        self.wait_started = None
        self.preflight_timeout_s = float(self.declare_parameter("preflight_timeout_s", 45.0).value)
        if not math.isfinite(self.preflight_timeout_s) or self.preflight_timeout_s <= 0:
            raise ValueError("preflight_timeout_s must be finite and positive")
        self.last_command = 0.0

        self.x = self.y = self.z = self.yaw = 0.0
        self.target_z = 0.0
        self.home_x = self.home_y = self.initial_yaw = 0.0
        self.arrival_since = None
        self.yaw_progress = 0.0
        self.position_resets = None
        self.last_motion_time = None

        # Control loop at 10 Hz
        self.create_timer(0.1, self.tick)

    def on_position(self, msg):
        resets = (msg.xy_reset_counter, msg.z_reset_counter, msg.heading_reset_counter)
        if self.position_resets is not None and resets != self.position_resets:
            if self.phase == "PRIME":
                self.phase = "WAIT"  # Recapture home and earn a fresh prime after a reset.
            elif self.phase not in ("WAIT", "DONE", "ABORT"):
                self.get_logger().error("Estimator frame reset; yielding control")
                self.phase = "ABORT"
        self.position_resets = resets
        self.pos = msg
        self.pos_time = self.clock_seconds()

    def on_status(self, msg):
        self.status = msg
        self.status_time = self.clock_seconds()

    def on_landed(self, msg):
        self.landed = bool(msg.landed)
        self.landed_time = self.clock_seconds()

    def on_estimator_flags(self, msg):
        previous_stamp = self.yaw_alignment_timestamp
        self.estimator_flags = msg
        self.yaw_aligned = bool(msg.cs_yaw_align)
        # Replayed/duplicate packets never renew the freshness interval.
        if int(msg.timestamp) > previous_stamp:
            self.yaw_alignment_time = self.clock_seconds()
        self.yaw_alignment_timestamp = int(msg.timestamp)

    def on_slam_odometry(self, msg):
        now = self.clock_seconds()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        frames = (msg.header.frame_id, msg.child_frame_id)
        valid = (all(math.isfinite(v) for v in values)
                 and 0.9 <= sum(v * v for v in values[3:]) <= 1.1
                 and all(frames) and stamp > 0
                 and -SOURCE_FUTURE_TOLERANCE <= now - stamp <= VISION_MAX_AGE)
        self.last_slam_observation = dict(
            stamp=stamp, received=now, frame_id=frames[0], child_frame_id=frames[1],
            finite_pose=all(math.isfinite(v) for v in values),
            quaternion_norm_sq=sum(v*v for v in values[3:]))
        self.slam_valid = valid
        if not valid or (self.slam_stamp is not None and stamp <= self.slam_stamp):
            return  # Repeated/reordered samples never refresh their age.
        if self.slam_frames is not None and frames != self.slam_frames:
            if self.phase == "PRIME":
                self.phase = "WAIT"  # Recapture home and earn a fresh prime after a reset.
            elif self.phase not in ("WAIT", "DONE", "ABORT"):
                self.get_logger().error("SLAM odometry frame changed; yielding control")
                self.phase = "ABORT"
        self.slam_frames = frames
        self.slam_stamp, self.slam_time = stamp, now

    def on_vision_odometry(self, msg):
        now = self.clock_seconds()
        stamp = int(msg.timestamp_sample) * 1e-6
        published = int(msg.timestamp) * 1e-6
        values = (*msg.position, *msg.q)
        valid = (msg.pose_frame in (VehicleOdometry.POSE_FRAME_NED, VehicleOdometry.POSE_FRAME_FRD)
                 and all(math.isfinite(v) for v in values)
                 and 0.9 <= sum(v * v for v in msg.q) <= 1.1
                 and stamp > 0 and stamp-published <= SOURCE_FUTURE_TOLERANCE
                 and -SOURCE_FUTURE_TOLERANCE <= now - stamp <= VISION_MAX_AGE
                 and -SOURCE_FUTURE_TOLERANCE <= now - published <= VISION_MAX_AGE)
        self.vision_valid = valid
        if not valid or (self.vision_stamp is not None and stamp <= self.vision_stamp):
            return
        if self.vision_reset is not None and msg.reset_counter != self.vision_reset:
            if self.phase == "PRIME":
                self.phase = "WAIT"  # Recapture home and earn a fresh prime after a reset.
            elif self.phase not in ("WAIT", "DONE", "ABORT"):
                self.get_logger().error("External-vision reset; yielding control")
                self.phase = "ABORT"
        self.vision_reset = msg.reset_counter
        self.vision_stamp, self.vision_time = stamp, now

    def on_vision_health(self, msg):
        self.vision_healthy = bool(msg.data)
        self.vision_health_time = self.clock_seconds()

    def vision_problem(self, now):
        # Monitor the established bridge. Never publish a second odometry stream
        # or feed raw SLAM coordinates into PX4 NED position commands.
        if (not self.slam_valid or self.slam_stamp is None or self.slam_time is None
                or not -SOURCE_FUTURE_TOLERANCE <= now - self.slam_stamp <= VISION_MAX_AGE
                or not 0 <= now - self.slam_time <= VISION_MAX_AGE):
            return "SLAM odometry missing, invalid or stale"
        if (not self.vision_healthy or self.vision_health_time is None
                or not 0 <= now - self.vision_health_time <= VISION_MAX_AGE):
            return "vision bridge health missing, false or stale"
        if (not self.vision_valid or self.vision_stamp is None or self.vision_time is None
                or not -SOURCE_FUTURE_TOLERANCE <= now - self.vision_stamp <= VISION_MAX_AGE
                or not 0 <= now - self.vision_time <= VISION_MAX_AGE):
            return "PX4 external-vision input missing, invalid or stale"
        flags = self.estimator_flags
        if (flags is None or self.yaw_alignment_time is None or self.pos is None
                or not 0 <= now - self.yaw_alignment_time <= FUSION_STATUS_MAX_AGE
                or int(flags.timestamp) <= 0
                or not -1_000_000 <= int(self.pos.timestamp)-int(flags.timestamp) <= FUSION_STATUS_MAX_AGE*1e6):
            return "PX4 fusion status missing, stale or incompatible with position time"
        if not (flags.cs_ev_pos and flags.cs_ev_yaw):
            return "PX4 external-vision position/yaw fusion is not active"
        if (flags.cs_ev_yaw_fault or flags.cs_inertial_dead_reckoning
                or flags.reject_hor_pos or flags.reject_yaw):
            return "PX4 reports rejected/faulted vision or inertial dead reckoning"
        return None

    def vision_diagnostics(self, now):
        def age(stamp):
            return round(now-stamp, 6) if stamp is not None else None
        latest = self.last_slam_observation
        detail = dict(slam_valid=self.slam_valid,
                      accepted_slam_capture_age_s=age(self.slam_stamp),
                      accepted_slam_receipt_age_s=age(self.slam_time),
                      vision_valid=self.vision_valid,
                      vision_capture_age_s=age(self.vision_stamp),
                      vision_receipt_age_s=age(self.vision_time),
                      bridge_healthy=self.vision_healthy,
                      bridge_health_age_s=age(self.vision_health_time),
                      fusion_status_age_s=age(self.yaw_alignment_time),
                      fusion_status_source_age_s=(round((int(self.pos.timestamp)-self.yaw_alignment_timestamp)*1e-6,6)
                                                  if self.pos is not None else None),
                      fusion_status_max_age_s=FUSION_STATUS_MAX_AGE)
        if latest is not None:
            detail.update(latest_slam_capture_age_s=age(latest['stamp']),
                          latest_slam_receipt_age_s=age(latest['received']),
                          latest_slam_finite_pose=latest['finite_pose'],
                          latest_slam_quaternion_norm_sq=latest['quaternion_norm_sq'],
                          slam_frames=[latest['frame_id'], latest['child_frame_id']])
        return json.dumps(detail, sort_keys=True)

    def position_ready(self, now):
        if self.pos is None:
            return False
        # PX4's compass final-heading flag waits for an in-flight alignment.
        # Its native yaw alignment is usable on the ground, as in refnav/node.py.
        # Require fresh reception AND compatible source time; replayed/future
        # flags must not make an unknown heading eligible for an arm request.
        flags_age_us = int(self.pos.timestamp) - self.yaw_alignment_timestamp
        aligned = (self.yaw_aligned and self.yaw_alignment_time is not None
                   and 0 <= now - self.yaw_alignment_time <= 3.0
                   and self.yaw_alignment_timestamp > 0
                   and 0 <= flags_age_us <= 3_000_000)
        return bool(self.pos.xy_valid and self.pos.z_valid
                    and (self.pos.heading_good_for_control or aligned)
                    and not self.pos.dead_reckoning
                    and all(math.isfinite(value) for value in
                            (self.pos.x, self.pos.y, self.pos.z, self.pos.heading)))

    def clock_seconds(self):
        # Standard ROS time: hardware uses system time; deployment selects
        # use_sim_time for simulation. No simulator or lab API is imported.
        return self.get_clock().now().nanoseconds * 1e-9

    def timestamp(self):
        return self.get_clock().now().nanoseconds // 1000

    def command(self, cmd, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp()
        msg.command = cmd
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def publish_setpoint(self):
        # Tell PX4 we're controlling position
        mode = OffboardControlMode()
        mode.timestamp = self.timestamp()
        mode.position = True
        self.offboard_pub.publish(mode)

        # PX4 uses NED coordinates: negative Z is UP
        sp = TrajectorySetpoint()
        sp.timestamp = self.timestamp()
        sp.position = [self.x, self.y, self.target_z]
        sp.velocity = [math.nan] * 3
        sp.acceleration = [math.nan] * 3
        sp.yaw = self.yaw
        self.setpoint_pub.publish(sp)

    def at_target(self):
        return (math.hypot(self.pos.x - self.x, self.pos.y - self.y) <= POSITION_TOLERANCE
                and abs(self.pos.z - self.target_z) <= HEIGHT_TOLERANCE
                and abs(math.atan2(math.sin(self.pos.heading - self.yaw),
                                   math.cos(self.pos.heading - self.yaw))) <= YAW_TOLERANCE)

    def settled_for(self, now, duration):
        if not self.at_target():
            self.arrival_since = None
            return False
        if self.arrival_since is None:
            self.arrival_since = now
        return now - self.arrival_since >= duration

    def start_phase(self, phase, now):
        self.phase = phase
        self.phase_start = now
        self.arrival_since = None
        self.last_motion_time = now
        self.get_logger().info("Phase: " + phase)

    def advance_maneuver(self, now):
        dt = max(0.0, min(0.2, now - self.last_motion_time))
        self.last_motion_time = now
        if self.phase == "YAW":
            self.yaw_progress = min(math.pi, self.yaw_progress + YAW_RATE * dt)
            angle = self.initial_yaw + self.yaw_progress
            self.yaw = math.atan2(math.sin(angle), math.cos(angle))
            if self.yaw_progress >= math.pi and self.settled_for(now, HOVER_TIME):
                self.begin_landing()
        elif self.phase in ("OUTBOUND", "RETURN"):
            if self.phase == "OUTBOUND":
                goal_x = self.home_x + DISTANCE * math.cos(self.initial_yaw)
                goal_y = self.home_y + DISTANCE * math.sin(self.initial_yaw)
            else:
                goal_x, goal_y = self.home_x, self.home_y
            dx, dy = goal_x - self.x, goal_y - self.y
            distance = math.hypot(dx, dy)
            step = min(distance, MOVE_SPEED * dt)
            if distance > 0:
                self.x += dx * step / distance
                self.y += dy * step / distance
            if distance <= MOVE_SPEED * dt + 1e-9:
                self.x, self.y = goal_x, goal_y
                if self.settled_for(now, SETTLE_TIME):
                    if self.phase == "OUTBOUND":
                        self.start_phase("FORWARD_HOLD", now)
                    else:
                        self.begin_landing()
        elif self.phase == "FORWARD_HOLD":
            if self.settled_for(now, FORWARD_HOLD):
                self.start_phase("RETURN", now)
        if (self.phase in ("YAW", "OUTBOUND", "FORWARD_HOLD", "RETURN")
                and now - self.phase_start > MANEUVER_TIMEOUT):
            self.get_logger().warning("Maneuver timeout; landing at current estimated position")
            self.x, self.y = float(self.pos.x), float(self.pos.y)
            self.yaw = float(self.pos.heading)
            self.begin_landing()

    def begin_landing(self):
        self.get_logger().info("Landing")
        self.phase = "LAND"
        self.last_command = 0.0
        self.ground_confirmation_started = None
        self.auto_land_observed = False

    def tick(self):
        now = self.clock_seconds()

        if self.phase in ("DONE", "ABORT"):
            return

        if self.last_tick_time is not None and now < self.last_tick_time:
            self.get_logger().error("ROS clock moved backwards; yielding control")
            self.phase = "ABORT"
            return
        self.last_tick_time = now
        if self.wait_started is None:
            self.wait_started = now
        if self.phase in ("WAIT", "PRIME") and now - self.wait_started >= self.preflight_timeout_s:
            self.get_logger().error("Preflight timeout: SLAM/bridge/PX4 fusion or position checks not ready; no arm request sent")
            self.phase = "ABORT"
            return

        # Wait for position and status telemetry
        if self.pos is None or self.status is None:
            return

        armed = (
            self.status.arming_state ==
            VehicleStatus.ARMING_STATE_ARMED
        )
        offboard = (
            self.status.nav_state ==
            VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

        # Applies during landing too: stale status must never authorize retries.
        if now - self.pos_time > 1.0 or now - self.status_time > 1.0:
            if self.phase in ("WAIT", "PRIME"):
                # No mode/arm request has been sent. Cancel priming and require
                # a whole fresh interval after recovery; emit nothing meanwhile.
                self.phase = "WAIT"
                return
            self.get_logger().error("Telemetry lost; stopping Offboard stream")
            self.phase = "ABORT"
            return

        if self.phase == "PRIME" and armed:
            self.get_logger().warning("Vehicle armed before our request; yielding control")
            self.phase = "ABORT"
            return

        if self.phase == "LAND":
            if armed and self.ground_confirmation_started is not None:
                self.get_logger().error("Vehicle re-armed during ground confirmation; yielding control")
                self.phase = "ABORT"
                return
            if armed and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                self.auto_land_observed = True
            if not armed:
                if self.landed is True and now - self.landed_time <= LAND_STATUS_MAX_AGE:
                    self.get_logger().info("Landed and disarmed")
                    self.phase = "DONE"
                else:
                    if (self.status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND
                            and not self.auto_land_observed):
                        self.get_logger().error("Unexpected disarm without confirmed ground; inspect vehicle")
                        self.phase = "ABORT"
                        return
                    # Status and land detection arrive independently. Keep the
                    # bounded land-status freshness requirement; wait for a new ground
                    # report without publishing anything to a disarmed vehicle.
                    if self.ground_confirmation_started is None:
                        self.ground_confirmation_started = now
                    if now - self.ground_confirmation_started >= 5.0:
                        self.get_logger().error("Disarmed without fresh ground confirmation; inspect vehicle")
                        self.phase = "ABORT"
                return

            # Let PX4 handle the automatic descent
            if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                return

            if self.auto_land_observed:
                self.get_logger().warning("Flight mode changed after automatic landing began; yielding control")
                self.phase = "ABORT"
                return

            if offboard:
                problem = self.vision_problem(now)
                if problem is not None or not self.position_ready(now):
                    self.get_logger().error("Landing handover: " + (problem or "position/heading invalid") + "; yielding control; " + self.vision_diagnostics(now))
                    self.phase = "ABORT"
                    return
                self.publish_setpoint()
                if now - self.last_command > 1.0:
                    self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    self.last_command = now
            else:
                self.get_logger().warning(
                    "Flight mode changed; yielding control to PX4/pilot"
                )
                self.phase = "ABORT"
            return

        # Yield before publishing another setpoint after loss of our authority.
        if self.phase in ("CLIMB", "HOVER", "YAW", "OUTBOUND", "FORWARD_HOLD", "RETURN") and (not armed or not offboard):
            self.phase = "ABORT"
            return

        if self.phase not in ("WAIT", "PRIME") and not self.position_ready(now):
            self.get_logger().error("Position/heading invalid; stopping Offboard stream")
            self.phase = "ABORT"
            return

        problem = self.vision_problem(now)
        if problem is not None:
            if self.phase in ("WAIT", "PRIME"):
                self.phase = "WAIT"  # Recovery must earn a new full priming interval.
                if now - self.last_wait_log >= 5.0:
                    self.get_logger().info("Waiting: " + problem + "; " + self.vision_diagnostics(now))
                    self.last_wait_log = now
            else:
                self.get_logger().error(problem + "; stopping Offboard stream; " + self.vision_diagnostics(now))
                self.phase = "ABORT"
            return

        ground_ready = self.landed is True and 0 <= now - self.landed_time <= LAND_STATUS_MAX_AGE
        if self.phase == "WAIT":
            if armed or not ground_ready or not self.position_ready(now):
                if now - self.last_wait_log >= 5.0:
                    self.get_logger().info("Waiting for fresh landed/disarmed state and valid PX4 position/heading")
                    self.last_wait_log = now
                return

            # Record the current position as our starting point
            self.x = float(self.pos.x)
            self.y = float(self.pos.y)
            self.z = float(self.pos.z)
            self.yaw = float(self.pos.heading)
            self.target_z = self.z - HEIGHT
            self.home_x, self.home_y = self.x, self.y
            self.initial_yaw = self.yaw

            self.get_logger().info("Position valid; priming Offboard")
            self.phase = "PRIME"
            self.phase_start = now

        if self.phase == "PRIME" and (not ground_ready or not self.position_ready(now)):
            self.get_logger().warning("Position/heading lost while priming; withholding arm request")
            self.phase = "WAIT"
            return

        if self.phase in ("YAW", "OUTBOUND", "FORWARD_HOLD", "RETURN"):
            self.advance_maneuver(now)
            if self.phase == "LAND":
                return

        # Continuously send Offboard heartbeat and position target
        self.publish_setpoint()

        if self.phase == "PRIME":
            # PX4 needs over 1 second of Offboard heartbeats
            if now - self.phase_start >= 2.0:
                self.command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0
                )
                self.command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0
                )
                self.get_logger().info("Offboard and arm requested")
                self.phase = "ACTIVATE"
                self.phase_start = now

        elif self.phase == "ACTIVATE":
            if armed and offboard:
                self.get_logger().info("Armed; climbing")
                self.phase = "CLIMB"
                self.phase_start = now
            elif now - self.phase_start > 8.0:
                self.get_logger().error("Arming/Offboard timeout")
                if armed:
                    self.begin_landing()
                else:
                    self.phase = "ABORT"

        elif self.phase == "CLIMB":
            if not armed or not offboard:
                self.get_logger().error("Offboard control lost")
                self.phase = "ABORT"
            elif self.settled_for(now, SETTLE_TIME):
                self.get_logger().info("Reached takeoff hover")
                self.start_phase("HOVER", now)
            elif now - self.phase_start > 25.0:
                self.get_logger().warning("Climb timeout")
                self.begin_landing()

        elif self.phase == "HOVER":
            if self.settled_for(now, HOVER_TIME):
                self.begin_landing()
            elif now - self.phase_start > MANEUVER_TIMEOUT:
                self.get_logger().warning("Hover failed to settle; landing")
                self.begin_landing()


def ros_arguments():
    # Ordinary ROS deployment data, used on both the Jetson and the simulator.
    # No file means standard ROS system time. An explicitly configured missing
    # file is an installation error, not permission to silently select a clock.
    configured = os.environ.get("DRONE_PARAMS_FILE")
    parameters = Path(configured or "/etc/flyx/mission.yaml")
    args = list(sys.argv[1:])
    if parameters.is_file():
        return ["--ros-args", "--params-file", str(parameters), *args]
    if configured:
        raise FileNotFoundError("Configured ROS parameter file is missing: " + str(parameters))
    return args


def main():
    rclpy.init(args=ros_arguments())
    node = TestFlight()
    try:
        while rclpy.ok() and node.phase not in ("DONE", "ABORT"):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warning(
            "Node interrupted; verify vehicle state in QGroundControl"
        )
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if node.phase == "DONE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
