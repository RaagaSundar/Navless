import json
import math

# This is ordinary ROS/PX4 code: no runtime, cloud API or mission wrapper.
# Use the topic versions exported by the installed px4_msgs generation.
def px4_topic(name, message_type):
    version = int(getattr(message_type, "MESSAGE_VERSION", 0))
    return "/fmu/out/" + name + (f"_v{version}" if version else "")

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy,
    DurabilityPolicy, HistoryPolicy
)

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from px4_msgs.msg import (
    VehicleLocalPosition,
    VehicleStatus,
    VehicleLandDetected,
    EstimatorStatusFlags,
    VehicleOdometry
)

VISION_MAX_AGE = 0.5


class Telemetry(Node):
    """Read-only checks from the installed test scripts; owns no timers or control writers."""
    def __init__(self, name):
        super().__init__(name)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
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
            "vision_health_topic", "/onboard/vision_health").value)
        self.create_subscription(Odometry, self.slam_topic, self.on_slam_odometry, sensor_qos)
        self.create_subscription(VehicleOdometry, self.vision_topic, self.on_vision_odometry, sensor_qos)
        self.create_subscription(Bool, self.health_topic, self.on_vision_health, sensor_qos)
        self.pose = None
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
        self.phase = "WAIT"
        self.position_resets = None

    def on_position(self, msg):
        resets = (msg.xy_reset_counter, msg.z_reset_counter, msg.heading_reset_counter)
        if self.position_resets is not None and resets != self.position_resets:
            if self.phase == "WAIT":
                self.prime = None
            elif self.phase not in ("OBSERVE", "DONE", "ABORT"):
                self.get_logger().error("Estimator frame reset; yielding control")
                self.phase = "ABORT"
        self.position_resets = resets
        if self.pos is not None and msg.timestamp <= self.pos.timestamp:
            return
        self.pos = msg
        self.pos_time = self.clock_seconds()

    def on_status(self, msg):
        if self.status is not None and msg.timestamp <= self.status.timestamp:
            return
        self.status = msg
        self.status_time = self.clock_seconds()

    def on_landed(self, msg):
        if msg.timestamp <= getattr(self, "_land_stamp", 0):
            return
        self._land_stamp = msg.timestamp
        self.landed = bool(msg.landed)
        self.landed_time = self.clock_seconds()

    def on_estimator_flags(self, msg):
        if int(msg.timestamp) <= self.yaw_alignment_timestamp:
            return
        self.estimator_flags = msg
        self.yaw_aligned = bool(msg.cs_yaw_align)
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
                 and frames == ("odom", "base_link") and stamp > 0
                 and 0 <= now - stamp <= VISION_MAX_AGE)
        self.last_slam_observation = dict(
            stamp=stamp, received=now, frame_id=frames[0], child_frame_id=frames[1],
            finite_pose=all(math.isfinite(v) for v in values),
            quaternion_norm_sq=sum(v*v for v in values[3:]))
        self.slam_valid = valid
        if not valid or (self.slam_stamp is not None and stamp <= self.slam_stamp):
            return  # Repeated/reordered samples never refresh their age.
        if self.slam_frames is not None and frames != self.slam_frames:
            if self.phase == "WAIT":
                self.prime = None
            elif self.phase not in ("OBSERVE", "DONE", "ABORT"):
                self.get_logger().error("SLAM odometry frame changed; yielding control")
                self.phase = "ABORT"
        self.slam_frames = frames
        self.pose = (p.x, p.y, p.z, math.atan2(2*(q.w*q.z + q.x*q.y),
                                             q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z))
        self.slam_stamp, self.slam_time = stamp, now

    def on_vision_odometry(self, msg):
        now = self.clock_seconds()
        stamp = int(msg.timestamp_sample) * 1e-6
        published = int(msg.timestamp) * 1e-6
        values = (*msg.position, *msg.q)
        valid = (msg.pose_frame in (VehicleOdometry.POSE_FRAME_NED, VehicleOdometry.POSE_FRAME_FRD)
                 and all(math.isfinite(v) for v in values)
                 and 0.9 <= sum(v * v for v in msg.q) <= 1.1
                 and stamp > 0 and stamp <= published
                 and 0 <= now - stamp <= VISION_MAX_AGE
                 and 0 <= now - published <= VISION_MAX_AGE)
        self.vision_valid = valid
        if not valid or (self.vision_stamp is not None and stamp <= self.vision_stamp):
            return
        if self.vision_reset is not None and msg.reset_counter != self.vision_reset:
            if self.phase == "WAIT":
                self.prime = None
            elif self.phase not in ("OBSERVE", "DONE", "ABORT"):
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
                or not 0 <= now - self.slam_stamp <= VISION_MAX_AGE
                or not 0 <= now - self.slam_time <= VISION_MAX_AGE):
            return "SLAM odometry missing, invalid or stale"
        if (not self.vision_healthy or self.vision_health_time is None
                or not 0 <= now - self.vision_health_time <= VISION_MAX_AGE):
            return "vision bridge health missing, false or stale"
        if (not self.vision_valid or self.vision_stamp is None or self.vision_time is None
                or not 0 <= now - self.vision_stamp <= VISION_MAX_AGE
                or not 0 <= now - self.vision_time <= VISION_MAX_AGE):
            return "PX4 external-vision input missing, invalid or stale"
        flags = self.estimator_flags
        if (flags is None or self.yaw_alignment_time is None or self.pos is None
                or not 0 <= now - self.yaw_alignment_time <= 1.0
                or int(flags.timestamp) <= 0
                or abs(int(self.pos.timestamp) - int(flags.timestamp)) > 1_000_000):
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
                      fusion_status_age_s=age(self.yaw_alignment_time))
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
        return bool(self.pos.xy_valid and self.pos.z_valid and not self.pos.dead_reckoning
                    and (self.pos.heading_good_for_control or aligned)
                    and all(math.isfinite(value) for value in
                            (self.pos.x, self.pos.y, self.pos.z, self.pos.heading)))

    def clock_seconds(self):
        # Standard ROS time: hardware uses system time; deployment selects
        # use_sim_time for simulation. No simulator or lab API is imported.
        return self.get_clock().now().nanoseconds * 1e-9

