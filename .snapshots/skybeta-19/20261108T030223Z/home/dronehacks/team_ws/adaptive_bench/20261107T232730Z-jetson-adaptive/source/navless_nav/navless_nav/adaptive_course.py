"""Opt-in ROS adapter for surveyed-course local detours; never a commander.

Requires calibrated camera->base TF and acquisition-aligned continuous odom.
Planning runs outside the setpoint timer; pending/failed plans hold a fixed
pose only while current odometry and control authority remain fresh.
"""
from concurrent.futures import ThreadPoolExecutor
import json
import math
import time

import numpy as np

from .adaptive_route import (Bounds, ConfirmedDepth, NoDetour, adapt_route,
                             direction_observed, source_identity, survey_anchors, surveyed_course)
from .route import Carrot, course_to_odom, odom_to_course, resolve_route_path
from .stereo_tube import OdomHistory, rotation


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class AdaptiveCourse:
    def __init__(self, mission, survey_path, route_path):
        import tf2_ros
        from rclpy.time import Time
        from sensor_msgs_py import point_cloud2
        self.tf_error, self.Time, self.cloud_reader = tf2_ros.TransformException, Time, point_cloud2
        self.node = mission
        self.bounds = Bounds()
        self.prior = surveyed_course(survey_path, self.bounds)
        route_data = json.load(open(route_path))
        declared = route_data.get('course_file')
        if not declared or source_identity(resolve_route_path(declared)) != source_identity(survey_path):
            raise ValueError('adaptive route must identify this exact as-built survey file')
        if route_data.get('route_type') != 'WINDOW_CENTRELINE':
            raise ValueError('adaptive survey crossing constraints currently support the window course only')
        self.route = self.to_course(mission.route)
        if max(p[2] for p in self.route) > 1.1:
            raise ValueError('adaptive route exceeds the range-beam model height bound')
        self.anchors = survey_anchors(self.route, survey_path)
        if not self.prior.route_free(self.route):
            try:
                initial = adapt_route(self.prior, self.route, self.anchors,
                                      self.route[0], 0, budget_s=2.)
            except NoDetour as exc:
                raise ValueError('baseline cannot satisfy adaptive envelope: '+str(exc)) from exc
            self.route, self.anchors = initial.points, initial.anchors
            mission.route = [course_to_odom(p, mission.frame[0], mission.frame[1], mission.start_height)
                             for p in self.route]
            mission.get_logger().info('adaptive preflight adjusted baseline for stricter envelope; '
                                      'required crossings retained; original route file unchanged')
        self.depth = ConfirmedDepth(max_voxels=2000)
        self.map = self.prior
        self.history = OdomHistory()
        self.odom_frame, self.base_frame = 'odom', 'base_link'
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, mission)
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='local-route')
        self.future = None
        # Preflight just validated this exact prior and complete route. Runtime
        # already checks every actual-current -> proposed-target edge. Repeating
        # whole-course verification on revision zero wastes the worker budget
        # alongside real ROS/cloud processing on Jetson and can latch a false hold.
        self.checked_revision = self.depth.revision
        self.failed_revision = None
        self.hold = None
        self.hold_reason = None
        self.fault = None
        self.pending_cloud = None
        self.last_cloud = -math.inf
        self.valid_cloud_at = None
        self.visible = np.empty((0, 3))
        self.last_now = -math.inf
        self.route_epoch = 0
        self.registration_offset = None
        self.camera_origin = None
        self.camera_returns = np.empty((0, 3))
        mission.create_timer(.02, self.process_cloud)
        mission.get_logger().info('EXPERIMENTAL adaptive as-built routing: survey_sha256='
                                  +source_identity(survey_path)+' route_sha256='+source_identity(route_path)
                                  +' assumed map/tracking errors=.02/.02m; attitude<=5deg; no flight qualification')

    def close(self):
        # The worker has its own bounded deadline and makes no ROS calls.
        self.executor.shutdown(wait=True, cancel_futures=True)

    def to_course(self, points):
        n = self.node
        return [odom_to_course(p, n.frame[0], n.frame[1], n.start_height) for p in points]

    def latch(self, reason):
        if self.fault is None:
            self.node.get_logger().error('adaptive routing stopped: '+reason)
        self.fault = reason

    def observe_odom(self, msg):
        now = self.node.now_s()
        try:
            stamp = seconds(msg.header.stamp)
            if (msg.header.frame_id != self.odom_frame or msg.child_frame_id != self.base_frame
                    or not 0 <= now-stamp <= .25):
                raise ValueError('odometry frame or acquisition age invalid')
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            epoch = self.history.epoch
            if not self.history.add(stamp, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)):
                if self.history.epoch != epoch:
                    self.latch('odometry discontinuity; map registration must be reviewed')
                return
            b = math.radians(self.node.frame[1])
            course_from_odom = np.array([[math.sin(b), -math.cos(b), 0],
                                         [math.cos(b), math.sin(b), 0], [0, 0, 1]])
            desired = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
            actual = course_from_odom @ rotation((q.x, q.y, q.z, q.w))
            angle = math.degrees(math.acos(float(np.clip((np.trace(desired.T @ actual)-1)/2, -1, 1))))
            if angle > self.bounds.attitude_deg:
                self.latch('attitude outside collision-envelope assumption')
            measured = np.asarray((p.x, p.y, p.z))-np.asarray(self.node.odom_zero_offset)
            if self.to_course([measured])[0][2] > 1.1:
                self.latch('height outside range-beam model assumption')
        except (ValueError, AttributeError, TypeError) as exc:
            self.latch(str(exc))

    def healthy(self, now):
        n = self.node
        if now < self.last_now:
            self.latch('clock moved backwards')
        self.last_now = now
        if (n.state not in ('WAIT_FOR_OFFBOARD', 'DONE')
                and not (n.state == 'LAND' and n.landed)
                and (not n.armed or not n.offboard_active)):
            self.latch('control authority lost; restart after team review')
        return (self.fault is None and n.vo_state == 1 and n.vo_at is not None
                and 0 <= now-n.vo_at <= .25 and self.history.samples
                and 0 <= now-self.history.samples[-1][0] <= .25
                and n.state_at is not None and 0 <= now-n.state_at <= .5)

    def cloud(self, msg):
        try:
            stamp = seconds(msg.header.stamp)
            if stamp <= self.last_cloud or not 0 <= self.node.now_s()-stamp <= .25:
                raise ValueError('stale or out-of-order depth')
            self.last_cloud, self.pending_cloud = stamp, msg
        except (ValueError, AttributeError, TypeError):
            self.valid_cloud_at = None
            self.pending_cloud = None

    def process_cloud(self):
        msg, n = self.pending_cloud, self.node
        now = n.now_s()
        if msg is None or not self.healthy(now) or not n.odom_zeroed:
            return
        offset = tuple(n.odom_zero_offset)
        if self.registration_offset is None:
            self.registration_offset = offset
        elif offset != self.registration_offset:
            self.latch('launch registration changed after depth mapping began')
            return
        stamp = seconds(msg.header.stamp)
        if not 0 <= now-stamp <= .25:
            self.pending_cloud = None
            self.valid_cloud_at = None
            return
        pose = self.history.at(stamp)
        if pose is None:
            return  # only measured bracketing poses, never extrapolate
        self.pending_cloud = None
        try:
            tf = self.tf.lookup_transform(self.base_frame, msg.header.frame_id,
                                          self.Time.from_msg(msg.header.stamp))
            t, q = tf.transform.translation, tf.transform.rotation
            mount = rotation((q.x, q.y, q.z, q.w))
            raw = np.asarray(self.cloud_reader.read_points_numpy(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)).reshape(-1, 3)
            if len(raw) > 6000:
                raw = raw[np.linspace(0, len(raw)-1, 6000, dtype=int)]
            valid = np.isfinite(raw).all(axis=1)
            ranges = np.linalg.norm(raw, axis=1)
            raw = raw[valid & (ranges >= .30) & (ranges <= 2.80)]
            if len(raw) < 30:
                raise ValueError('insufficient valid stereo returns')
            position, attitude = pose
            pts = (raw @ mount.T + (t.x, t.y, t.z)) @ attitude.T + position
            pts -= np.asarray(n.odom_zero_offset)
            course_points = np.asarray(self.to_course(pts))
            camera_odom = np.asarray((t.x, t.y, t.z)) @ attitude.T + position - np.asarray(n.odom_zero_offset)
            self.camera_origin = self.to_course([camera_odom])[0]
            self.camera_returns = course_points
            self.visible = self.prior.novel_points(course_points)
            keys = np.unique(np.floor(self.visible/self.depth.voxel).astype(np.int64), axis=0)
            if len(keys) > self.depth.limit:
                raise ValueError('depth map capacity exceeded')
            self.visible = (keys+.5)*self.depth.voxel
            self.depth.add(self.visible, stamp, now)
            self.map = self.prior.with_points(self.depth.points(), self.depth.voxel)
            self.valid_cloud_at = stamp
        except (self.tf_error, ValueError, TypeError, AttributeError) as exc:
            self.valid_cloud_at = None
            if 'capacity' in str(exc):
                self.latch(str(exc))

    def route_tick(self, now):
        """Called only in ROUTE; preserves the existing interface as commander."""
        n = self.node
        if not self.healthy(now) or not n.armed or not n.offboard_active:
            self.hold = None
            self.hold_reason = 'integrity_or_authority_stop'
            return  # stale estimates/authority: publish nothing
        current = self.to_course([n.position])[0]
        if self.hold is None:
            self.hold = tuple(n.position)
        if self.valid_cloud_at is None or not 0 <= now-self.valid_cloud_at <= .25:
            self.hold_reason = 'depth_stale_or_missing'
            n.send(*self.hold)
            return
        if not self.map.free(current):
            self.latch('current pose is inside the collision envelope')
            return
        revision = self.depth.revision
        if self.future is not None and self.future.done():
            future, self.future = self.future, None
            try:
                update = future.result()
                if (revision != self.request_revision or self.route_epoch != self.request_epoch
                        or time.monotonic()-self.request_wall > .5
                        or math.dist(current, self.request_position) > .05):
                    # Discard stale work, do not install a route for an old pose/map.
                    self.checked_revision = -1
                else:
                    self.checked_revision = revision
                    if update.changed:
                        self.route, self.anchors = update.points, update.anchors
                        n.route = [course_to_odom(p, n.frame[0], n.frame[1], n.start_height)
                                   for p in self.route]
                        n.carrot = Carrot(n.route, n.cruise_speed, n.max_lead,
                                          n.max_lat_accel, .01)
                        n.progress_at = (now, 0, n.carrot.point)
                        n.get_logger().info('adaptive local detour installed; remaining ordered anchors='
                                            +str(len(self.anchors)))
            except (NoDetour, ValueError) as exc:
                self.failed_revision = revision
                n.get_logger().warn('adaptive route blocked; holding: '+str(exc))
            except Exception as exc:
                self.latch('planner worker failed: '+type(exc).__name__)
                return
        if self.failed_revision == revision:
            self.hold_reason = 'planner_refused'
            n.send(*self.hold)
            return
        if self.future is None and self.checked_revision != revision:
            self.request_revision, self.request_epoch = revision, self.route_epoch
            self.request_position, self.request_wall = current, time.monotonic()
            self.future = self.executor.submit(adapt_route, self.map, self.route,
                                               self.anchors, current, n.carrot.index)
        if self.future is not None:
            self.hold_reason = 'planner_pending'
            n.send(*self.hold)
            return
        old_index, old_point = n.carrot.index, n.carrot.point
        point = n.carrot.step(n.position, min(.1, max(0, now-n.last_tick)))
        target = self.to_course([point])[0]
        # An immediate single-frame guard brakes before temporal confirmation.
        guard = self.map.with_points(self.visible, self.depth.voxel)
        clear = guard.free(current, target)
        observed = direction_observed(current, target, self.camera_origin, self.camera_returns)
        if not clear or not observed:
            self.hold_reason = 'collision_guard' if not clear else 'travel_direction_unobserved'
            n.carrot.index, n.carrot.point = old_index, old_point
            n.send(*self.hold)
            return
        self.hold = None
        self.hold_reason = None
        n.send(*point)
        if n.carrot.done(n.position, .08):
            n.goto('LAND' if n.land_at_end else 'HOLD', point)
        n.last_tick = now
