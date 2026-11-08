"""Navless course node: take over on the pilot's arm, fly a planner route.

It uses the stack's competitor topics, and nothing else (FI_NS below is the
flight interface's topic namespace):

    subscribe  /visual_slam/tracking/odometry   where we are (ENU, odom)
    subscribe  FI_NS/state                      what PX4 is doing (JSON)
    subscribe  /tube/offset                     tube centre from the side ToF
                                                sensors (tof_tube_centering only)
    subscribe  /fmu/out/vehicle_local_position  PX4's heading, read only (the
                                                heading guard; never /fmu/in)
    subscribe  /team/localize/odom_correction   cloud_localize's drift correction
                                                (wall_correction only), used only when
    subscribe  /team/localize/status            its status vouches for that exact cloud
    subscribe  /visual_slam/status              cuVSLAM tracking (required)
    publish    FI_NS/setpoint/{pose,twist}      where we want to be (ENU, odom),
                                                with the takeoff heading held
    publish    /team/navigation/status          JSON, every tick: state, fault, readiness

Never publishes to /fmu/*, never arms, never changes mode: flight_interface
owns that, and enters Offboard once when the pilot arms, then switches to
Position on the ground.

States (10 Hz timer):

    WAIT_FOR_OFFBOARD  stream the current position. A setpoint from us in the
                       last second is one of flight_interface's conditions
                       for taking over. Pre-arm height check: we stream only
                       once PX4's height (FI_NS/state `local_enu` z) has
                       matched odom z for prearm_settle_s. PX4 can boot with
                       its height ~0.6 m off (EKF2 parks the error in its
                       terrain estimate, and with EV_CTRL 9 nothing pulls it
                       back); flight_interface passes odom z to PX4 unchanged,
                       so every setpoint would then be flown that far off.
                       A steady offset within prearm_height_tol_m (19: ~0.19
                       m) is measured here and added to every height sent
                       (match_px4_height). Both PX4's heading and the
                       camera's must agree too (prearm_heading_tol_deg).
                       With no stream, flight_interface refuses the takeover
                       and the pilot keeps Position.
    REFUSED            Offboard came anyway while the heights disagreed (the
                       stream had stopped less than flight_interface's
                       setpoint timeout before the switch): descend in place
                       with twist setpoints until PX4 disarms, never climb.
    TAKEOFF            climb straight up. PX4 disarms if we are not off the
                       ground 10 s after arming, so this starts on the first
                       tick of Offboard. The heading the drone has now is
                       held from here on (hold_heading, with the heading
                       guard), and the route starts only once the target
                       has been held within takeoff_settle_radius_m for
                       takeoff_settle_s.
    ROUTE              walk route_file's waypoints with a speed-limited
                       carrot (navless_nav/route.py). The carrot freezes
                       while cuVSLAM reports lost tracking or odometry is
                       stale, and resumes when it is back. With blind_carry,
                       a loss inside the dark tube instead keeps the carrot
                       pacing PX4's own estimate for a few seconds, so the
                       drone does not stop where tracking cannot return.
                       With wall_correction, the route is flown in the frame
                       cloud_localize's drift correction keeps on the walls.
    LAND               route done: descend onto its end point (the landing
                       pad) with twist setpoints until PX4 detects the
                       touchdown and disarms (COM_DISARM_LAND). The twist
                       never stops before that: an expired twist makes
                       flight_interface fly back to the last pose, 0.85 m
                       up.
    DONE               landed. Publishes nothing, so a later arming cannot
                       hand the vehicle to us again. With fly_home, the
                       pad touchdown goes back to WAIT_FOR_OFFBOARD instead,
                       with the route swapped for the home leg (climb to
                       home_altitude_m, straight to the launch mark); the
                       pilot re-arms, and the touchdown there is DONE.
    HOLD               hover at the last target.
    PAUSED             the pilot took over. Nothing advances; once the
                       vehicle is disarmed we go back to waiting.
    FAULT              latched for good (see latch_navigation_fault): an
                       odometry jump/reset/stale clock, a PX4 heading reset
                       in flight, VIO lost outside ROUTE, a localizer epoch
                       change, or sustained lateral error in ROUTE. Nothing
                       is commanded again (a twist descent keeps descending);
                       the pilot takes over; restart the node to fly again.

Every setpoint outside the idle states is pulled inside the net by
net_margin_m: flight_interface clamps altitude only, never x/y.
"""
import json
import math
import threading
import time
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String

from navless_nav.route import (Carrot, clamp_to_net, course_to_odom,
                               create_live_course_planner, load_route,
                               odom_to_course, plan_dynamic_course_route,
                               resolve_route_path)
from navless_nav.tube_guidance import TubeGuidance
from navless_nav.flight_log import FlightLog, LoggingLogger, resolve_dir
from navless_nav.pad_tracker import PadTracker
from navless_nav.navigation_integrity import PoseIntegrity
from navless_nav.correction_contract import CorrectionContract
from navless_nav.cloud_transform import (CloudTransformError, rigid_transform,
                                        stamp_s, transform_pointcloud)

try:
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
except ImportError:   # off the drone: fall back to odometry staleness only
    VisualSlamStatus = None

try:
    from sensor_msgs.msg import PointCloud2
except ImportError:
    PointCloud2 = None

try:
    from px4_msgs.msg import VehicleLocalPosition
except ImportError:   # off the drone: the heading guard uses the camera only
    VehicleLocalPosition = None

VO_LOST = 2           # VisualSlamStatus.vo_state: 0 unknown, 1 tracking, 2 lost
# PX4's own heading and its reset counter, read only (the heading guard).
PX4_LOCAL_TOPIC = '/fmu/out/vehicle_local_position'
PX4_TIMEOUT_S = 0.5   # an older PX4 heading is not used

# The flight interface's topics. Their names are fixed by the image.
FI_NS = '/skybeta'
STATE_TOPIC = FI_NS + '/state'
POSE_TOPIC = FI_NS + '/setpoint/pose'
TWIST_TOPIC = FI_NS + '/setpoint/twist'
ODOM_TOPIC = '/visual_slam/tracking/odometry'

IDLE_STATES = ('WAIT_FOR_OFFBOARD', 'PAUSED', 'DONE')

# Pre-arm height check (see heights_agree): once passed, it holds until the
# disagreement is this much past prearm_height_tol_m, or PX4 has sent no
# height for PREARM_PX4_TIMEOUT_S.
PREARM_HYSTERESIS_M = 0.05
PREARM_PX4_TIMEOUT_S = 1.0


def wrap_pi(a):
    """Angle wrapped to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def drop_floor_points(pts, min_height_m):
    """Live stereo points (N x 3, course frame, z up from the floor) without
    those below min_height_m, before they go into the planner's obstacle map.
    The decoder already drops z < 0.15 m, but it places points with the
    drone's yaw only: pitched forward in flight (~5 deg), the floor 2-3 m ahead
    lands 0.15-0.25 m up and, inflated, blocks detours. Nothing that matters
    to the drone at flight height is lower than this (boulders 0.53 m, walls
    1.73 m)."""
    if pts is None or len(pts) == 0 or min_height_m <= 0.0:
        return pts
    return pts[pts[:, 2] >= min_height_m]

# The tube (course_v3.json `tunnel`: footprint x 182-240 in, y 108-168 in,
# axis north-south, "Dark inside"), in course metres (in * 0.0254 - 3.048).
TUBE_X = (1.5748, 3.048)
TUBE_Y = (-0.3048, 1.2192)

PARAMETERS = (
    # Navigation integrity (navigation_integrity.PoseIntegrity). An odometry sample counts only
    # with a source stamp no older than this, in these frames, and no faster than these bounds
    # since the last accepted one (source time). A frame/reset change or a jump latches FAULT:
    # nothing more is commanded and the node must be restarted (disarmed, realigned).
    ('odom_source_max_age_s', 0.25),
    ('odom_future_tolerance_s', 0.05),    # hardware_slam/odom_gate's camera-vs-node clock allowance
    ('odom_frame', 'odom'),
    ('odom_body_frame', 'base_link'),
    ('odom_max_speed_mps', 2.0),
    ('odom_max_angular_rate_rps', 3.0),
    # In commanded flight, the first sample after a source gap longer than this must be within
    # odom_gap_px4_tol_m of PX4's own estimate (plus their usual offset), or FAULT.
    ('odom_gap_check_s', 0.3),
    ('odom_gap_px4_tol_m', 0.35),
    # ROUTE only: a lateral distance from the route polyline beyond this for this long latches
    # FAULT. Unqualified candidate values, NOT a clearance guarantee for the 6-14 cm course gaps.
    ('tracking_error_max_m', 0.35),
    ('tracking_error_timeout_s', 0.75),
    # wall_correction: the localizer's D must hold still (2 cm, 0.5 deg) this long before the
    # pre-arm stream starts; the route is then flown from that registration.
    ('initial_alignment_settle_s', 2.0),
    ('cloud_capture_max_age_s', 0.5),
    ('takeoff_altitude_m', 0.85),
    # Climb target this far along the route's first leg instead of straight
    # above the drone, m (see takeoff_target). 0 = straight up. Gazebo, drone
    # profile, centreline route: Barrier 1 clearance during the climb 3.1-4.7
    # cm at 0, 16.1 cm at 0.15, 18.3 cm at 0.25. The first corridor is open
    # for 1.6 m ahead, so the larger lean is free if the real drone does not
    # drift at all, and it covers more backward drift if it does. Never on
    # the fly_home take-off from the pad (see lead_m).
    ('takeoff_lead_m', 0.25),
    ('arrival_radius_m', 0.25),
    # Settle before the route: hold the takeoff target within this (odom,
    # horizontal) for takeoff_settle_s first. On 19 (2026-11-07 test
    # flights) both liftoffs slid 25-33 cm sideways and took 10-15 s to come
    # back. If it never settles by takeoff_timeout_s, it holds there and the
    # route never starts. 0 s = off.
    ('takeoff_settle_radius_m', 0.05),
    ('takeoff_settle_s', 2.0),
    ('takeoff_timeout_s', 30.0),
    # Hold drift correction (the stabilisation from navless_flight_tests,
    # 2026-11-07: hover hold 6.7 cm mean on 19). While the target stands
    # still (TAKEOFF once the climb is done, HOLD), a steady offset of the
    # drone from it is integrated into a correction on the commanded point:
    # drift_ki per second, ignored inside drift_deadband_m, not learned beyond
    # drift_max_error_m (a tracking problem or the pilot, not a push), capped
    # at drift_max_m, only above drift_min_height_m. Never learned while the
    # target moves (that lag overshot the forward test). The push on 19 is
    # body-fixed (the liftoff slide flipped with a 180 deg yaw), so the
    # correction is kept in the body frame and turns with the nose; with
    # drift_apply_in_route it keeps cancelling the push along the route.
    # Arrival and settle checks use the true target, never the corrected one.
    ('drift_correction', True),
    ('drift_ki', 0.3),
    ('drift_deadband_m', 0.02),
    ('drift_max_error_m', 0.40),
    ('drift_max_m', 0.35),
    # Takeoff slide pre-compensation: the drift correction starts each first
    # takeoff already at this BODY-frame value (x forward, y left, m) instead of
    # zero, so the liftoff is pushed against a slide that happens before the
    # correction can learn (it only learns above drift_min_height_m). On 19
    # every liftoff slid 25-40 cm to the drone's LEFT (2026-11-07: 25-33 cm;
    # 2026-11-08: 21 cm, then 34 cm into the west net on Favour's north-leaning
    # takeoff), so the course launch seeds (0, -0.28): 28 cm to the right.
    # Learning refines it from there; 0, 0 = off (the node default).
    ('drift_seed_body_x_m', 0.0),
    ('drift_seed_body_y_m', 0.0),
    ('drift_min_height_m', 0.40),
    ('drift_apply_in_route', True),
    # Flight recorder (navless_nav/flight_log.py): a CSV row per tick and per
    # log line in flight_log_dir ('' = $NAVLESS_FLIGHT_LOG_DIR, else
    # ~/team_ws/flight_logs), and the parameters as JSON.
    ('flight_log', True),
    ('flight_log_dir', ''),
    # Hold the heading the drone took off with. A zero quaternion makes
    # flight_interface send yaw NaN, and PX4's PositionControl then sets its
    # yaw setpoint to the current heading every cycle ("TODO: better way to
    # disable yaw control"), so nothing turns the nose back after a nudge. On
    # 19, 2026-11-07: commanded, the heading held within 1.4 deg; with yaw
    # NaN (the twist descents) it wandered 6 deg in 10 s. Twist setpoints
    # (LAND) carry yaw rate only: heading_rate_gain * error, capped.
    ('hold_heading', True),
    ('heading_rate_gain', 1.0),           # 1/s
    ('heading_rate_max_dps', 20.0),
    # Which heading is held (needs hold_heading):
    #   'travel' (team decision, 2026-11-07): the nose points along the
    #   route, looking travel_heading_lookahead_m ahead of the carrot, turned
    #   at most travel_heading_rate_dps; EXCEPT through the tube, flown facing
    #   course north (the ToF sensors' side-on geometry and the tube's
    #   north-south axis): from when the route tube_heading_lookahead_m ahead
    #   of the carrot enters the tube zone (blind_zone_margin_m around the
    #   tube) until the carrot has left it. The camera then looks where the
    #   drone goes: the localizer sees walls ahead (sim: <= 13 cm vs 51 cm
    #   nose-north) and the planner's live obstacle map covers the path.
    #   'hold': the takeoff heading for the whole flight.
    # The heading guards below apply to both.
    ('heading_mode', 'travel'),
    ('travel_heading_lookahead_m', 0.5),
    ('travel_heading_rate_dps', 25.0),
    ('tube_heading_lookahead_m', 0.8),
    # The route enters the tube at its north mouth flying south, so facing
    # north there is a turn of up to 180 deg: the carrot waits just outside
    # the tube's footprint (the drone hovers at the mouth) until the nose is
    # within this of north, then flies in. It never waits inside the tube.
    ('tube_align_deg', 10.0),
    # A commanded heading is only safe while PX4's heading agrees with the
    # camera's. In a vision gap over 0.4 s, PX4 (EKF2_MAG_TYPE 0) resets its
    # heading to the magnetometer: ~100 deg off on 19's floor, 2026-11-07, four
    # times after landing. PX4 would then spin the drone to "correct" it. So:
    # if PX4 and the camera disagree by more than this, if PX4 resets its
    # heading in flight, or if the camera sees the nose pushed this far off
    # the held heading, command no heading until PX4 and the camera agree
    # again for heading_guard_s, then hold wherever the nose is.
    ('heading_guard_deg', 15.0),
    ('heading_guard_s', 1.0),
    # Pre-arm: also refuse to stream while PX4's heading and the camera's
    # disagree by more than this (PX4 on the magnetometer), deg. A fresh PX4
    # heading that is good for control is required; 0 never streams.
    ('prearm_heading_tol_deg', 10.0),
    ('publish_rate_hz', 10.0),
    # Where we read our own position. Use the same odometry that feeds PX4's
    # vision fusion: cuVSLAM's raw topic through the stock px4_odom_bridge,
    # or Koji's gated /mapping/validated_odometry (which drops samples while
    # cuVSLAM is not tracking) if PX4 is fed that. Then a VIO loss looks the
    # same to this node as to PX4 (see blind_carry and odom_arriving).
    ('odometry_topic', ODOM_TOPIC),
    # Route: a planner file, and how the drone sat on the launch mark at
    # power-up (nose bearing clockwise from course north; height of the odom
    # origin above the floor).
    ('route_file', 'route_gate_b.json'),
    ('start_bearing_deg', 0.0),
    # Pixhawk IMU on its feet: 0.07 m on the Gazebo stand-in, built from the
    # airframe's dimensions. Measure it on the real drone.
    ('start_height_m', 0.07),
    ('cruise_speed_mps', 0.5),
    ('max_lead_m', 0.6),
    # Tight turns (the wall-end arcs round C1, C2b and A1 on the Gate B
    # route): chasing a carrot `lead` ahead round an arc of radius R, the
    # drone cuts inside by ~lead^2 / (2 R). With this > 0 the lead shrinks
    # there so the cut stays under it, m (Carrot.lead_limit). 0 = off.
    ('max_turn_sagitta_m', 0.0),
    # Slow through bends: speed <= sqrt(this * bend radius). 0 = off.
    ('max_lat_accel_mps2', 0.25),
    # Refuse to start the route from somewhere else (e.g. a re-arm after the
    # pilot moved the drone): its first point is the launch mark.
    ('max_start_offset_m', 0.6),
    ('stall_timeout_s', 20.0),
    # Keep-out from the net (drone is 0.475 m wide with props).
    ('net_margin_m', 0.4),
    ('odom_timeout_s', 0.5),
    # Landing on the route's last point (scored: 7 points for the pad).
    ('land_at_end', True),
    ('land_speed_mps', 0.3),
    ('land_xy_max_mps', 0.3),     # centring on the pad while descending
    # Bonus return leg after the pad landing. Off by default. When on, the
    # pilot's next arming climbs to home_altitude_m (above the floor; the
    # walls are 1.727 m), flies straight over the wall to the launch mark and
    # lands there. At 2.2 m the feet clear the wall by ~0.38 m; PX4's floor
    # offset is flown out (match_px4_height), so that holds. The ceiling is
    # navless.launch.py's max_altitude_m 2.5 m above the odom origin (2.57 m
    # above the floor; flight_interface's own default is 3.0): the 2.13 m
    # odom setpoint plus PX4's offset (~0.19 m on 19) stays under it. Lower
    # that clamp and this too.
    ('fly_home', False),
    ('home_altitude_m', 2.2),
    # The hop's horizontal leg starts only once odom z, or PX4's own height,
    # is this close to the hop height (not at arrival_radius_m): starting
    # early crossed Barrier 1 at 1.91-1.99 m in SITL with home_altitude_m
    # 2.03. See climbed().
    ('home_climb_tol_m', 0.05),
    # The home leg lands only within this of the launch mark (not
    # arrival_radius_m): Barrier 1 is right behind the mark, and descending
    # while still closing the last 0.25 m left 14.2 cm to it in SITL.
    ('home_arrival_radius_m', 0.10),
    # Pre-arm height check (see WAIT_FOR_OFFBOARD). Largest |PX4 z - odom z|
    # we will take off with, m; 0 = off. Assumes flight_interface's
    # anchor_setpoints is false (its default, and navless.launch.py's): odom
    # z goes to PX4 unchanged. It refuses the ~0.6 m boot error (EKF2 parking
    # it in its terrain estimate). It was 0.15, but on skybeta-19 PX4 read
    # 0.185-0.198 m above odom on the floor in all three 2026-11-07 test
    # logs, so the course mission would never have taken over; with
    # match_px4_height that steady offset is measured and flown out.
    ('prearm_height_tol_m', 0.30),
    ('prearm_settle_s', 2.0),     # ...and for this long without a break
    # Add PX4's height minus odom's, measured on the floor (median of the last
    # 2 s of the pre-arm check, latched at takeoff), to every height sent.
    # PX4 holds its own z at the setpoint, so without this the whole flight is
    # flown that far off the route's heights (19: ~0.19 m low). 19 is only
    # 0.116 m tall, so its PX4 z of ~0.19 m on the floor is an offset, not a
    # height; odom (start_height_m above the floor) is the reference.
    ('match_px4_height', True),
    # The tube is dark inside, so cuVSLAM may lose tracking there. Off: a VIO
    # loss freezes the carrot, which in the tube means hovering blind until
    # PX4's dead reckoning times out (EKF2_NOAID_TOUT, 5 s on the fleet) and
    # it fails safe. On: while the carrot is in the tube (plus
    # blind_zone_margin_m at each mouth), a VIO loss keeps it pacing PX4's
    # own estimate (FI_NS/state local_enu) along the route for at most
    # blind_carry_max_s per loss. A loss anywhere else still freezes it.
    ('blind_carry', False),
    ('blind_carry_max_s', 4.0),
    ('blind_zone_margin_m', 0.4),
    # Experimental stereo wall correction. Never enabled by a hardware default.
    ('stereo_tube_centering', False),
    ('stereo_tube_max_shift_m', 0.20),
    ('stereo_tube_rate_mps', 0.10),
    ('stereo_tube_hold_s', 3.0),
    # Side ToF tube centring, off by default. tube/tube_node.py publishes
    # /tube/offset (how far the tube centreline is to the drone's left) only
    # while both side readings span the tube's width. While the drone and
    # the carrot are in the tube, the target is pulled toward that centre:
    # never more than tof_tube_max_shift_m from the route, so a misreading
    # can do no worse, and at tof_tube_rate_mps. It drops back to the route
    # at once when the offset is older than tof_tube_timeout_s, outside the
    # tube, or with nothing to measure from (VIO lost, no blind carry).
    # Needs the bus-7 sensor on the drone's LEFT: swapped sensors pull
    # toward a wall, up to the shift limit.
    ('tof_tube_centering', False),
    ('tof_tube_max_shift_m', 0.15),
    ('tof_tube_rate_mps', 0.10),
    ('tof_tube_timeout_s', 0.3),
    ('replan_on_obstacles', True),
    # Live stereo points lower than this (above the floor) never enter the
    # obstacle map: tilted-camera floor returns otherwise block detours.
    ('obstacle_min_height_m', 0.30),
    # 'planner' (Favour's EGO planner, default) or 'file' (fly route_file as
    # exported: Raaga's gated routes; no planning, no re-planning).
    ('route_source', 'planner'),
    # Wall correction, off until it has been gated. depthcloud's cloud_localize
    # measures how far cuVSLAM's odometry frame has slid against the walls and
    # publishes /team/localize/odom_correction while that is valid: a planar
    # transform D in odom (position x/y = dx/dy, orientation = dyaw about z),
    # with corrected = D * course_to_odom(p). On, ROUTE is flown in D's frame:
    # the carrot paces the odometry mapped through D^-1 (where the walls say
    # the drone is), every setpoint goes out as D * target after the net clamp
    # in that frame, and the held heading turns by dyaw. Odometry stays the
    # position source. D is the identity until the first message, keeps its
    # last value when messages stop (the drift doesn't go away), only changes
    # in ROUTE, and freezes while VIO is lost or the carrot is in the tube
    # zone. It moves toward each new value at most wall_correction_rate_*, so
    # a late first message or the end of a freeze never steps a setpoint
    # (cloud_localize itself rate-limits D to 0.08 m/s and 3 deg/s).
    ('wall_correction', False),
    ('wall_correction_topic', '/team/localize/odom_correction'),
    ('wall_correction_stale_s', 3.0),     # older than this is logged; D stays
    # A NEW correction is taken only this soon after its cloud was captured (source stamp),
    # so a delayed message never re-applies old data; cloud_localize itself allows 0.25 s.
    ('wall_correction_max_age_s', 0.5),
    ('wall_correction_rate_mps', 0.12),
    ('wall_correction_rate_dps', 4.5),
    ('wall_correction_max_m', 3.0),       # messages beyond these are ignored
    ('wall_correction_max_deg', 40.0),
    # Camera pad landing, off by default. The course (and the pad's place on
    # it) can change, so the pad is found by camera, not taken from the map:
    # pad_observer.py publishes every pad fix it sees (/team/pad/center, raw
    # odom) and pad_tracker.py clusters the whole flight's fixes (the pad:
    # pad_vision_confirm agreeing ones). At the route's end, PAD_FIND:
    #   search    not confirmed yet: turn on the spot in pad_vision_search_step_deg
    #             steps (the forward camera sees ~1-3 m ahead from 0.50 m); still
    #             not confirmed but glimpsed (fewer fixes, e.g. past a wall end):
    #             approach that sighting's look point instead, and land only if
    #             it is confirmed there
    #   approach  fly to a look point pad_vision_look_dist_m from the pad on
    #             whichever side the planner's map shows clear (nearest first),
    #             facing it; straight if clear, else the planner's path
    #   look      wait up to pad_vision_look_s there for fresh fixes
    #   over      walk over the pad at pad_vision_speed_mps, then LAND on it.
    # Nothing found after a full turn (the pad isn't 1-3 m around), or no clear
    # way to it: land on the route's end as before. Look height: 0.50 m, the
    # planner's own landing-stage floor; lower, live stereo floor points
    # (inflated) block it on the map (master sim).
    ('pad_vision', False),
    ('pad_vision_timeout_s', 0.5),
    ('pad_vision_confirm', 3),
    ('pad_vision_look_dist_m', 1.0),
    ('pad_vision_look_height_m', 0.50),
    ('pad_vision_look_s', 3.0),
    ('pad_vision_search_step_deg', 45.0),
    ('pad_vision_search_dwell_s', 1.0),
    ('pad_vision_speed_mps', 0.3),
)


class CourseMission(Node):

    def __init__(self, **kwargs):
        super().__init__('navless_nav', **kwargs)

        # dynamic_typing: `start_bearing_deg:=90` arrives as an int, and a
        # typed double parameter would kill the node at startup -- which on
        # the day means no takeover at all.
        any_type = ParameterDescriptor(dynamic_typing=True)
        for name, default in PARAMETERS:
            self.declare_parameter(name, default, any_type)

        def param(name):
            return float(self.get_parameter(name).value)

        self.takeoff_altitude = param('takeoff_altitude_m')
        self.takeoff_lead = max(0.0, param('takeoff_lead_m'))
        self.arrival_radius = param('arrival_radius_m')
        self.takeoff_timeout = param('takeoff_timeout_s')
        self.settle_radius = param('takeoff_settle_radius_m')
        self.settle_s = param('takeoff_settle_s')
        self.settle_since = None
        self.hold_heading = str(self.get_parameter('hold_heading').value
                                ).lower() in ('true', '1')
        self.heading_gain = param('heading_rate_gain')
        self.heading_rate_max = math.radians(param('heading_rate_max_dps'))
        self.heading_guard = math.radians(param('heading_guard_deg'))
        self.heading_guard_s = param('heading_guard_s')
        self.prearm_heading_tol = math.radians(param('prearm_heading_tol_deg'))
        mode = str(self.get_parameter('heading_mode').value).strip().lower()
        if mode not in ('travel', 'hold'):
            self.get_logger().error(f"heading_mode {mode!r} is not 'travel' or 'hold'; using 'travel'.")
            mode = 'travel'
        self.heading_mode = mode
        self.travel_look = max(0.05, param('travel_heading_lookahead_m'))
        self.travel_rate = math.radians(param('travel_heading_rate_dps'))
        self.tube_look = max(0.0, param('tube_heading_lookahead_m'))
        self.tube_align = math.radians(param('tube_align_deg'))
        self.tube_waiting = False
        self.want_yaw = None        # the travel/tube heading aimed for (route frame)
        self.tube_heading = False   # facing north for the tube right now
        self.heading_at = None      # last travel-heading update (for its rate)
        self.drift_on = str(self.get_parameter('drift_correction').value).lower() in ('true', '1')
        self.drift_ki = param('drift_ki')
        self.drift_deadband = param('drift_deadband_m')
        self.drift_max_error = param('drift_max_error_m')
        self.drift_max = param('drift_max_m')
        self.drift_min_height = param('drift_min_height_m')
        self.drift_in_route = str(self.get_parameter('drift_apply_in_route').value).lower() in ('true', '1')
        self.drift_body = (0.0, 0.0)    # learned correction, body frame (x fwd, y left), m
        self.drift_seed = (param('drift_seed_body_x_m'), param('drift_seed_body_y_m'))
        self.drift_seeded = False
        self.drift_at = None            # last learning step
        self.drift_learning = False
        self.last_target = None         # the last setpoint before the drift correction
        self.last_cmd_yaw = None        # the yaw in the last pose setpoint
        self.replans = 0
        self.takeoff_carrot = None
        self.hold_yaw = None        # the heading we hold (odom ENU yaw), or None
        self.px4_yaw_ref = None     # PX4 yaw minus odom yaw when it was latched
        self.heading_suspended = False
        self.heading_agree_since = None
        self.px4_yaw = None         # PX4's heading as an odom-comparable ENU yaw
        self.px4_yaw_at = None
        self.px4_heading_resets = None
        self.heading_warned_at = None
        self.last_pose = None       # the last pose setpoint, after the fence
        rate_hz = param('publish_rate_hz')
        self.rate_hz = rate_hz
        self.cruise_speed = param('cruise_speed_mps')
        self.max_lead = param('max_lead_m')
        self.max_sagitta = param('max_turn_sagitta_m')
        self.max_lat_accel = param('max_lat_accel_mps2')
        self.max_start_offset = param('max_start_offset_m')
        self.stall_timeout = param('stall_timeout_s')
        self.net_margin = param('net_margin_m')
        self.odom_timeout = param('odom_timeout_s')
        self.integrity = PoseIntegrity(
            max_age_s=param('odom_source_max_age_s'),
            future_tolerance_s=param('odom_future_tolerance_s'),
            max_speed_mps=param('odom_max_speed_mps'),
            max_yaw_rate_rps=param('odom_max_angular_rate_rps'),
            frame_id=str(self.get_parameter('odom_frame').value),
            child_frame_id=str(self.get_parameter('odom_body_frame').value))
        self.navigation_fault = None
        self.fault_descending = False   # latched during LAND/REFUSED: keep pushing down
        self.vio_lost_at = None         # first tick of a VIO loss while commanding
        self.takeover_ready = False     # every pre-arm check passed (logged on each change)
        self.odom_gap = param('odom_gap_check_s')
        self.odom_gap_tol = param('odom_gap_px4_tol_m')
        self.px4_ref = None             # odom - PX4 position, horizontal (update_px4_reference)
        self.error_since = None
        self.tracking_error_max = param('tracking_error_max_m')
        self.tracking_error_timeout = param('tracking_error_timeout_s')
        self.initial_alignment_settle = param('initial_alignment_settle_s')
        self.cloud_max_age = param('cloud_capture_max_age_s')
        self.cloud_tf = None
        self._frame_history = deque(maxlen=300)
        self._mapping_epoch = 0
        try:
            import tf2_ros
            self.cloud_tf = tf2_ros.Buffer()
            self._cloud_listener = tf2_ros.TransformListener(self.cloud_tf, self)
        except ImportError:
            pass  # no TF means no map insertion; never use a guessed camera pose
        self.land_at_end = str(self.get_parameter('land_at_end').value
                               ).lower() in ('true', '1')
        self.land_speed = param('land_speed_mps')
        self.land_xy_max = param('land_xy_max_mps')
        self.fly_home = str(self.get_parameter('fly_home').value
                            ).lower() in ('true', '1')
        # Odom z of the hop: odom starts start_height_m above the floor.
        self.home_z = param('home_altitude_m') - param('start_height_m')
        self.home_climb_tol = param('home_climb_tol_m')
        self.home_arrival_radius = param('home_arrival_radius_m')
        self.going_home = False
        self.prearm_tol = param('prearm_height_tol_m')
        self.prearm_settle = param('prearm_settle_s')
        self.match_height = str(self.get_parameter('match_px4_height').value
                                ).lower() in ('true', '1')
        self.z_samples = []         # PX4 z - odom z on the floor, last 2 s
        self.z_offset = 0.0         # added to every height sent (see send)
        self.px4_z = None           # PX4's last valid height (local_enu z)
        self.px4_z_at = None
        self.px4_pos = None         # PX4's position (local_enu), odom-comparable
        self.px4_at = None
        self.blind_carry = str(self.get_parameter('blind_carry').value
                               ).lower() in ('true', '1')
        self.blind_max_s = param('blind_carry_max_s')
        self.blind_margin = param('blind_zone_margin_m')
        self.blind_since = None     # start of the current VIO loss in ROUTE
        self.blind_carrying = False
        self.stereo_tube = str(self.get_parameter('stereo_tube_centering').value).lower() in ('true', '1')
        self.tube_guidance = TubeGuidance(param('stereo_tube_max_shift_m'), param('stereo_tube_rate_mps'),
                                          param('stereo_tube_hold_s'))
        self.tube_invalid_at = -math.inf
        self.tube_status_at = None
        self.tof_tube = str(self.get_parameter('tof_tube_centering').value).lower() in ('true', '1')
        if self.tof_tube and self.stereo_tube:
            self.get_logger().error(
                'stereo_tube_centering and tof_tube_centering are both on: '
                'steering on stereo only.')
            self.tof_tube = False
        self.tof_guidance = TubeGuidance(
            param('tof_tube_max_shift_m'), param('tof_tube_rate_mps'),
            param('tof_tube_timeout_s'), clamp=True)
        self.tof_offset = None      # (stamp, metres to the left) from /tube/offset
        self.tof_used_stamp = None
        self.wall_corr = str(self.get_parameter('wall_correction').value).lower() in ('true', '1')
        self.corr_stale = param('wall_correction_stale_s')
        self.correction_contract = CorrectionContract(param('wall_correction_max_age_s'))
        self.alignment_since = None
        self.alignment_candidate = None
        self.alignment_ready = False
        self.corr_rate_m = param('wall_correction_rate_mps')
        self.corr_rate_rad = math.radians(param('wall_correction_rate_dps'))
        self.corr_max_m = param('wall_correction_max_m')
        self.corr_max_rad = math.radians(param('wall_correction_max_deg'))
        self.corr = (0.0, 0.0, 0.0)     # applied D (dx, dy, dyaw): odom <- route frame
        self.corr_rx = None             # the last D received (dx, dy, dyaw), and when
        self.corr_rx_at = None
        self._pending_corrections = {}
        self.corr_at = None             # when the applied D was last stepped
        self.corr_stale_logged = False
        self.yaw = None             # odometry heading: the offset is along body y
        self.vo_at = None
        self.state_at = None
        self.heights_agree_since = None
        self.prearm_passed = False
        self.prearm_warned_at = None
        self.prearm_anchor_pos = None
        self.prearm_stability_tol = 0.05
        self.raw_position = None
        self.odom_zero_offset = (0.0, 0.0, 0.0)
        self.odom_zeroed = False
        self.start_bearing = param('start_bearing_deg')
        self.start_height = param('start_height_m')
        self.preferred_branch = (
            'gate_b' if 'gate_b' in str(self.get_parameter('route_file').value).lower()
            else 'window'
        )
        # route_source 'planner' (default): the active 3D occupancy grid + EGO
        # B-spline planner on the as-built course map, with live re-planning.
        # 'file': fly route_file's waypoints as exported (Raaga's gated
        # drone19 routes, gazebo_sim/course/route_centreline_*_asbuilt.json),
        # with no planning and no re-planning: the fallback.
        source = str(self.get_parameter('route_source').value).strip().lower()
        if source not in ('planner', 'file'):
            self.get_logger().error(f"route_source {source!r} is not 'planner' or 'file'; using 'planner'.")
            source = 'planner'
        self.static_asbuilt_route = source == 'file'
        self.replan_on_obstacles = str(self.get_parameter('replan_on_obstacles').value).lower() in ('true', '1')
        self.obstacle_min_height = param('obstacle_min_height_m')
        self.pad_vision = str(self.get_parameter('pad_vision').value).lower() in ('true', '1')
        self.pad_timeout = param('pad_vision_timeout_s')
        self.pad_tracker = PadTracker(confirm=int(param('pad_vision_confirm')))
        self.pad_look_dist = param('pad_vision_look_dist_m')
        self.pad_look_z = param('pad_vision_look_height_m') - param('start_height_m')  # odom z
        self.pad_look_s = param('pad_vision_look_s')
        self.pad_search_step = math.radians(param('pad_vision_search_step_deg'))
        self.pad_search_dwell = param('pad_vision_search_dwell_s')
        self.pad_speed = param('pad_vision_speed_mps')
        self.pad_step = self.pad_speed / rate_hz  # m per tick
        self.pad = {}               # PAD_FIND working state (see _state_pad_find)
        self.planner_cfg, self.planner_grid, self.planner_router = create_live_course_planner()
        self.live_grid_updated = False
        self.last_replan_at = 0.0
        self.cloud_rx_count = 0
        self.route_kind = 'DYNAMIC_3D_ASTAR_BSPLINE'
        self.initial_route_kind = self.route_kind
        self._planner_lock = threading.Lock()
        self._bg_worker_enabled = False
        self._bg_stop = threading.Event()
        self._bg_wake = threading.Event()
        self._bg_cloud_batch = []
        self._bg_replan_req = None
        self._bg_replan_result = None
        self._bg_replanning = False
        self._bg_thread = None
        self.map_viz = None
        self.frame = None          # (launch mark xy, start bearing) once known
        self.route = self.load_route(
            str(self.get_parameter('route_file').value),
            self.start_bearing, self.start_height)
        self.initial_route_odom = list(self.route or [])
        self.carrot = None
        self.last_tick = None
        self.progress_at = None

        self.setpoint_pub = self.create_publisher(
            PoseStamped, POSE_TOPIC, 10)
        self.twist_pub = self.create_publisher(
            TwistStamped, TWIST_TOPIC, 10)
        self.navigation_status_pub = self.create_publisher(String, '/team/navigation/status', 10)
        self.create_subscription(
            Odometry, str(self.get_parameter('odometry_topic').value),
            self.odometry_cb, 10)
        self.create_subscription(String, STATE_TOPIC, self.state_cb, 10)
        if self.stereo_tube:
            self.create_subscription(PoseStamped, '/tube/stereo/centerline', self.tube_center_cb, 2)
            self.create_subscription(String, '/tube/stereo/status', self.tube_status_cb, 2)
        if self.tof_tube:
            # tube_node publishes best effort; a reliable subscriber would
            # never match it.
            self.create_subscription(PointStamped, '/tube/offset', self.tof_offset_cb,
                                     qos_profile_sensor_data)
        if self.wall_corr:
            self.create_subscription(String, '/team/localize/status', self.localizer_status_cb, 10)
            self.create_subscription(
                PoseStamped, str(self.get_parameter('wall_correction_topic').value),
                self.odom_correction_cb, 10)
        if self.pad_vision:
            self.create_subscription(PointStamped, '/team/pad/center', self.pad_center_cb, 10)
        if VisualSlamStatus is not None:
            self.create_subscription(
                VisualSlamStatus, '/visual_slam/status', self.status_cb, 10)
        if PointCloud2 is not None and not hasattr(PointCloud2, '__getattr__'):
            try:
                from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
                self._cloud_cb_group = MutuallyExclusiveCallbackGroup()
                self.create_subscription(
                    PointCloud2, '/team/stereo/points', self.pointcloud_cb, 10,
                    callback_group=self._cloud_cb_group)
            except (ImportError, TypeError):
                self.create_subscription(
                    PointCloud2, '/team/stereo/points', self.pointcloud_cb, 10)
        if VehicleLocalPosition is not None:
            # PX4's uXRCE-DDS publishers: best effort, transient local (the
            # QoS navless_flight_tests reads this topic with on 19).
            try:
                from rclpy.qos import (DurabilityPolicy, HistoryPolicy,
                                       QoSProfile, ReliabilityPolicy)
                px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                     history=HistoryPolicy.KEEP_LAST, depth=1)
            except ImportError:     # a ROS stand-in without QoS classes
                px4_qos = None
                self.get_logger().warn('no rclpy QoS classes: not reading '
                                       f'{PX4_LOCAL_TOPIC}; heading guard on the camera only.')
            if px4_qos is not None:
                self.create_subscription(VehicleLocalPosition, PX4_LOCAL_TOPIC,
                                         self.px4_local_cb, px4_qos)

        self.state = 'WAIT_FOR_OFFBOARD'
        self.state_since = self.now_s()
        self.target = None
        self.position = None
        self.odom_at = None
        self.vo_state = None
        self.vio_was_ok = True
        self.offboard_active = False
        self.armed = False
        self.landed = False
        self.log_once = set()

        self.flog = None
        if str(self.get_parameter('flight_log').value).lower() in ('true', '1'):
            params = {name: self.get_parameter(name).value for name, _ in PARAMETERS}
            self.flog = FlightLog(resolve_dir(self.get_parameter('flight_log_dir').value), params,
                                  warn=lambda m: self.get_logger().warn(m))
            if self.flog.on:
                self.get_logger().info(f'flight log: {self.flog.path}')

        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            f'navless_nav ready: take off to {self.takeoff_altitude:.2f} m, '
            f'then fly {len(self.route or [])} waypoints at '
            f'{self.cruise_speed:.2f} m/s. Waiting for the pilot to arm, '
            'then switch to Position...')

    def load_route(self, name, bearing_deg, height_m):
        """Plan dynamically via the active 3D A* + EGO B-spline planner against
        the as-built course prior, or (route_source 'file') read route_file."""
        if self.static_asbuilt_route:
            try:
                waypoints, launch, kind = load_route(resolve_route_path(name))
            except (OSError, ValueError, KeyError) as exc:
                self.get_logger().error(
                    f'route file not loaded ({exc}); will take off and hover only.')
                return None
            kind = f'file {name} ({kind})'
            self.route_kind = kind
            self.initial_route_kind = kind
            odom = [course_to_odom(p, launch, bearing_deg, height_m) for p in waypoints]
            self.frame = (launch, bearing_deg)
            self.get_logger().info(
                f'route {kind}: {len(odom)} waypoints, launch mark {launch}, start bearing '
                f'{bearing_deg:.1f} deg; no planning or re-planning (route_source file).')
            return odom
        try:
            low = str(name).lower()
            if not any(k in low for k in ('window', 'gate_b', 'dynamic', 'route.json')):
                resolve_route_path(name)
            branch = 'gate_b' if 'gate_b' in low else 'window'
            waypoints, launch, kind = plan_dynamic_course_route(
                branch=branch, router=self.planner_router)
        except (OSError, ValueError, KeyError, RuntimeError) as exc:
            self.get_logger().error(
                f'route not loaded ({exc}); will take off and hover only.')
            return None
        self.route_kind = kind
        self.initial_route_kind = kind
        odom = [course_to_odom(p, launch, bearing_deg, height_m)
                for p in waypoints]
        self.frame = (launch, bearing_deg)
        self.get_logger().info(
            f'route {kind}: {len(odom)} waypoints, launch mark '
            f'{launch}, start bearing {bearing_deg:.1f} deg, ends at odom '
            f'({odom[-1][0]:.2f}, {odom[-1][1]:.2f}, {odom[-1][2]:.2f})')
        return odom

    def start_background_planner(self, start_http=False, viz_port=8082):
        """Start permanent dedicated worker threads for 3D pointcloud mapping and 3D A* + B-spline replanning."""
        if not self._bg_worker_enabled:
            self._bg_stop.clear()
            self._bg_worker_enabled = True
            self._cloud_wake = threading.Event()
            self._cloud_thread = threading.Thread(
                target=self._bg_cloud_loop, name='navless_cloud_mapper', daemon=True)
            self._bg_thread = threading.Thread(
                target=self._bg_replan_loop, name='navless_ego_replanner', daemon=True)
            self._cloud_thread.start()
            self._bg_thread.start()
        if start_http and self.map_viz is None:
            try:
                from navless_nav.live_map_viz import LiveMapVisualizer
                self.map_viz = LiveMapVisualizer(self, port=viz_port)
                self.map_viz.start()
            except ImportError:
                pass

    def stop_background_planner(self):
        """Stop the dedicated mapping and replanning worker threads cleanly."""
        if self.map_viz is not None:
            self.map_viz.stop()
            self.map_viz = None
        if not self._bg_worker_enabled:
            return
        self._bg_worker_enabled = False
        self._bg_stop.set()
        if getattr(self, '_cloud_wake', None) is not None:
            self._cloud_wake.set()
        self._bg_wake.set()
        for t in (getattr(self, '_cloud_thread', None), self._bg_thread):
            if t is not None and t.is_alive():
                t.join(timeout=1.0)

    def _bg_cloud_loop(self):
        """Permanent dedicated thread: decode & insert stereo pointclouds into the 3D occupancy grid."""
        while not self._bg_stop.is_set():
            self._cloud_wake.wait(timeout=0.1)
            self._cloud_wake.clear()
            if self._bg_stop.is_set():
                break
            with self._planner_lock:
                if not self._bg_cloud_batch:
                    continue
                # Process the freshest stereo cloud frame
                item = self._bg_cloud_batch[-1]
                self._bg_cloud_batch.clear()
            pts_world, origin, epoch = item
            if epoch != self._mapping_epoch or self.navigation_fault:
                continue
            if pts_world.size > 0:
                with self._planner_lock:
                    n_added = self.planner_grid.insert_point_cloud(
                        pts_world, sensor_origin_m=origin, min_live_hits=2)
                    if n_added > 0:
                        self.live_grid_updated = True
                if n_added > 0:
                    self._bg_wake.set()

    def _bg_replan_loop(self):
        """Permanent dedicated thread: check route clearance and run 3D A* + EGO B-spline replanning."""
        while not self._bg_stop.is_set():
            self._bg_wake.wait(timeout=0.1)
            self._bg_wake.clear()
            if self._bg_stop.is_set():
                break
            with self._planner_lock:
                req = self._bg_replan_req
                self._bg_replan_req = None
            if req is None:
                continue
            now, cur_course, rem_enu, launch_xy, bearing_deg, epoch = req
            self._bg_replanning = True
            try:
                with self._planner_lock:
                    blocked = any(
                        self.planner_grid.is_discrepancy_occupied(
                            c_pt, clearance_m=self.planner_cfg.obstacle_inflation_m)
                        for c_pt in rem_enu
                    )
                    if not blocked:
                        continue
                    wps_enu, launch, kind = plan_dynamic_course_route(
                        branch=self.preferred_branch,
                        start_enu=cur_course,
                        router=self.planner_router,
                        remaining_enu=rem_enu,
                    )
                new_odom = [course_to_odom(p, launch, bearing_deg, self.start_height)
                            for p in wps_enu]
                with self._planner_lock:
                    self._bg_replan_result = (now, new_odom, kind, epoch)
            except (ValueError, RuntimeError) as exc:
                self.get_logger().warn(f'Online replan skipped ({exc}).')
            finally:
                self._bg_replanning = False

    def pointcloud_cb(self, msg):
        """Admit only calibrated, capture-time geometry from the current navigation epoch."""
        self.cloud_rx_count += 1
        if (not self.replan_on_obstacles or self.static_asbuilt_route or self.position is None
                or self.frame is None or self.navigation_fault or not self.vio_ok()):
            return
        epoch = self._mapping_epoch
        try:
            pts_world, origin = self.capture_cloud(msg)
        except Exception as exc:    # tf2 raises its own types; any failure skips the cloud
            self.info_once('cloud_tf_missing', 'Live map skipped: capture-time calibrated TF required: '
                           + str(exc))
            return
        if self._bg_worker_enabled:
            with self._planner_lock:
                self._bg_cloud_batch.append((pts_world, origin, epoch))
            self._cloud_wake.set()
            return
        if pts_world.size > 0 and epoch == self._mapping_epoch:
            with self._planner_lock:
                n_added = self.planner_grid.insert_point_cloud(
                    pts_world, sensor_origin_m=origin, min_live_hits=2)
                if n_added > 0:
                    self.live_grid_updated = True

    def _apply_replanned_route(self, now, new_odom, kind):
        """Swap in a newly planned route and align the carrot to the drone's current progress."""
        self.route = new_odom
        self.route_kind = kind
        if self.carrot is not None:
            effective_sagitta = self.max_sagitta if self.max_sagitta > 0.0 else 0.04
            self.carrot = Carrot(
                self.route, self.cruise_speed, self.max_lead,
                self.max_lat_accel, effective_sagitta)
            if self.position is not None:
                self.carrot.catch_up(self.corr_to_route(self.position))
        self.last_replan_at = now
        self.replans += 1
        self.get_logger().info(
            f'Online 3D A* + B-spline replan ({kind}): {len(new_odom)} waypoints around obstacle.')

    def maybe_replan_for_discrepancies(self, now):
        """Replan remaining route via 3D A* + B-spline if live stereo depth reveals a map discrepancy."""
        if not self.replan_on_obstacles or self.static_asbuilt_route or self.going_home or self.frame is None or not self.route:
            return
        if self._bg_worker_enabled:
            with self._planner_lock:
                res = self._bg_replan_result
                self._bg_replan_result = None
            if res is not None:
                r_now, new_odom, kind, epoch = res
                if epoch == self._mapping_epoch and not self.navigation_fault:
                    self._apply_replanned_route(r_now, new_odom, kind)
            if not self.live_grid_updated or (now - self.last_replan_at) < 0.35 or self._bg_replanning:
                return
            self.live_grid_updated = False
            start_idx = self.carrot.index if self.carrot is not None else 0
            rem_odom = self.route[start_idx:]
            rem_enu = [odom_to_course(q, self.frame[0], self.frame[1], self.start_height) for q in rem_odom]
            if self.carrot is not None and self.position is not None:
                cur_course = odom_to_course(
                    self.corr_to_route(self.position), self.frame[0], self.frame[1], self.start_height)
            else:
                cur_course = rem_enu[0]
            with self._planner_lock:
                self._bg_replan_req = (now, cur_course, rem_enu, self.frame[0], self.frame[1], self._mapping_epoch)
            self._bg_wake.set()
            return

        if not self.live_grid_updated or (now - self.last_replan_at) < 0.5:
            return
        self.live_grid_updated = False
        start_idx = self.carrot.index if self.carrot is not None else 0
        rem_odom = self.route[start_idx:]
        rem_enu = [odom_to_course(q, self.frame[0], self.frame[1], self.start_height) for q in rem_odom]
        blocked = False
        with self._planner_lock:
            for c_pt in rem_enu:
                if self.planner_grid.is_discrepancy_occupied(c_pt, clearance_m=self.planner_cfg.obstacle_inflation_m):
                    blocked = True
                    break
        if not blocked:
            return
        # Re-plan from where the walls say the drone is: the route lives in
        # that frame (the wall correction's; plain odometry when it is off).
        if self.carrot is not None and self.position is not None:
            cur_course = odom_to_course(
                self.corr_to_route(self.position), self.frame[0], self.frame[1], self.start_height)
        else:
            cur_course = rem_enu[0]
        try:
            with self._planner_lock:
                wps_enu, launch, kind = plan_dynamic_course_route(
                    branch=self.preferred_branch,
                    start_enu=cur_course,
                    router=self.planner_router,
                    remaining_enu=rem_enu,
                )
            new_odom = [course_to_odom(p, launch, self.frame[1], self.start_height)
                        for p in wps_enu]
            self._apply_replanned_route(now, new_odom, kind)
        except (ValueError, RuntimeError) as exc:
            self.get_logger().warn(f'Online replan skipped ({exc}).')

    def live_obstacle_speed_scale(self, pos):
        """Slow the carrot when a newly discovered live stereo obstacle is close ahead or replanning is in flight."""
        if (not self.replan_on_obstacles or self.static_asbuilt_route or self.carrot is None
                or self.frame is None or not getattr(self.planner_grid, 'has_live_discrepancies', False)):
            return 1.0
        if self._bg_replanning:
            return 0.30
        probe = self.route_point_ahead(self.carrot, 0.45)
        c_probe = odom_to_course(probe, self.frame[0], self.frame[1], self.start_height)
        with self._planner_lock:
            near_obs = self.planner_grid.is_discrepancy_occupied(
                c_probe, clearance_m=self.planner_cfg.obstacle_inflation_m + 0.12)
        return 0.35 if near_obs else 1.0

    def latch_navigation_fault(self, reason):
        """Stop commanding for good; only an operator restart (disarmed, realigned) clears it.

        What happens next is flight_interface's: a pose setpoint persists, a twist expires
        after its setpoint_timeout_s (1 s) and it then holds the last pose. That hold is in
        PX4's frame, which may be the broken one, so FAULT is not a safe state by itself: the
        safety pilot must take over. Never a new HOLD or route from here. One exception: a
        fault during a twist descent (LAND, REFUSED) keeps pushing straight down, with no
        horizontal or yaw command, until PX4 disarms; stopping it would make flight_interface
        climb back to the last pose (0.85 m up over the pad).
        """
        if self.navigation_fault is not None:
            return
        self.navigation_fault = str(reason)
        self.fault_descending = self.state in ('LAND', 'REFUSED')
        # No _planner_lock: a long background A* must not delay the descent twist. The epoch
        # bump alone discards anything the workers still hand back.
        self._mapping_epoch += 1
        self._bg_cloud_batch.clear()
        self._bg_replan_req = self._bg_replan_result = None
        self.goto('FAULT')
        self.get_logger().error(
            'NAVIGATION FAULT LATCHED: ' + self.navigation_fault + '. '
            + ('Descending straight down until PX4 disarms. ' if self.fault_descending else
               'No more setpoints: flight_interface holds the last pose; PILOT TAKE OVER. ')
            + 'No automatic resume; restart the node after disarming.')
        self._state_fault()

    def _state_fault(self):
        if not (self.armed and self.offboard_active):
            self.fault_descending = False   # disarmed or the pilot has it: never again
        if not self.fault_descending:
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = -float(self.land_speed)
        msg.twist.angular.z = 0.0
        self.twist_pub.publish(msg)

    def publish_navigation_status(self):
        """/team/navigation/status, every tick: why the node is (not) commanding."""
        vio = self.vio_ok()
        message = String()
        message.data = json.dumps({
            'state': self.state, 'fault_latched': self.navigation_fault is not None,
            'reason': self.navigation_fault, 'fault_descending': self.fault_descending,
            'vio_ok': vio, 'odometry': self.integrity.fault or self.integrity.reason,
            'alignment_ready': self.alignment_ready if self.wall_corr else None,
            'correction': self.correction_contract.reason if self.wall_corr else None,
            'armed': self.armed, 'offboard_active': self.offboard_active})
        self.navigation_status_pub.publish(message)

    def check_tracking_error(self):
        """ROUTE: lateral distance from the route polyline (not the carrot, which leads on
        purpose). TAKEOFF is left to the settle check: faulting there changes nothing PX4
        does (flight_interface holds the same target) and the route never starts unsettled."""
        error = 0.0
        if self.state == 'ROUTE' and len(self.route or []) >= 2:
            p = self.corr_to_route(self.position)
            error = math.inf
            for a, b in zip(self.route, self.route[1:]):
                dx, dy = b[0] - a[0], b[1] - a[1]
                length = dx * dx + dy * dy
                t = min(1.0, max(0.0, ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length)) if length else 0.0
                error = min(error, math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy))
        if error > self.tracking_error_max:
            if self.error_since is None:
                self.error_since = self.now_s()
            elif self.now_s() - self.error_since >= self.tracking_error_timeout:
                self.latch_navigation_fault(f'sustained lateral tracking error {error:.2f} m')
        else:
            self.error_since = None

    def initial_alignment(self):
        if not self.wall_corr or self.going_home:
            return True
        if (self.corr_rx is None or self.corr_rx_at is None
                or self.now_s() - self.corr_rx_at > self.corr_stale
                or not self.correction_contract.current_valid):
            self.alignment_candidate = self.alignment_since = None   # settle again from scratch
            self.alignment_ready = False
            return False
        d = self.corr_rx
        old = self.alignment_candidate
        if (old is None or math.hypot(d[0]-old[0], d[1]-old[1]) > 0.02
                or abs(wrap_pi(d[2]-old[2])) > math.radians(0.5)):
            self.alignment_candidate, self.alignment_since = d, self.now_s()
            self.alignment_ready = False
            return False
        if self.now_s() - self.alignment_since < self.initial_alignment_settle:
            return False
        # No flight command is displaced here: WAIT streams the current raw pose.
        self.corr = self.correction_target()
        self.alignment_ready = True
        self.remember_map_frame()
        return True

    def remember_map_frame(self):
        if not hasattr(self, 'corr'):
            return
        sample = (self.now_s(), tuple(self.odom_zero_offset), tuple(self.corr), self._mapping_epoch)
        self._frame_history.append(sample)

    def capture_cloud(self, msg):
        """(course points, camera origin in the course) from the raw odom <- camera TF at
        the cloud's own stamp; never the receipt-time pose. The origin comes from the same
        transform as the points (the planner ray-casts free space from it)."""
        acquired = stamp_s(msg.header.stamp)
        if self.cloud_tf is None or self.frame is None or self.navigation_fault:
            raise CloudTransformError('capture-time camera TF unavailable')
        from rclpy.time import Time
        from rclpy.duration import Duration
        tf = self.cloud_tf.lookup_transform(
            self.integrity.frame_id, msg.header.frame_id,
            Time.from_msg(msg.header.stamp), timeout=Duration(seconds=0.0))
        return self.project_cloud(msg, tf, acquired)

    def course_from_raw_odom(self, acquired):
        """4x4 course <- raw odom at `acquired`, from the alignment recorded then (this epoch)."""
        history = tuple(self._frame_history)    # one C-level copy: tick appends from another thread
        sample = next((s for s in reversed(history)
                       if s[0] <= acquired and s[3] == self._mapping_epoch), None)
        if sample is None:
            raise CloudTransformError('no capture-time course alignment in this epoch')
        _, zero, correction, _ = sample
        a = correction[2]
        D = rigid_transform((correction[0], correction[1], 0), (0, 0, math.sin(a/2), math.cos(a/2)))
        raw_to_mission = np.eye(4)
        raw_to_mission[:3, 3] = -np.asarray(zero)
        angle = math.pi/2 - math.radians(self.frame[1])
        nominal_course = rigid_transform((*self.frame[0], self.start_height),
                                        (0, 0, math.sin(angle/2), math.cos(angle/2)))
        return nominal_course @ np.linalg.inv(D) @ raw_to_mission

    def project_cloud(self, msg, tf, acquired):
        """capture_cloud without the TF query (tf: a geometry_msgs TransformStamped)."""
        stamp = stamp_s(tf.header.stamp)
        if tf.child_frame_id != msg.header.frame_id or tf.header.frame_id != self.integrity.frame_id:
            raise CloudTransformError('TF frames do not match odom <- cloud frame')
        t, q = tf.transform.translation, tf.transform.rotation
        raw_camera = rigid_transform((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        course_camera = self.course_from_raw_odom(acquired) @ raw_camera
        points = transform_pointcloud(
            msg, course_camera, transform_stamp_s=stamp,
            transform_source_frame=tf.child_frame_id, now_s=self.now_s(),
            max_capture_age_s=self.cloud_max_age, min_depth_m=0.3, max_depth_m=2.8,
            crop_min=(-3.1, -3.1, self.obstacle_min_height), crop_max=(3.1, 3.1, 1.70))
        return points, tuple(float(v) for v in course_camera[:3, 3])

    # -- inputs ----------------------------------------------------------
    def odometry_cb(self, msg):
        """Only a sample PoseIntegrity accepts moves the pose; anything else (no header,
        stale, duplicate) is dropped, and a discontinuity latches FAULT. After a gap in
        commanded flight the speed bound has grown with the gap, so the first sample back must
        also agree with PX4's own (dead-reckoned) estimate: a relocalised or reset pose never
        becomes the pose the route resumes from (carrot.catch_up)."""
        previous = self.integrity.last_stamp_s
        try:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            raw = (float(p.x), float(p.y), float(p.z))
            source_stamp = stamp_s(msg.header.stamp)
            accepted = self.integrity.observe(
                stamp_s=source_stamp, now_s=self.now_s(), frame_id=msg.header.frame_id,
                child_frame_id=msg.child_frame_id, position=raw,
                quaternion=(q.x, q.y, q.z, q.w),
                reset_counter=getattr(msg, 'reset_counter', None))
        except (AttributeError, TypeError, ValueError):
            accepted = False
            if self.armed:
                self.latch_navigation_fault('malformed odometry source/pose')
        if not accepted:
            if self.integrity.fault:
                self.latch_navigation_fault(self.integrity.fault)
            return
        position = tuple(r - z for r, z in zip(raw, self.odom_zero_offset))
        commanding = self.armed and self.offboard_active and self.state not in IDLE_STATES
        if commanding and previous is not None and source_stamp - previous > self.odom_gap:
            why = self.gap_disagreement(position)
            if why:
                self.latch_navigation_fault(why)
                return
        self.raw_position = raw
        self.position = position
        self.odom_at = self.now_s()
        self.update_px4_reference()
        q = getattr(msg.pose.pose, 'orientation', None)
        if q is not None:
            self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def update_px4_reference(self):
        """Smoothed odom minus PX4 position (horizontal) while both are fresh: the offset the
        post-gap check expects (their origins and update rates differ; 5 Hz PX4 lags a bit)."""
        if self.px4_pos is None or self.px4_at is None or self.now_s() - self.px4_at > 0.25:
            return
        d = (self.position[0] - self.px4_pos[0], self.position[1] - self.px4_pos[1])
        r = self.px4_ref
        self.px4_ref = d if r is None else (r[0] + 0.1 * (d[0] - r[0]), r[1] + 0.1 * (d[1] - r[1]))

    def gap_disagreement(self, position):
        """Why the first odometry sample after a gap cannot be trusted, or None."""
        if (self.px4_pos is None or self.px4_at is None or self.px4_ref is None
                or self.now_s() - self.px4_at > self.odom_timeout):
            return 'odometry back after a gap with no fresh PX4 estimate to check it against'
        off = math.hypot(position[0] - self.px4_pos[0] - self.px4_ref[0],
                         position[1] - self.px4_pos[1] - self.px4_ref[1])
        if off > self.odom_gap_tol:
            return f"odometry back after a gap {off:.2f} m from PX4's estimate (relocalised?)"
        return None

    def status_cb(self, msg):
        self.vo_state = int(msg.vo_state)
        self.vo_at = self.now_s()
        if self.vo_state != 1:
            self.tube_guidance.clear()
            self.tube_invalid_at = self.now_s()

    def tube_status_cb(self, msg):
        self.tube_status_at = self.now_s()
        try:
            valid = json.loads(msg.data).get('valid') is True
        except (ValueError, AttributeError):
            valid = False
        if not valid:
            self.tube_guidance.clear()
            self.tube_invalid_at = self.now_s()

    def tube_center_cb(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        values = (q.x, q.y, q.z, q.w)
        if (msg.header.frame_id != 'odom' or stamp <= self.tube_invalid_at
                or not all(math.isfinite(v) for v in values)
                or abs(sum(v*v for v in values)-1) > 0.01
                or abs(q.x) > 1e-6 or abs(q.y) > 1e-6):
            self.tube_guidance.clear()
            return
        self.tube_guidance.update(stamp, (p.x, p.y, p.z),
                                  2*math.atan2(q.z, q.w), self.now_s())

    def px4_local_cb(self, msg):
        """PX4's heading (NED, clockwise from its x) as an ENU yaw, the same
        convention as odom's (flight_interface: yaw_ned = pi/2 - yaw_enu)."""
        heading = float(msg.heading)
        if math.isfinite(heading) and getattr(msg, 'heading_good_for_control', False) is True:
            self.px4_yaw = wrap_pi(math.pi / 2.0 - heading)
            self.px4_yaw_at = self.now_s()
        else:
            self.px4_yaw = self.px4_yaw_at = None
        resets = int(msg.heading_reset_counter)
        if self.px4_heading_resets is not None and resets != self.px4_heading_resets:
            why = (f'PX4 reset its heading (reset counter {self.px4_heading_resets} -> '
                   f'{resets}; a vision gap?)')
            if self.armed and self.offboard_active and self.state not in IDLE_STATES:
                # A yaw reset turns PX4's local frame under every odom setpoint
                # flight_interface converts: the same discontinuity as an odometry jump.
                self.latch_navigation_fault(why + ' while commanding flight')
            elif self.hold_yaw is not None and not self.heading_suspended:
                self.suspend_heading(why, republish=True)
        self.px4_heading_resets = resets

    def tof_offset_cb(self, msg):
        """tube_node's /tube/offset: point.y = how far the tube centreline is
        to the drone's left (body frame), point.z = measured width."""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        offset = float(msg.point.y)
        self.tof_offset = ((stamp, offset) if msg.header.frame_id == 'base_link'
                           and math.isfinite(offset) else None)

    def localizer_status_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                self.correction_contract.status(data)
                if self.correction_contract.fault:
                    self.latch_navigation_fault(self.correction_contract.fault)
                for pending in list(self._pending_corrections.values()):
                    self.odom_correction_cb(pending)
        except (ValueError, TypeError):
            pass

    def odom_correction_cb(self, msg):
        """cloud_localize's drift correction D (see wall_correction): a
        PoseStamped in odom, position x/y = dx/dy, orientation = dyaw about z.
        It only arrives while cloud_localize considers it valid."""
        try:
            p, q = msg.pose.position, msg.pose.orientation
            acquired = stamp_s(msg.header.stamp)
            acquired_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            rigid_transform((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
            dx, dy = float(p.x), float(p.y)
            dyaw = math.atan2(2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)),
                              1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2))
        except (AttributeError, TypeError, ValueError, CloudTransformError):
            return
        self._pending_corrections[acquired_ns] = msg
        if len(self._pending_corrections) > 8:
            self._pending_corrections.pop(next(iter(self._pending_corrections)))
        if not self.correction_contract.accept(acquired_ns, msg.header.frame_id, self.now_s()):
            if self.correction_contract.reason == 'localization epoch changed':
                if self.armed or self.state not in IDLE_STATES or self.going_home:
                    self.latch_navigation_fault(self.correction_contract.reason)
                else:
                    # On the ground before any arming: register again from scratch.
                    self.get_logger().warn('wall correction: localizer epoch changed before '
                                           'arming; realigning.')
                    self.correction_contract.epoch = None
                    self.corr_rx = self.corr_rx_at = None
                    self.alignment_candidate = self.alignment_since = None
                    self.alignment_ready = False
            return
        self._pending_corrections.pop(acquired_ns, None)
        if msg.header.frame_id != 'odom' or not all(map(math.isfinite, (dx, dy, dyaw))):
            self.info_once('corr_bad', 'wall correction: ignoring a message that is not '
                           'in odom or not finite.')
            return
        if math.hypot(dx, dy) > self.corr_max_m or abs(dyaw) > self.corr_max_rad:
            self.info_once('corr_big', f'wall correction: ignoring D ({dx:+.2f}, {dy:+.2f} m, '
                           f'{math.degrees(dyaw):+.1f} deg), beyond wall_correction_max_*.')
            return
        if self.corr_rx is None:
            self.get_logger().info(
                f'wall correction: first D ({dx:+.3f}, {dy:+.3f} m, '
                f'{math.degrees(dyaw):+.2f} deg); it is applied in ROUTE.')
        self.corr_rx = (dx, dy, dyaw)
        self.corr_rx_at = acquired
        self.corr_stale_logged = False

    # -- wall correction (identity unless wall_correction is on) -----------
    def corr_to_route(self, q):
        """Odom point -> the route's frame (D^-1: where the walls say it is)."""
        dx, dy, a = self.corr
        if dx == 0.0 and dy == 0.0 and a == 0.0:
            return tuple(q)
        c, s = math.cos(a), math.sin(a)
        x, y = q[0] - dx, q[1] - dy
        return (c * x + s * y, -s * x + c * y, q[2])

    def corr_from_route(self, q):
        """Route-frame point -> odom (D): the frame setpoints are flown in."""
        dx, dy, a = self.corr
        if dx == 0.0 and dy == 0.0 and a == 0.0:
            return tuple(q)
        c, s = math.cos(a), math.sin(a)
        return (c * q[0] - s * q[1] + dx, s * q[0] + c * q[1] + dy, q[2])

    def step_correction(self, frozen):
        """Move the applied D toward the last D received, by at most the rate
        limits. Held while `frozen`; the last D stays when messages stop."""
        now = self.now_s()
        dt = 0.0 if self.corr_at is None else min(max(now - self.corr_at, 0.0), 0.2)
        self.corr_at = now
        if not self.wall_corr or self.corr_rx is None or frozen or not self.correction_contract.current_valid:
            return
        if now - self.corr_rx_at > self.corr_stale and not self.corr_stale_logged:
            self.corr_stale_logged = True
            self.get_logger().warn(
                f'wall correction: no message for {now - self.corr_rx_at:.1f} s; keeping '
                f'the last D ({self.corr_rx[0]:+.3f}, {self.corr_rx[1]:+.3f} m, '
                f'{math.degrees(self.corr_rx[2]):+.2f} deg).')
        if now - self.corr_rx_at > self.corr_stale:
            return  # hold the last APPLIED transform; do not keep chasing stale input
        self.corr = self.ease_correction(self.corr, self.correction_target(), dt)
        self.remember_map_frame()

    def correction_target(self):
        """The applied correction that the last D asks for, in this node's
        frame. cloud_localize's D maps its nominal odometry (raw odom, launch
        mark at the origin) to raw odom. This node flies odom minus
        odom_zero_offset (the pose at arming counts as the launch mark), so
        the walls' answer for a route point r is D*r - z0: rotate by dyaw,
        then shift by (dx, dy) - z0. With no D this node keeps flying r."""
        dx, dy, a = self.corr_rx
        z0 = self.odom_zero_offset
        return (dx - z0[0], dy - z0[1], a)

    def ease_correction(self, cur, tgt, dt):
        """Move the applied correction toward tgt with ONE fraction for its
        translation and rotation together, sized so that where it puts the
        drone moves at most wall_correction_rate_mps and its heading at most
        wall_correction_rate_dps. The rotation is about this frame's origin
        (the launch mark), so limiting dx/dy and dyaw separately would let a
        heading change alone move a point 5 m out by r * dyaw, with its
        compensating shift lagging on its own clock."""
        (cx, cy, ca), (tx, ty, ta) = cur, tgt
        step = (tx - cx, ty - cy, wrap_pi(ta - ca))
        if dt <= 0.0 or step == (0.0, 0.0, 0.0):
            return cur
        p = self.corr_to_route(self.position) if self.position is not None else (0.0, 0.0, 0.0)

        def place(e):
            c, s = math.cos(e[2]), math.sin(e[2])
            return (c * p[0] - s * p[1] + e[0], s * p[0] + c * p[1] + e[1])

        a0 = place(cur)
        a1 = place((cx + step[0], cy + step[1], ca + step[2]))
        moved = math.hypot(a1[0] - a0[0], a1[1] - a0[1])
        f = 1.0
        if moved > self.corr_rate_m * dt:
            f = self.corr_rate_m * dt / moved
        if abs(step[2]) * f > self.corr_rate_rad * dt:
            f = self.corr_rate_rad * dt / abs(step[2])
        return (cx + f * step[0], cy + f * step[1], wrap_pi(ca + f * step[2]))

    def held_yaw(self):
        """The held heading in odom: hold_yaw (route frame) turned by D's dyaw."""
        dyaw = self.corr[2]
        return self.hold_yaw if dyaw == 0.0 else wrap_pi(self.hold_yaw + dyaw)
    def pad_center_cb(self, msg):
        """pad_observer's /team/pad/center: one pad fix in raw odom, anywhere."""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = float(msg.point.x) - self.odom_zero_offset[0]
        y = float(msg.point.y) - self.odom_zero_offset[1]
        if (msg.header.frame_id != 'odom' or not (math.isfinite(x) and math.isfinite(y))
                or not 0.0 <= self.now_s() - stamp <= self.pad_timeout or self.going_home):
            return
        had = self.pad_tracker.best()
        self.pad_tracker.add(self.now_s(), x, y)
        best = self.pad_tracker.best()
        if had is None and best is not None:
            self.get_logger().info(f'pad confirmed at odom ({best[0]:.2f}, {best[1]:.2f}) '
                                   f'by {best[2]} camera fixes.')

    def vio_ok(self):
        now = self.now_s()
        return (self.navigation_fault is None and self.integrity.fresh(now)
                and self.vo_state == 1 and self.vo_at is not None
                and 0 <= now - self.vo_at <= self.integrity.max_age_s)

    def vio_recent(self):
        """vio_ok(), or a gap of at most odom_timeout_s with no explicit loss report: for the
        drift offset, so a status at the 0.25 s limit never steps the setpoint for a tick."""
        if self.vio_ok():
            return True
        return (self.navigation_fault is None and self.vo_state != VO_LOST
                and self.vio_lost_at is not None and self.now_s() - self.vio_lost_at <= self.odom_timeout)

    def state_cb(self, msg):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        self.state_at = self.now_s()
        self.offboard_active = bool(state.get('offboard_active', False))
        self.armed = bool(state.get('armed', False))
        self.landed = bool(state.get('landed', False))
        local = state.get('local_enu')
        try:
            xyz = tuple(float(v) for v in local[:3])
        except (TypeError, ValueError):
            xyz = ()
        ok = len(xyz) == 3 and all(math.isfinite(v) for v in xyz)
        self.px4_pos = (
            (xyz[0] - self.odom_zero_offset[0],
             xyz[1] - self.odom_zero_offset[1],
             xyz[2] - self.odom_zero_offset[2])
            if ok else None
        )
        self.px4_at = self.now_s()
        if ok:
            # Keep the last valid height: one message without a position
            # must not refuse a takeover (heights_agree checks its age).
            self.px4_z, self.px4_z_at = xyz[2], self.px4_at
        if state.get('clamped'):
            self.info_once(
                'clamped:' + ','.join(state['clamped']),
                f'flight_interface is CLAMPING our setpoints '
                f'({", ".join(state["clamped"])}); envelope '
                f'{state.get("envelope")}')

    # -- outputs ---------------------------------------------------------
    def send(self, x, y, z, fence=True, yaw=None):
        """One ENU pose setpoint in odom, with the held heading (heading_cmd);
        a zero quaternion (yaw NaN: PX4 does not steer the heading) when
        there is none to hold. `yaw` (ENU, odom) commands that heading instead:
        PAD_FIND turning on purpose to look at the pad (no heading guard).
        With drift_correction, the learned hold correction is added first
        (last_target keeps the true target for the settle check and the log)."""
        self.last_target = (x, y, z)
        if self.drift_applies():
            wx, wy = self.drift_world()
            x, y = x + wx, y + wy
        if fence and self.frame is not None:
            # The net is fixed to the walls: clamp in the route's frame
            # (odom through the wall correction; the same frame when it is off).
            r, moved = clamp_to_net(
                self.corr_to_route((x, y, z)), self.frame[0], self.frame[1], self.net_margin)
            if moved:
                x, y, z = self.corr_from_route(r)
                self.info_once(
                    'fenced', f'setpoint pulled inside the net (margin '
                    f'{self.net_margin:.2f} m); route or start bearing wrong?')
        # PX4 holds its own z at the setpoint: send heights in PX4's frame.
        z = z + self.z_offset
        self.last_pose = (x, y, z)
        self.publish_pose(x, y, z, self.heading_cmd() if yaw is None else yaw)

    def publish_pose(self, x, y, z, yaw):
        if self.navigation_fault:
            return
        self.last_cmd_yaw = yaw
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(x + self.odom_zero_offset[0])
        msg.pose.position.y = float(y + self.odom_zero_offset[1])
        msg.pose.position.z = float(z + self.odom_zero_offset[2])
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        if yaw is None:
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 0.0
        else:
            msg.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.setpoint_pub.publish(msg)

    def send_twist(self, vx, vy, vz):
        """ENU world velocity in odom; expires after 1 s unless refreshed.
        Yaw rate steers the nose back to the held heading, if any."""
        if self.navigation_fault:
            return
        rate = 0.0
        yaw = self.heading_cmd()
        if yaw is not None and self.yaw is not None:
            rate = max(-self.heading_rate_max, min(
                self.heading_rate_max,
                self.heading_gain * wrap_pi(yaw - self.yaw)))
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.z = float(rate)
        self.twist_pub.publish(msg)

    # -- heading ---------------------------------------------------------
    def latch_heading(self):
        """Hold the heading the drone has now (called as the takeoff starts)."""
        self.heading_suspended = False
        self.heading_agree_since = None
        if not self.hold_heading or self.yaw is None:
            self.hold_yaw = None
            return
        # hold_yaw is in the route's frame (odom until a wall correction has
        # turned it): the nose is held where it is now, whatever dyaw is.
        self.hold_yaw = self.yaw if self.corr[2] == 0.0 else wrap_pi(self.yaw - self.corr[2])
        px4 = self.px4_heading()
        self.px4_yaw_ref = wrap_pi(px4 - self.yaw) if px4 is not None else None
        self.get_logger().info(
            f'holding heading {math.degrees(self.yaw):.1f} deg (odom)'
            + (f'; PX4 agrees to {math.degrees(self.px4_yaw_ref):+.1f} deg'
               if self.px4_yaw_ref is not None else '; no PX4 heading to cross-check'))

    def release_heading(self):
        self.want_yaw = None
        self.tube_heading = False
        self.heading_at = None
        self.hold_yaw = None
        self.px4_yaw_ref = None
        self.heading_suspended = False
        self.heading_agree_since = None

    def px4_heading(self):
        """PX4's latest heading (odom-comparable ENU yaw) if fresh, else None."""
        if self.px4_yaw is None or self.now_s() - self.px4_yaw_at > PX4_TIMEOUT_S:
            return None
        return self.px4_yaw

    def px4_heading_off(self):
        """How far PX4's heading is from the camera's, beyond what it was when
        the heading was latched (rad, >= 0), or None if not known."""
        px4 = self.px4_heading()
        if px4 is None or self.yaw is None or self.px4_yaw_ref is None:
            return None
        return abs(wrap_pi(px4 - self.yaw - self.px4_yaw_ref))

    def suspend_heading(self, why, republish=False):
        self.heading_suspended = True
        self.heading_agree_since = None
        self.get_logger().warn(
            f'heading guard: {why}. Commanding no heading (PX4 will not turn '
            'the nose) until PX4 and the camera agree again.')
        if (republish and self.state in ('TAKEOFF', 'ROUTE', 'HOLD')
                and self.last_pose is not None):
            self.publish_pose(*self.last_pose, None)     # now, not next tick

    def heading_cmd(self):
        """The heading to command (odom ENU yaw), or None for no heading."""
        if self.hold_yaw is None or self.state in IDLE_STATES:
            return None
        off_px4 = self.px4_heading_off()
        if not self.heading_suspended:
            pushed = (abs(wrap_pi(self.yaw - self.held_yaw()))
                      if self.yaw is not None else 0.0)
            if off_px4 is not None and off_px4 > self.heading_guard:
                self.suspend_heading(
                    f'PX4 heading {math.degrees(off_px4):.0f} deg off the camera')
            elif pushed > self.heading_guard:
                self.suspend_heading(
                    f'nose {math.degrees(pushed):.0f} deg off the held heading')
            else:
                return self.held_yaw()
        # Suspended: resume once PX4 and the camera have agreed (within a
        # third of the guard) for heading_guard_s -- or, with no PX4 heading
        # to compare, once heading_guard_s has passed -- holding the nose
        # where it is now, never turning it back.
        now = self.now_s()
        agree = off_px4 is None or off_px4 <= self.heading_guard / 3.0
        if not agree:
            self.heading_agree_since = None
            return None
        if self.heading_agree_since is None:
            self.heading_agree_since = now
        if now - self.heading_agree_since < self.heading_guard_s or self.yaw is None:
            return None
        self.heading_suspended = False
        self.hold_yaw = self.yaw if self.corr[2] == 0.0 else wrap_pi(self.yaw - self.corr[2])
        self.get_logger().info(
            f'heading guard: PX4 and the camera agree again; holding heading '
            f'{math.degrees(self.yaw):.1f} deg (odom), where the nose is.')
        return self.held_yaw()

    # -- travel heading (heading_mode 'travel') ---------------------------
    def route_point_ahead(self, carrot, distance):
        """The route point `distance` metres along the path past the carrot
        (the carrot itself is not moved)."""
        path, cum = carrot.path, carrot.cum
        i = carrot.index
        s = cum[i] + math.dist(path[i], carrot.point) + max(0.0, distance)
        while i < len(path) - 1 and cum[i + 1] < s:
            i += 1
        if i >= len(path) - 1:
            return path[-1]
        seg = cum[i + 1] - cum[i]
        f = 0.0 if seg <= 0.0 else (s - cum[i]) / seg
        return tuple(a + (b - a) * f for a, b in zip(path[i], path[i + 1]))

    def travel_heading(self, carrot):
        """(route-frame yaw to aim for, facing north for the tube?) or
        (None, False) when there is nothing to aim along (route end)."""
        if carrot is None or self.frame is None:
            return None, False
        if (self.tof_tube or self.static_asbuilt_route) and (
                self.in_blind_zone(carrot.point)
                or self.in_blind_zone(self.route_point_ahead(carrot, self.tube_look))):
            # Course north in odom: odom +x points along the start bearing
            # (degrees clockwise from north), so north is +bearing from it.
            return wrap_pi(math.radians(self.frame[1])), True
        a = carrot.point
        b = self.route_point_ahead(carrot, self.travel_look)
        if math.hypot(b[0] - a[0], b[1] - a[1]) < 0.05:
            return None, False
        return math.atan2(b[1] - a[1], b[0] - a[0]), False

    def update_travel_heading(self, carrot):
        """heading_mode 'travel': turn hold_yaw (route frame) toward the
        travel / tube heading at most travel_heading_rate_dps. Leaves it alone
        while the heading guard has suspended the heading (it resumes from
        wherever the nose is, as before)."""
        now = self.now_s()
        dt = 0.0 if self.heading_at is None else min(max(now - self.heading_at, 0.0), 0.2)
        self.heading_at = now
        if self.heading_mode != 'travel' or self.hold_yaw is None or self.heading_suspended:
            return
        want, tube = self.travel_heading(carrot)
        if tube != self.tube_heading:
            self.tube_heading = tube
            self.get_logger().info('tube ahead: turning to face north for the tube.' if tube
                                   else 'out of the tube: facing the direction of travel again.')
        if want is None:
            return
        self.want_yaw = want
        step = max(-self.travel_rate * dt, min(self.travel_rate * dt, wrap_pi(want - self.hold_yaw)))
        self.hold_yaw = wrap_pi(self.hold_yaw + step)

    def wait_to_face_tube(self):
        """Hold the carrot at the tube mouth until the nose faces north
        (travel heading): true while the tube heading is wanted, the nose is
        more than tube_align_deg off it, and the carrot is still outside the
        footprint with the footprint 0.3 m ahead."""
        wait = False
        if (self.heading_mode == 'travel' and self.tube_heading and self.yaw is not None
                and self.hold_yaw is not None and not self.heading_suspended
                and not self.in_tube_footprint(self.carrot.point)
                and self.in_tube_footprint(self.route_point_ahead(self.carrot, 0.3))):
            north = wrap_pi(math.radians(self.frame[1]))
            nose = wrap_pi(self.yaw - self.corr[2])
            wait = abs(wrap_pi(nose - north)) > self.tube_align
        if wait != self.tube_waiting:
            self.tube_waiting = wait
            self.get_logger().info('at the tube mouth: hovering while the nose turns north.' if wait
                                   else 'facing north: into the tube.')
        return wait

    # -- hold drift correction (drift_correction) -------------------------
    def drift_world(self):
        """The learned correction in odom (it turns with the nose)."""
        bx, by = self.drift_body
        if bx == 0.0 and by == 0.0:
            return 0.0, 0.0
        yaw = self.yaw if self.yaw is not None else 0.0
        c, s = math.cos(yaw), math.sin(yaw)
        return c * bx - s * by, s * bx + c * by

    def seed_drift(self):
        """First takeoff of this node: start the drift correction at the known
        liftoff slide's compensation (drift_seed_body_*), so the climb is pushed
        against it from the first instant. Later takeoffs (the fly_home re-arm)
        keep what was learned."""
        if self.drift_seeded or not self.drift_on or self.drift_seed == (0.0, 0.0):
            return
        self.drift_seeded = True
        bx, by = self.drift_seed
        n = math.hypot(bx, by)
        if n > self.drift_max:
            bx, by = bx * self.drift_max / n, by * self.drift_max / n
        self.drift_body = (bx, by)
        self.get_logger().info(
            f'takeoff slide pre-compensation: drift correction starts at '
            f'{bx:+.2f} m forward, {by:+.2f} m left (body); learning refines it after the climb.')

    def drift_applies(self):
        """Only with live odometry: when VIO is lost the node holds at the
        last pose (zero error against a frozen estimate) or paces PX4's own
        estimate through the tube, and an offset added there would have PX4
        push with no feedback (Gazebo 'freeze': into the south net)."""
        if not self.drift_on or not self.vio_recent() or self.blind_carrying:
            return False
        return (self.state in ('TAKEOFF', 'HOLD')
                or (self.state == 'ROUTE' and self.drift_in_route))

    def learn_drift(self, target):
        """One integral step toward putting the drone itself on `target`, a
        target standing still (TAKEOFF after the climb, HOLD)."""
        now = self.now_s()
        dt = 0.0 if self.drift_at is None else min(max(now - self.drift_at, 0.0), 0.2)
        self.drift_at = now
        self.drift_learning = False
        if not self.drift_on or target is None or not self.vio_ok() or dt <= 0.0:
            return
        if self.position[2] < self.drift_min_height or abs(self.position[2] - target[2]) > 0.15:
            return                       # still climbing: the drone lags by design
        ex, ey = target[0] - self.position[0], target[1] - self.position[1]
        e = math.hypot(ex, ey)
        if e > self.drift_max_error:
            self.info_once(f'drift_big:{self.state}:{int(self.state_since)}',
                           f'drift correction paused: {e:.2f} m from the target is too far '
                           'to be a steady push.')
            return
        if e <= self.drift_deadband:
            return
        self.drift_learning = True
        wx, wy = self.drift_world()
        k = self.drift_ki * dt
        wx, wy = wx + k * ex, wy + k * ey
        n = math.hypot(wx, wy)
        if n > self.drift_max:
            wx, wy = wx * self.drift_max / n, wy * self.drift_max / n
            self.info_once('drift_cap', f'drift correction at its {self.drift_max:.2f} m cap: '
                           'something keeps pushing the drone (level calibration, centre of '
                           'gravity, props).')
        yaw = self.yaw if self.yaw is not None else 0.0
        c, s = math.cos(yaw), math.sin(yaw)
        self.drift_body = (c * wx + s * wy, -s * wx + c * wy)

    # -- flight recorder ---------------------------------------------------
    def get_logger(self):
        """The ROS logger, with every line copied into the flight log."""
        return LoggingLogger(super().get_logger(), self._record_event)

    def _record_event(self, level, msg):
        flog = getattr(self, 'flog', None)
        if flog is not None and flog.on:
            flog.row({'wall_time': round(time.time(), 3), 't': round(self.now_s(), 3),
                      'kind': level, 'state': getattr(self, 'state', ''), 'event': str(msg)})

    def log_tick(self):
        flog = self.flog
        if flog is None or not flog.on:
            return
        now = self.now_s()
        deg = (lambda a: None if a is None else math.degrees(a))
        p, raw = self.position, self.raw_position
        tgt, sent, carrot = self.target, self.last_pose, self.carrot
        ref = self.last_target or tgt
        wx, wy = self.drift_world()
        v = {'wall_time': round(time.time(), 3), 't': round(now, 3), 'kind': 'tick',
             'state': self.state, 'yaw_deg': deg(self.yaw),
             'odom_age': None if self.odom_at is None else now - self.odom_at,
             'vio_ok': self.vio_ok(), 'vo_state': self.vo_state,
             'route_n': len(self.route or []), 'heading_mode': self.heading_mode,
             'cmd_yaw_deg': deg(self.last_cmd_yaw), 'hold_yaw_deg': deg(self.hold_yaw),
             'want_yaw_deg': deg(self.want_yaw), 'heading_suspended': self.heading_suspended,
             'tube_heading': self.tube_heading, 'px4_yaw_deg': deg(self.px4_yaw),
             'px4_heading_resets': self.px4_heading_resets,
             'drift_body_x': self.drift_body[0], 'drift_body_y': self.drift_body[1],
             'drift_x': wx, 'drift_y': wy, 'drift_learning': self.drift_learning,
             'corr_dx': self.corr[0], 'corr_dy': self.corr[1], 'corr_dyaw_deg': deg(self.corr[2]),
             'corr_rx_age': None if self.corr_rx_at is None else now - self.corr_rx_at,
             'armed': self.armed, 'offboard': self.offboard_active, 'landed': self.landed,
             'z_offset': self.z_offset, 'prearm_passed': self.prearm_passed,
             'blind_carrying': self.blind_carrying, 'replans': self.replans,
             'tof_shift': getattr(self.tof_guidance, 'shift', None),
             'stereo_shift': getattr(self.tube_guidance, 'shift', None)}
        for name, q in (('', p), ('raw_', raw), ('target_', tgt), ('sent_', sent),
                        ('px4_', self.px4_pos), ('zero_', self.odom_zero_offset)):
            if q is not None:
                v.update({f'{name}x': q[0], f'{name}y': q[1], f'{name}z': q[2]})
        if carrot is not None:
            v.update({'carrot_x': carrot.point[0], 'carrot_y': carrot.point[1],
                      'carrot_z': carrot.point[2], 'carrot_i': carrot.index})
            v['in_tube_zone'] = self.in_blind_zone(carrot.point)
        if p is not None and ref is not None:
            v['err_xy'] = math.hypot(ref[0] - p[0], ref[1] - p[1])
            v['err_z'] = ref[2] - p[2]
        flog.row(v)

    # -- helpers ---------------------------------------------------------
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def info_once(self, key, message):
        if key not in self.log_once:
            self.log_once.add(key)
            self.get_logger().info(message)

    def arrived(self, target):
        return math.dist(self.position, target) <= self.arrival_radius

    def goto(self, state, target=None):
        self.get_logger().info(f'{self.state} -> {state}')
        self.state = state
        self.state_since = self.now_s()
        self.target = target
        self.settle_since = None
        self.takeoff_carrot = None
        self.drift_at = None
        self.drift_learning = False
        if state in IDLE_STATES:
            self.release_heading()      # every takeoff latches its own
        if state == 'WAIT_FOR_OFFBOARD':
            # Every arming (the fly_home re-arm on the pad too) re-checks,
            # and re-measures PX4's floor offset.
            self.heights_agree_since = None
            self.prearm_passed = False
            self.takeover_ready = False
            self.prearm_anchor_pos = None
            self.z_samples = []
            self.z_offset = 0.0

    def zero_launch_origin(self):
        """Anchor odom_zero_offset so where the drone is when armed counts as the launch mark (0, 0, 0)."""
        if self.going_home or self.raw_position is None:
            return
        old = self.odom_zero_offset
        self.position = (0.0, 0.0, 0.0)
        self.odom_zeroed = True
        if tuple(old) == tuple(self.raw_position):
            return      # every waiting tick re-zeroes; only a move changes the map frame
        self.odom_zero_offset = self.raw_position
        if self.alignment_ready:
            # The same registration in the new zero: correction_target() is D - zero.
            self.corr = (self.corr[0] + old[0] - self.raw_position[0],
                         self.corr[1] + old[1] - self.raw_position[1], self.corr[2])
        self._mapping_epoch += 1
        self._frame_history.clear()
        self.remember_map_frame()

    def headings_agree(self):
        """Require a fresh PX4 heading; agreement alone does not prove course registration."""
        if self.prearm_heading_tol <= 0.0:
            return False
        px4 = self.px4_heading()
        if px4 is None or self.yaw is None:
            if VehicleLocalPosition is not None:
                self.info_once('prearm_heading_none', 'pre-arm heading check: '
                               f'no PX4 heading on {PX4_LOCAL_TOPIC}; not checked.')
            return False
        off = abs(wrap_pi(px4 - self.yaw))
        if off <= self.prearm_heading_tol:
            return True
        now = self.now_s()
        if self.heading_warned_at is None or now - self.heading_warned_at >= 5.0:
            self.heading_warned_at = now
            self.get_logger().warn(
                f'pre-arm heading check: PX4 heading {math.degrees(px4):.1f} deg '
                f'vs camera {math.degrees(self.yaw):.1f} deg: '
                f'{math.degrees(off):.1f} deg apart (limit '
                f'{math.degrees(self.prearm_heading_tol):.0f}). PX4 is not '
                'using the camera\'s heading (magnetometer?). Not streaming '
                'setpoints, so flight_interface will not take over. Keep the '
                'drone still; if it stays off, power-cycle the Pixhawk.')
        return False

    def heights_agree(self):
        """Pre-arm check: PX4's height matches odom z, VIO is stationary, and launch offset is zeroed."""
        if self.prearm_tol <= 0.0:
            self.info_once('prearm_off', 'pre-arm height check is OFF '
                           '(prearm_height_tol_m 0).')
            if not self.going_home and not self.armed:
                self.zero_launch_origin()
            return True
        now = self.now_s()
        if not self.vio_ok():
            self.heights_agree_since = None
            self.prearm_passed = False
            self.prearm_anchor_pos = None
            return False
        raw_pos = self.raw_position if self.raw_position is not None else self.position
        if self.prearm_anchor_pos is None:
            self.prearm_anchor_pos = raw_pos
        elif math.dist(raw_pos, self.prearm_anchor_pos) > self.prearm_stability_tol:
            self.prearm_anchor_pos = raw_pos
            self.heights_agree_since = None
            self.prearm_passed = False
            return False
        fresh = (self.px4_z is not None
                 and now - self.px4_z_at <= PREARM_PX4_TIMEOUT_S)
        dz = abs(self.px4_z - raw_pos[2]) if fresh else None
        # PX4's floor offset against RAW odom z (publish_pose adds
        # odom_zero_offset back, so PX4 holds its z at raw odom + this): the
        # median of the last 2 s (20 ticks), so one noisy sample does not set
        # it. It stops updating at takeoff (this runs only while waiting),
        # which latches it.
        if fresh:
            self.z_samples = (self.z_samples + [self.px4_z - raw_pos[2]])[-20:]
            if self.match_height:
                self.z_offset = sorted(self.z_samples)[len(self.z_samples) // 2]
        else:
            self.z_samples = []
        # Hysteresis: pass at prearm_height_tol_m held for prearm_settle_s;
        # once passed, only a clear disagreement (PREARM_HYSTERESIS_M more)
        # or no PX4 height for PREARM_PX4_TIMEOUT_S revokes it. Otherwise
        # noise at the threshold or an arming transient can drop the stream
        # just as the pilot switches, and the takeover becomes a REFUSED
        # descent (Gazebo, simulated pilot arming at a 0.15 m boot offset).
        limit = self.prearm_tol + (PREARM_HYSTERESIS_M if self.prearm_passed
                                   else 0.0)
        if dz is None or dz > limit:
            self.heights_agree_since = None
            self.prearm_passed = False
            if (self.prearm_warned_at is None
                    or now - self.prearm_warned_at >= 5.0):
                self.prearm_warned_at = now
                seen = ('no valid local position from PX4' if dz is None else
                        f'PX4 height {self.px4_z:.2f} m vs odom '
                        f'{raw_pos[2]:.2f} m: {dz:.2f} m apart')
                self.get_logger().warn(
                    f'pre-arm height check: {seen} (limit '
                    f'{limit:.2f} m). Not streaming setpoints, so '
                    'flight_interface will not take over. Keep the drone '
                    'still on the floor; if it stays off, power-cycle the '
                    'Pixhawk.')
            return False
        if self.heights_agree_since is None:
            self.heights_agree_since = now
        if now - self.heights_agree_since < self.prearm_settle:
            return False
        if not self.going_home and not self.armed:
            self.zero_launch_origin()
        if not self.prearm_passed:
            self.prearm_passed = True
            self.prearm_warned_at = None
            self.get_logger().info(
                f'pre-arm height & zero check passed: PX4 and odom agree to '
                f'{dz:.2f} m. Streaming; ready for the arming.')
        return True

    # -- state machine ---------------------------------------------------
    def tick(self):
        self.remember_map_frame()
        self.publish_navigation_status()
        if self.integrity.fault:
            self.latch_navigation_fault(self.integrity.fault)
        if self.navigation_fault:
            self._state_fault()
            self.log_tick()
            return
        if self.state not in IDLE_STATES and self.armed and self.offboard_active:
            now = self.now_s()
            if self.state_at is None or not 0 <= now - self.state_at <= 1.0:
                self.latch_navigation_fault('flight_interface state older than 1 s while commanding')
                return
            # The twist descents (LAND, REFUSED) need no position. ROUTE rides out a loss
            # (frozen carrot / blind carry) for at most blind_carry_max_s + odom_timeout_s;
            # elsewhere a loss longer than odom_timeout_s ends the mission: there is no
            # position to command from. (A latched odom_gate looks like this too.)
            # A stale wall correction is not a fault: D keeps its last applied value.
            allowed = self.odom_timeout + (self.blind_max_s if self.state == 'ROUTE' else 0.0)
            if self.state in ('LAND', 'REFUSED') or self.vio_ok():
                self.vio_lost_at = None
            elif self.vio_lost_at is None:
                self.vio_lost_at = now
            elif now - self.vio_lost_at > allowed:
                self.latch_navigation_fault(f'VIO tracking lost for over {allowed:.2f} s '
                                            f'in {self.state}')
                return
            self.check_tracking_error()
            if self.navigation_fault:
                return
        else:
            self.vio_lost_at = None
        if self.position is None:
            self.info_once(
                'no_odom', 'no /visual_slam/tracking/odometry yet; nothing '
                'is commanded until cuVSLAM is tracking.')
            return
        if self.stereo_tube and self.state not in ('WAIT_FOR_OFFBOARD', 'DONE'):
            fresh = self.state_at is not None and 0 <= self.now_s()-self.state_at <= 0.5
            if not fresh or (not self.offboard_active and not self.landed):
                self.tube_guidance.clear()
                if not self.offboard_active and self.state != 'PAUSED':
                    self.goto('PAUSED')
                return  # let the existing flight interface own loss-of-stream behavior
        if self.state not in IDLE_STATES and not self.offboard_active:
            if self.state == 'LAND' and self.landed and (
                    self.fly_home and not self.going_home):
                # Home leg: straight up, then over the wall to the launch
                # mark, which is the odom origin. The route start check
                # (max_start_offset_m) keeps it from flying from anywhere
                # but this touchdown.
                # In the route's frame (the wall correction's, if any), whose
                # origin is the launch mark where the walls put it.
                x, y, _ = self.corr_to_route(self.position)
                self.route = [(x, y, self.home_z), (0.0, 0.0, self.home_z)]
                self.going_home = True
                self.get_logger().info(
                    'touched down on the pad and disarmed. fly_home: the '
                    'next arming flies back to the launch mark at '
                    f'{self.home_z:.2f} m (odom). Leave it disarmed to stop.')
                self.goto('WAIT_FOR_OFFBOARD')
                return
            if self.state == 'LAND' and self.landed:
                self.get_logger().info(
                    'touched down and disarmed: mission done. This node '
                    'will not take the vehicle again.')
                self.goto('DONE')
                return
            self.get_logger().warn(
                f'Offboard lost in {self.state}: the pilot has control. '
                'Paused until the vehicle is disarmed.')
            self.goto('PAUSED')
        getattr(self, '_state_' + self.state.lower())()
        self.log_tick()

    def _state_wait_for_offboard(self):
        checks = {'height': self.heights_agree(), 'PX4 heading': self.headings_agree(),
                  'VIO': self.vio_ok(), 'course alignment': self.initial_alignment()}
        ready = all(checks.values())
        if ready != self.takeover_ready:
            # One line per change, no commas (tools/flight_day/fly_course.sh cuts the CSV log).
            self.takeover_ready = ready
            if ready:
                self.get_logger().info('ready for takeover: height / PX4 heading / VIO / '
                                       'course alignment all pass; streaming.')
            else:
                self.get_logger().warn('not ready for takeover: ' + ' / '.join(
                    k for k, v in checks.items() if not v) + ' failing; not streaming.')
        if not self.offboard_active:
            if self.position is not None:
                self.maybe_replan_for_discrepancies(self.now_s())
            if ready:
                self.send(*self.position, fence=False)
            return
        if not ready:
            self.get_logger().error(
                'Offboard came while not ready for takeover ('
                + ', '.join(k for k, v in checks.items() if not v)
                + ' failing): not taking off. Descending in place until PX4 disarms.')
            self.goto('REFUSED', self.position)
            self._state_refused()       # no tick without a setpoint
            return
        if not self.going_home:
            self.zero_launch_origin()
        self.goto('TAKEOFF', self.takeoff_target())
        self.latch_heading()
        self.seed_drift()
        if self.match_height:
            self.get_logger().info(
                f'PX4 reads {self.z_offset * 100:+.1f} cm from odom on the floor '
                f'(median of {len(self.z_samples)} samples): added to every '
                'height sent, so the route\'s heights are flown as planned.')
        self._state_takeoff()       # climb on this tick, not the next one

    def takeoff_target(self):
        """Straight up to the route's first height (no height step when the
        route opens; Gazebo: 7 cm once start_height_m is set), leaning
        takeoff_lead_m into the route's first leg. Barrier 1 is behind the
        launch mark and the first corridor ahead of it; in Gazebo the liftoff
        drifts ~15 cm backwards (thrust-model mismatch), to 3 cm from the
        barrier, and the lean takes that margin back."""
        x, y, _ = self.position
        if not self.route:
            return (x, y, self.takeoff_altitude)
        r0 = self.route[0]
        p = Carrot(self.route, 1.0, math.inf).advance(self.lead_m())
        dx, dy = p[0] - r0[0], p[1] - r0[1]
        c, s = math.cos(self.corr[2]), math.sin(self.corr[2])
        return (x + c * dx - s * dy, y + s * dx + c * dy, r0[2])

    def lead_m(self):
        """takeoff_lead_m, but 0 on the fly_home take-off: the pad has room
        all round (over 0.7 m to anything in Gazebo's model), and a lean
        there only starts the hop over Barrier 1 closer to the wall, with
        less run-up to finish the climb."""
        return 0.0 if self.going_home else self.takeoff_lead

    def _state_takeoff(self):
        self.learn_drift(self.target)
        if (self.heading_mode == 'travel' and self.route and self.position[2] >= self.drift_min_height
                and abs(self.position[2] - self.target[2]) <= 0.15):
            # climb done: turn to the first leg (or north, if it starts at the tube) while settling
            if self.takeoff_carrot is None:
                self.takeoff_carrot = Carrot(self.route, 1.0, math.inf)
                self.takeoff_carrot.advance(self.lead_m())
            self.update_travel_heading(self.takeoff_carrot)
        self.send(*self.target)
        there = self.arrived(self.target) and self.climbed()
        if there and self.settled():
            self.get_logger().info(
                f'takeoff settled: within {self.settle_radius * 100:.0f} cm of '
                f'the target for {self.settle_s:.1f} s, '
                f'{self.now_s() - self.state_since:.1f} s after the climb began.')
            self.start_route()
        elif self.now_s() - self.state_since > self.takeoff_timeout:
            self.get_logger().error(
                f'takeoff did not settle in {self.takeoff_timeout:.0f} s '
                f'({math.dist(self.position, self.target):.2f} m off); '
                f'holding here. Check `clamped` in {STATE_TOPIC} and VIO.')
            self.goto('HOLD', self.position)

    def settled(self):
        """Within takeoff_settle_radius_m of the setpoint actually sent (the
        target after the net fence; horizontally, by odom) without a break
        for takeoff_settle_s."""
        if self.settle_s <= 0.0:
            return True
        now = self.now_s()
        # the true target (before the drift correction, after the fence would
        # be the same unless fenced): the correction must not fake a settle
        aim = self.last_target if self.last_target is not None else self.target
        if math.dist(self.position[:2], aim[:2]) > self.settle_radius:
            self.settle_since = None
            return False
        if self.settle_since is None:
            self.settle_since = now
        return now - self.settle_since >= self.settle_s

    def climbed(self):
        """On the fly_home hop, the climb must be (nearly) complete before
        the leg over Barrier 1 starts; elsewhere arrival_radius_m is enough.
        Complete by odom, or by PX4's own height: PX4 holds where its
        estimate says the setpoint is, which can sit a few cm off odom (7 cm
        short at 2.2 m in SITL), so waiting longer would never get there."""
        if not self.going_home:
            return True
        need = self.target[2] - self.home_climb_tol
        return (self.position[2] >= need
                or (self.px4_z is not None
                    and self.now_s() - self.px4_z_at <= PREARM_PX4_TIMEOUT_S
                    and (self.px4_z - self.z_offset - self.odom_zero_offset[2]) >= need))

    def start_route(self):
        if not self.route:
            self.goto('HOLD', self.target)
            return
        effective_sagitta = self.max_sagitta if self.max_sagitta > 0.0 else 0.04
        carrot = Carrot(self.route, self.cruise_speed, self.max_lead,
                        self.max_lat_accel, effective_sagitta)
        carrot.advance(self.lead_m())       # never pull back behind the lean
        # Measured from where the route starts after the lean, so the lean
        # does not use up max_start_offset_m.
        # Never further off than ROUTE's own lateral-error FAULT allows.
        offset = math.dist(self.corr_to_route(self.position)[:2], carrot.point[:2])
        limit = min(self.max_start_offset, self.tracking_error_max)
        if offset > limit:
            self.get_logger().error(
                f'{offset:.2f} m from the route start (limit '
                f'{limit:.2f} m): not flying it from here. '
                'Holding. Power up on the launch mark.')
            self.goto('HOLD', self.target)
            return
        self.carrot = carrot
        self.blind_since = None     # a fly_home leg starts with a clean slate
        self.blind_carrying = False
        self.last_tick = self.now_s()
        self.progress_at = (self.last_tick, 0, self.carrot.point)
        self.goto('ROUTE')

    def _state_route(self):
        now = self.now_s()
        dt = min(now - self.last_tick, 0.2)
        self.last_tick = now
        vio_ok = self.vio_ok()
        # The carrot lives in the route's frame: odometry through the wall
        # correction (wall_correction; the identity when it is off). D holds
        # still while blind and while the carrot is in the tube zone.
        self.step_correction(not vio_ok or self.in_blind_zone(self.carrot.point))
        pos = self.corr_to_route(self.position)
        if vio_ok != self.vio_was_ok:
            self.vio_was_ok = vio_ok
            if vio_ok:
                # If the drone got ahead of the carrot while blind, move the
                # carrot up to it rather than flying back (in the tube, back
                # into the dark).
                moved = self.carrot.catch_up(pos)
                self.get_logger().info(
                    'VIO back: resuming the route'
                    + (f' ({moved:.2f} m further on, where the drone is).'
                       if moved > 0.0 else '.'))
            else:
                self.get_logger().warn(
                    f'VIO lost (vo_state {self.vo_state}, odometry '
                    f'{now - (self.odom_at or now):.2f} s old).')
        if vio_ok:
            self.blind_since = None
            self.set_carrying(False, 'VIO back')
            self.maybe_replan_for_discrepancies(now)
            self.update_travel_heading(self.carrot)
            if self.wait_to_face_tube():
                point = self.carrot.point       # hover at the mouth while the nose turns north
            else:
                dt_eff = dt * self.live_obstacle_speed_scale(pos)
                point = self.carrot.step(pos, dt_eff)
            ref = self.position     # where the drone is, in the flown (odom) frame
        else:
            # With no trustworthy position, do not move the target: PX4
            # falls back on its own estimate and holds the last point. In
            # the dark tube (blind_carry) keep pacing that estimate instead,
            # so the drone does not stop where tracking cannot come back.
            if self.blind_since is None:
                self.blind_since = now
            px4 = self.blind_carry_position(now)
            ref = px4               # None unless carrying on PX4's estimate
            if px4 is not None:
                # at the route's own speed: a faster carry (1.0 m/s, 1.2 m
                # lead) drifted 49 cm and touched the tube in Gazebo
                point = self.carrot.step(self.corr_to_route(px4), dt)
            elif self.odom_arriving():
                # cuVSLAM reports lost but keeps publishing (a frozen pose,
                # or worse), and the stack's px4_odom_bridge forwards every
                # message, so PX4 fuses it. Against that estimate a carrot
                # ahead makes PX4 push on with no feedback (Gazebo 'freeze':
                # through the tube into the south net). Hold at the latest
                # pose instead: zero error, whatever the stream does.
                self.info_once(f'odom-during-loss:{self.blind_since}',
                               'VIO lost but odometry still arriving (frozen '
                               'or unreliable): holding at its latest pose.')
                point = pos
            else:
                point = self.carrot.point
        point = self.corr_from_route(point)     # the target, in odom
        if self.stereo_tube:
            inside = self.in_tube(self.position, point)
            healthy = (vio_ok and self.vo_state == 1 and self.vo_at is not None
                       and 0 <= now-self.vo_at <= 0.25 and self.armed and self.offboard_active
                       and self.tube_status_at is not None and 0 <= now-self.tube_status_at <= 0.30)
            if healthy and not inside:
                # Observe the tube on approach, without steering outside it.
                self.tube_guidance.shift = 0.0
                self.tube_guidance.last_time = None
            else:
                point = self.tube_guidance.apply(
                    point, self.position, math.radians(self.frame[1]) if self.frame else 0,
                    now, enabled=inside and healthy, max_lead=self.max_lead)
        if self.tof_tube:
            point = self.tof_tube_point(point, ref, now)
        self.send(*point)
        if self.going_home:
            # Over the launch mark by x/y only: PX4 can hold the hop height a
            # few cm off odom (7 cm in SITL), which a 3D 0.10 m test could
            # never meet.
            arrived = (self.carrot.at_end and math.dist(
                pos[:2], self.carrot.point[:2])
                <= self.home_arrival_radius)
        else:
            arrived = self.carrot.done(pos, self.arrival_radius)
        if vio_ok and arrived:
            if self.land_at_end and self.pad_vision and not self.going_home:
                self.start_pad_find()
                return
            if self.land_at_end:
                self.get_logger().info('route complete; landing on its end.')
                self.goto('LAND', point)
                self._state_land()      # no tick without a setpoint
            else:
                self.get_logger().info('route complete; holding at its end.')
                self.goto('HOLD', point)
            return
        if (self.carrot.index, point) != self.progress_at[1:]:
            self.progress_at = (now, self.carrot.index, point)
        elif now - self.progress_at[0] > self.stall_timeout:
            self.get_logger().error(
                f'no progress for {self.stall_timeout:.0f} s at waypoint '
                f'{self.carrot.index + 1}/{len(self.route)} '
                f'({math.dist(self.position, point):.2f} m off); holding. '
                f'Check `clamped` in {STATE_TOPIC} and VIO.')
            self.goto('HOLD', self.position)

    def in_tube(self, position, point):
        """Drone and target both inside the tube's footprint, at flying
        height, and not on the fly_home hop."""
        if self.frame is None or self.going_home:
            return False
        # odom points: through the wall correction to where the walls are
        x, y, _ = odom_to_course(self.corr_to_route(position), self.frame[0], self.frame[1])
        gx, gy, _ = odom_to_course(self.corr_to_route(point), self.frame[0], self.frame[1])
        height = position[2] + self.start_height
        return (TUBE_X[0] < x < TUBE_X[1] and TUBE_Y[0] < y < TUBE_Y[1]
                and TUBE_X[0] < gx < TUBE_X[1] and TUBE_Y[0] < gy < TUBE_Y[1]
                and 0.3 < height < 1.5)

    def tof_tube_point(self, point, ref, now):
        """The route target pulled toward the tube centre the side ToF
        sensors measure (see tof_tube_centering). ref is where the drone is,
        in the frame the target is flown in: odometry, or PX4's own estimate
        during a blind carry. None means nothing to measure from."""
        m = self.tof_offset
        if not (ref is not None and m is not None and self.yaw is not None
                and self.armed and self.offboard_active and self.in_tube(ref, point)):
            self.tof_guidance.clear()       # no correction outlives its data
            return point
        axis = math.radians(self.frame[1])  # the tube runs course north-south
        if m[0] != self.tof_used_stamp:
            self.tof_used_stamp = m[0]
            # The offset is along the body's y axis: the centre in odom.
            left = (-math.sin(self.yaw), math.cos(self.yaw))
            centre = (ref[0] + m[1] * left[0], ref[1] + m[1] * left[1], ref[2])
            self.tof_guidance.update(m[0], centre, axis, now)
        return self.tof_guidance.apply(point, ref, axis, now, enabled=True,
                                       max_lead=self.max_lead)

    def in_tube_footprint(self, point):
        """The (route-frame) point is inside the tube's footprint itself."""
        if self.frame is None:
            return False
        x, y, _ = odom_to_course(point, self.frame[0], self.frame[1])
        return TUBE_X[0] <= x <= TUBE_X[1] and TUBE_Y[0] <= y <= TUBE_Y[1]

    def in_blind_zone(self, point):
        """The odom point is in the tube, or within blind_zone_margin_m of a mouth."""
        if self.frame is None:
            return False
        x, y, _ = odom_to_course(point, self.frame[0], self.frame[1])
        return (TUBE_X[0] <= x <= TUBE_X[1]
                and TUBE_Y[0] - self.blind_margin <= y
                <= TUBE_Y[1] + self.blind_margin)

    def blind_carry_position(self, now):
        """PX4's own position to pace the carrot with during a VIO loss, or
        None to freeze it (blind_carry off, outside the tube zone, past
        blind_carry_max_s, or no fresh PX4 position)."""
        why = None
        if not self.blind_carry:
            why = 'blind_carry off'
        elif self.odom_arriving():
            why = 'odometry still arriving during the loss'
        elif now - self.blind_since > self.blind_max_s:
            why = f'no VIO for {self.blind_max_s:.1f} s'
        elif not self.in_blind_zone(self.carrot.point):
            why = 'outside the tube'
        elif (self.px4_pos is None or self.px4_at is None
              or now - self.px4_at > self.odom_timeout):
            why = 'no fresh PX4 position'
        self.set_carrying(why is None, why)
        return self.px4_pos if why is None else None

    def odom_arriving(self):
        """Odometry kept coming after VIO was flagged lost (a message at
        least 0.1 s after the loss, so the last one before a real stop does
        not count). cuVSLAM runs with IMU fusion off on skybeta, so that is a
        frozen or unreliable pose, not dead reckoning: never carry on it."""
        return (self.blind_since is not None and self.odom_at is not None
                and self.odom_at > self.blind_since + 0.1)

    def set_carrying(self, carrying, why):
        if carrying == self.blind_carrying:
            if not carrying and self.blind_since is not None:
                self.info_once(f'frozen:{self.blind_since}',
                               f'carrot frozen ({why}).')
            return
        self.blind_carrying = carrying
        if carrying:
            self.get_logger().warn(
                'VIO lost in the tube: carrying the carrot on PX4\'s own '
                f'estimate for up to {self.blind_max_s:.1f} s.')
        else:
            self.get_logger().warn(f'blind carry ended ({why}).')

    # -- camera pad landing (pad_vision) ---------------------------------
    def start_pad_find(self):
        x, y, _ = self.position
        self.get_logger().info('route complete; finding the landing pad by camera.')
        self.pad = {'phase': 'search', 'goal': (x, y, self.pad_look_z), 'goal_at': None,
                    'yaw0': self.yaw or 0.0, 'k': 0, 'since': self.now_s()}
        self.goto('PAD_FIND', tuple(self.position))
        self._state_pad_find()

    def pad_land(self, target):
        """PAD_FIND -> LAND on `target`, holding the nose where PAD_FIND turned it (the
        descent's twist would otherwise steer it back to the takeoff heading)."""
        self.latch_heading()
        self.goto('LAND', target)
        self._state_land()

    def pad_clear(self, a, b):
        """The straight odom segment a -> b is inside the net, with no obstacle on the planner's
        RAW map within 0.30 m across (the drone is 0.475 m wide) from 0.25 m above the floor up to
        0.4 m over the segment. Lower is ignored: inflation spreads anything near the floor up to
        ~0.3-0.4 m (the live insertion drops points under obstacle_min_height_m, the prior map's
        floor-level geometry stays), and the course's low things (boulders: 0.53 m) still block."""
        if self.frame is None:
            return True
        n = max(1, int(math.dist(a, b) / 0.1))
        qs = [[u + (v - u) * i / n for u, v in zip(a, b)] for i in range(n + 1)]
        if any(clamp_to_net(q, self.frame[0], self.frame[1], self.net_margin)[1] for q in qs):
            return False
        g = self.planner_grid
        c = np.array([odom_to_course(self.corr_to_route(q), self.frame[0], self.frame[1],
                                     self.start_height) for q in qs])
        top = float(c[:, 2].max()) + 0.4
        offs = np.array([(dx, dy, z) for dx in np.arange(-0.3, 0.301, 0.1)
                         for dy in np.arange(-0.3, 0.301, 0.1) if math.hypot(dx, dy) <= 0.301
                         for z in np.arange(0.25, top + 1e-6, 0.1)])
        pts = np.stack([c[:, None, 0] + offs[None, :, 0], c[:, None, 1] + offs[None, :, 1],
                        np.broadcast_to(offs[None, :, 2], (len(c), len(offs)))], axis=-1).reshape(-1, 3)
        idx = np.floor((pts - g.bounds_min) / g.resolution).astype(int)
        inside = np.all((idx >= 0) & (idx < np.array(g.shape)), axis=1)
        if not inside.all():
            return False
        return not g.raw_grid[idx[:, 0], idx[:, 1], idx[:, 2]].any()

    def pad_path(self, start, goal):
        """Odom waypoints start -> goal: straight if clear, else the planner's A* + B-spline."""
        if self.pad_clear(start, goal):
            return [tuple(start), tuple(goal)]
        if self.frame is None:
            return None
        try:
            from planner.fsm_router import CourseStage
            to_c = lambda q: odom_to_course(q, self.frame[0], self.frame[1], self.start_height)  # noqa: E731
            spline, _ = self.planner_router.optimizer.plan_spline(
                to_c(start), to_c(goal),
                self.planner_router.profile_for_stage(CourseStage.STAGE_5_LANDING_PAD))
        except (ValueError, RuntimeError, AttributeError) as exc:
            self.get_logger().warn(f'pad approach not planned ({exc}).')
            return None
        if spline is None:
            return None
        pts = [course_to_odom(tuple(p), self.frame[0], self.frame[1], self.start_height)
               for p in spline.sample_points(40)]
        if not self.pad_clear(pts[-1], goal):
            return None
        return [tuple(start)] + [tuple(p) for p in pts] + [tuple(goal)]

    def plan_pad_approach(self, pad):
        """Look point + path: pad_vision_look_dist_m (then other distances) from the pad, every
        30 deg around it, nearest the drone first; it and its line to the mat must be clear."""
        px, py = pad
        cands = []
        for d in (self.pad_look_dist, self.pad_look_dist + 0.3, self.pad_look_dist - 0.2,
                  self.pad_look_dist + 0.6):
            for k in range(12):
                b = math.radians(30 * k)
                look = (px + d * math.cos(b), py + d * math.sin(b), self.pad_look_z)
                cands.append((math.dist(self.position[:2], look[:2]), look))
        seen = 0
        for _, look in sorted(cands):
            # Line of sight to the mat's edge only: nothing stands on the mat (0.76 m
            # radius), and VIO drift can put the pad itself near a wall on the map.
            d = math.dist(look[:2], pad)
            edge = (px + (look[0] - px) * 0.6 / d, py + (look[1] - py) * 0.6 / d, self.pad_look_z)
            if not self.pad_clear(look, edge):
                continue
            seen += 1
            path = self.pad_path(tuple(self.position), look)
            if path:
                return look, path
        self.get_logger().warn(f'pad approach: {seen} of {len(cands)} look points see the pad '
                               'on the map, none reachable.')
        return None, None

    def pad_walk(self, goal, yaw):
        """Move the commanded point toward goal at pad_vision_speed_mps, facing `yaw`. True once
        there: the drone within 10 cm across, or 2 s after the commanded point arrived (PX4 can
        hold a few cm off odom, more while its estimate absorbs drift)."""
        now = self.now_s()
        d = math.dist(self.target, goal)
        f = 1.0 if d <= self.pad_step else self.pad_step / d
        self.target = tuple(a + (b - a) * f for a, b in zip(self.target, goal))
        self.send(*self.target, yaw=yaw)
        if f < 1.0:
            self.pad['goal_at'] = None
            return False
        if self.pad['goal_at'] is None:
            self.pad['goal_at'] = now
        return (math.dist(self.position[:2], goal[:2]) <= 0.10
                or now - self.pad['goal_at'] >= 2.0)

    def facing(self, xy):
        return math.atan2(xy[1] - self.position[1], xy[0] - self.position[0])

    def _state_pad_find(self):
        pd, now = self.pad, self.now_s()
        best = self.pad_tracker.best()
        if pd['phase'] == 'search':
            if best is not None:
                look, path = self.plan_pad_approach(best[:2])
                if look is None:            # no look point: fly over it as it stands, then land
                    over = (best[0], best[1], self.pad_look_z)
                    path = self.pad_path(tuple(self.position), over)
                    if path is None:
                        self.get_logger().warn('pad seen, but no clear way to it: landing on the route end.')
                        return self.pad_land(tuple(self.route[-1]))
                    self.get_logger().warn('pad seen, but no clear look point around it: flying over it.')
                    look = over
                self.get_logger().info(
                    f'pad at ({best[0]:.2f}, {best[1]:.2f}): approaching the look point '
                    f'({look[0]:.2f}, {look[1]:.2f}) over {len(path)} waypoints.')
                pd.update(phase='approach', look=look, goal_at=None,
                          carrot=Carrot(path, self.pad_speed, self.max_lead, self.max_lat_accel,
                                        self.max_sagitta))
                return self._state_pad_find()
            if pd['k'] * self.pad_search_step >= 2 * math.pi:
                cand = self.pad_tracker.candidate()
                if cand is not None and cand[2] < 2:        # one glimpse: noise as often as not
                    cand = None
                look, path = self.plan_pad_approach(cand[:2]) if cand else (None, None)
                if look is None:
                    self.get_logger().warn('no pad confirmed in a full turn: landing on the route end.')
                    return self.pad_land(tuple(self.route[-1]))
                self.get_logger().info(
                    f'pad glimpsed ({cand[2]} fix{"es" if cand[2] > 1 else ""}) at ({cand[0]:.2f}, '
                    f'{cand[1]:.2f}), not confirmed: going to look from ({look[0]:.2f}, {look[1]:.2f}).')
                pd.update(phase='approach', look=look, goal_at=None, sighting=cand[:2],
                          carrot=Carrot(path, self.pad_speed, self.max_lead, self.max_lat_accel,
                                        self.max_sagitta))
                return self._state_pad_find()
            yaw = pd['yaw0'] + pd['k'] * self.pad_search_step
            if not self.pad_walk(pd['goal'], yaw):
                pd['since'] = now                   # not at the search spot yet
                return
            turned = abs(math.remainder((self.yaw or 0.0) - yaw, 2 * math.pi)) <= math.radians(10)
            if not turned and now - pd['since'] < 4.0:
                pd['turned_at'] = None
                return                              # still turning (4 s cap)
            pd['turned_at'] = pd.get('turned_at') or now
            if now - pd['turned_at'] >= self.pad_search_dwell:
                pd.update(k=pd['k'] + 1, since=now, turned_at=None)
        elif pd['phase'] == 'approach':
            point = pd['carrot'].step(self.position, 1.0 / max(1e-3, self.rate_hz))
            self.target = point
            self.send(*point, yaw=self.facing(best[:2] if best else pd['sighting']))
            if pd['carrot'].at_end and math.dist(self.position[:2], pd['look'][:2]) <= 0.15:
                pd.update(phase='look', since=now, goal_at=None)
        elif pd['phase'] == 'look':
            self.pad_walk(pd['look'], self.facing(best[:2] if best else pd['sighting']))
            if best is None:                            # a sighting, not confirmed (yet)
                if now - pd['since'] > self.pad_look_s:
                    end = self.route[-1]
                    path = self.pad_path(tuple(self.position), (end[0], end[1], self.pad_look_z))
                    self.get_logger().warn('the sighting was not confirmed from its look point: '
                                           'back to the route end to land.')
                    if path is None:
                        return self.pad_land(tuple(self.position))
                    pd.update(phase='return', goal_at=None,
                              carrot=Carrot(path, self.pad_speed, self.max_lead, self.max_lat_accel,
                                            self.max_sagitta))
                return
            fresh = best[3] > pd['since'] + 0.5
            if (fresh and now - pd['since'] >= 1.0) or now - pd['since'] > self.pad_look_s:
                self.get_logger().info(
                    ('fresh pad fixes' if fresh else 'no fresh pad fix') + ' from the look point: '
                    f'flying over ({best[0]:.2f}, {best[1]:.2f}).')
                pd.update(phase='over', yaw=self.facing(best[:2]), goal_at=None)
        elif pd['phase'] == 'return':
            point = pd['carrot'].step(self.position, 1.0 / max(1e-3, self.rate_hz))
            self.target = point
            self.send(*point)
            if pd['carrot'].at_end and math.dist(self.position[:2], self.route[-1][:2]) <= 0.15:
                self.pad_land(tuple(self.route[-1]))
        elif pd['phase'] == 'over':
            over = (best[0], best[1], self.pad_look_z)
            if self.pad_walk(over, pd['yaw']):
                self.pad_land(over)

    def _state_land(self):
        # Straight down at land_speed, nudged back over the pad. Keep pushing
        # down after touchdown: PX4's land detector needs it, and it disarms
        # COM_DISARM_LAND (2 s) later, which ends Offboard -> DONE in tick().
        tx, ty = self.target[0], self.target[1]
        best = (self.pad_tracker.best() if self.pad_vision and not self.going_home
                and self.pad.get('phase') == 'over' else None)    # only once flown over it
        if best is not None:
            tx, ty = best[0], best[1]   # where the camera saw the pad (pad_tracker)
            self.info_once('pad_land', f'landing on the camera pad fix ({tx:.2f}, {ty:.2f}).')
        ex = tx - self.position[0]
        ey = ty - self.position[1]
        norm = math.hypot(ex, ey)
        scale = min(1.0, self.land_xy_max / norm) if norm > 0.0 else 0.0
        self.send_twist(ex * scale, ey * scale, -self.land_speed)

    def _state_refused(self):
        # Push down, never up: on the floor this is PX4's land detector, then
        # its disarm. Ends like any lost Offboard: PAUSED, then waiting again.
        self.send_twist(0.0, 0.0, -self.land_speed)

    def _state_done(self):
        pass

    def _state_hold(self):
        self.learn_drift(self.target)
        self.send(*self.target)

    def _state_paused(self):
        # PX4 ignores setpoints outside Offboard; keep the stream up so a
        # later arming can take over again, and re-arm the mission only
        # once the vehicle has been disarmed.
        self.send(*self.position, fence=False)
        if not self.armed:
            self.goto('WAIT_FOR_OFFBOARD')


def main(args=None):
    rclpy.init(args=args)
    node = CourseMission()
    node.start_background_planner(start_http=True)
    try:
        try:
            from rclpy.executors import MultiThreadedExecutor
            executor = MultiThreadedExecutor(num_threads=4)
            executor.add_node(node)
            executor.spin()
        except (ImportError, AttributeError):
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_background_planner()
        if node.flog is not None:
            node.flog.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
