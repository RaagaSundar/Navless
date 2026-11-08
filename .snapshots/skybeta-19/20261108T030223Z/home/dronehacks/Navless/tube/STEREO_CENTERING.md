# Experimental stereo tube centering

Implemented and tested offline on 2026-11-07. Disabled in all launch defaults.
No hardware access, flight, deployment, or PX4 parameter changes were performed.
This feature estimates a lateral centerline; it is not a replacement estimator,
an altitude controller, or a collision-avoidance system.

## Data path and behavior

Calibrated stereo images + CameraInfo + camera extrinsics TF
→ `depthcloud/stereo_cloud.py` CPU OpenCV SGBM
→ `/team/stereo/points` in left optical coordinates
→ `navless_nav.stereo_tube` + continuous SLAM odometry + camera-to-body TF
→ `/tube/stereo/centerline` (`PoseStamped`, `odom`)
→ optional correction in the existing `course_mission` commander.

Use the same validated odometry source for this node, the mission and PX4's
existing vision bridge. This change does not rewire that bridge. Map-loop-closure
poses must remain outside the continuous odometry/control path.

The estimator interpolates position and orientation at the cloud's original
timestamp, transforms camera points through measured body TF, and fits both tube
sides in a horizontal slice ±10 cm from flight height. Robust parallel-line
fits require independent support on both walls, at least 30 cm of axial coverage
per wall, shared coverage, low residuals, consistent orientation and plausible
width. The default width is 1.45 m at flight height; an oval's width changes with
height. The node rejects unsupported geometry instead of substituting a side
from the course model. Floor/roof-only or single-wall observations do not suffice.

`/tube/stereo/centerline` carries a point on the observed centerline and its
undirected axis orientation. The header retains the **sensor** timestamp.
The mission changes only its lateral target relative to that line. It preserves
altitude and caps the overall carrot lead. The target's displacement from the
nominal route is limited to 20 cm and acquisition ramps at 10 cm/s. It acts only
when the aircraft **and target** are inside the tube, below its roof, with fresh
tracking, odometry, stereo diagnostics and flight authority. It never commands
yaw, arms, changes mode or publishes a position fix.

An observation can be anchored in continuous, healthy SLAM odometry for at most
3 seconds and 1.5 m from the observation location. This is necessary because a
forward camera cannot see both walls throughout a short tube. New images without
a fit do not renew the observation timestamp. `/tube/stereo/status` distinguishes
`observed: true` from `observed: false` (an existing anchor). Lost tracking, an
odometry position/heading jump, clock rollback, invalid frames/transforms, stale
images/diagnostics or an expired anchor revoke correction. Invalid correction
returns to the nominal target immediately; that removal can be a step of up to
20 cm. Telemetry/authority loss stops publication by the opt-in mission path.
Existing tracking-loss/`blind_carry` policy remains separate; centering does not
make that policy safe or create odometry when vision is absent.

The new upstream `texture_filter` rejects low-contrast pixels that SGBM can fill
across the otherwise featureless opening. `require_calibration_tf` prevents use
of the stereo node's development baseline fallback. Both retain their old false
defaults for existing users. Enable both explicitly for this experiment.

## Offline simulation and tests

From the Navless checkout, in a Python environment containing pytest, NumPy and
OpenCV (the session used `uv run --with pytest python`):

```bash
python -m pytest navless_nav/test gazebo_sim/dh/test_tube_blackout.py gazebo_sim/mirror/test_tube_test.py -q
python tube/sim_stereo_centering.py --output tube/stereo_evidence/final-rendered.json
```

The simulator raycasts left/right **images** against a textured, 1.524 m long,
58 × 72 inch oval tube and runs the production SGBM, wall fitting and bounded
guidance. It models continuous odometry with a 12 cm lateral bias, a 0.5 m/s
traversal and second-order lateral response. Seeds 1–3 are tested in both
directions, with centering on/off. The camera faces travel in those comparisons.
Pass criteria: centering improves final error, final lateral error <3 cm, and
modeled lateral clearance >10 cm. Untextured and fixed-backward-camera cases must
produce no accepted wall fits. The command exits nonzero if any criterion fails.

Oracle use is confined to the simulation's rendering, dynamics and scoring.
Navigation sees reconstructed stereo points and modeled odometry, not true pose.
This is **not Gazebo/PX4 SITL or actual cuVSLAM**: it does not qualify the
autopilot, real tracking, magnetic heading resets, calibration, CPU latency,
motion blur, exposure changes, netting or physical clearance. Its footprint
score checks lateral clearance at fixed height, not full swept-volume collisions.
The M1 local environment cannot execute NVIDIA cuVSLAM; the configured ROS/Gazebo
container image is not installed. No aircraft was used to bypass that limitation.

Evidence in `stereo_evidence/` records source SHA-256 hashes, assumptions, runtime
versions, per-frame traces and failures. Baseline before editing: **79 passed**.
Final regression: **100 passed**. All six camera-facing-travel comparisons
passed: final lateral error was 0.21–2.38 cm with correction versus 11.88 cm
without it. Minimum modeled lateral clearance with correction was 38.6 cm.
These are outcomes of the declared idealized sensor/dynamics model.
The first image experiment had zero fits: untextured opening pixels contaminated
the cloud, and both walls were not continuously visible. The preserved
`initial-rendered.log` also records an initial NumPy-boolean JSON serialization
failure. After converting score scalars to Python types, `rendered.json` records
the zero-fit result. The correction was the explicit texture check and bounded
odometry anchoring, not a relaxed two-wall fit or score threshold. An initial
integration test also caught floating-point timestamp roundoff at identical
times; a 1 microsecond comparison tolerance corrected it. Future-dated samples
outside that tolerance remain rejected.

## Integration for a later isolated ROS simulation

These are component invocations, not a flight-arming script:

```bash
# Existing stereo producer, isolated simulated camera topics and measured TF:
python depthcloud/stereo_cloud.py --ros-args -p use_sim_time:=true \
  -p texture_filter:=true -p require_calibration_tf:=true

# Read-only centering node (requires the installed package / built overlay):
ros2 run navless_nav stereo_tube --ros-args -p use_sim_time:=true \
  -p odometry_topic:=/mapping/validated_odometry
```

Both standard/course launches expose `stereo_tube_centering:=true`, which starts
the read-only node and enables correction in the existing commander. They do not
start another stereo producer. Match `odometry_topic` to the approved existing
vision path, set `start_bearing_deg` consistently, and use a single commander.
Do not launch a second standalone centering node alongside the launch-managed one.
ROS simulation must propagate `/clock` and `use_sim_time` to every component.
No launch default enables this experiment for hardware.

**Out of scope:** backward support is unnecessary, as clarified by the user.
The fixed north-facing camera in the existing route traverses the tube
backward. That arrangement produced no valid two-wall observations in this image
simulation. Centering therefore stayed off. Successful camera-facing-travel
runs do not qualify the current backward route. Establish a measured centerline
on approach with adequate camera visibility, or separately implement and qualify
a camera/heading strategy, before claiming that route benefits from this feature.
