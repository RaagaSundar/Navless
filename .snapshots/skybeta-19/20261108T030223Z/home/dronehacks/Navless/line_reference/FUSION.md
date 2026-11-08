# SLAM plus floor line — offline fusion core

`FusionSession.fuse` combines continuous SLAM x/y/yaw with floor-line distance
and direction. It returns a planar pose and full 3x3 covariance for analysis.
It is a per-sample fusion function, not a temporal motion estimator, and it
does not add IMU fusion to cuVSLAM. It has no ROS/PX4 publishers or subscribers.

SLAM supplies full pose; an associated line constrains the position normal to
that line and yaw relative to its direction. A straight line adds no along-line
position measurement. Neither this code nor a line alone can recover full
navigation when SLAM is lost. Height, roll/pitch and velocity are not estimated.

## Why this fusion rule

Both estimates can use the same camera, so independent-measurement Kalman fusion
could count information twice. This implementation uses fixed-weight covariance
intersection with SLAM weight w=0.75. The information matrix is

```
Pf^-1 = w Pslam^-1 + (1-w) H' Rline^-1 H
delta = Pf (1-w) H' Rline^-1 (z - H xslam)
```

Here H selects cross-line position and yaw only. The yaw residual is resolved
modulo pi near the current SLAM yaw (tape has no arrow). With identical sensor
variances, the observed variance stays unchanged, rather than being halved.
The unmeasured along-line variance can increase; correlations in the SLAM prior
can also move its along-line mean. No along-line information is added by H.
Covariance intersection addresses unknown cross-correlations **only if** the
input error bounds are valid and the local model/association are correct. It
cannot remove systematic calibration error or make a false tape match safe.

The primary reference is Julier and Uhlmann,
[Using covariance intersection for SLAM](https://www.sciencedirect.com/science/article/pii/S0921889006001436).
ROS's [sensor-data preparation guidance](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/preparing_sensor_data.rst)
also explains the need for correct frames and meaningful covariances. This
prototype does not wrap or configure robot_localization.

## Required real inputs

- `SlamSample`: a new RAW, validated `odom -> base_link` continuous pose at the
  image capture time, full x/y/yaw covariance, tracking status and timestamps.
  Extract indices [0, 1, 5] from ROS's 6x6 pose covariance, keeping cross terms.
  Use the existing odometry validation gate before this adapter; reject the
  cuVSLAM covariance-unavailable sentinel. `covariance_available` is an explicit
  caller assertion, not a substitute for validating the ROS message. Never use
  loop-closure map poses, PX4's already-fused output or this module's output as
  the prior. Raw SLAM may include temporal correlations; it remains the only
  temporal estimator in this prototype.
- `LineSample`: the existing rectified-pixel geometry output, PLUS externally
  assessed 2x2 uncertainty of signed offset and heading. Include pixel/segment
  error, height, attitude, calibration, timing and map registration uncertainty;
  fit RMS alone is insufficient. The sign/order is (offset metres, heading
  radians), including cross-covariance if known. A successful geometry fit
  does not imply valid tape identification or association.
- `MapSegment`: measured endpoints registered into the SAME odom epoch, with
  a unique segment ID. This is a caller-supplied, reviewed association, not a
  lookup into the simulator's truth or the course JSON. A line observed without
  a registered map can still guide relative centering, but supplies no absolute
  position correction. When heading/origin resets, invalidate registration and
  create a new session only after re-registering the measured map.
- Fresh synchronized image, matching rectified CameraInfo/TF, attitude and
  corrected base-to-floor range. An eventual ROS adapter must interpolate raw
  SLAM to image time and validate all those sources before asserting
  `sensors_validated`. This library only checks supplied timestamps and flags.

`requirements.txt` declares NumPy for the matrix operations; the original
geometry module still needs only the Python standard library.

## Rejection and fallback

Prototype defaults: SLAM/image age <=150 ms, tracking-status age <=250 ms,
no future timestamps, SLAM/image timestamp skew <=30 ms, positive-definite full
covariances, validated unique segment, same frame/registration epoch, and the
existing line geometry quality gates. Timestamp skew is a tolerance, not motion
compensation; its error must be included in R. Covariance values and thresholds
remain unqualified for hardware.

Do not extrapolate past the segment endpoints. Reject distance disagreement
over 0.35 m or heading disagreement over 30 degrees. The innovation gate uses
`2*(H P H' + R)`, an upper bound with unknown cross-correlation, and threshold
9.21; it is a diagnostic threshold, not an independent-noise statistical claim.
Reject corrections over 0.15 m in XY or 10 degrees yaw; never clip a bad fix
and pretend it passed. Each raw SLAM stamp and each line image may be consumed
at most once. The adapter must discard its buffers on resets or clock rewinds.

Healthy SLAM + missing/rejected line returns EXACT raw SLAM (`slam_only`). Bad,
stale or repeated SLAM returns no pose (`unavailable`), even with a valid line.
Every result includes a reason. The caller still owns estimator health, motion
jump detection, authority checks and the approved flight-abort policy.

**Do not wire this output into the existing PX4 bridge.** Fused-to-raw fallback
can introduce a pose step, and neither output continuity nor full 6D covariance
has been qualified. Before any live use, add a reviewed shadow ROS adapter on a
separate diagnostic topic, evaluate camera recordings, and design continuous
local odometry versus map correction explicitly. Keep one TF authority and one
PX4 vision producer. This work changes neither today.

## Run and interpret the checks

From the repo root:

```bash
uv run --no-project --with numpy python -m unittest discover -s line_reference -p 'test_*.py' -v
uv run --no-project --with numpy python line_reference/demo_fusion.py
```

The seeded demo uses synthetic SLAM drift and independently noisy line
observations, with 10 deliberate dropped images out of 100. Its synthetic map
and truth are test fixtures only. It reports error against that fixture, not
measured camera or flight performance. The equal-information correlation test
separately checks that duplicate information does not halve the uncertainty.
There are no recorded camera inputs, aircraft connections, moving simulator
tests, detector metrics or hardware qualification in this change.
