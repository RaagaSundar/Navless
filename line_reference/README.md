# Floor-line reference investigation — Koji, 2026-11-07

The maze floor line can supply a local lateral offset and an undirected heading
reference. It cannot supply position along a straight line: sliding along the
same line leaves those measurements unchanged. For stationary hover, retain
validated visual odometry / optical flow for along-line motion and range/height
control for altitude. This does not repair a failed position estimator.

## What exists

`reference.py` is a dependency-free **offline geometry prototype**. It takes
already selected tape-centre pixels, rectified intrinsics, optical-frame TF,
capture-time roll/pitch and base-to-floor height; it projects rays onto a flat
floor and fits one local straight segment. It returns timestamp, signed offset
in metres, relative heading, normal, residual and span. No image segmentation,
ROS node, flight integration or automatic bend selection is implemented.
It reads no simulator truth, map centreline, private host or reset API.

Frames: gravity-level, body-heading x forward / y left / z up. Optical axes
are x right / y down / z forward. Transform the **rectified** optical frame,
including its rectification rotation, rather than blindly reusing raw TF.
Height is the base origin's distance above the local floor; correct rangefinder
slant and mount offset using capture-time attitude. Raw local z is not height.
The function cannot verify sensor freshness/health: its caller must check image,
height and attitude timestamps, calibration matching and sensor validity first.

Default prototype gates: image age 0–150 ms; at least 8 selected points; span
at least 0.30 m; RMS residual at most 0.02 m; maximum residual at most 0.05 m;
heading within 60 degrees of forward. Reject upward or nearly horizontal rays
(downward ray component must exceed 0.1 of ray length) and floor intersections
over 3 m horizontal range. These are assumptions, not hardware-qualified values.
A rejected observation returns no reference, never a cached or invented fix.

Only feed a uniquely identified tape centre, not generic edges. Residual gates
can reject some corners and mixed candidates; they cannot prove that a clean
wall seam is tape, reject every corner, or resolve nearby parallel candidates.
The fit has a 180-degree direction ambiguity. Planner context must choose the
travel direction and branch; the prototype simply orients toward body forward.

## Proposed use in the maze (not activated)

With valid navigation and flight authority, a bounded lateral correction could
be proportional to `reference.closest_xy` (the vector toward the line), damped
by measured lateral velocity. Advance along the planner's collision-checked path
while centering on the observed line. A hover request sets forward travel to
zero, but still needs along-line velocity/position feedback. Use the existing
controller's frame conversion, speed limits, obstacle clearance and estimator
checks. Do not publish a second competing setpoint stream or turn this partial
measurement into a full PX4 odometry fix. Choose gains and limits only after
recorded-data assessment and approved bounded tests.

At bends, fit the near segment separately and let the planner choose the next;
do not average two legs into a diagonal shortcut. At line loss, reject line
corrections immediately and use only healthy existing navigation; if navigation
is also unhealthy, follow the reviewed abort policy. No "hover on the line"
claim is valid after all along-line motion sensing is lost. The line ends at
Gate A, so tunnel and later stages need their existing navigation strategies.

## Main feasibility question: can the camera see it?

The documented OAK mount faces forward; the downward VL53L1X is a rangefinder,
not a camera. The existing camera topics are `/oak/left/image_raw` and
`/oak/left/camera_info` (see `depthcloud/stereo_cloud.py`). Use matching
calibration/TF and rectification, not guessed simulator intrinsics.

For a level camera at height h with vertical half-FOV a, the closest visible
floor is approximately h/tan(a) ahead of the camera. As an **illustration**,
h=0.9 m and a=25 degrees gives 1.93 m; a maze wall may hide that patch entirely.
This is not a measured OAK FOV. Body pitch changes both visibility and apparent
line position, which is why attitude compensation matters. A downward-looking
camera would improve nearby floor coverage, but hardware/mount changes require
team review and recalibration; do not rotate the stereo SLAM camera casually.

Next validation is passive, props-off recording: actual rectified images,
CameraInfo, camera/base TF, attitude and height while hand-carrying over the
real line at operating height and through each bend. Confirm visibility first;
then select/segment tape centres and compare signed offsets against measured
left/right displacements. Test seams, shadows, multiple lines, blur and line
loss. OpenCV's [Hough line documentation](https://docs.opencv.org/4.x/d9/db0/tutorial_hough_lines.html)
describes a candidate detector, but a Hough segment alone does not identify tape.
No aircraft connection or recording was attempted in this investigation.

## Offline verification

Run from the repository root:

```bash
python3 -m unittest discover -s line_reference -p 'test_*.py' -v
```

Nine synthetic tests cover camera projection, mount translation, tilt,
lateral sign, heading, endpoint reversal, along-line unobservability,
stale/future timestamps, invalid calibration, horizon rejection and ambiguous
geometry. Synthetic points establish geometry only; there are no real images,
detector accuracy measurements, moving simulator tests or flight results.
See `evidence.txt` for exact commands, settings, hashes and results.
