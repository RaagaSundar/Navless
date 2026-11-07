# Mac local course: experimental SITL and evidence tools

This extends [PR #32's Apple Silicon base install](https://github.com/RaagaSundar/Navless/pull/32).
It does not duplicate the base Dockerfile or installation runbook. This PR is
based on main; until #32 merges, obtain its `local_sim` directory in a separate
checkout as below. No sibling `Navless`, `jetson_sim`, existing PX4 binary or
private workstation directory is required. Generated course assets derive from
this repository's tracked `master_sim` and `gazebo_sim` sources.

**Status: experimental, three airborne attempts and zero course passes.**
No flight controller is included here. The ongoing mission adapter, route policy,
PX4 integrity changes, raw recordings and private evidence are deliberately
outside this environment PR. Starting the environment starts disarmed SITL,
simulated sensors and perception; it never selects Offboard or arms. Team
approval and fresh telemetry are required before any separately supplied mission.
No physical aircraft interfaces, devices, SSH or hardware parameter instructions
are used.

## Fresh checkout and build

Use an Apple Silicon Mac, Docker Desktop configured for Linux ARM64, Git, Python
3 and enough disk for PX4 and Docker build caches. Coordinate compute ownership:
do not build during another person's flight. The base helper bounds PX4 to two
compiler jobs; setup adds 2 CPU / 6 GiB container limits. Build duration and disk
requirements have not been measured from this clean packaging.

```bash
git clone https://github.com/RaagaSundar/Navless.git Navless-course
cd Navless-course
git fetch origin codex/macos-local-course
git switch --track origin/codex/macos-local-course
# Separate base checkout; no modification to the course checkout or live install.
git fetch origin refs/pull/32/head:refs/remotes/origin/pr-32
git worktree add --detach ../Navless-base origin/pr-32
cd local_course
BASE_SIM_DIR="$PWD/../../Navless-base/local_sim" bash setup.sh
```

Once #32 is merged, use `BASE_SIM_DIR="$PWD/../local_sim"` instead. Setup builds
its base image and CPU RTAB image, then uses #32's build-only helpers for PX4
v1.16.1, release/1.16 px4_msgs and XRCE Agent v2.4.3. The messages release branch
and apt packages can move: `state/build-identity.txt`, image inspection and setup
log record actual identities. This is a usable build recipe, not a hermetic
lockfile. Existing state is reused; archive it before intentionally rebuilding.
A failed setup exits before runtime launch; inspect `state/setup.log`.

## Start and inspect, without flight

```bash
# Preserve any previous run before recreating its container.
COURSE_START_BEARING=90 COURSE_RTF=0.5 \
  VISION_PROFILE=baseline-descriptor bash start-local.sh --fresh
docker exec local-course-sim bash -lc \
  'source /local_course/env.sh; python3 /local_course/sim_preflight.py --duration 35 --output /course-state/current/stationary35s.json'
```

North-facing bearing 0 and east-facing 90 are the only accepted fixtures.
`fixture.json` records the actual spawn and common bridge bearing; it is a
static surveyed assumption, never a live position fix. `COURSE_RTF` is in (0,1]
and changes only the runtime world's requested real-time factor. Actual achieved
speed can be lower. `physics-settings.json` records XML readback.

ROS domain **220**, Gazebo partition `local_course`, localhost DDS and an internal
Docker network isolate this from aircraft domain 19 and Jetson bench domain 219.
Only noVNC binds host loopback port 6082. Docker Desktop host networking is not
needed. Runtime has no network egress; setup downloads happen separately.
`state/home` is writable container HOME, including SITL binaries and build output;
repository sources are mounted read-only. `state/runs/<UTC>/` retains logs,
immutable fixture and PX4 ULogs. Stop with `bash sim-stop.sh`; this terminates the
whole simulation and is not an airborne landing command. Keyboard RC sessions
must land/release RC and reset to a fresh world before automated programs.

Inspect `/clock`, actual image timestamps/CameraInfo/TF, fresh landed/disarmed
PX4 status, EV XY/yaw fusion and bridge diagnostics. Startup success or topic
names alone prove no readiness. There is no full-course preflight/flight command
in this PR. Stop if telemetry is stale or estimator validity is lost.

## Stereo and optional OBJ observer

`VISION_PROFILE=baseline-descriptor` and `temporal-flow` select CPU RTAB-Map;
`opencv-measured-stereo` selects the separate measured four-view OpenCV prototype.
Switch only through a fresh run. The wrapper checks paired acquisition times,
frame/calibration validity, tracking, finite positive covariance, source age and
wall stall; it emits no invented pose after loss. See source/tests for exact gate
thresholds. The launch uses the static simulated camera mounting TF; actual
hardware needs measured calibration and qualified drivers. No IMU estimator is
implemented. Neither CPU profile has passed the full course.

The optional OBJ observer uses the current checkout's existing `depthcloud`
modules and tracked course OBJ. It is constrained to the reviewed east fixture:

```bash
docker exec local-course-sim bash /local_course/map-observer.sh
```

This starts exact-camera-timestamp paired stereo/CAD observation, writes
`map-observer/` evidence, and publishes observer outputs only. It does not replace
the EV source. Its known initial course pose and exact CAD prior are strong
simulation assumptions; results remain weak/unqualified. Stop the observer with
Ctrl-C; do not run it during a resource-sensitive moving test. No HTTP port for
its process-internal viewers is exposed by Compose.

## Independent recording, grading and viewer

For a separately reviewed mission, start the recorder *before* arming in another
terminal. There must be one pilot/mission owner. These commands never arm:

```bash
docker exec local-course-sim bash -lc \
  'source /local_course/env.sh; python3 /local_course/score_record.py --output /course-state/current/flight/track.json'
# After recorder exits following landing/disarm (or an explicit stop):
docker exec local-course-sim bash -lc \
  'source /local_course/env.sh; python3 /local_course/score_course.py /course-state/current/flight/track.json --repo /workspace/Navless --output /course-state/current/flight/grade.json'
python3 viewer/server.py
```

Open [the loopback run viewer](http://127.0.0.1:8774). It has no flight/reset/ROS
controls and serves only allowlisted bounded evidence; missing grades remain
unqualified. Optional `flight/planned-route.json` is a plan, never actual flight.
The independent grade checks ordered gates, contacts, conservative aircraft
envelope, ceiling, telemetry freshness and stable disarmed landing on the pad.
Preserve all failed/incomplete recordings. A mission process exit is not a pass.

**Oracle disclosure:** `range_sensor.py` alone converts Gazebo truth and collision
geometry to a noisy downward range model (40 Hz, 1 cm Gaussian error, ideal ray,
2 cm–4 m); independent scoring reads truth/contact sensors. The generated textured
course is an optimistic visual assumption. Neither oracle enters portable visual
navigation. A simulator result is not hardware qualification.

## Results and limits

The latest observed airborne attempt used the physical east fixture and CPU RTAB:
real arm/takeoff, range height plus EV XY/yaw fusion, zero ordered checkpoints,
zero obstacle contacts, landed/disarmed 2.2836 m from the pad. The then-current
mission integrity gate stopped at 0.215 m 3D estimator disagreement (0.191 m in
unused stereo height; 0.099 m horizontal). Acquisition-matched fused-axis checks
are ongoing separately. Three airborne attempts produced **zero passes**.
These are operator-reported prior-session observations, not reproduced by this
packaging or independent hardware evidence. No raw session logs are published.

Measured OpenCV replay previously tracked a short pre-contact interval but lost
tracking on the full camera-only route. Low-rate or stationary success does not
qualify moving flight. This draft's clean-checkout validation is recorded in
`VERIFICATION.md`; a clean Docker build/start remains required before readiness.
