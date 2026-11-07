# Install the Mac + NVIDIA Jetson course stack

This package fills the missing runner between the public Mac simulator and the
organiser's NVIDIA companion image. The Mac renders stereo and runs PX4 SITL;
the Jetson runs actual cuVSLAM and the companion controller. The preview and
video workers run on the Mac. ROS domain 219 is simulation-only; physical domain 19
and Mac-only domain 220 are separate.

The packaged controller now includes the exact frozen 221438 independently
passing runtime and centered window-forward profile. The original 213553 airborne sill-stop
failure and manifests are preserved for provenance. The sealed 221438 independent grader passed; this is one simulation result,
not repeatability or hardware qualification. Actual laser
freshness, fixed EV/barometer height, bounded sill-event evidence and all
existing authority/estimator/recorder gates remain mandatory.

## What is available in GitHub

As checked 2026-11-07:

| Change | Actual availability |
|---|---|
| [#32](https://github.com/RaagaSundar/Navless/pull/32) | Mac ARM64 base image and PX4 build recipe, merged main |
| [#35](https://github.com/RaagaSundar/Navless/pull/35) | Mac-only domain 220 course + RTAB-Map environment, merged main; expressly excludes flight controller |
| [#36](https://github.com/RaagaSundar/Navless/pull/36) | Network setup doctor/fix in `split_setup/`, now consolidated into main (originally merged into the Mac course branch after #35) |
| [#40](https://github.com/RaagaSundar/Navless/pull/40), [#41](https://github.com/RaagaSundar/Navless/pull/41) | Portable synthetic appearance/open-net generators, merged main; no automatic flight readiness |
| This package | Complete split runner, frozen controller/route, source gate, recorders, install scripts and preview/video workers; no private sibling workspace required |

The procedural appearance is synthetic. The public portable schema2 manifests
are regenerated from tracked assets; they are not falsely presented as the old
path-bearing renderer manifests from a recorded flight. No recordings, maps,
build output, passwords or private keys are distributed.

## Prerequisites

Use an Apple Silicon Mac, Git, Python3.10+, Docker Desktop with Linux ARM64,
OpenSSH and sufficient free disk for PX4/Gazebo. Coordinate build ownership;
compilation must not compete with another flight. The build-only PX4 container
uses two CPUs and 6 GiB. Full image/PX4 clean builds were not rerun for this PR.

The Jetson must already have its **organiser-provided compatible `skybeta:0.3.3`
image**, NVIDIA runtime, and authorised SSH account. This image contains ROS2
Humble, CUDA/cuVSLAM, `isaac_ros_visual_slam`, `px4_msgs`, the existing
`oakd_vslam_bringup` PX4 odometry bridge and `/usr/local/bin/skybeta_env.sh`.
Image distribution, JetPack installation, firmware and physical aircraft setup
are organiser prerequisites; this repository cannot recreate them from public
source. Installer checks the image before copying and records the actual image
ID. If unavailable, obtain it from the organiser; do not replace hardware drivers.

## Obtain and validate the sources

Check out this PR's branch in a new folder. Once merged, ordinary main suffices:

```bash
git clone https://github.com/RaagaSundar/Navless.git Navless-split
cd Navless-split
# Until passing-profile PR #49 merges, fetch its reviewed head:
git fetch origin pull/49/head:review-pass-49
git switch review-pass-49
python3 split_sim/doctor.py --source-only
python3 -m unittest split_sim.test_install -v
bash split_sim/setup_jetson.sh --dry-run
```

The doctor verifies every tracked dependency and all 39 packaged runtime hashes.
The dry-run invokes neither SSH nor Docker. Preserve the checkout commit in your
run notes. Existing hardware/team checkouts need no changes.

For preview/video dependencies and actual rendering smoke tests, follow
[preview/README.md](preview/README.md). Those install commands are included there;
the live preview is not dependent on a pre-existing private frontend checkout.

## Build the Mac side, without starting simulation

```bash
bash split_sim/bootstrap_mac.sh --build
python3 split_sim/doctor.py --local
```

This builds the tracked `local_sim/Dockerfile` image and the tracked PX4
v1.16.1 helper into `split_sim/state/PX4-Autopilot`. It starts no simulator,
PX4 process or flight. Downloads/apt packages are not hermetically pinned;
record actual Git/image identities. Build failures remain in Docker logs and
state; do not silently proceed after failure.

## Install the separate Jetson simulation container

Use the organiser's verified SSH host fingerprint. Establish known_hosts through
normal interactive SSH after checking that fingerprint. No password is stored
by these scripts. Override the example account/host if yours differs:

```bash
export JETSON_SSH_TARGET=dronehacks@skybeta-19.local
export JETSON_HOST_KEY_ALIAS=skybeta-19.local
# Use your existing trusted organiser host identity; do not disable host checking.
cd split_sim
bash setup_jetson.sh --install
```

Installation refuses an existing split container or a running Jetson-local
bench instead of overwriting a teammate's installation. It copies only packaged
simulation files into `~/team_ws/split_sim` and creates a **stopped** container
`drone19-split-sim`, without USB/serial mounts. No second autopilot or real
aircraft service is started. An existing installation needs a separately reviewed
update, not deletion or force replacement.

Default estimator is stereo-only. To use the measured250 Hz body-IMU transport,
select it while the separate simulation container is stopped:

```bash
bash configure_estimator.sh stereo-imu
export SPLIT_INERTIAL=1
export SPLIT_CAMERA_RATE_HZ=31.25
```

For stereo-only use `configure_estimator.sh stereo`, `SPLIT_INERTIAL=0` and the
reviewed camera rate. Both ends must agree. These are simulation calibration/
noise assumptions; they do not alter the physical camera or qualify hardware.

## Start the disarmed environment and inspect

```bash
# From split_sim, inactive setup; select reviewed simulated height profile.
export SPLIT_HEIGHT_PROFILE=ev-terrain
# Explicit split mode, never the auto Jetson-local selection.
bash start.sh --split
bash start_px4_from_mac.sh
```

The first starts Gazebo, sensor transport and the separate NVIDIA estimator.
The second starts PX4 SITL plus a heartbeat-only local station; neither arms or
requests Offboard. Initial original renderer uses tracked Gazebo assets. Endpoints:

- Gazebo desktop: `http://127.0.0.1:6081/vnc.html?autoconnect=true&resize=scale`
- NVIDIA map/camera: `http://127.0.0.1:18765/`
- Stereo cloud: `http://127.0.0.1:18081/`
- Experimental OBJ observer: `http://127.0.0.1:18082/`

The authenticated SSH server address determines the XRCE target; optional
`JETSON_IP` must be a literal reviewed IPv4. Do not use a stale mDNS address or
change physical PX4 parameters. SITL alone uses domain 219, UDP8889 and
`UXRCE_DDS_SYNCT=0` to retain shared simulation time.

Inspect fresh actual images/CameraInfo, `/clock`, validated odometry and
PX4 local position/estimator flags. Do not infer readiness from process existence.
The subsequent prepared-run gates check actual rates, IMU subscription when
selected, range fusion, stationary frame/datum, continuous observation and the
independent recorder. No made-up visual fix or simulator position is used by
navigation. The separately identified range sensor model and independent grader
use simulator truth; the OBJ observer does not feed PX4.

## Optional portable neutral appearance

Install Pillow as documented by `appearance_profile/README.md`. In an inactive
setup, create a new output and review its manifest:

```bash
# From repository root, before starting the renderer for this comparison.
python3 appearance_profile/generate.py --source gazebo_sim \
  --world-source gazebo_sim/worlds/dronehacks.sdf \
  --output split_sim/state/portable-neutral --seed 20261107 --pixels-per-m 256
```

The generator is deterministic within its stated
dependency limits and refuses an existing output. Then select it explicitly:

```bash
cd split_sim
export SPLIT_APPEARANCE=portable-neutral-v1
export SPLIT_APPEARANCE_DIR="$PWD/state/portable-neutral"
export APPEARANCE_MANIFEST_SHA256="$(shasum -a256 state/portable-neutral/appearance.json | awk '{print $1}')"
bash start.sh --split
```

For open-net visuals, use the tracked `appearance_profile/net.py` instructions
with the **official contest OBJ ZIP supplied by the organiser**, a new output,
`SPLIT_APPEARANCE=portable-open-net-v2`, its directory and reviewed manifest hash.
The ZIP is not secretly downloaded or assumed to exist at a private path.
Collision net remains the conservative solid envelope; optical holes are not
free flight space. Each changed appearance is an unqualified simulation variant.

## Prepare a recorded course and approve only after team review

**This baseline has no verified full-course pass.** Review the actual route,
yaw/height settings and corrected height follow-up before a moving test. The
source-pinned default route uses the existing conservative grader; failures are
retained. Preparing alone does not authorize arming:

```bash
SPLIT_YAW_POLICY=first-exterior bash run_split_course.sh --prepare-flight
# The command prints a fresh RUN_ID and records a passive prepared observer.
# After reviewing its plan and fresh evidence, team approval for that exact run:
bash run_split_course.sh --approve-prepared RUN_ID
```

The prepared launcher remains running while awaiting approval. Approval must be
sent from another terminal. It is bound to run ID, frozen source hashes and the
same world. No old marker authorizes a new world. The runtime retains the full
30-second healthy observation, source/estimator checks and continuous Offboard
stream before the existing single-arm arbiter acts. Stop publication on stale
telemetry or lost authority. No second keyboard/controller/program may run.

The default yaw remains `first-exterior`; the explicit `SPLIT_YAW_POLICY=window-forward`
option selects the now-integrated route-pinned camera candidate. The corrected
`ev-terrain` height profile is selected before starting the disarmed container.
Their exact recorded configuration passed one bounded simulation trial; each new run still needs qualification. Do not pass an
unsupported flag because it exists in another workspace. No argument disables
the independent grader or fabricates a position fix.

Run evidence is stored at `local_course/state/nvidia-runs/RUN_ID/`: original
mission/source hashes, sensor recording, ULog, logs, actual track and independent
grade. A mission exit 0 alone is not a pass. Preview/video workers use these files;
see [preview/README.md](preview/README.md). Failed recordings/grading remain visible.

After a recorder-confirmed landed/disarmed stop, `reset_split_after_flight.sh`
verifies fresh local station telemetry before resetting this simulation. It is
not a portable flight API. After keyboard RC, land/release RC and reset before
any automated mission. `stop.sh` ends the separate simulation/tunnel; it is not
an airborne landing command and does not stop the physical stack.

## What was checked for this package

Source-only doctor and complete frozen manifest; a copy into a fresh folder
whose name contains spaces; missing-file rejection; shell syntax; pure transport/
readiness/runtime tests; viewer HTTP/ranges and actual synthetic video/chase
rendering. No clean NVIDIA build, remote installation, simulator lifecycle,
arming or flight was performed for packaging. Preserve that boundary when
reporting installation success.

## Height-candidate checks from the retrieved checkout

```bash
python3 -m unittest split_height.test_height_profile -v
python3 -m unittest discover -s split_sim/course_runtime -v
python3 -m unittest discover -s split_sim -p test_sensor_link.py -v
```

The runtime suite needs NumPy; optional retained-ULog replay needs `pyulog` and
an operator-retained original log. No raw logs are distributed. The source doctor,
fresh-layout checks and dry-run need only Python. See `../split_height/README.md`
for exact candidate provenance, known prior failure, finite measured transition
conditions and replay limits. The passing221438 profile and its current source
identity are documented separately in profiles/README.md. Do not change a running
container's profile: coordinate a disarmed fresh world, preserve failed evidence,
and use the documented setup/start sequence under the team's approval workflow.


## Independently passing centered-window recipe

See [profiles/README.md](profiles/README.md) for the exact frozen221438 runtime,
centered0.762m route, matching window-forward yaw, EV-height and original-perimeter
neutral appearance. All39 packaged source members match its archived manifest.
The sealed independent grade passed all six checkpoints with zero contacts,
12.70cm minimum clearance and13.23cm pad landing followed by normal disarm.
This is one simulation pass, not hardware qualification. The older215942 sole
window-height failure remains archived separately, not relabeled.

Until PR49 merges, the actual passing code must be fetched explicitly using
`git fetch origin pull/49/head:review-pass-49`; do not assume it is already in
main. The normal launcher now actually archives/invokes the passing route/config.
No new return-hop, heading-all or obstacle prototype belongs to this profile.

## Optional bounded M3 capture pair

The separate [M3 recipe](m3_bench/README.md) provides reproducible nominal and
physically rotated-M3 worlds, exact frozen finite-capture contract, explicit
compose/prepared-launch selection and independent replay. Both met bounded
capture goals, not full-course or avoidance goals. Ordinary grades are unchanged
and incomplete. The passing49 profile remains unchanged without this opt-in.
