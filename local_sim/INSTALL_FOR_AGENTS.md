# Installation runbook for coding agents

This runbook installs the Mac base environment and optional build tools. The
default launch opens a paused course for inspection. It never starts a mission.
Read the repository's AGENTS.md and the team's current simulator instructions
before changing shared files or starting services.

## 1. Establish ownership and prerequisites

- Use an Apple Silicon Mac with Docker Desktop installed and its ARM64 Linux
  engine running. This setup was validated on an M1 Max with 32 GB host RAM and
  an 8 GB Docker VM. Allow roughly 20 GB free for initial images, caches and
  source builds; the observed first installation consumed about 12 GB.
- Run `git status --short` and `docker ps` first. Preserve teammate edits and
  active containers. Use an isolated worktree for packaging or unrelated edits.
- Reserve this preview's ports 6080/8081/8082 and ROS domain 219. Other simulators
  can use different ports/domains, but still compete for CPU and RAM. Do not
  stop another owner's jobs or run a second message build without coordination.
- Keep passwords, SSH keys, login cookies and tokens outside Git. These public
  source downloads require no stored project credentials.

## 2. Start the base image

From the repository checkout:

```bash
cd local_sim
bash start.sh
```

`start.sh` finds Docker Desktop's bundled CLI on macOS, starts Docker if needed,
validates the repository path, builds the image and starts the desktop. The
first download can take a while. Build output is retained in `evidence/`.
Subsequent builds reuse Docker's cache. Do not prune other images or caches to
work around a slow download.

For manual Docker commands, if the CLI is absent from the terminal PATH:

```bash
export PATH="$HOME/.docker/bin:$PATH"
```

The default bind source is the repository containing `local_sim/`. To use a
separate checkout, set `NAVLESS_SOURCE_DIR` to its absolute path before starting.
The source is read-only inside the container. Generated runs, build workspaces
and logs live under `local_sim/state/`; do not commit that directory.

## 3. Inspect the course without flight

```bash
docker compose exec gazebo sim-course
```

Open <http://localhost:6080/vnc.html?autoconnect=true&resize=scale>. Confirm that
the course renders. It starts paused. No PX4 process or flight controller is
started by this command. Close this scene before starting another simulator in
this container. Stop only this preview with `docker compose stop` when handing
CPU ownership to another agent.

## 4. Check the installation

These checks do not arm or publish flight commands:

```bash
docker compose config --quiet
bash -n start.sh desktop.sh course.sh bootstrap-px4.sh bootstrap-agent.sh
docker compose exec gazebo bash -lc 'source /opt/ros/jazzy/setup.bash; gz sim --versions; glxinfo -B; python3 -c "import rclpy, cv2, numpy, gz.transport13"'
docker compose exec gazebo bash -lc 'source /opt/ros/jazzy/setup.bash; SDF_PATH=/workspace/Navless/gazebo_sim/models gz sdf -k /workspace/Navless/gazebo_sim/worlds/dronehacks.sdf'
docker compose exec gazebo bash -lc 'source /opt/ros/jazzy/setup.bash; PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider tests/test_planner.py gazebo_sim/dh/test_tube_blackout.py -q'
```

Expect llvmpipe software OpenGL, rather than GPU acceleration. A missing
`dronehacks-course-v3.obj` causes an existing planner test to skip; preserve and
report it. Never alter an estimator check or grader to obtain a passing result.

`check_sensors.py` is an optional passive 20-second check. It requires an
already-running, coordinated bench that supplies advancing `/clock`, both
1024x640 stereo images and both CameraInfo topics. The paused preview alone
does not supply those streams. With that bench running in this container:

```bash
docker compose exec -T gazebo bash -lc 'source /opt/ros/jazzy/setup.bash; python3 -' < check_sensors.py
```

It checks message presence, advancing timestamps and dimensions, and reports
counts. Passing it does not establish stereo synchronization, calibrated
accuracy, sufficient estimator rate or flight readiness.

## 5. Optional PX4 and transport builds

Coordinate build ownership first. These commands compile tools and start no
PX4 runtime, agent transport or mission:

```bash
docker compose exec -T gazebo bash -s < bootstrap-px4.sh
docker compose exec -T gazebo bash -s < bootstrap-agent.sh
```

PX4 v1.16.1 uses a sparse partial checkout to avoid unused large test data. Its
Makefile needs explicit `j=2` to enforce two compile workers in a noninteractive
shell; ordinary `-j2` was not propagated to Ninja during the original attempt.
The agent is v2.4.3, using Jazzy's existing dependencies. Do not substitute a v3
agent for PX4's v2 client. See [PX4's compatibility guide](https://docs.px4.io/v1.16/en/middleware/uxrce_dds#micro-xrce-dds-agent-installation).

The PX4 helper skips `px4_msgs` by default. Only when assigned that build:

```bash
docker compose exec -T gazebo bash -s -- --with-messages < bootstrap-px4.sh
```

The source and installs persist under `state/home/`. Python build dependencies
in the container's virtualenv are recreated by the helper if the container was
removed. Do not copy PX4 simulator parameter files onto hardware.

## 6. Jetson follow-up and handoff

Before choosing a Jetson image, record the board model, `uname -m`,
`/etc/nv_tegra_release`, JetPack, CUDA, ROS and cuVSLAM versions. Preserve an
existing working installation and measured camera calibration. This generic
Mac simulation image is not a Jetson CUDA/cuVSLAM deployment.

A later split setup may send simulated stereo, CameraInfo, TF and `/clock` to
the Jetson and return continuous SLAM odometry. Confirm compatible ROS message
types, explicit networking/domain isolation, simulation time, stereo timestamps,
calibration, frames and stale-data rejection. Disable truth-derived odometry
producers when evaluating real SLAM; do not mix them with the measured estimate.
Keep map loop closures separate from continuous control odometry.

At handoff, record code revisions, settings, checks and failures; save logs and
stop your unnecessary preview/build workers. A simulator pass is not hardware
qualification. Arming requires the team's approved flight plan and fresh
telemetry. See [the project Gazebo documentation](../gazebo_sim/README.md) for
existing harness behavior, including its synthetic sensor assumptions.
