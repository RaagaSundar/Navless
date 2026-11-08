# Installed drone 19 simulation bench

This records the installation verified on **2026-11-07**. Inspect live state
before acting: the recorded running/stopped state may have changed. This is an
operations guide for files already deployed on the Jetson, not an installer;
the runtime source and evidence described below are not bundled with this guide.

Gazebo Harmonic renders headlessly with OGRE2/EGL on the Jetson's NVIDIA Tegra
Orin GPU. Simulated stereo images feed actual stereo-only cuVSLAM, an independent
odometry gate, map viewer and stereo depth. Optional CAD localization remains
an observer. The stationary profile has **no PX4 autopilot, XRCE agent, external
vision bridge, commander, arming or setpoint publisher**. It does not establish
flight readiness or hardware qualification. A separate Mac PX4/split-Jetson
path was previously verified disarmed; that is not the profile described here.

## Isolation and installed paths

| Workload | Container / owner | ROS domain |
| --- | --- | --- |
| Physical aircraft | Existing `skybeta-stack`; leave untouched | 19 |
| Jetson-local stationary bench | `drone19-gazebo-bench` | 219, localhost DDS |
| Alternative Mac-rendered split bench | `drone19-split-sim`; coordinate with its owner | 219 |
| Other team's Mac fixture | `local_course`; coordinate with its owner | 220 |
| Other team's camera recording bench | `local_course/camera_bench*`; coordinate with its owner | 221 |

Only one domain-219 profile may run at a time. At verification, the local
bench was running, the split container was stopped, and our Mac renderer/PX4
were off. Domain separation does not isolate CPU, memory or GPU contention.

- Jetson source: `~/team_ws/split_sim`, mounted read-only at `/split`.
- Local bench launcher: `~/team_ws/split_sim/jetson_gazebo/start_remote.sh`.
- State, logs and maps: `~/team_ws/gazebo_bench_state`, mounted at
  `/root/gazebo_state`. Preserve these before replacing containers.
- Passive collector output: `~/team_ws/gazebo_bench_evidence/<label>`.
- Runtime image: `drone19-gazebo-deps:humble-harmonic`; recorded image ID:
  `sha256:86ef7cb0cad0612ff2f9ee1407a77fd87c1a5652dff145cc8871726ea6ce1bf2`.
- Runtime limits: 3 GiB RAM with no additional swap, four CPUs, 512 processes.
  Rendering uses NVIDIA runtime and `/dev/dri/renderD128,129`; there are no
  physical camera, USB or serial device mounts.

## Inspect and operate

Use the operator-provided password interactively; never save it in a script,
command line, repository or Slack message. From the laptop:

```bash
ssh -o PubkeyAuthentication=no -o PreferredAuthentications=password dronehacks@skybeta-19.local
```

First inspect on the Jetson, without restarting processes:

```bash
hostname
uptime
docker ps -a --format '{{.Names}} {{.Image}} {{.Status}}'
docker inspect drone19-gazebo-bench --format 'State={{json .State}} Image={{.Image}} Memory={{.HostConfig.Memory}} MemorySwap={{.HostConfig.MemorySwap}} NanoCpus={{.HostConfig.NanoCpus}} Devices={{json .HostConfig.Devices}} Env={{json .Config.Env}}'
```

Check that domain 219 is available and coordinate any profile switch with its
owner. If the existing local bench is stopped, start that container:

```bash
docker start drone19-gazebo-bench
```

Only if the container is absent and its source/image are present, the deployed
launcher creates it without compilation:

```bash
bash ~/team_ws/split_sim/jetson_gazebo/start_remote.sh
```

The launcher rejects a running `drone19-split-sim`. Starting an existing
container does not run that launcher check, so inspect first. To yield the
local bench, stop only its container; do not remove it or its state:

```bash
docker stop -t 10 drone19-gazebo-bench
```

For the split profile, its owner must coordinate the Mac renderer as well.
Do not stop the physical stack or another team's domain-220 fixture.

## Viewers and passive checks

Keep this tunnel open on the laptop; all laptop listeners bind to loopback:

```bash
ssh -o PubkeyAuthentication=no -o PreferredAuthentications=password \
  -o ExitOnForwardFailure=yes -N \
  -L 127.0.0.1:18765:127.0.0.1:8765 \
  -L 127.0.0.1:18081:127.0.0.1:18081 \
  -L 127.0.0.1:18082:127.0.0.1:18082 \
  dronehacks@skybeta-19.local
```

Open the SLAM map at `http://127.0.0.1:18765/`, stereo depth at port `18081`,
and optional CAD view at port `18082`. These laptop URLs require the tunnel.

On the Jetson, inspect ROS nodes, verify actual calibration, and collect a
passive 35-second sample. Choose a new label each time to preserve evidence:

```bash
docker exec drone19-gazebo-bench bash -lc 'export SKYBETA_UNDERLAY_ONLY=1; source /usr/local/bin/skybeta_env.sh; ros2 node list'
docker exec drone19-gazebo-bench bash -lc 'export SKYBETA_UNDERLAY_ONLY=1; source /usr/local/bin/skybeta_env.sh; python3 /split/jetson_gazebo/verify_calibration.py'
bash ~/team_ws/split_sim/jetson_gazebo/collect_remote.sh drone19-gazebo-bench "agent-$(date -u +%Y%m%dT%H%M%SZ)" 35
```

The deployed `loss_test.py` deliberately withholds images for six seconds.
It is a disruptive simulation sensor test, not a passive inspection command.
Coordinate it and use it only on the stationary bench without an autopilot or
commander; never apply it to physical aircraft sensors.

## Sensor and dependency contracts

The simulation-only `jetson_gazebo/native_sensor_bridge.py` consumes native
Gazebo `/clock`, left/right `/oak/*/image_raw`, and left/right
`/oak/*/camera_info`. It publishes standard ROS Clock, Image and CameraInfo,
plus `image_mono` aliases. Preserve actual source stamps, optical frame IDs,
L8 image bytes/dimensions/step, distortion model and D/K/P/R. Reject malformed
data. The model's `/split/unused_truth` topic is not connected to ROS; the
adapter must not feed truth pose/odometry or PX4 commands into the pipeline.

The simulated stereo pair is 640×400 at 10 Hz simulation time, with a 0.075 m
baseline, 80° horizontal FOV and Gaussian image noise standard deviation 0.007.
These are model assumptions, not measured aircraft calibration. IMU fusion is
disabled. Portable navigation and the independent odometry gate stay unchanged.

The Jetson image is Ubuntu 22.04 ARM64 / ROS Humble / Python 3.10 / glibc 2.35.
Do not copy Ubuntu 24.04/Jazzy/Python 3.12/glibc 2.39 binaries from the Mac
fixture into it. The native adapter alone uses
`PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python` for the installed Gazebo Python
bindings; do not replace global protobuf packages to bypass an ABI conflict.

## Recorded evidence and limits

The setup's saved evidence label is `jetson-gazebo-native-local-35s`, including
`measurement.json`; preserve the original evidence bundle alongside source
hashes, image metadata, renderer logs and settings when transferring results.

| Observation on 2026-11-07 | Result |
| --- | --- |
| Wall observation / images / raw odometry / validated odometry | 35.04 s / 323 / 315 / 310 |
| Image wall rate / simulation rate / simulation-to-wall clock ratio | 9.20 Hz / 10 Hz / 0.9204 |
| Tracking status | All 314 observed `vo_state` samples were 1 |
| Gate age in simulation time | Median 80 ms; p95 104 ms; maximum 108 ms |
| Same-stamp image-to-odometry callback arrival delta | Median 51.4 ms; p95 66.2 ms |
| Image-loss test, settled five-second window | Zero raw and validated odometry messages |
| Recovery after image restoration | 47 validated messages in five seconds |
| Resources | Peak container memory 1.217 GiB; mean CPU 280%; zero swap |

Gate age is simulation timestamp age. Callback delta is an observer's arrival
time difference affected by DDS scheduling. Neither is exact end-to-end wall
latency. A separate Wi-Fi/SSH sender-socket-to-receiver-socket measurement gave
17.7 ms median and 98.8 ms p95 over 240 images, with approximately 3 ms clock
offset uncertainty. Different clocks and workloads prevent assigning that
measurement as an additive portion of the earlier 124 ms gate age. The prior
split test also had concurrent build load, so this is not a controlled latency
comparison. CAD localization remained `weak` and did not feed flight control.

## Build and recovery guidance

The native adapter is operational without the unfinished optional C++
`ros_gz_bridge` build. Preserve its failed/partial build logs and caches; do
not silently retry heavy compilation. Jetson disconnects/reboots were observed
during earlier builds, but their cause was not established.

Coordinate before any future optional build, especially during a teammate's
flight window. Use a named build container capped at 2 GiB RAM with no extra
swap, two CPUs, one compile job and a 1.5 GiB per-process address-space limit.
Stop the entire build container/process tree when yielding and verify it has
stopped; canceling a Docker/BuildKit client alone is insufficient. Validate the
resulting ARM64 Humble ABI and real topic conversion before changing backends.
Do not change host power modes, drivers, firmware or PX4 parameters to make a
simulation or build pass.
