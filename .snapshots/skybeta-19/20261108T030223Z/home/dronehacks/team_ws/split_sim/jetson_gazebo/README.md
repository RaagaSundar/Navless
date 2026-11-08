# Operational Jetson-local Gazebo + cuVSLAM stationary bench

Gazebo Harmonic renders on drone 19's **NVIDIA Tegra Orin GPU**. A small typed
simulation IO adapter passes actual camera images, CameraInfo and simulation
clock to ROS Humble. The existing stereo-only cuVSLAM, independent odometry
gate, map viewer, stereo depth and CAD localization workload run alongside it.
The 35-second stationary test and camera-loss/recovery test passed.

The active container is `drone19-gazebo-bench`, using
`drone19-gazebo-deps:humble-harmonic`, image
`sha256:86ef7cb0cad0612ff2f9ee1407a77fd87c1a5652dff145cc8871726ea6ce1bf2`.
The original `drone19-split-sim` is stopped; our Mac renderer and PX4 are off.
The physical aircraft stack and ROS domain 19 were not changed. This local
bench has **no PX4 process, XRCE agent, EV bridge or commander**. The previous
Mac-PX4/split-Jetson path was separately verified disarmed by the parent agent.
This stationary simulation is not hardware or flight qualification.

## Recorded results

`../evidence/jetson-gazebo-native-local-35s/measurement.json` records:

| Quantity | Jetson-local native adapter |
|---|---:|
| Observation | 35.04 s wall time |
| Images / tracking odometry / validated odometry | 323 / 315 / 310 |
| Image / odometry / validated wall rates | 9.20 / 9.20 / 9.19 Hz |
| Image and odometry simulation rate | 10 Hz |
| Simulation / wall clock rate | 0.9204 |
| Tracking status | All 314 samples `vo_state=1` |
| Gate age, simulation milliseconds | median 80; p95 104; max 108 |
| Same-stamp image→odom callback arrival delta | median 51.4; p95 66.2; max 89.3 ms |

The callback delta is an observer's arrival-time difference, not exact tracker,
render-to-odometry or network latency. DDS scheduling can affect it. Gate age
uses simulation time; it is not wall time.

The earlier 35-second Mac→Jetson baseline delivered 6.44 Hz images, clock ratio
0.6435, and median/max gate simulation age 112/168 ms. It included concurrent
build load and preceded removal of an extra Mac ROS/Gazebo bridge. These are
observed results under different loads, **not a controlled causal comparison**.
They support operating this local alternative near real time, but do not
establish a precise end-to-end latency improvement or isolate network cost.

The full local renderer log identifies `GL_VENDOR = NVIDIA Corporation`,
`GL_RENDERER = NVIDIA Tegra Orin (nvgpu)/integrated`, OpenGL 4.5 NVIDIA 540.5.0.
A separate bounded native camera probe captured 101 640×400 mono frames over
12.05 seconds, at 9.96 wall Hz / 10.02 simulation Hz, and exited cleanly.

Full-bench samples: mean CPU 280% (about 2.8 cores), peak container memory
1.217 GiB, host GPU temperature at most 55.44°C, zero swap. The container is
capped at 3 GiB RAM, no extra swap, four CPUs and 512 processes. The physical
stack remained concurrent. No power settings or host drivers were changed.

Camera loss test: images were withheld for six wall seconds while actual
clock and CameraInfo continued. During the settled five-second loss window,
raw and validated odometry each had **zero** messages. After restoring images,
47 validated messages arrived in five seconds. The gate was unchanged.

Actual static TF and source projection independently give a 0.075 m stereo
baseline. Frame IDs match `oak_left_camera_optical_frame` and
`oak_right_camera_optical_frame`; K/P/R/D and dimensions are copied from native
Gazebo CameraInfo. The saved stereo cloud has 14,395 points. Optional CAD
localization reported `weak` (fitness about 0.648, RMSE 0.0247 m); this result is
preserved, not presented as a qualified global pose. Its truth field was null.

## Operation

All commands below run on the Jetson in an already authorized session. The
parent `jetson_sim/start.sh` manages laptop SSH forwards and detects an active
local bench, keeping Mac rendering off. With those forwards the map viewer is
at `http://127.0.0.1:18765`, stereo at port 18081 and CAD view at port 18082.

To restart this existing local container:

```bash
docker start drone19-gazebo-bench
```

For a new deployment, coordinate domain 219 with the split owner first, stop
only `drone19-split-sim`, then:

```bash
bash ~/team_ws/split_sim/jetson_gazebo/start_remote.sh
```

The default backend is `SENSOR_ADAPTER=native`, using the existing dependencies
image. No new compilation is required. `run_inside.sh` uses headless OGRE2/EGL,
starts the world and stationary quad, then launches the native sensor adapter,
unchanged `/split/sim_slam.launch.py`, gate/viewer, stereo cloud and optional
CAD localization. Files are mounted read-only at `/split`; state/logs are in
`~/team_ws/gazebo_bench_state`. No USB, serial or camera devices are mounted;
only NVIDIA runtime and `/dev/dri/renderD128,129` provide rendering access.

To return to the original split pipeline, coordinate with its owner, stop
`drone19-gazebo-bench`, start `drone19-split-sim`, and have the Mac owner restart
only their domain-219 renderer. Do not touch the other chat's `local_course`,
domain 220, or its flight container.

## Sensor contract and verification

`native_sensor_bridge.py` subscribes to exactly five native Gazebo topics:
`/clock`, left/right `/oak/*/image_raw`, and left/right `/oak/*/camera_info`.
It publishes standard ROS Clock, Image and CameraInfo. Images also receive
`image_mono` aliases for SLAM/depth. It preserves source stamps/frame IDs,
640×400 L8 bytes/step, distortion model and actual D/K/P/R arrays. It rejects
unexpected pixel formats, malformed dimensions/calibration or missing frame
IDs. It never subscribes to pose, odometry, truth, PX4 or command topics.
The model's truth topic `/split/unused_truth` is not connected to ROS.

The simulated camera uses a 7.5 cm baseline, body mount (0.1053, ±0.0375,
0.0304) m, 10 Hz, 80° horizontal FOV, Gaussian image noise stddev 0.007,
and a textured course. This is generated simulation calibration, not measured
hardware calibration. IMU fusion remains disabled. Portable SLAM/navigation and
the independent validity gates were not modified.

The adapter alone sets `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python`. This
handles the base image's newer Python protobuf runtime with OSRF's old generated
Gazebo Python messages. No packages are changed. `pause_images` in its state
directory is only a simulation producer loss-test hook; navigation does not
read it.

```bash
bash ~/team_ws/split_sim/jetson_gazebo/collect_remote.sh drone19-gazebo-bench local-35s 35
docker exec drone19-gazebo-bench bash -lc 'export SKYBETA_UNDERLAY_ONLY=1; source /usr/local/bin/skybeta_env.sh; python3 /split/jetson_gazebo/verify_calibration.py'
# Stationary simulation only; temporarily withholds camera output for six seconds.
docker exec drone19-gazebo-bench bash -lc 'export SKYBETA_UNDERLAY_ONLY=1; source /usr/local/bin/skybeta_env.sh; python3 /split/jetson_gazebo/loss_test.py'
```

Evidence under `../evidence/jetson-gazebo-native-*` includes measurements,
resources/tegrastats, complete renderer/node logs, native source calibration,
actual static TF, loss/recovery events, camera image, depth PLY and final state.
`jetson-gazebo-sha256.txt` records exact source/assets; runtime metadata records
image, environment, limits, devices and state. Four sensor-protocol unit tests
and Python/shell syntax checks passed. Initial pytest was unavailable locally;
the protocol's native unittest suite was used.

## Optional official C++ bridge: unfinished, not required

The initial binary build failed: OSRF Jammy arm64 had no
`ros-humble-ros-gzharmonic`. An official source build at commit
`856e319ee4da713609b3f6e58c74ac8db7662482` then exposed mixed protobuf26 headers
in `/usr/local` versus Jammy protobuf3.12/`libprotobuf.so.23`. The corrected
build passed that error, but the target became unreachable and rebooted twice.
The cause is unknown; previous-boot journals were not persistent and pstore was
empty. Heavy Jetson compilation remains discontinued.

The Mac ARM64 Jammy/Humble offload compiled interfaces, then was interrupted
for another chat's recorded flight window. Mac Docker subsequently became
unresponsive amid reported OOM events. Preserve those partial-build logs and
caches. No completed C++ bridge artifact is claimed or used by the operational
native adapter. No Mac Docker actions may resume until the coordinator releases
the window.

For a future optional build, `BridgeBuilder.Dockerfile` now materializes only
pinned ARM64 Jammy/Humble dependencies/source. `build_bridge_mac.sh` compiles
with one job in a named container capped at 2 GiB RAM/no extra swap, two CPUs
and a 1.5 GiB per-process address space. It can be stopped as a complete process
tree with `docker stop -t 5 drone19-bridge-compile`. This replaces the earlier
unbounded BuildKit compiler RUN. These revised optional scripts have syntax
validation only and have not been executed.

`export_bridge.sh` exports the successful stopped container's install tree,
package versions, commit and SHA256 tarball. Transfer only that artifact to the
Jetson under `~/team_ws/split_sim/jetson_gazebo/bridge_install`. The optional
`Runtime.Dockerfile` and `build_remote.sh` assemble it on the cached dependency
image with no compilation. Then validate runtime link/ABI and actual topic
conversion before selecting `SENSOR_ADAPTER=ros_gz` and
`GAZEBO_IMAGE=drone19-gazebo:humble-harmonic`. The native path does not depend on
this work. Never substitute Mac Jazzy/glibc2.39 libraries for Jammy/Humble.

The cached dependency image uses glibc 2.35, Python 3.10.12, ROS Humble rclcpp
16.0.15, Gazebo sim 8.15.0, msgs 10.4.0, transport 13.6.0 and protobuf 3.12.4.
References: [official headless OGRE2/EGL](https://gazebosim.org/api/sim/8/headless_rendering.html),
[official ROS/Gazebo combinations](https://gazebosim.org/docs/harmonic/ros_installation/).
