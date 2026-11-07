# Verification — 2026-11-07 Pacific

The original installation ran on an M1 Max Mac with 32 GB RAM and an 8 GB ARM64
Docker VM. The image provided Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic 8.15.0,
OpenCV 4.6.0, NumPy 1.26.4 and Mesa llvmpipe OpenGL 4.5.

Observed before PR packaging:

- The existing course rendered in the noVNC desktop.
- SDF validation returned `Valid.` with the model lookup path supplied.
- Project tests passed: 15 passed, 1 skipped. The existing skip required an
  absent `dronehacks-course-v3.obj`.
- A stationary quad without PX4 or a flight brain supplied stereo and clock
  through ROS. The exact passive observation is in
  [verification/ros-sensors-2026-11-07.json](verification/ros-sensors-2026-11-07.json).
  Clock advanced 16.572 simulation seconds in 20.001 wall seconds. Image
  delivery was 25 left / 29 right frames, below the configured simulation rate.
  This verifies stream delivery, not real-time SLAM performance or calibration.
- PX4 v1.16.1, commit `94cb2012792b2ae89f0b147cfee53ee31ae550be`, built with
  `make px4_sitl_default j=2`. Its ARM64 binary linked with resolved dependencies.
  Binary SHA-256: `1f1faaaa343616a81de0a7cb8ee8f1ac69c70b315aaafc756fc8e4e99c290464`.
- XRCE-DDS Agent v2.4.3, commit
  `73622810d984349b80bbac0ef55fc0b694d62222`, built using Jazzy dependencies and
  started idle on UDP 8888. PX4 runtime/ROS telemetry was not tested here.

The first full PX4 clone timed out. A partial sparse checkout succeeded. The
first compile was interrupted to correct its worker limit. Later, after make
completed with exit 0, the bootstrap wrapper was deliberately terminated before
its queued message build to avoid overlapping another agent's assignment. No
`px4_msgs` build or flight ran in this base-install work. Raw build/failure logs
remain local under the original installation's ignored `evidence/` directory.

The original runtime used the team workspace at commit
`3d9371128664b9b4537edf0cbe8474e9515930af`, with contemporaneous teammate edits.
PR packaging starts from main `6a12608`; its unit, configuration and path checks
are recorded with the PR. No full-course success or Jetson qualification is
claimed. The preview was stopped to free resources for the separately owned
full-course simulation.

PR packaging checks on main `6a12608`: the same unit suite passed (15 passed,
1 skipped); shell/Python syntax, Compose configuration, repository source
resolution, read-only mounting, ARM64 platform and loopback port bindings passed.
The first packaging SDF check was interrupted when the shared Docker daemon
became unresponsive. After coordinated VM recovery, the SDF check was repeated
successfully against this PR checkout and returned `Valid.` with exit code 0.
The check used a read-only repository mount, no network, a 0.5 CPU limit and a
256 MiB memory limit. No new preview, source build or flight was started while
packaging. The original runtime observations and their limits remain above.
