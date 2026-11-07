# Gazebo base environment on Apple Silicon

Ubuntu 24.04 ARM64, ROS 2 Jazzy and Gazebo Harmonic run in Docker Desktop.
Gazebo uses Mesa software rendering; this image does not provide NVIDIA CUDA.

```bash
cd Navless/local_sim
bash start.sh
docker compose exec gazebo sim-course
```

Open [the desktop](http://localhost:6080/vnc.html?autoconnect=true&resize=scale).
The course opens paused, without an autopilot, flight brain or arming commands.
Use Ctrl+C in the scene terminal to close it. `docker compose stop` stops this
preview; `docker compose down` removes its container while retaining `state/`.

Read [INSTALL_FOR_AGENTS.md](INSTALL_FOR_AGENTS.md) before installation or
handoff. [VERIFICATION.md](VERIFICATION.md) records actual results and limits.

The repository is mounted read-only, except for generated run output. Local
state and raw logs are excluded from Git and the Docker build context. Ports
6080, 8081 and 8082 bind only to laptop loopback. ROS uses domain 219 and
localhost discovery. Coordinate ports, domains and CPU ownership with other
simulator users before starting the preview.

Optional PX4 and XRCE-DDS builds install tools without starting an autopilot:

```bash
docker compose exec -T gazebo bash -s < bootstrap-px4.sh
docker compose exec -T gazebo bash -s < bootstrap-agent.sh
```

PX4 is v1.16.1; the Jazzy-compatible agent is v2.4.3. `px4_msgs` generation is
skipped by default. Full-course flight and real cuVSLAM integration are outside
this base-install change. The existing simulator's truth-derived odometry is a
sensor model and does not validate visual SLAM. A flight requires the team's
approved plan and fresh telemetry, as described in the project instructions.
