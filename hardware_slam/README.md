# Hand-carried stereo SLAM — Sky Beta 19

Open <http://127.0.0.1:8765/> on this Mac. The map, live camera, trajectory,
tracking health and PX4 fusion status are displayed together. The viewer uses
the Jetson's existing calibrated OAK-D-S2 and NVIDIA cuVSLAM 3.2.6, with stereo
cameras only; IMU fusion remains disabled as planned.

Double-click **Open SLAM.command** if the SSH tunnel needs reopening. It asks
for the Jetson's existing password when required; credentials are not stored.
The laptop and Jetson must be on a network that can reach skybeta-11.local.

## Make a map

1. Keep the drone disarmed, with props removed for hand-carried testing.
2. Start with the drone level and the camera looking at a textured scene.
3. Walk slowly with overlapping camera views; avoid pointing only at blank
   walls, covering the camera, rapid spins or motion blur.
4. Revisit the starting view to help loop closure. Tracking must show TRACKING.
5. Click **Save SLAM map** and wait for **saved**. **Export PLY** downloads the
   currently displayed landmarks; **Snapshot JSON** exports the displayed map
   and trajectory.

This is a **sparse visual landmark map**, not a dense mesh or an occupancy map.
Unobserved space is unknown. The tracker does not use an external north or
gravity reference. Start level; this setup has not been qualified for flight.

## Data and processes

- Jetson team files: `/home/dronehacks/team_ws/handheld_slam/`.
- Saved sessions: `maps/<UTC timestamp>/` within that directory. Each contains
  `cuvslam/data.mdb`, `landmarks.ply`, `snapshot.json`, and `save_result.json`.
- Saved maps and raw recordings are excluded from Git. Laptop backups remain
  in the original local workspace; this repository contains source only.
- `handheld-slam.service` starts the mapper on Jetson boot and restarts its two
  team nodes if they exit. It reconfigures the SLAM/bridge panes in the existing
  container and keeps the XRCE agent running. The organisers' checkout and
  monitoring are unchanged.
- The example competitor/flight interface is stopped while this mapping mode
  runs. No arming, offboard heartbeat, flight setpoint or mode requests are sent.
- Viewer HTTP binds only to Jetson loopback and reaches the Mac over SSH.
  ROS DDS stays on domain 19 with the existing localhost transport.
- Check service: `systemctl status handheld-slam.service` on the Jetson.
- Restart only the team nodes: `sudo systemctl restart handheld-slam.service`.
  This preserves the current tracker/map and serial agent.

## PX4 path

`/visual_slam/tracking/odometry` (continuous odom → base_link, ROS FLU)
→ independent `odom_gate.py`
→ `/mapping/validated_odometry`
→ the existing `px4_odom_bridge`
→ `/fmu/in/vehicle_visual_odometry`
→ existing Micro XRCE agent / TELEM2 / PX4.

Map-loop-closure poses are used only for visualization. PX4 receives continuous
tracking odometry through the original frame conversion, with arbitrary-heading
world FRD (`pose_frame=2`). The original bridge's tracking-reset handling stays
in place. The gate rejects stale, out-of-order, failed-tracking, wrong-frame,
nonfinite and invalid-covariance estimates. The stamp must be at most 250 ms old
and no more than 50 ms ahead; tracking status must be fresh within 250 ms.
Visualization and exports run in another process and cannot block the gate.

The measured body→camera mount is preserved: translation
`(0.1053, 0, 0.0304)` metres FLU; rotation `(0, 0, 0)` radians, as provisioned.
Camera calibration, IMU settings, PX4 parameters and firmware were not changed.
Vision horizontal position and yaw fusion were observed active. Vision height
and velocity fusion are **not** enabled in the existing PX4 configuration.
PX4's height estimate therefore comes from its other configured sensors.

Powering off Pixhawk stops its telemetry while mapping continues. The viewer
marks telemetry stale instead of claiming current fusion. When powered back on,
the existing transport reconnects; wait for Connected and fresh fusion status.

## Verification and evidence — 2026-11-06 Pacific

The original bridge tests passed before changes: **23 passed**. Final checks
also passed: **23 bridge tests + 11 new relay validation tests**. New tests use
isolated ROS domain 219 and never inject estimates into the aircraft domain.

Recorded evidence is retained outside Git in the original workspace’s
`hardware_slam/evidence/`, including runtime settings, health checks,
passive 35-second observations, and test output. The first viewer attempt failed
to JSON-serialize a NumPy float from PX4; the original failure log is retained.
Converting message-array elements to Python floats corrected it. A later
observation coincided with Pixhawk being powered off: ROS odometry continued
at ~29 Hz while FC telemetry stopped. That recording is retained separately.

The first saved map contained **23,213 landmarks and 7,047 trajectory poses**,
with a trajectory span of approximately **6.78 × 8.15 × 3.18 m**. Those spans
are camera-derived map coordinates, not surveyed room dimensions. Subsequent
mapping continues in the running tracker. No flight was commanded or tested.

The final powered-PX4 observation lasted **35.0 s**: visual odometry output
**28.43 Hz**, PX4 local-position output **92.17 Hz**, vehicle disarmed,
vision XY/yaw fusion active, and XY/Z validity true. Validated sample age was
**59.89 ms median / 83.80 ms p95**. See
`evidence/verification-powered-px4-35s.json`. This measures the bench pipeline;
it does not establish safe control performance or airborne tracking reliability.

Useful references: [NVIDIA's version-matched SLAM interfaces](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html)
and [PX4 external estimation interfaces](https://docs.px4.io/v1.16/en/ros/external_position_estimation.html).
