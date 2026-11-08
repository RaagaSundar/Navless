# Hand-carried stereo SLAM — Sky Beta 19

Open <http://127.0.0.1:8765/> on this Mac. The map, live camera, trajectory,
tracking health and PX4 fusion status are displayed together. The viewer uses
the Jetson's existing calibrated OAK-D-S2 and NVIDIA cuVSLAM 3.2.6, with stereo
cameras only; IMU fusion remains disabled as planned.

Double-click **Open SLAM.command** if the SSH tunnel needs reopening. It asks
for the Jetson's existing password when required; credentials are not stored.
The laptop and Jetson must be on a network that can reach skybeta-19.local.

## Modes: flight (default) and mapping

`handheld-slam.service` runs in one of two modes, set per drone with `HANDHELD_SLAM_MODE=` in
`/etc/skybeta/drone.env`. With nothing set, it uses `flight`.

| | `flight` (default) | `mapping` |
|---|---|---|
| Stack `competitor` window | **left alone**, so `COMPETITOR_PKG` (the team's flight code) runs | replaced by `sleep infinity`, so **no flight code runs** |
| SLAM | the stack's stock settings | localization + mapping, with visualisation |
| PX4 vision feed | the stack's stock bridge, or the gate with `HANDHELD_SLAM_PX4_GATE=1` (below) | fed through `odom_gate.py` |
| Runs | `odom_gate.py` only, in its own tmux window `gate` (→ `/mapping/validated_odometry`, used by `depthcloud/cloud_localize.py` and the planner), restarted by the service if it exits | `odom_gate.py` + `map_server.py` (viewer, ~50% CPU) as children of the service |

Flight mode also **undoes** a previous mapping session: if the competitor window is `sleep infinity`,
SLAM is in mapping mode, or the bridge is gated (without `HANDHELD_SLAM_PX4_GATE=1`), those windows are
restarted with the stack's own commands (`run_competitor.sh`, `run_slam.sh <camera pose>`,
`run_bridge.sh`). Restarting SLAM resets odometry, so only switch modes with the drone on the ground.
It's idempotent: windows that are already right are left alone, so the service's automatic restarts
never disturb a running stack.

### Feeding PX4 through the gate: `HANDHELD_SLAM_PX4_GATE=1` (flight mode, default 0)

- **Stock (0):** the bridge forwards every cuVSLAM sample. While tracking is lost (the dark tube, a
  covered lens), a frozen pose can keep arriving, so PX4 believes the drone is stationary, doesn't brake,
  and coasts blind.
- **Gate (1):** PX4 gets only samples that pass the gate (tracking, ≤ 250 ms old, sane). During a loss
  PX4 gets nothing, knows it's blind, and dead-reckons. The bridge is rerouted only after the gate is
  seen passing samples. If it isn't (cuVSLAM not tracking, CPU starved), PX4 stays on the stock feed and
  the service log says why.
- **Costs (a team decision):**
  - Team code sits in the VIO → PX4 path. The gate runs in its own window (`Ctrl-b`, window `gate`),
    so stopping or restarting this service doesn't cut PX4's vision, and the service restarts the
    gate within 5 s if it exits.
  - It's CPU-sensitive: at 5 Hz stereo plus the map viewer, sample age reached about 245 ms of its
    250 ms limit and most samples were dropped. Keep `stereo_cloud` at ≤ 3 Hz, run no `map_server.py`,
    and watch `/mapping/gate_status` `sample_age_ms` with the whole flight stack up.
  - With the magnetometer on (`EKF2_MAG_TYPE` 0), PX4 swings its heading about 90° after a 0.5 s vision
    gap (Gazebo finding). `EKF2_MAG_TYPE=5` avoids it.
- These are the manual tmux steps from the team's tube thread, automated.

Preview without changing anything:

```bash
HANDHELD_SLAM_DRYRUN=1 ~/team_ws/handheld_slam/run_mapping.sh
```

To make a map, set `HANDHELD_SLAM_MODE=mapping` in `/etc/skybeta/drone.env`, then run
`sudo systemctl restart handheld-slam.service`. Set it back to `flight` (or delete the line) and restart
the service before the drone flies. Mapping mode closes the flight-mode `gate` window, so only one gate
ever runs.

To install or update on a drone, copy `run_mapping.sh`, `container_mode.sh`, `stop_mapping.sh`,
`odom_gate.py`, `map_server.py` and `../navless_nav/navless_nav/navigation_integrity.py` to
`~/team_ws/handheld_slam/` (without `navigation_integrity.py` the gate refuses to start). Run the dry run above, then
`sudo systemctl enable --now handheld-slam.service` (or `restart`). All of this is on the ground,
disarmed.

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
  ROS DDS stays on the drone's ROS_DOMAIN_ID from drone.env (19 on skybeta-19, by the stack's
  convention; don't change it) with the existing localhost transport.
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
Each relayed sample must also continue the last one in source time (at most 2 m/s
and 3 rad/s, plus 5 cm / 0.05 rad): a jump or a backwards stamp **latches** the gate
and nothing more is relayed until it is restarted (`/mapping/gate_status` `fault`).
A SLAM restart or mode switch on the ground therefore needs a gate restart too
(`sudo systemctl restart handheld-slam.service`). With the stock bridge feeding PX4,
this protects only the gate's consumers (localizer, course mission), not PX4.
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
