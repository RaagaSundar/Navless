# Practice launch 1 (skybeta-19, 2026-11-08, ~01:14–01:24 PST)

This branch is **exactly the code that flew** on skybeta-19 in the first autonomous practice runs
(commit `2ffac5d`, confirmed on the drone), plus that night's flight logs. Both runs crashed; see
[What happened](#what-happened) and [Known problems to fix](#known-problems-to-fix). It is the base for the
next round of changes.

- Code: `main` `f590e00` + `nav-stabilize-travel-heading` (travel heading with the tube flown facing north,
  hold drift correction, flight recorder).
- Logs: [`flight_logs/2026-11-08_skybeta19_course/`](flight_logs/2026-11-08_skybeta19_course/) (with its own README).

## What ran on the drone

One command in the stack's `competitor` tmux window:

```
ros2 launch navless_nav navless_course_slow.launch.py route_source:=file
```

| File | Role |
|---|---|
| `navless_nav/launch/navless_course_slow.launch.py` | Entry point: `navless_course.launch.py` at the slow speeds (0.5 m/s cruise). |
| `navless_nav/launch/navless_course.launch.py` | Course settings (route, net margin 0.25, ToF on); starts the ToF chain and includes `navless.launch.py`. |
| `navless_nav/launch/navless.launch.py` | Starts `flight_interface` (skybeta image, envelope 0.3–2.5 m, 1 m/s) and `course_mission`. |
| `navless_nav/navless_nav/course_mission.py` | The flight program: pre-arm checks, takeoff + settle, route carrot, heading (travel / tube north), drift correction, landing. |
| `navless_nav/navless_nav/route.py` | Route loading, course↔odom frames, the carrot, the net clamp; also builds the planner's map at startup. |
| `navless_nav/navless_nav/flight_log.py` | The flight recorder (CSV per tick + every log line, parameters JSON). |
| `navless_nav/navless_nav/tube_guidance.py` | ToF / stereo tube-centring filter (ToF centring was on; neither run reached the tube). |
| `navless_nav/navless_nav/pad_tracker.py` | Imported (pad vision is off by default). |
| `gazebo_sim/course/route_centreline_window_asbuilt.json` | **The route flown**: Raaga's new-maze window route, NE first leg, 286 waypoints, 14.1 m (`route_source:=file`: flown as exported, no planning/re-planning). |
| `planner/config.py`, `planner/grid_map.py`, `planner/fsm_router.py`, `planner/config/prior_course_map.json`, `dronehacks-course-asbuilt.obj` | Favour's planner and its map: **loaded at startup** (`create_live_course_planner`) but not used for the path with `route_source:=file`; re-planning needs `/team/stereo/points`, which was not running. |
| `tube/tof_driver.py`, `tube/tube_node.py` | Side ToF sensors → `/tube/offset` (both sensors were still mounted on one side). |
| skybeta stack (not in this repo) | uXRCE-DDS agent, cuVSLAM (`/visual_slam/tracking/odometry`, the only position source), `px4_odom_bridge` (vision → PX4 EKF2), `flight_interface`. |

**Not running:** `depthcloud/stereo_cloud.py`, `depthcloud/cloud_localize.py` (no wall-based localization; `wall_correction` false), `navless_flight_tests` (no hover/yaw/forward tests that night), any other team process (all stopped before the runs).

Effective settings (from `*_course_params.json`): `route_source file`, `heading_mode travel`, `hold_heading true`,
`drift_correction true`, `drift_apply_in_route true`, `tof_tube_centering true`, `wall_correction false`,
`replan_on_obstacles true` (no input), `cruise_speed_mps 0.5`, `net_margin_m 0.25`, `prearm_height_tol_m 0.30`,
`match_px4_height true`, `start_bearing_deg 0`.

## How to run it (exactly as on the night)

On skybeta-19's Jetson host (`ssh dronehacks@skybeta-19.local`, check `hostname` = `skybeta-19`), disarmed, props on only for the flight:

1. **Code onto the drone.** `~/team_ws/src/Navless` on this branch, then build:
   ```bash
   cd ~/team_ws/src/Navless && git fetch origin && git checkout practiceLaunch1 && git pull --ff-only
   docker exec skybeta-stack bash -lc "team_build.sh"
   ```
   (No internet on 19? From a laptop with the repo: `git bundle create pl1.bundle practiceLaunch1 ^origin/drone19/flight`,
   `scp` it to 19, then `git fetch /tmp/pl1.bundle practiceLaunch1:practiceLaunch1 && git checkout practiceLaunch1`.)
2. **Nothing else on the Jetson:** stop other containers (`docker ps`; keep `skybeta-stack`, `skybeta-ros-status`),
   no sims, no test nodes. Load raised cuVSLAM gaps and PX4 compass heading resets before.
3. **Pixhawk linked:** power-cycle it if `/fmu/out/vehicle_local_position` has no publisher.
4. **Restart the stack with the drone EXACTLY on the launch mark, nose EXACTLY north, still** (cuVSLAM's origin and
   heading are where it starts; the whole route is placed relative to it):
   ```bash
   docker rm -f skybeta-stack        # the skybeta-stack service recreates it; wait ~60 s
   ```
5. **Start the course program** (`drone.env` has `COMPETITOR_PKG=none`, so start it by hand):
   ```bash
   docker exec skybeta-stack tmux respawn-pane -k -t skybeta:competitor "bash -lc 'source skybeta_env.sh; source /root/team_ws/install/setup.bash; ros2 launch navless_nav navless_course_slow.launch.py route_source:=file'"
   docker exec skybeta-stack tmux capture-pane -p -t skybeta:competitor | tail -15
   ```
   Expect the route line (286 waypoints), `flight log: /root/team_ws/flight_logs/<time>_course.csv`, and
   `pre-arm height & zero check passed`.
6. **Fly:** pilot arms in Altitude, switches to Position; the program takes off, settles, flies, lands on the pad.
   Any mode change hands control back to the pilot. Disarm promptly after touchdown.
7. **Logs:** `~/team_ws/flight_logs/<time>_course.csv` + `_params.json` on the host.
8. **Afterwards:** competitor window idle with
   `docker exec skybeta-stack tmux respawn-pane -k -t skybeta:competitor "sleep infinity"`.

Recommended for the next attempt (launch arguments; not what ran): `heading_mode:=hold cruise_speed_mps:=0.3 tof_tube_centering:=false`.

## What happened

Details and numbers: [`flight_logs/2026-11-08_skybeta19_course/README.md`](flight_logs/2026-11-08_skybeta19_course/README.md).

- **Run 1** (`20261108-091401_course.csv`): takeoff slid ~40 cm left (west); the drift correction pulled it back
  and it settled in 14 s. It flew the NE first leg **on its planned line by odometry** (cuVSLAM tracking throughout)
  and hit a wall 1.6 m in. Most likely the route was not aligned with the real course (nose not exactly north / not
  exactly on the mark at stack start): ~25 cm of planned room there, and nothing corrects the start pose.
- **Run 2** (`20261108-092321_course.csv`): armed further back than the mark (the route moves with the arming
  pose). Through the 1 m gap at MD's tip; turning east it swung **18 cm outside its setpoint** (north) while lagging
  ~25 cm, side-on to the north net (travel heading, ~17 cm planned), and hit it. **cuVSLAM then jumped 1–2 m per
  0.1 s while still reporting tracking; the program kept flying the route for 20 s** (PX4 accepted the jump after
  1.6 s and climbed to ~2.8 m) until the 20 s no-progress HOLD.

## Known problems to fix

1. **No crash / odometry-jump protection.** An odometry step faster than the drone can move (e.g. > 0.5 m in one
   0.1 s tick) or a sustained large error to the target should end the route and descend in place (like `REFUSED`).
2. **Corner overshoot at the maze gap** (and other turns): the drone runs ~18 cm outside a turn at 0.5 m/s. Needs a
   path with more room at the turn past MD's tip / slower into turns (planner side), and/or lower cruise speed.
3. **Travel heading puts the wide side (props 0.2375 m each side vs 0.1575 ahead) toward walls** on legs that pass
   close: 16.8 cm planned at the north net vs 19.6 cm nose-north. Not Gazebo-gated.
4. **Start pose / alignment:** the route is placed relative to cuVSLAM's origin; misplacement or heading error goes
   straight into wall clearance. The wall correction (`depthcloud/cloud_localize.py` → `wall_correction:=true`)
   is the fix but is not validated on the real course (walk test first).
5. **ToF tube centring was on** with both sensors on one side: launch with `tof_tube_centering:=false` until moved.
6. Favour's planner (`route_source:=planner`) came within ~13 cm of MD's tip in the stub check: tighter still.
7. 19's PX4 failsafe actions (`COM_OBL_RC_ACT`, `NAV_RCL_ACT`, `COM_RCL_EXCEPT`) are unknown; PX4 RTL would climb
   into the net. Read them in QGroundControl (do not change without the organisers).

## Tests

`python -m pytest navless_nav/test gazebo_sim/dh/test_route_export.py` → 186 passed, 1 skipped (laptop, stub
ROS). `tests/test_planner.py` → 18 passed. Not Gazebo-gated. A stub full-course check (perfect tracking) gave
closest walls 16.8 cm (file + travel), 19.6 cm (file + hold), ~13 cm (planner).
