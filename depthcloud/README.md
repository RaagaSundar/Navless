# depthcloud: stereo depth + point cloud

Owner: Shining (`@depthcloud`)

`stereo_cloud.py` is a ROS 2 node that reads the skybeta stack's raw OAK-D-S2 stereo pair and computes depth on the CPU (OpenCV SGBM). It never opens the camera and never publishes to `/fmu/in/*`.

| Output | Type | Notes |
|---|---|---|
| `/team/stereo/points` | `sensor_msgs/PointCloud2` | left camera optical frame, 0.3–5 m |
| `/team/stereo/depth` | `sensor_msgs/Image` 32FC1 | metres, 512x320 (rectified left) |
| `http://<drone>:8081/` | browser | rectified left + colour depth, and a 3D cloud viewer |
| `http://<drone>:8081/cloud.ply` | download | latest cloud for CloudCompare / MeshLab |

Inputs: `/oak/left/image_raw`, `/oak/right/image_raw`, `/oak/left/camera_info`, `/oak/right/camera_info`, plus TF between the two camera frames. Without that TF it falls back to parallel cameras 7.5 cm apart.

## Run on the drone

On the drone, `~/team_ws` is bind-mounted into the stack container at `/root/team_ws`. Copy the file there:

```bash
scp depthcloud/stereo_cloud.py dronehacks@skybeta-19.local:~/team_ws/src/dronehacks
```

Then, on the drone in `~/skybeta`, start the stack if `docker ps` doesn't show `skybeta-stack`:

```bash
SKYBETA_DETACH=1 host/run.sh stack
```

Open a shell in the running stack:

```bash
host/run.sh shell
```

Check the camera is streaming (about 30 Hz), then start the node:

```bash
ros2 topic hz /oak/left/image_raw
```

```bash
python3 /root/team_ws/src/dronehacks/stereo_cloud.py
```

## Parameters

Pass with `--ros-args -p name:=value`.

| Parameter | Default | |
|---|---|---|
| `rate_hz` | 5.0 | processing rate |
| `scale` | 0.5 | image scale before matching |
| `cloud_stride` | 2 | keep every Nth point in the cloud |
| `num_disparities`, `block_size` | 64, 5 | SGBM settings |
| `min_depth`, `max_depth` | 0.3, 5.0 | metres |
| `fallback_baseline_m` | 0.075 | used only if the camera-to-camera TF is missing |
| `require_calibration_tf` | false | reject image pairs when stereo extrinsics TF is unavailable; required for the experimental tube-centering pipeline |
| `texture_filter` | false | reject low local contrast pixels that SGBM may fill across an opening; use true for tube centering |
| `http_port` | 8081 | browser view |
| `max_view_points` | 40000 | points sent to the 3D viewer per update |

## Localization against the course model: `cloud_localize.py`

This runs alongside `stereo_cloud.py`, which it doesn't modify. It matches each `/team/stereo/points` cloud to an .obj using point-to-plane ICP, starting from the previous pose, and outputs:

- `/team/localize/pose` (`geometry_msgs/PoseStamped`, `frame_id: course`): the camera's pose in the .obj frame, Z up, metres
- `http://<drone>:8082/`: a 3D viewer of the best estimate. It shows the model (ceiling hidden) and the live cloud placed in it (green matched the model, orange didn't). The camera is drawn as a dot with a field-of-view wedge, a heading arrow on the floor and a trail. A HUD shows the position in feet and metres, the heading, and what the latest frame actually **measured** versus **held** (x, y, height, heading). It's coloured green when everything is measured, yellow when partly held (with a dashed ring if horizontal position is held), and grey when stale (no accepted frame for over 2 s). There's also a collapsible form to reset the pose. If `skybeta-19.local` doesn't open in the browser, use the drone's IPv4 address (DHCP: check with `ip -4 -br addr show wlP1p1s0` on the drone), as `http://<address>:8082/`.

It needs a starting pose, and then it tracks frame to frame, so move slowly. It reports the camera's position, not the drone's centre.

Check the maths offline first (no ROS needed):

```bash
python3 cloud_localize.py --selftest test_room.obj
```

### Two courses: as-built (real) and plan (FlyX Lab)

The organisers' `course_v3.json` / `dronehacks-course-v3.obj` were a **temporary plan**. The real course, from a phone LiDAR scan on 2026-11-06, matches the plan for the net, the barrier line, the window wall C2a, the tube and the boulders (within about 2–6 cm). But five walls are elsewhere:

| Wall | Moved | Leans |
|---|---|---|
| M1 | 17 cm | 8° |
| M2 | 29 cm | 5° |
| M3 | **76 cm east** | 2° |
| A1 | 45 cm | 13° |
| C1 | 9 cm | 13° |

FlyX Lab still simulates the plan. So there are two courses, and every tool picks one:

| Course | json | .obj | For |
|---|---|---|---|
| `asbuilt` | `gazebo_sim/course/course_asbuilt.json` | `dronehacks-course-asbuilt.obj` | the real drone, and our own sims of it |
| `plan` | `gazebo_sim/course/course_v3.json` | `dronehacks-course-v3.obj` | FlyX Lab |

- **How the as-built course is made:** `asbuilt/make_asbuilt.py` takes the plan plus the scanned walls (`asbuilt/scan_walls.json`, fitted by `asbuilt/scan/`). It writes the as-built json and, given the plan .obj, the as-built .obj. It needs only Python 3, no numpy:
  ```bash
  python3 depthcloud/asbuilt/make_asbuilt.py --obj path/to/dronehacks-course-v3.obj
  ```
  The .obj lands next to the plan's. The as-built .obj is also committed, at the repo root (`dronehacks-course-asbuilt.obj`), so you only need to re-run this after changing `scan_walls.json`. The plan .obj comes from the organisers' zip and isn't in git. `asbuilt/asbuilt_vs_scan.png` shows the as-built walls over the scan.
- **What it recomputes:** the moved walls keep their names. The floor line, gate A and the wall-bounded checkpoint zones are worked out from the walls, using the same rule that reproduces the plan's own floor line exactly.
- **Status:** Shining confirmed the as-built walls against the real course on 2026-11-07, so this is the course to localize, simulate and plan routes on. The one computed part is the floor line, because the tape doesn't show in the scan.
- **Maze changed later on 2026-11-07.** The organisers removed M1, M2 and M3 and put up three new walls, measured from Shining's sketch:

  | Wall | From → to (m, x east / y north) |
  |---|---|
  | north wall, along the net | (−2.547, 2.985) → (−0.830, 2.985) |
  | long diagonal, tip 1 m from the north net | (−1.996, 0.508) → (−0.169, 2.048) |
  | wall to the tube | (−0.182, 0.508) → (1.535, 1.352) |

  - **The change is data:** `asbuilt/maze_2026-11-07.json`. `make_asbuilt.py` applies it after the scan walls (`--changes ''` skips it).
  - **`course_asbuilt.json`:**
    - M1, M2 and M3 move to `removed_walls`; `MN`, `MD` and `MT` are added;
    - **gate A** is now the maze exit: the 1 m gap between MD's tip and the north net;
    - the checkpoint zones are redrawn roughly around the new walls.
  - **The repo-root `.obj`** is regenerated the same way. It matches the first hand-made version (`asbuilt/maze_change_2026-11-07.js`, top view `.svg`) to within 0.12 mm.
  - :warning: **The taped floor line was not re-laid, so it's the old maze's tape.**
    - Its southward leg at x ≈ −0.85 runs under the new diagonal (MD) at about (−0.85, 1.47), and its top stretch (y ≈ 2.79) passes about 14 cm from the new north wall.
    - So `floor_line_in` is **removed**. The tape is kept as `floor_tape_in`, for reference only (a camera sees it), with a note: **do not follow**.
    - **Plan the maze path from the walls.**
    - `walk_test.py` uses the tape corners only as known floor spots.

`cloud_localize.py` uses the **as-built** course by default: `-p course:=asbuilt` (default) or `plan`. It finds the .obj next to the script, in `~/team_ws/src/dronehacks`, or in the repo root; `-p obj:=...` overrides it. `walk_test.py --course asbuilt|plan` works the same way.

Simulated on the as-built course (2026-11-07, generated camera data, so this tests the matching on the new walls, not real stereo):

| Test | As-built | Plan, same test |
|---|---|---|
| Self-test, 40 random views | 24 pass / 9 close / 5 held / **2 bad** | 28 / 7 / 5 / 0 bad |
| Bench, drone still on the launch mark | 0.0–0.3 cm, fit 0.75–0.81 | 0.0–0.2 cm |
| Window route (test route from `course_asbuilt.json`), facing travel, drifting odometry, 2 seeds | median 4–5 cm, 90% 8–10 cm, max 18–23 cm; heading median 1.5° | median 3–4 cm, max 9–11 cm |
| Same, with 15% VSLAM dropouts + 2 reset jumps | median 10 cm, 90% 22 cm, max 40 cm | median 8–10 cm, max 23–37 cm |
| Same route, odometry only (no course matching) | median 26 cm, max 143 cm | |

- **The 2 bad views are borderline:** both have status `weak` and fewer than 100 matched points. One looks north-west from beside A1, the other looks at the tube from the south-east. Treat `weak` as "don't steer on it", as the planner already does (it fuses only `tracking`).
- **Flying the plan's route through the as-built walls:** 26 of its 341 points come within 24 cm (the drone's half-width) of a wall, and its centre line passes **3.6 cm** from one. The drone needs a route exported from the as-built course.

### Running it

On the drone, with `stereo_cloud.py` already running, start it against the course model (metres, Z up, origin at the centre of the floor). With no `init_*` parameters, the **drone's centre (`base_link`)** starts on the model's **launch mark** (−2.515, 0.914), standing at `init_base_z` (0.07 m, like `navless_nav`'s `start_height_m`), facing **north (90°)**. The drone is always placed facing forward along the guideline, which runs north from the mark. The camera is placed from that through the `base_link` → camera mount from TF, on the first cloud. If TF has no mount after 3 s, it uses `mount_fallback` (Koji's measured `0.1053,0,0.0304`). `-p init_z:=<camera height>` gives the old behaviour, putting the camera itself at (`init_x`, `init_y`, `init_z`).

```bash
python3 /root/team_ws/src/dronehacks/cloud_localize.py                         # as-built course
python3 /root/team_ws/src/dronehacks/cloud_localize.py --ros-args -p course:=plan   # FlyX Lab's plan
```

Start it with `docker exec -d ...` or in its own shell: a background `&` inside a plain `docker exec` is killed when that exec ends.

**Odometry fusion:** each frame, the camera's motion since the previous frame comes from the stack's Isaac ROS Visual SLAM. It's applied first as a prediction, then ICP corrects it against the course. When nothing matches (looking through the net), the estimate keeps moving on odometry, with status `odometry` shown in blue. Turn it off with `use_odometry:=false`.

- **Source:** Koji's fail-closed gate, `/mapping/validated_odometry` (`hardware_slam/odom_gate.py`). Only samples that passed its checks are used: tracking OK, at most 250 ms old, right frames, finite, valid covariance. If that topic is silent for more than 2 s, it falls back to raw TF `odom` → camera, but only while `/visual_slam/status` reports tracking.
- **Gaps:** after untrusted frames, the next trusted one bridges the gap, because VSLAM's frame is continuous. Gaps over 3 s restart the chain.
- **Resets:** a step faster than 3 m/s (or 360°/s) is treated as a VSLAM reset and rejected, and the last good step is repeated in its place, so a mid-turn reset doesn't freeze the heading.

**Outputs for consumers:**

| Topic | Type | Contents |
|---|---|---|
| `/team/localize/pose` | `PoseStamped`, `frame_id: course` | the left camera's pose; orientation = camera forward/left/up |
| `/team/localize/base_pose` | `PoseStamped`, `frame_id: course` | `base_link` (drone centre), via the static TF `base_link` → camera; use this one for planning |
| `/team/localize/status` | `std_msgs/String` (JSON) | `status` (`tracking` / `weak` / `odometry` / `holding` / `lost` / ...), `measured` (fraction of x / y / height / heading the frame measured), `all_measured` (every direction measured), `age` (s since the last course match), `odom_source`, `odom_rejects`, `odom_correction` (see below) |
| `/team/localize/odom_correction` | `PoseStamped`, `frame_id: odom` | drift correction for odometry-frame flight (navless_nav): planar transform D, with `position.x/y` = dx/dy and `orientation` = dyaw about z. Published only while valid. |

**Drift correction for navless_nav (`odom_correction`).** navless_nav maps course points into cuVSLAM's odometry frame once, from the launch mark and the start bearing (`route.course_to_odom`). As cuVSLAM drifts, or if the drone was placed slightly off the mark, the route slides relative to the real walls. The localizer measures that slide, comparing its course-matched pose with odometry at the same instant, and publishes the correction:

```
corrected_odom_point = D * course_to_odom(p)       # x, y and heading; z unchanged
D = (dx, dy, dyaw) from /team/localize/odom_correction
```

- **Fed by:** frames with a course match (`tracking` or `weak`), never `odometry`.
- **Guards:**
  - a jump of more than 15 cm / 3° is only accepted once 3 frames in a row agree;
  - anything over 3 m / 40° in total is rejected;
  - it's low-passed (1.5 s), and its output moves the drone's corrected position at most 8 cm/s and its heading at most 3°/s, so setpoints never jump. (Translation and rotation ramp together, by one fraction: D rotates about the launch mark, so 3°/s on its own would move a point 5 m away by 26 cm/s.)
- **Valid** while the last accepted update is under 3 s old. The status JSON's `odom_correction` has `valid`, `dx`, `dy`, `dyaw_deg`, `age`, `accepted` and `rejected`.
- **Consumer rules:**
  - keep the last valid D while it's stale; the drift doesn't go away;
  - use zero until the first valid one;
  - freeze it inside the tube, where the camera sees little.
- **Tuning:** `corr_*` parameters.

**Simulated results** (new as-built maze, a simulation-only test route through it, generated camera data, the odometry error navless_nav would fly with):

| Simulated drift | Without | With `odom_correction` |
|---|---|---|
| 3 % distance + 0.3°/s heading (2 seeds) | median 61–65 cm, max 1.4–1.5 m | **median 5–9 cm, max 13–22 cm**, valid 100 % |
| double that | median 1.34 m, max 2.5 m | median 25 cm, but falls behind late (max 1.35 m) |
| 1.5 m VSLAM reset jumps + 15 % dropouts | metres | not handled: the jumps are rejected |

Real cuVSLAM drift on 19 hasn't been measured yet. The two real flights show PX4 and cuVSLAM agreeing to within 6 cm, but neither is an absolute reference.

The two pose topics are published only while status is `tracking`, `weak` or `odometry`.

**Which status to steer on** (changed 2026-11-07):

| Status | Meaning | Steer on it? |
|---|---|---|
| `tracking` | The course match measured **x, y and heading** (not held at the odometry prediction), at least 300 points matched, and the frame is **settled**: the match moved the prediction by less than 3 cm and 1°. | **Yes, for x, y and heading.** |
| `weak` | The match was accepted, but one of those didn't hold. | No |
| `odometry` | No course match right now; the pose is moving on odometry alone. | No |

- **Height may be held, even in `tracking`.** Take height from PX4's rangefinder, not from this pose's z. The old, stricter meaning (every direction measured, including height, roll and pitch) is `all_measured`. From flight height that almost never happened, so `tracking` was effectively never reported.
- **Simulated on the as-built window route:**
  - normal laps: 88–91 of 371 frames are `tracking`, with x/y error up to 1.2 cm;
  - with 15% VSLAM dropouts and 2 reset jumps: 44 frames, of which 6 were about 10 cm off during the final descent onto the pad;
  - self-test: no wrong `tracking` views.

ICP uses Tukey weighting with an annealed cut-off (`max(6 cm, 0.5 × search radius)`), so coarse passes can still correct 10–15° heading errors.

**Junk-point defences** (from the master sim's 0.13-fitness problem, reproduced with `--benchtest`):
- `crop_margin` (0.5 m): drops points that land outside the course's bounds plus this margin at the predicted pose. Beyond the net is unmodelled: the venue on the real drone, plain ground and sky in Gazebo, where SGBM invents depths from camera noise. To effectively turn it off, set a large margin such as `crop_margin:=50`.
- `min_neighbors` (5, within 12 cm after the 5 cm voxel step): radius outlier removal, because real surfaces are dense and invented depths are scattered. `min_neighbors:=0` turns it off.
- `max_corr_m` / `max_corr_deg` (5 cm / 1.5° per frame): with odometry, the course match may only correct the odometry prediction this much per frame. Real VSLAM drift needs millimetres; a bad match could otherwise yank the pose tens of cm at once.

Simulated results: the bench case (drone still on the mark, camera 8 cm up, invented background) went from fit 0.19 and *rejected every frame* (old start pose, no crop) to fit 0.75 and 0.0–0.2 cm. With the old start pose plus crop, it was accepted but drifted 17 → 39 cm. Nose-north window flight, perfect odometry plus background: max 84 cm → 20 cm. **Unsolved:** nose-north plus drifting odometry plus heavy invented background stays bad (136–181 cm in every variant), and it depends entirely on how much junk real stereo produces, which only real or Gazebo data can show.

**Frame dumps:** `-p dump_dir:=/tmp/loc_dump -p dump_every:=5` saves every 5th processed frame as `.npz`: the raw cloud (up to 20k points), the points used, the predicted and final pose, odometry delta, mount, status and fit info. Send these for offline analysis.

`--benchtest <obj> [background=0.6] [frames=8] [init=new|old] [crop=0.5|off] [mount=x,y,z] [base_z=0.07]` reproduces the master sim's bench case. `--simflight` also takes `background=`, `mount=x,y,z`, `base_z=`, `max_corr=off` and `min_neighbors=`.

**Window route** (`route_centreline_window.json`, simulated, 2 seeds each):

| Drone heading | Normal VSLAM | VSLAM with 15% dropouts + 2 resets |
|---|---|---|
| nose north | median ~9 cm, up to ~1 m | median 19 cm up to 4 m |
| facing direction of travel | median 3–4 cm, max 9–11 cm | median 8–10 cm, max 23–37 cm |

Nose north, the camera stares through the north net for about 24 s along the north edge, then flies backwards through the tube. The error peaks there, until the window wall snaps it back.

Earlier simulated flights of `route_gate_b.json` (`--simflight dronehacks-course-v3.obj route=route_gate_b.json speed=0.3 yaw=travel`), with pessimistic odometry drift (3% scale, 0.3°/s heading), before annealing:

| | Position median / max / end | Heading median / max |
|---|---|---|
| odometry only | 17 / 98 / 98 cm | 6.2° / 18.6° |
| fused, fixed heading 90° | 11 / 50 / 20 cm | 2.1° / 11.1° |
| fused, facing direction of travel | 7 / 13 / 3 cm | 1.1° / 6.0° |
| fused, facing travel, 1 m/s | 3 / 19 / 3 cm | 0.6° / 6.0° |

Facing the direction of travel keeps the camera on the course; with a fixed heading the camera looks away from the tunnel and south side while flying through them.

To watch a simulated flight live, add `view=8083`. It runs in real time, loops the route, and serves the same viewer, with the true pose drawn in white. Run it inside the stack container from `~/team_ws/src/dronehacks`:

```bash
python3 cloud_localize.py --simflight dronehacks-course-v3.obj route=route_centreline_window.json speed=0.3 yaw=travel view=8083
```

Other options: `yaw=fixed|travel`, `seed=`, `odom_drop=0.15` (untrusted VSLAM frames), `odom_resets=60,140` (fake VSLAM reset jumps), `max_speed=1000` (turns the reset check off, for comparison), `odom_only=1` (baseline), and `icp.anneal=`.

**CPU:** run `stereo_cloud.py` at 3 Hz (`-p rate_hz:=3.0`); the localizer uses only 3 clouds per second. At 5 Hz, together with Koji's mapping viewer, the Jetson overloaded, and Koji's odometry gate, which also feeds PX4, saw sample age of about 245 ms and rejected most samples. At 3 Hz it was back to about 57 ms.

**What the simulations do NOT show:** the camera data in them is generated from the .obj, with made-up noise and clutter. They test the matching and fusion logic, not real stereo depth, the real net, real lighting, or the real VSLAM's drift. See the team Slack post for the full list.

Course .objs: `dronehacks-course-asbuilt.obj` (5.7 MB) is at the repo root, where `cloud_localize.py` finds it with no parameters. `dronehacks-course-v3.obj` (the plan) comes from the organisers' zip and isn't in git; put it in the repo root too for `course:=plan`. On the drone, both can also go in `~/team_ws/src/dronehacks`.

`search_heading:=true` (or the viewer's checkbox) tries every heading from the start position instead. That's unreliable on the launch mark, because Barrier 1 and the M1 divider, both about 0.5 m away, look alike.

Useful parameters: `obj_up` (`z` or `y`), `obj_scale`, `max_range` (5 m), `max_points` (1500), `min_fitness` (0.35) or `min_matches` (300) matched points, `max_rmse` (0.06 m), `rate_hz` (3), `http_port` (8082), `match_ceiling` (true; the viewer always hides the ceiling, but matching keeps it because it's often a third of the cloud).

The ICP only moves along directions the visible surfaces actually pin down. A view of just floor and table tops can't fix horizontal position, so it holds those directions at the previous estimate and shows status `weak` instead of drifting. ICP uses Tukey weighting (c = 6 cm) and a 0.3 → 0.08 m search schedule, so people and chairs near modelled surfaces can't drag the pose.

Self-test options: `trials=`, `range=`, `perturb=` (0.4 is about frame-to-frame tracking; 1.0 is a rough manual reset), `points=`, `clutter=` (unmodelled points as a fraction of model points; 1.0 is about a crowded room), and `icp.kernel=`, `icp.c=`, `icp.schedule=` for experiments.

## Walk test: does it know where it is on the real course? (`walk_test.py`)

**Only if walking the drone through the course is allowed. Props OFF.** You set the drone down on spots whose positions are known (the launch mark, the six corners of the taped floor line, the tube's mouths and middle, the landing pad centre), nose along the direction of travel, and press Enter. The script takes the median of the localizer's last 2 s of `/team/localize/base_pose` there and compares it with the true spot. It does the same for **cuVSLAM odometry alone**, anchored on the launch mark, so the result shows whether matching against the course model beats plain odometry drift. `python3 walk_test.py --list` prints the checkpoints.

1. Put the drone on the launch mark, nose north. Restart the stack (it re-zeroes odometry) and keep the drone still.
2. Start `stereo_cloud.py` (3 Hz) and `cloud_localize.py` as above, in their own `docker exec -d` shells.
3. Start the walk test in an interactive shell, still on the mark:
   ```bash
   docker exec -it skybeta-stack bash -lc "source skybeta_env.sh; python3 /root/team_ws/src/dronehacks/walk_test.py --course /root/team_ws/src/Navless/gazebo_sim/course/course_asbuilt.json"
   ```
4. Press Enter for checkpoint 0 (the launch mark) straight away. That also anchors the odometry-only comparison.
5. Carry the drone at a steady walk, roughly along the route, facing the direction of travel, and put it down on each checkpoint. Wait 3 s, press Enter. `s` skips a checkpoint, `b` goes back one, `q` finishes. A live line every 2 s shows the status, fit, current estimate and distance to the next checkpoint. Watch `http://<drone>:8082/` too.

It writes `results.json`, `track.csv` and `plan.svg` (top view: walls, checkpoints, localizer track in blue, odometry-only in grey) to `~/team_ws/walk_tests/<time>/` on the drone. The summary prints both errors per checkpoint, plus how much of the walk was `tracking` / `weak` / `odometry` / `lost`. Placing by hand is only good to about ±3 cm, so anything under about 5 cm is as good as this test can show. `checked = NO` means the pose there came from odometry only, not a course match (expected in the dark tube).

`python3 walk_test.py --selftest` runs a synthetic walk with no ROS, to check the maths and the output files.

## Walk session: live view + full recording (`walk_session.sh`)

The same walk with the drone (**props OFF, disarmed, only if the organisers allow it**), but one script starts everything, records everything, and cleans up after itself. Run it on 19's Jetson host, from `~/team_ws/src/dronehacks`. Not while anyone is flying or prepping 19: it uses the Jetson.

```bash
docker rm -f skybeta-stack        # drone on the launch mark, nose north, still; wait ~60 s for the stack
bash walk_session.sh start        # stereo_cloud (3 Hz) + cloud_localize (as-built) + walk_recorder
bash walk_session.sh checkpoints  # optional, 2nd terminal: walk_test, Enter at known spots = ground truth
bash walk_session.sh status
bash walk_session.sh stop         # stops exactly what start started, checks nothing is left, writes the summary
```

- **Live:** `http://<drone>:8082` shows the localizer's best estimate on the course model with its trail and the live cloud (green points = matched the model); `http://<drone>:8081` is the camera and depth.
- **Recording** (`~/team_ws/walk_sessions/<time>/`):
  - `track.csv` at 10 Hz from `walk_recorder.py`: localizer pose, status, fit, matches, measured axes; `odom_correction`; cuVSLAM pose, velocity and `vo_state`; PX4's EKF2 position, velocity, acceleration, heading, reset counters, rangefinder, accuracy, dead reckoning, armed/mode, battery; how fresh the vision feed into PX4 and the camera IMU are; stereo cloud rate.
  - `raw/`: **every message, every field, at full rate**: `px4_local_position`, `px4_status`, `px4_battery`, `px4_vision_in` (what the bridge feeds EKF2), `cuvslam_odometry` (with covariances), `oak_imu` (camera IMU, ~200 Hz).
  - `frames/` (every 5th cloud as the localizer saw it, for replaying offline), and each node's log.
  - **Not available:** the Pixhawk's own IMU, attitude and EKF2 odometry. 19's PX4 publishes only `vehicle_local_position`, `vehicle_status_v1` and `battery_status` over uXRCE-DDS, and adding topics is a PX4 config change.
- **Summary** (`walk_summary.py`, also runs on a laptop: `python3 walk_summary.py <session> course_asbuilt.json`): `summary.txt` gives the share of time in each status, the longest stretch without `tracking`, estimate jumps, cuVSLAM stalls over 0.4 s, how far plain odometry drifted from the localizer, what `odom_correction` did, and checkpoint errors. `plan.svg` draws the walk on the course: localizer trail coloured by status, plain odometry from the launch mark (what navless_nav would believe uncorrected), odometry + correction, checkpoints (truth vs estimate).

"Odometry vs localizer" compares two estimates. Only checkpoints measure against the truth, so do at least a few.

## Stand-in course .obj for testing

`scene_to_obj.js` turns tape measurements into an .obj, so localization can be tested before we have the real course. `test_room.json` is our 30 x 34 x 9 ft hackathon room with its 12 tables: 4 rows, each made of two tables joined end to end, an aisle, then one table, all running along the 30 ft wall. Positions are **scaled from a sketch** until someone measures them.

```bash
node depthcloud/scene_to_obj.js depthcloud/test_room.json depthcloud/test_room.obj --svg depthcloud/test_room_plan.svg
```

- The origin is the floor corner on the wall opposite the door, at the end farther from the door. x runs along the 30 ft wall (toward the door's wall), y runs along the 34 ft wall (toward the door's end), and z is up. Output is in metres.
- `--svg` writes a top-down plan, with the origin at the bottom-left, drawn the same way as the room sketch. Compare it with the real room.
- Item types, all in the scene's units:
  - `{"type": "table", "center": [cx, cy], "yaw_deg": 0}`: yaw 0 means the long side runs along x. Sizes come from `defaults.table`, or set `length`, `width`, `height` or `top` per table. Only the top slab is modelled.
  - `{"type": "table_grid", "rows": 4, "cols": 3, "first_center": [cx, cy], "pitch": [dx, dy], "yaw_deg": 0}`: a grid of tables. `first_center` is the centre of the table nearest the origin, and `pitch` is the centre-to-centre spacing along x and y. For uneven spacing, give either entry as a list, for example `[[6, 12], 6]`; a spacing equal to the table's length (across) or width (rows) makes the tables touch. Tables are named `T_r<row>c<col>`.
  - `{"type": "chair", "center": [cx, cy], "yaw_deg": 90}`: a seat and a backrest, with the seat facing +x at yaw 0.
  - `{"type": "box", "center": [cx, cy], "size": [sx, sy, sz], "yaw_deg": 0, "z0": 0}`
- `--up y` writes Y-up instead. Match whatever the real course .obj uses.
- An empty rectangular room looks the same after a 180° turn, and stereo depth only reaches about 5 m. The tables, measured accurately near where the drone will be, give the alignment something unique to lock onto.

## CPU cost

It uses about 160 ms of CPU per frame at 5 Hz (roughly 80% of one core), on the same CPU as the flight stack. Lower `rate_hz` (for example to 3.0) or stop it before flying.
