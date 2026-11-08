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

On the drone, with `stereo_cloud.py` already running, start it against the course model (`dronehacks-course-v3.obj`: metres, Z up, origin at the centre of the floor). With no `init_*` parameters, the **drone's centre (`base_link`)** starts on the model's **launch mark** (−2.515, 0.914), standing at `init_base_z` (0.07 m, like `navless_nav`'s `start_height_m`), facing **north (90°)**. The drone is always placed facing forward along the guideline, which runs north from the mark. The camera is placed from that through the `base_link` → camera mount from TF, on the first cloud. If TF has no mount after 3 s, it uses `mount_fallback` (Koji's measured `0.1053,0,0.0304`). `-p init_z:=<camera height>` gives the old behaviour, putting the camera itself at (`init_x`, `init_y`, `init_z`).

```bash
python3 /root/team_ws/src/dronehacks/cloud_localize.py --ros-args -p obj:=/root/team_ws/src/dronehacks/dronehacks-course-v3.obj
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
| `/team/localize/status` | `std_msgs/String` (JSON) | `status` (`tracking` / `weak` / `odometry` / `holding` / `lost` / ...), `measured` (fraction of x / y / height / heading the frame measured), `age` (s since the last course match), `odom_source`, `odom_rejects` |

The two pose topics are published only while status is `tracking`, `weak` or `odometry`. Check `status` and `measured` before trusting a pose: `odometry` means no course match right now.

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

The course .obj (`dronehacks-course-v3.obj`, 5.7 MB, from the organisers' zip) is not in the repo. Copy it next to the script.

`search_heading:=true` (or the viewer's checkbox) tries every heading from the start position instead. That's unreliable on the launch mark, because Barrier 1 and the M1 divider, both about 0.5 m away, look alike.

Useful parameters: `obj_up` (`z` or `y`), `obj_scale`, `max_range` (5 m), `max_points` (1500), `min_fitness` (0.35) or `min_matches` (300) matched points, `max_rmse` (0.06 m), `rate_hz` (3), `http_port` (8082), `match_ceiling` (true; the viewer always hides the ceiling, but matching keeps it because it's often a third of the cloud).

The ICP only moves along directions the visible surfaces actually pin down. A view of just floor and table tops can't fix horizontal position, so it holds those directions at the previous estimate and shows status `weak` instead of drifting. ICP uses Tukey weighting (c = 6 cm) and a 0.3 → 0.08 m search schedule, so people and chairs near modelled surfaces can't drag the pose.

Self-test options: `trials=`, `range=`, `perturb=` (0.4 is about frame-to-frame tracking; 1.0 is a rough manual reset), `points=`, `clutter=` (unmodelled points as a fraction of model points; 1.0 is about a crowded room), and `icp.kernel=`, `icp.c=`, `icp.schedule=` for experiments.

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
