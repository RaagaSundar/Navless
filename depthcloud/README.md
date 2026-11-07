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

## CPU cost

It uses about 160 ms of CPU per frame at 5 Hz (roughly 80% of one core), on the same CPU as the flight stack. Lower `rate_hz` (for example to 3.0) or stop it before flying.
