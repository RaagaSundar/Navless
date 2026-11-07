# Local recorded-run preview and offline videos

This loopback-only viewer reviews **archived simulation evidence**. It does not
connect to ROS, start a simulator, arm, reset, or issue flight commands. Recorded
simulator truth and the chase reconstruction are review aids, never navigation
sensors or hardware qualification. Keep real recordings outside Git.

## Mac installation

Install Homebrew from its official instructions if it is not already available.
Then, from the repository root:

```bash
brew install uv ffmpeg
uv python install 3.12
uv run --python 3.12 split_sim/preview/smoke.py
uv run --python 3.12 split_sim/preview/smoke.py --with-chase
```

The first smoke uses Python's standard library plus `ffmpeg` and `ffprobe`
(included together by Homebrew). It creates a **synthetic grayscale test pattern**
in a temporary SQLite database, exports an H.264 MP4/JPEG/JSON, and verifies the
read-only HTTP catalog, UI, JPEG, byte-range seeking, invalid-range rejection,
symlink exclusion, and absence of write endpoints. It deletes its own temporary
fixtures; it uses no aircraft, network service, Docker, ROS, or existing run data.
The generated disarmed fixture is explicitly not counted as an observed flight.

`--with-chase` also builds a minimal synthetic scene and exercises actual OpenGL
chase rendering and H.264 encoding. `uv` installs the inline pinned dependencies
for `render_chase.py`: moderngl 5.13.0, numpy 2.3.2, Pillow 13.0.0,
trimesh 5.2.0, and pyulog 1.3.0. The supported interpreter for these commands is
Python 3.12 (the renderer declares Python >=3.11,<3.14). Rendering requires an
OpenGL 3.3-capable local desktop context on macOS; Linux uses EGL. A headless Mac
without an accessible graphics context may pass the first smoke but cannot
qualify the chase renderer. There is no Pillow, ROS, or GPU requirement for the
viewer or onboard exporter themselves.

## Open archived runs

```bash
uv run --python 3.12 split_sim/preview/server.py --port 8774
```

Open <http://127.0.0.1:8774>. Stop the foreground process with Ctrl-C. The server
binds only `127.0.0.1`; do not publish or reverse-proxy it. It has no authentication
and is intended for local evidence review. It serves neither arbitrary repository
files nor writes. Defaults are relative to this checkout, independent of CWD:

- `local_course/state/runs/`: Mac runs.
- `local_course/state/nvidia-runs/`: NVIDIA runs (require `platform.json` with
  `verified: true`; this verifies archive provenance, not successful flight).
- `local_course/state/flyx-runs/`: FlyX runs (require `assessment.json`).
- `gazebo_sim/course/`: `course_v3.json` and `route_centreline_window.json`.

Relocate archives without editing source:

```bash
uv run --python 3.12 split_sim/preview/server.py --runs /path/to/mac-runs --nvidia-runs /path/to/nvidia-runs --flyx-runs /path/to/flyx-runs --course /path/to/course
```

Each run uses `flight/track.json` (`samples` with simulation seconds `t`, world
ENU metres `x,y,z`, optional yaw and recorded arm/landed state), optional
`flight/grade.json`, `flight/planned-route.json`, and `review.json`. Only an actual
sample with `armed: true`, `landed: false`, finite `z > 0.25` counts as observed
flight. A passing course badge requires that evidence plus independent
`grade.passed: true`. Missing state, native completion, or geometric crossings
are never converted into a pass. FlyX truth and estimates remain separate and
ungraded unless their independent grade is archived through an explicit contract.

Direct links use `?run=nvidia%3A<folder-name>` or `?run=flyx%3A<folder-name>`;
Mac folder IDs are unprefixed. Comparisons keep independent run plans. Legacy
centreline fallbacks, estimator overlays and OBJ observer data are clearly
labelled. Runtime freshness indicates output activity, not successful flight.

## Export onboard video from a sealed recording

The exporter reads the existing sensor-recorder SQLite schema: `samples(kind,
stamp,frame,data)`, `stamp` in simulation nanoseconds, `left_info` JSON
CameraInfo (`width`,`height`), and zlib-compressed `left` mono8 image bytes.
It opens SQLite read-only and rejects active journal/WAL files; stop/finalize the
recorder first. Ensure at least 256 MiB free and do not export an actively copied
or changing database. No hardware is contacted:

```bash
uv run --python 3.12 split_sim/export_drone_video.py --database /path/to/sealed-camera-imu.sqlite --run-dir /path/to/run
```

The run directory must already exist. `flight/track.json` automatically bounds
the clip to its first/last simulation timestamp; `--start-ns` and `--end-ns`
provide explicit cropping. Use `--fps 15` or `--fps 30`. Missing frame intervals
hold the previous real frame; there is no interpolation or enhancement.
`drone-view.mp4`, `.jpg`, and `.json` are published with the JSON manifest last,
with `status: complete`, hashes, source frame stamps and timing disclosure.
Generated manifests can contain local source paths: keep them with private local
evidence, and inspect/redact copies before intentionally sharing.

## Export a behind-drone reconstruction

```bash
uv run --python 3.12 split_sim/preview/render_chase.py /path/to/run --assets /path/to/prepared/assets --world /path/to/matching/world.sdf
```

This creates `chase-view.mp4`, `.jpg`, and `.json` from archived trajectory poses.
It is a new third-person reconstruction, not an onboard camera recording. The
world SDF and prepared mesh/material/texture tree must match the recorded run.
The tree needs `models/dh_course/model.sdf`, its referenced OBJ/material/textures,
plus `dh_quad_stereo.sdf` unless `simulated-vehicle.sdf` is archived in the run.
Use the prerequisite split harness's asset preparation; this tool does not
silently download or regenerate an appearance. Without `--assets`, an explicit
run-local `appearance.json` renderer path is used, otherwise `split_sim/state/assets/` in this checkout. Without `--world`, the appearance sidecar is used, otherwise
`gazebo_sim/worlds/dronehacks.sdf`. Synthetic profiles require their archived
asset hashes; relocated files are checked and mismatches fail. `--world` does
not bypass synthetic hash verification.

Recorded quaternion attitude is preferred; an independently matching archived
ULog can supply it. Missing attitude is disclosed as measured yaw with level
body, and pose gaps are not interpolated into invented motion. Foreground
geometry is clipped to keep the drone visible and this is disclosed. FlyX
reconstructions explicitly use local geometry and do not reproduce FlyX imagery.

The convenience coordinator runs both workers on an already sealed flight:

```bash
uv run --python 3.12 split_sim/export_run_videos.py /path/to/run
```

It requires genuine airborne samples, skips existing video sidecars, and uses
`camera-imu-capture.json` (`integrity: "ok"`, `path` absolute or relative to the
run) for the sealed camera DB. It may wait up to ten minutes for that manifest.
`--enqueue` runs the same offline export detached with `video-export.log` in the
run; use the foreground form for reproducible first setup and troubleshooting.

## Playback and evidence boundary

The selected run has separate onboard and behind-drone players, provenance
sidecars, and explicit missing/unfinalized states. MP4s stream in bounded chunks
with HTTP byte-range seeking and no 16 MiB document cap. JSON/log/text/YAML/CSV/
PNG/JPEG evidence remains capped at 16 MiB; symlinks and secret-like filenames
are excluded. `appearance.json` supplies only a brief visible profile and a link
to source evidence. Videos are independent of trajectory playback by default;
optional seeking uses the explicit `first_stamp_ns` or `start_sim_s` origin.
Recent/live selections poll for newly finalized clips for at most ten minutes
without replacing an existing player or moving the replay timestep. Older
missing clips stay explicitly absent. Refresh manually after a later export.
