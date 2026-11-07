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

## Named runs and What changed

Every experiment uses run-local `run-comparison.json` schema 1 (formal schema:
`run-comparison.schema.json` alongside the writer). Required identity
fields are `run_id` (the exact catalog ID, including `nvidia:`/`flyx:`), `name`,
`kind` (`flight attempted`, `bench`, `replay`), and `baseline_id` (or null).
`changes` is a list of `{field,before,after,evidence}`; evidence filenames belong
to this run. `notes`, `confounds`, `source_sha256`, a seeded settings `snapshot`,
and measured `telemetry` retain context. Labels never override independent grade.
The same physical flight may have separate derived replay/localization artifacts;
do not rename it as though a new flight happened.

From the repository root (stdlib Python >=3.11):

```bash
python3 split_sim/run_comparison.py /path/to/run --name "Centered window · PX4 auto-land" --baseline /path/to/baseline
```

`--kind bench` or `--kind replay` labels other experiment types. The portable
installation uses `split_sim/run_comparison.py` with the same arguments.
`--annotations /path/to/annotations.json` can supply `name`, `notes`, `confounds`,
and evidenced `changes`; it cannot alter run ID, baseline, kind, or schema. The
writer does not run software or estimate results: it reads archived settings,
scene hashes, planned-route data and actual arm/landed samples, and atomically
writes metadata. Missing archived fields are “not archived”, not defaults. Equal
recorded fields mean no recorded differences, not proof of identical software,
measurements, startup state or physics. No baseline is selected automatically.

Names appear in the archive and selected heading alongside the immutable ID.
The prominent What changed section shows before → after, confounds and source
links. Clicking an archived baseline overlays its track while preserving the
selected run. Missing comparison metadata stays explicitly unknown.

Offline comparison checks (no recording or hardware needed):

```bash
uv run --python 3.12 -m unittest discover -s split_sim -p test_run_comparison.py -v
node split_sim/preview/test_comparison_ui.cjs
```

Node is needed only for this optional small UI test; the server and writer use
Python's standard library. Install it with `brew install node` if needed.

Per-source localization uses run-relative `localization-poses.json`, optional
`localization-live.json`, and `map-localization.json`. Synchronized source cards
and coloured trails/markers show raw NVIDIA, admitted VO, PX4 fused, independent
truth, and measured map hypotheses separately. The cursor uses source simulation
stamps, never future samples; samples older than 1s have no current marker and
trajectory gaps over 1s are not joined. The height plot uses registered course
metres. Origin, units, status, source/cursor age and validity remain visible.
Unknown/unregistered frames and rejected/ambiguous map results retain diagnostics
but cannot place a marker on the course. Raw scan poses remain withheld without
a qualified transform. Raw VO, admitted VO, PX4 and truth are visible by default; source
checkboxes control each comparison. Map studies are labelled offline/non-controller.

Live latest updates actual track/source snapshots in place every3s, retaining
video elements and replay selection. Playing video suspends cursor movement and
map redraw; scrubbing turns live latest off. Snapshot generation wall age is
separate from simulation source age; archived snapshots are not claimed live.
Direct demonstrations: `?run=nvidia:20261107T215942Z-split-astar-flight` (near-pass
flight, original independent grade retained) and `?run=nvidia:20261107T215630Z-split-astar-flight`
(issue flight with measured CAD/as-built/withheld raw-scan comparison).

Portable offline UI validation (Node.js standard library only):

```bash
node --check split_sim/preview/app.js
node split_sim/preview/localization_test.cjs
```

The inline test fixture has all four source metadata/poses and explicit synthetic
origin; it is never registered as a run or presented as flight evidence. Tests
cover exact-value preservation, source stamps/no future samples, withheld map
poses, invalid estimator flags/unregistered frames, and video/cursor continuity.
Actual local review additionally checked the 215942/215630/221438 source archives
and rendered the two demo links in headless Chrome. No real recording, private
absolute path, binary asset, or credentials are included in this feature.

## All saved tests in the common picker

The normal homepage now has one searchable, paginated picker for flight archives,
ungraded native trajectories, interrupted setup, passive benches, offline sensor
and map replays, and saved unit/planner checks. Selecting any item stays in the
same viewer and produces a stable `?run=test:<id>` link. Existing flight and
localization views remain separate from generic artifact details. The optional
`/tests` index uses the same inventory; it is not another site.

`test_index.py` scans only its explicit evidence-root allowlist, with bounded
nesting, 128 KiB receipt parsing, an 18,000-file cap and a 15-second cache. It does
not execute tests or crawl vendor/build/source/binary trees. File-modification
only dates and unknown outcomes stay labelled; conflicting receipts show mixed
checks. Capture complete means capture complete, not a flight pass. Administrative
source inventories and code-only files are not completed experiment cards.
Derived analyses link their parent flight and do not create another flight count.

The server accepts `--test-evidence-root /path/to/workspace` for the allowlisted
workspace layout, in addition to its existing run-root options. Evidence is
served read-only from exact catalogued paths; traversal, hidden files, credentials,
symlinks and unsupported binaries are excluded. Views above 16 MiB are withheld.
No recordings, model assets, source data or private paths ship with this feature.

## One map for actual and preloaded geometry

The optional archived M3 bench appears only when its experiment artifacts exist
under `--wall-tests`. Its one shared map overlays actual orange and preloaded
purple geometry, with labelled recorded baseline localization or camera-rig
reference. Paired camera media is explicitly passive, never a flown trajectory.
No side-by-side map mode is provided.

A genuine flight may archive `map-comparison.json` schema_version 1 with ENU metre
poses, baseline/actual model hashes, changed wall geometry, unchanged onboard map
and run-relative `fixture_provenance`. The viewer verifies the fixture SHA256 and
exact dimensions/poses, then requires a separately hashed, verified renderer
receipt matching the actual model/profile before calling it actual scene geometry.
Prepared options without a renderer receipt are withheld. Existing truth and
measured estimator trajectories keep their own sources, colours and time cursor.
Missing change metadata means one baseline map with equivalence unknown; it does
not prove the world matched. No fabricated scan or scene pose is introduced.

Portable checks (synthetic fixtures only):

```bash
python3 -m unittest discover -s split_sim/preview -p test_test_index.py -v
node split_sim/preview/all_tests_picker_test.cjs
node split_sim/preview/map_geometry_test.cjs
node split_sim/preview/test_comparison_ui.cjs
python3 split_sim/preview/smoke.py
```

The four core localization sources are visible by default with distinct co-located
glyphs (truth dot, raw ring, admitted diamond, PX4 cross). `?t=48.6` selects
recorded simulation seconds and disables live latest; the airborne comparison
button selects only an actual armed, airborne sample with fresh joint coverage.
Live latest can use a jointly covered snapshot up to 15 simulated seconds behind
truth publication, labelled explicitly; it never bridges a source outage or
unknown registration. Turning trails off also suppresses preceding split segments.

Sealed `m3-capture-result.json` and `m3-physical-clearance.json` expose bounded
experiment goals independently from the original full-course grade. A bounded
capture pass never increments course-pass totals. Offline shadow results remain
offline evidence, not a flight-control claim. Non-finite numeric evidence is
returned as null in API catalogs so strict browsers can read other runs; original
files and grading booleans remain unchanged.
