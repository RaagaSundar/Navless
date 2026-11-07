# Centered window · PX4 auto-land

The immutable **20261107T221438Z-split-astar-flight** recorded simulation run passed the unchanged independent course
grader: all six ordered checkpoints, no rejection reasons, zero contacts,
12.70 cm minimum clearance, and landing 13.23 cm from the pad center followed by
normal PX4 auto-disarm. The companion verified the fused route endpoint, sent
one `NAV_LAND`, observed its accepted ACK, then remained passive through PX4's
landing. This is one bounded simulation pass, not repeatability or hardware
qualification.

`passed-centered-window/grade.json` is the sealed grade, unchanged. The compact
settings/source/evidence manifests retain input/output hashes without raw bags,
private paths or videos. The earlier **215942** full physical traversal remains
in `known-window-transit/`: zero contacts and a pad landing, but its sole grade
failure was the vertical window envelope by 5.60 cm. It is not relabeled a pass.

## Exact code and route installed by this PR

All **39** `course_runtime/sources.sha256` members are byte-identical to the
frozen passing 221438 archive, including the centered route, its corresponding
window-forward yaw config, `NAV_LAND` handoff/tests and startup timing behavior.
The runtime is the normal `split_sim/course_runtime/` tree; it is actually invoked
by the existing `run_split_course.sh`, not a dormant alternate snippet. The
profile checker verifies those byte hashes and the selected environment.

## What changed vs Forward window · high transit (215942)

- The same 879 XY points lower the window target from 0.85 m to 0.762 m and re-pinned 3D yaw
  arclengths. Forward-window heading remains 292.099° with a 10°/s rate cap.
- A fresh fused route-end/authority check hands off once to standard PX4
  `NAV_LAND`, then checks ACK/mode and final landed/disarmed state passively.
- Waiting for a stable landed translation reports diagnostics without resetting
  the independent 30-second healthy observation. Both checks remain required.

The config's historical `route_source`/`scope` labels are retained byte-exact
for source identity; they are descriptive, not paths used by the loader. The
loader reads the packaged `astar-route.json` and verifies its hash. No return-hop,
heading-all, obstacle-probing or other workspace prototypes are included.

## Retrieve and verify the actual passing source

Until this PR merges, obtain its reviewed head explicitly; do not assume these
updates are already in main:

```bash
git clone https://github.com/RaagaSundar/Navless.git Navless-pass
cd Navless-pass
git fetch origin pull/49/head:review-pass-49
git switch review-pass-49
python3 split_sim/doctor.py --source-only
python3 split_sim/verify_transit_profile.py
python3 -m unittest split_sim.test_transit_profile split_sim.test_install -v
```

After merge, normal main contains the profile. Use the [installation guide](../INSTALL.md)
for Mac Docker/PX4 build, organizer Jetson Humble/NVIDIA image, verified SSH and
stopped-container installation. Vendor image/fleet access are external
prerequisites; no credentials or hardware setup are invented here.

## Regenerate and pin the passing visual inputs

Install Pillow for generation. Choose new immutable paths. These commands are
offline asset generation only; they do not start ROS/Gazebo/PX4:

```bash
SPLIT_CAMERA_RATE_HZ=31.25 python3 split_sim/make_assets.py split_sim/state/pass-base
python3 appearance_profile/generate.py --source split_sim/state/pass-base \
  --world-source gazebo_sim/worlds/dronehacks.sdf \
  --output split_sim/state/pass-neutral --seed 20261107 --pixels-per-m 256
python3 split_sim/verify_transit_profile.py \
  --appearance split_sim/state/pass-neutral \
  --camera split_sim/state/pass-base/dh_quad_stereo.sdf
```

The offline generation chain matched all archived visual bytes and the
qualified camera/vehicle hash. The portable schema2 sidecar digest changes
because host paths are removed; visual hashes and camera are compared separately.
The passing run used stereo-only at 31.25 Hz, procedural ground-inclusive neutral
textures and the **original simplified perimeter**, not the separate open-strand
net. A net/surface change is another unqualified variant, not this passing input.

## Disarmed startup and explicit profile selection

In an inactive coordinated setup, after building/installing the published stack:

```bash
cd split_sim
bash configure_estimator.sh stereo  # Separate Jetson simulation container stopped.
export SPLIT_INERTIAL=0 SPLIT_CAMERA_RATE_HZ=31.25
export SPLIT_HEIGHT_PROFILE=ev-terrain SPLIT_YAW_POLICY=window-forward
export SPLIT_APPEARANCE=portable-neutral-v1
export SPLIT_APPEARANCE_DIR="$PWD/state/pass-neutral"
export APPEARANCE_MANIFEST_SHA256="$(shasum -a256 state/pass-neutral/appearance.json | awk '{print $1}')"
bash start.sh --split
bash start_px4_from_mac.sh
```

These starts remain disarmed. The selected profile must read back actual PX4
`EKF2_HGT_REF=3`, `EKF2_EV_CTRL=11`, `EKF2_RNG_CTRL=1`, `EKF2_BARO_CTRL=1`.
Verify actual image/CameraInfo/TF/source stamps, camera hash, 31.25 Hz simulation
rate and delivered wall rate, tracking, same-source Range, fixed EV/barometer/
range health, fresh local position and landed/disarmed reports. The measured
sill exception remains finite and conditioned on real range/geometry/authority;
it does not disable range, residual, uncertainty, reset or integrity checks.

The normal `run_split_course.sh --prepare-flight` now archives and invokes the
passing centered route and exact corresponding yaw config from the manifest.
Preparation starts a passive observer and waits for run-bound approval; it is
not authorization to arm. Review the plan and fresh evidence and obtain the
team's explicit approval before using the existing activation workflow. After
keyboard RC, land/release RC and **Reset world** before automation. Preserve
failures and stop publication on stale telemetry or lost authority.

The [preview/video guide](../preview/README.md) describes actual recorded
footage, playback and source provenance. Viewer loading, a process exit or a
prior pass does not waive a new run's health, recorder, authority or independent
grade checks. Keep the actual `grade.json`, full source/asset identities and
sensor/ULog archives locally for each run.

## Focused validation

The runtime suite includes actual passing `test_landing.py` plus existing
conditional-range, yaw, frame, integrity, datum and recorder tests. Profile
checks reject changed source, config, camera and route while preserving the
prior failed grade. One optional historical ULog test skips when the retained
raw log or `pyulog` is absent; raw captures are intentionally not distributed.
A fresh network clone/source doctor and offline preview smoke are recorded in
this PR before readiness. No simulator/Jetson/hardware action is performed by
the packaging worker.
