# Recorded autonomous window transit and a separate centered candidate

The `known-window-transit/` profile records the **215942 physical full transit**:
all ordered checkpoints, window-forward camera yaw about 291.6° (292.099°
commanded), landing/disarm on the pad at 14.13 cm offset, and zero recorded
contacts. The independent grade **failed** its full-envelope vertical window
check: center Z 0.97524 m exceeded the 0.9192 m upper bound by 5.60 cm for the
0.30 m envelope. That sole failure is preserved in `grade.json`; this is not a
fully graded pass or hardware qualification. The small evidence summary carries
source/grade/track hashes instead of raw recordings, private paths or videos.

## Exact source relationship

The official main baseline for this package is `cd139e1` (PR #46 merge). The
archived 215942 runtime has 38 source/route/config members. All 37 other members
are byte-identical to the published runtime. Only two descriptive yaw-config
fields (`scope`, `route_source`) differ because PR #46 made provenance portable;
all behavioral settings are identical. The verifier checks each member against
the actual runtime manifest and the frozen source hashes, comparing that one
config semantically after removing only those two fields. Changes to rate,
route hash, control policy or other settings are rejected.

No duplicate flight controller is added. The recipe uses the existing
`start.sh` → `start_px4_from_mac.sh` and `run_split_course.sh` →
`course_runtime/mission.py` entrypoints. New return-hop or heading-all work is
not part of this profile. Future runtime changes must not inherit this label
without passing its explicit source/profile checks.

## Reproduce from GitHub, disarmed first

Use the [full installation guide](../INSTALL.md) for Mac Docker/PX4 build and the
organizer-provided Jetson Humble/NVIDIA image and verified SSH access. This
profile does not distribute that vendor image or authorize hardware operation.
Until this profile PR merges, fetch its exact reviewed PR head into a fresh
checkout; afterward ordinary main includes the recipe. Run checks before any
simulation lifecycle action:

```bash
python3 split_sim/doctor.py --source-only
python3 split_sim/verify_transit_profile.py
python3 -m unittest split_sim.test_transit_profile -v
```

Install Pillow for offline generation as described in `appearance_profile/`.
Select new immutable paths; neither generator overwrites existing output:

```bash
# Repository root; offline generation only, no ROS/Docker/Gazebo/PX4 starts.
SPLIT_CAMERA_RATE_HZ=31.25 python3 split_sim/make_assets.py split_sim/state/transit-base
python3 appearance_profile/generate.py --source split_sim/state/transit-base \
  --world-source gazebo_sim/worlds/dronehacks.sdf \
  --output split_sim/state/transit-neutral --seed 20261107 --pixels-per-m 256
python3 split_sim/verify_transit_profile.py \
  --appearance split_sim/state/transit-neutral \
  --camera split_sim/state/transit-base/dh_quad_stereo.sdf
```

This exact published generation chain was run offline for the package. All
archived visual bytes (120 atlases, ground map, OBJ/MTL/world and supporting
models) and the qualified 31.25 Hz camera/vehicle hash matched. Schema 2 sidecar
metadata has portable paths and therefore a new digest; it is deliberately not
claimed to be the old path-bearing sidecar. The checker compares visual hashes
and the camera separately instead of treating a changed manifest as a flight.
The appearance remains procedural, not measured course reflectance.

In an inactive reviewed setup, select stereo-only (not IMU fusion), the fixed
EV-height profile, the recorded forward-window option and ground-inclusive
neutral appearance. The 215942 profile used the **prior simplified perimeter**,
not the separately generated open-strand net. A changed net is a different
unqualified appearance variant; see merged [#41](https://github.com/RaagaSundar/Navless/pull/41).

```bash
cd split_sim
# While the separate Jetson simulation container is stopped:
bash configure_estimator.sh stereo
export SPLIT_INERTIAL=0 SPLIT_CAMERA_RATE_HZ=31.25
export SPLIT_HEIGHT_PROFILE=ev-terrain SPLIT_YAW_POLICY=window-forward
export SPLIT_APPEARANCE=portable-neutral-v1
export SPLIT_APPEARANCE_DIR="$PWD/state/transit-neutral"
export APPEARANCE_MANIFEST_SHA256="$(shasum -a256 state/transit-neutral/appearance.json | awk '{print $1}')"
# Coordinate domain219 ownership and fresh disarmed world first.
bash start.sh --split
bash start_px4_from_mac.sh
```

These starts remain disarmed. Verify the *actual* renderer/camera file hashes,
source stamps, stereo calibration/TF, 31.25 Hz simulation rate and wall-time
rate, tracking, Range delivery, fixed EV/barometer/range health, local position,
landed/disarmed state and recorder qualification. A loading viewer or topic
name is not evidence of those properties. Preview and video workers are
[documented separately](../preview/README.md); only finalized recorded footage
is labeled as onboard video. No viewer, process exit or physical traversal
changes the independent grade.

The normal `run_split_course.sh --prepare-flight` starts the passive prepared
observer and waits for a run-bound team decision. It does not authorize arming.
Review the known 5.60 cm vertical-envelope failure, plan and fresh telemetry
before any team-approved moving run, using the existing approval workflow.
After keyboard RC, land/release RC and **Reset world** before automation.
Keep failed evidence and stop publication on stale telemetry or lost authority.

## Separate optional centered-window candidate

`centered-window-candidate/route.json` is an independently staged altitude-only
candidate, SHA-256 `e25ce78ebb68f9b3ed2d65d30a01f0a66ad007b4fb597e35120a438ca5755b8a`.
Its 879 XY points are bitwise identical to the recorded route. It targets
0.762 m in the window band and retains the same 292.099° heading/10°/s limit;
its yaw profile re-pins the changed route and recomputed 3D arclengths.

It is **not selected by this recipe or the default runtime**. No successful
airborne verdict exists for this centered route at packaging time; preparation
attempts were blocked by unstable landed translation. Static geometry and
policy tests do not prove actual height tracking. Earlier offline screens
reported 0.145647 m minimum full-sphere clearance in the window band and
0.123913 m minimum over modeled collision parts. Those are ideal-trajectory
calculations, not a new moving-test result.

The original candidate yaw metadata digest was
`e06340f198a1b7d39a3c9759abad9fe43a17371c60c6fa1cd810ab5dbed1e61b`;
only its descriptive scope/route-source strings are made relative here. Its
behavioral route hash, bounds and yaw values are unchanged. Before activating
it in an inactive separate setup, replace *both* route and yaw config, regenerate
and verify the actual deployed manifest, preserve the original recorded-profile
files, run the existing policy/clearance checks and obtain a new run-specific
team approval. No activation is performed or automatically offered here.

## Packaging checks

Four profile tests verify source/config tamper rejection, preserved failed grade,
wrong-camera rejection and centered XY/route-hash separation. Existing runtime
checks remain applicable. Offline actual asset regeneration verified every
recorded visual and camera hash. No simulator, PX4, Jetson, flight, device,
network setting or live working file was changed by this packaging worker.
