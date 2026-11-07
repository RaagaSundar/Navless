# Reproduce the real bounded M3 capture pair

| Named experiment | Immutable run | Declared finite goal |
| --- | --- | --- |
| **Changed M3 −18° · bounded stereo capture** | `20261107T231918Z-split-astar-flight` | Met; zero contacts, normal land/disarm |
| **Nominal M3 · bounded stereo capture** | `20261107T232429Z-split-astar-flight` | Met; zero contacts, normal land/disarm |

These are real moving PX4/NVIDIA data captures, **not full-course passes or
successful obstacle avoidance**. Their unchanged ordinary full-course grades
remain incomplete in `evidence/`. No shadow detector receipts were available;
no detector decision controlled the aircraft. Actual stop observations are
individual transients, not guaranteed braking bounds.

## What changed and what was already published

Merged #49 supplies all39 executed core files, identical to both frozen runs.
Their47-member historical archives also contained eight unused return/registration
files, deliberately excluded here. This package adds the exact frozen capture
wrapper/233-point nominal prefix/contract, portable actual-scene generation,
range-model scene selection, prepared-launch wiring and independent replay.
#57 is the separate as-built MAVLink gate/side-ToF toolset; viewer #54/#56 is
separate. No duplicate core controller, hardware bench or viewer is added.

Changed M3 rotates its visual **and real collision** −18° about the north tip
`(0.3048, 2.12085)`m. Nominal restores original geometry. Other scene appearance,
world, original onboard prior, prefix, sensors, calibration, first-exterior yaw
and control contract remain unchanged. Fresh initialization/noise/trajectories
differ; this is not a claim that all measurements are identical.

Target pacing is0.20m/s, then0.08m/s near the endpoint. At prefix index35 the
wrapper commands actual position hold for≥2s until measured speed<0.03m/s for
0.5s;5s timeout requests normal landing if native health/authority allows.
Endpoint hold3s then uses the same sole Interface's normal PX4 Land handoff.
No fake position/Carrot completion/pad landing or grader modification occurs.
Original authority, freshness, frame, estimator, range, recorder and approval
gates remain. Shadow messages are logged if present, never used for control.

## Clean checkout and offline checks

Until this PR merges, use the actual published branch rather than assume main:

```bash
git clone --branch codex/m3-bounded-recipes https://github.com/zoutei/Navless.git Navless-m3
cd Navless-m3
python3 split_sim/doctor.py --source-only
python3 split_sim/m3_bench/mission_m3_capture.py --runtime split_sim/course_runtime \
  --contract split_sim/m3_bench/m3-capture-contract.json --check-contract
python3 -m unittest discover -s split_sim/m3_bench -p 'test_*.py' -v
```

Geometry/replay tests need NumPy; image generation also needs Pillow. Mocked
wrapper tests need no ROS. The [install guide](../INSTALL.md) provides Mac
Docker/PX4, organizer Jetson Humble/NVIDIA image, verified SSH, stopped isolated
container and preview/video prerequisites. No fresh full-image build or moving
runtime qualification of this portable integration is claimed.

## Generate both actual scenes without running a simulator

Use new immutable output directories. First regenerate the original neutral
appearance using the published passing-profile chain:

```bash
SPLIT_CAMERA_RATE_HZ=31.25 python3 split_sim/make_assets.py split_sim/state/m3-base
python3 appearance_profile/generate.py --source split_sim/state/m3-base \
  --world-source gazebo_sim/worlds/dronehacks.sdf --output split_sim/state/m3-neutral \
  --seed 20261107 --pixels-per-m 256
python3 split_sim/m3_bench/scene.py build --source split_sim/state/m3-neutral \
  --output split_sim/state/m3-changed --route split_sim/course_runtime/astar-route.json --variant changed
python3 split_sim/m3_bench/scene.py build --source split_sim/state/m3-neutral \
  --output split_sim/state/m3-nominal --route split_sim/course_runtime/astar-route.json --variant nominal
python3 split_sim/m3_bench/screen_m3_capture.py \
  --route split_sim/m3_bench/m3-capture-route.json \
  --clean split_sim/state/m3-nominal/assets/models/dh_course/model.sdf \
  --changed split_sim/state/m3-changed/assets/models/dh_course/model.sdf \
  --output split_sim/state/m3-prefix-screen.json
```

These generate/screen geometry only. At5mm spacing, the complete prefix and
terminal landing column have0.10510m minimum margin in both worlds for a0.30m
sphere enclosing every modeled part. This is ideal geometry, not a dynamic
bound. The altered full-course path still intrudes into M3; the separate oracle
feasibility corridor is **evaluation-only, never supplied to navigation**.
Regeneration matched the archived changed collision SDF `32d5e02c...` and OBJ
`272f872b...` byte-for-byte (full hashes in `evidence/PROVENANCE.json`). Portable
receipts have relative contained members/new hashes, not the old host paths.

## Explicit inactive scene selection and disarmed startup

Coordinate domain219 ownership and a fresh disarmed world. Physical domain19
is excluded. For nominal, replace `m3-changed` with `m3-nominal`:

```bash
cd split_sim
export COMPOSE_FILE=compose.yaml:compose.m3.yaml
export SPLIT_M3_SCENE_DIR="$PWD/state/m3-changed"
export SPLIT_M3_BUNDLE_SHA256="$(shasum -a256 "$SPLIT_M3_SCENE_DIR/bundle.json" | awk '{print $1}')"
export SPLIT_INERTIAL=0 SPLIT_CAMERA_RATE_HZ=31.25 SPLIT_HEIGHT_PROFILE=ev-terrain
export SPLIT_YAW_POLICY=first-exterior SPLIT_EXPERIMENT=m3-bounded
bash configure_estimator.sh stereo  # Separate simulation container stopped.
bash start.sh --split
bash start_px4_from_mac.sh
```

The explicit compose override mounts the actual scene only on the **Mac**.
Renderer and separate oracle range-sensor model verify the same pinned physical
collision SDF. Range noise/mount/cadence/quantization are unchanged. The Jetson
receives the original nominal route/prior and ordinary sensors. Altered geometry
stays outside portable navigation; defaults without the override are unchanged.

Verify actual renderer process/receipt, camera hash/calibration/TF, source stamps,
delivered stereo/odom/Range rates, selected PX4 parameters, tracking, fresh
landed/disarmed state and independent recorder. Then the normal entrypoint
stages the exact capture bundle and starts passive preparation:

```bash
bash run_split_course.sh --prepare-flight
```

This waits for approval and does not authorize arming. Review the bounded plan,
scene receipts, contract/screen and fresh telemetry; obtain explicit run-specific
team approval using the existing workflow. No concurrent controller. After
keyboard RC: land, release RC and **Reset world** before automation. Preserve
failures; stop publication on lost authority/stale telemetry.

The runner archives scene/prior receipts, wrapper/route/contract, native
telemetry, genuine stereo and independent truth/contact evidence. The original
full-course grader still runs unchanged and is expected incomplete. Separate
`m3-capture-result.json` reports only the finite goal; physical clearance uses
archived **actual** geometry and full attitude offline. Keep those meanings
separate in review and preview records.

## Replay retained sealed evidence without simulator operations

Raw captures are not in Git. Define `$RUN` as a retained run directory,
`$SEALED_SQLITE` as its sealed genuine camera archive, `$ACTUAL_SDF` as its actual
changed/nominal model, and new immutable output paths:

```bash
python3 split_sim/m3_bench/evaluate_m3_capture.py \
  --contract "$RUN/m3-capture/m3-capture-contract.json" --route "$RUN/flight/planned-route.json" \
  --events "$RUN/m3-capture-events.jsonl" --telemetry "$RUN/flight/telemetry.jsonl" \
  --truth "$RUN/flight/truth.json" --actual-course-sdf "$ACTUAL_SDF" \
  --sensors "$SEALED_SQLITE" --ordinary-grade "$RUN/flight/grade.json" \
  --name 'M3 bounded stereo capture' --experiment-id "$(basename "$RUN")" \
  --baseline-source-id 20261107T221438Z-split-astar-flight \
  --world-difference 'explicit actual variant; original prior unchanged' --output "$NEW_RESULT"
python3 split_sim/m3_bench/physical_clearance.py "$RUN/flight/track.json" \
  --map-comparison "$RUN/map-comparison.json" --repo "$PWD" --output "$NEW_CLEARANCE"
```

Missing valid stereo, actual motion, normal Land/landed-disarm, endpoint accuracy,
verified contacts or positive clearance prevents a finite pass. The original
grade is never changed. Both actual retained replays were run with this port
and regenerated SDFs: goals/reasons, endpoint/clearance and braking metrics
matched sealed reports. Shadow detector was **unavailable**, control use **false**.
Changed/nominal stops measured about2.508/2.808s and3.94/5.79cm forward excursion;
these are individual observations, not universal braking guarantees. Named
compact evidence preserves source hashes; no raws, private host paths,
credentials, large assets or experimental avoidance/return code is included.
