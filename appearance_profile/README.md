# Opt-in procedural appearance and a matched-pose optical bench

This package preserves the team's 2026-11-07 synthetic appearance work in a
portable, reviewable form. It generates neutral, nonrepeating surface patterns;
these are **synthetic, not scan-derived or measured course reflectance**. It is
separate from setup PR #36, navigation, flight approval and automatic recording.
No default world, source asset, camera calibration, collision model or grader
is changed. The historical local renderer used a different path-bearing
sidecar; this portable schema 2 profile must be generated and reviewed anew.

Requirements: Python 3.10+ and Pillow (`python -m pip install Pillow`). Gazebo,
ROS, CUDA and Docker are not needed to generate assets or run unit tests.

## Reproduce assets without changing the baseline

From this repository, choose a new output directory outside the source tree:

```bash
python appearance_profile/generate.py --source gazebo_sim \
  --world-source gazebo_sim/worlds/dronehacks.sdf \
  --output /path/to/new-appearance --seed 20261107 --pixels-per-m 256
python -m unittest appearance_profile.test_generate appearance_profile.test_integration -v
```

The source must have `models/dh_course/model.sdf`, its `meshes/course_visual.obj`
and `course.mtl`. The world must retain its existing named ground plane visual.
Use the exact source asset snapshot corresponding to the renderer being
compared; newer as-built course geometry is a separate change and requires its
own baseline identity. `--source gazebo_sim` demonstrates the checked-in course,
not a claim that it matches the latest as-built hardware course.

The generator refuses an existing output, overlapping trees and source
symlinks. It preserves exact vertex/normal/face geometry, groups, semantic
painted markings and collision/camera files. Only OBJ UV/material references
and MTL diffuse maps change; generated PNGs and a copied world with an added
ground albedo material are new files. A failed generation can leave a partial
new output: keep it as failure evidence and select another new directory.

Projection atlases use finite metric extents and unique per-surface seeds;
no reused 0.6 m tile or encoded fiducial is introduced. Pattern patches span
2.5–16 cm at 256 pixels/m by default. The separate existing ground visual gets
128 pixels/m over its full extent. These are model assumptions. Lighting, sky,
net transparency, geometry and collisions remain unchanged; assertions and
hashes enforce the generator invariants. Pillow/version differences may affect
PNG bytes: preserve the actual manifest digest and dependency versions.

`appearance.json` records relative renderer paths, source/generated SHA-256
hashes, seed, geometry signature and collision identity. It contains no absolute
host paths. Generation does not establish stereo tracking or flight readiness.
Archive the exact reviewed manifest and source identity with each selected run.
Never treat the procedural textures as a correction to a measured map.

## Explicit renderer integration

In an **operator-owned inactive** split renderer deployment, mount the generated
profile read-only at `/split-appearance` and this package read-only at
`/appearance-tools`. Retain the current qualified vehicle/camera SDF. Do not
replace it with a copied vehicle from the generated profile.

Review and record the SHA-256 of `appearance.json` on the host, then set
`APPEARANCE_MANIFEST_SHA256` to that digest in the renderer environment. Before
the existing `gz sim` command, select and verify the appearance explicitly:

```bash
world=/split-appearance/worlds/dronehacks-synthetic.sdf
export GZ_SIM_RESOURCE_PATH="/split-appearance/models:$GZ_SIM_RESOURCE_PATH"
python3 /appearance-tools/verify.py \
  --overlay /split-appearance --manifest-sha256 "$APPEARANCE_MANIFEST_SHA256" \
  --world "$world" --resource-path "$GZ_SIM_RESOURCE_PATH" \
  --camera /root/assets/dh_quad_stereo.sdf > /root/render-profile.json
# The reviewed existing launcher may now pass "$world" to its gz sim command.
```

This is a reviewable selection recipe, not an installer or an instruction to
restart a shared renderer. The verifier rejects wrong model-search precedence,
wrong world, unpinned manifests, missing/tampered assets and path escape. It
records the retained camera hash for comparison against the approved baseline;
it does not establish the camera's calibration. Preserve the default original
branch and require explicit operator selection of the synthetic branch. No
lifecycle or flight changes are provided by this package.

## Matched-pose static A/B

`static_bench.py prepare` makes a new world with a static stereo rig. It copies
only the original `oak_left` and `oak_right` camera sensor definitions, removes
world GUI/non-sensor plugins, and uses archived physical pose solely as an
**offline optical oracle**. It sends no commands to PX4 or an existing world.
Its two small input schemas are:

```json
{"nearest_physical_sample":{"x":1,"y":2,"z":0.5,"yaw":0}}
```

```json
{"roll_pitch_yaw_ENU_FLU_rad":[0,0,1.57]}
```

Positions are world ENU metres; attitude is already ENU/FLU radians. The
attitude's yaw overrides the pose field. Do not feed raw NED/FRD angles without
an independently reviewed transform. Historical nearest physical samples can
differ from the image timestamp; preserve that uncertainty with input hashes.
Use identical input files for both variants; the script does not infer pose
from an image or claim pixel-exact correspondence to a moving recorded frame.

The bounded optional wrapper requires an **already installed** operator-reviewed
image with Gazebo Harmonic, transport13/msgs10 Python bindings, NumPy, OpenCV
and Xvfb. It does not pull/build images. Select a new output directory:

```bash
APPEARANCE_OVERLAY=/path/to/new-appearance \
  bash appearance_profile/run_static_ab.sh REVIEWED_IMAGE \
  /path/to/baseline-assets /path/to/baseline-world.sdf \
  /path/to/dh_quad_stereo.sdf /path/to/pose.json /path/to/attitude.json \
  /path/to/new-ab-output
```

Each finite container has no network, no host devices, a unique Gazebo
partition, domain 223, read-only assets and bounded CPU/memory. It starts only
its own Gazebo; capture accepts a same-stamp 640×400 stereo pair after one
simulation second, with a 90 s wall timeout. It saves PNGs, source hashes and
static-capture metadata. Matching timestamps are copied under a lock before
writing. Containers are removed on normal exit; interrupted Docker execution
may require the operator to inspect and clean up only their own bench container.
Do not run it during another owner's resource-critical window.

No container or render was run for this packaging. Earlier local matched-pose
renders showed the ground texture loading at outward views; that result does
not validate this new package or temporal stereo/VIO, moving flight or hardware.

## Recording / video boundary

The live team's automatic recording and run-room video tools have separate
lifecycle and archive contracts. They are not copied here, and this package
does not claim to enable automatic recording or run-room video on a fresh clone.
For any future integration, archive the exact `appearance.json` **before** the
run, the verifier's camera/world hashes, original source snapshot, dependency
versions and renderer selection. A chase renderer must use those archived
assets, not today's defaults. Keep Mac SIM and NVIDIA SIM provenance distinct.
Do not classify an optical preview, a map, a video export or a process exit as
a course pass. Preserve failed recordings and independent grader outcomes.

Large PNG atlases, camera captures, videos, bags and raw run archives are
intentionally excluded from Git. Unit tests create small temporary fixtures.

## Provenance and packaging validation

Adapted from team-owned files inspected on 2026-11-07, SHA-256:

| Original module | SHA-256 |
| --- | --- |
| `synthetic_texture_profile.py` | `e5b758cc1bcfd80c8a43640f22df6ea0f993ee451c99db27e7aa4ae47d6c6ffa` |
| `synthetic_texture_profile_test.py` | `76120c5a4c8b36271df39a5b202d412f3ad7d69f66dab36d46e30e067152c9e7` |
| `split_appearance.py` | `4143bfa4409860e05adbf75a9d4d8b7d7adbbff33be3d3c98f84c965a5aaac86` |
| `fixed_pose_texture_ab.py` | `f8204e0f869be109d7279a70f3e8bc4cccaefb2ba4fcb3f3ce40f80e918de5cb` |
| `run_fixed_pose_texture_ab.sh` | `1e715f92592b8a96b467d9190715885cd58f27ac8b4cb3c20b03dbd313e64373` |

Portable changes remove host paths, correct repo-relative defaults, replace a
single deployment's hardcoded manifest digest with an explicit reviewed pin,
add contained-file verification and retain matched-frame snapshots safely.
Only this new directory changes. No experimental planner or flight code is
included. Baseline tests: 16 passed, 4 skipped. Initial existing test venv lacked
Pillow (`ModuleNotFoundError: PIL`); tests then used the installed Pillow Python.
Final generator/integration checks, syntax and source audit are recorded in the
PR. No build, simulator restart, recording lifecycle or flight/hardware action
was performed.
