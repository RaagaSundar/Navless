# NVIDIA split SITL height-reference correction (draft integration)

The previous range-height configuration treated a downward laser's distance to
raised course geometry as its height reference. At the window sill, the range
fell although launch-frame altitude did not. The reviewed simulation candidate
selects fixed launch-frame visual height (`EKF2_HGT_REF=3`, `EKF2_EV_CTRL=11`),
retains barometer (`EKF2_BARO_CTRL=1`) and conditional real range/terrain aiding
(`EKF2_RNG_CTRL=1`). It neither adds a synthetic position fix nor removes range
checks. These values apply only to the isolated simulated autopilot.

The normal runner is published under `split_sim/`, with the final frozen
215431 companion and explicit conditional-range policy integrated in the
standard `run_split_course.sh` → `course_runtime/mission.py` path. Follow the
[installation guide](../split_sim/INSTALL.md). The moving candidate result is
still pending; this is a reviewable draft, not flight approval.

From a clean checkout (Python 3.10+, no ROS/Docker/aircraft required):

```bash
python3 -m unittest split_height.test_height_profile -v
python3 split_height/height_profile.py shell --profile ev-terrain
```

The second command prints four reviewed environment exports; it does not apply
them or start a process. `check --profile ev-terrain --listener FILE
--parameters FILE` checks actual passive listener/parameter records and exits
nonzero on missing/stale input, mismatched settings or absent aiding. CLI output
is configuration evidence, not flight approval. The tiny sill test explains why
AGL and fixed launch height differ; it is not an estimator or a sensor source.

As-flown source reference: frozen companion `mission.py` SHA-256
`3a3166edb5adc59de48a577137705ce4c0d8865234c7811fee548f2f548ea6be`;
`px4_navigation.py` SHA-256
`0dfc45c51a923d3f6c94a7fa8261f8b3038def9d568e3da693b7236ae34071a4`.
The recorded candidate's actual parameter readback was HGT_REF=3, EV_CTRL=11,
RNG_CTRL=1, BARO_CTRL=1. Its moving trial was still in progress when this draft
was opened; no course pass or hardware qualification is claimed.

[Open-net appearance PR #41](https://github.com/RaagaSundar/Navless/pull/41)
is separate and is not duplicated. Window-forward yaw is included as an explicit optional profile; its moving
qualification remains pending. Current controls, course geometry, integrity bounds,
recorders, authority/freshness checks and approval requirements stay in scope
for integration review.

The first candidate stopped at the sill because its new guard required
continuous optional range/terrain fusion. That failure is preserved. The
integrated `WindowRangeHealth` now allows a finite exception only for a fresh
sill-sized measured step inside the mapped aperture, established prior normal
fusion, airborne authority, fixed EV/barometer health and measured clearance.
It pairs the range event with acquisition-aligned PX4 position history. Stale,
invalid/out-of-range or unexplained range, faults, lost authority, an out-of-
corridor position or expired transition still reject. Independent vision/PX4
residual, uncertainty, reset and source-freshness checks remain unchanged.
No continuous optional range-fusion flag is required during that bounded event.
Prearm continues to require actual normal range/terrain fusion.

`split_height/height_profile.py` is a passive profile/readback utility; the
companion uses the actual frozen `conditional_range.py` policy, not a duplicated
utility predicate. Source manifests retain both historical snapshots and pin
all 38 actual deployed runtime members. Only the yaw config's descriptive
route-source string is changed to the tracked relative `astar-route.json`.

The optional `SPLIT_YAW_POLICY=window-forward` is now wired through the normal
runner to the route-hash-pinned yaw module. It preserves the initial exterior
look, aligns forward through the window at at most 10°/s, holds during COAST
and LAND, and uses the independently registered PX4 yaw transform. Its tests
verify conventions, rate bounds, phase holds and image geometry. It is a
camera candidate, not a promise of tracking or successful flight.

Retrievable prerequisite: [split-runner PR #44](https://github.com/RaagaSundar/Navless/pull/44),
upstream branch `codex/split-sim-portable-install` at
`5da63f59dc78930c04e723dfd518485eb084554a`. It merged before final integration; this follow-up targets main.
Its [installation guide](../split_sim/INSTALL.md) supplies tracked dependencies,
Mac image/PX4 build, organizer Jetson image checks, asset generation, disarmed
startup, camera/estimator profile selection, preview/video workers and actual
health/grade inspection. The organizer's NVIDIA image and authorized fleet
access remain explicit external prerequisites. No credentials are distributed.

The Range source/transport in this follow-up mirrors exactly the existing
modeled, centimeter-quantized PX4 laser reading to `/split/downward_range`,
retaining the same acquisition stamp and invalid/out-of-range behavior. It
creates no new position source. The producer is simulation-only and uses
Gazebo truth solely inside the separate ray-cast sensor model; navigation sees
ordinary ROS Range. The native startup sources Jazzy before starting it.

Installed Jazzy serialization into installed Jetson Humble deserialization was
verified on common fields: frame `downward_range_link`, time 128.776 s, range
0.47 m survived. Jazzy's trailing Range variance is absent in Humble and is not
used by the consumer; this does not establish general ROS ABI compatibility.
A separate live matched-source observation preserved 84/84 exact centimeter
values over simulation 7.544–10.532 s, ranges 2–5 cm while landed. The tiny
[evidence summary](evidence/range-common-fields.json) records that comparison;
it is not an airborne transport/estimator qualification or a flight pass.

## Checks and retained replay

```bash
python3 split_sim/doctor.py --source-only
python3 -m unittest split_sim.test_install split_height.test_height_profile -v
python3 -m unittest discover -s split_sim/course_runtime -v
python3 -m unittest discover -s split_sim -p test_sensor_link.py -v
```

Runtime yaw tests need NumPy; install the project's declared Python dependencies
as in the install guide. The historical full ULog test skips without its optional
`pyulog` dependency and retained raw log, which is intentionally not in Git.
For the actual prior conditional-range interval, preserve your original ULog
and measured registration, then run the offline-only script:

```bash
python3 split_height/replay_range.py --ulog /path/to/retained-failure.ulg \
  --control-bearing 90 --start-bearing 0 --start 133 --end 136.30
```

Supply the actual frozen datum using `--datum-ned` if nonzero; the example
reproduces the reviewed replay's zero-XY-datum assumption. The tiny
[evidence events](evidence/conditional-replay.json) show normal fusion at
133.000 s, a bounded measured transition at 134.620 s, and restored normal
updates at 136.160 s. This is a narrow counterfactual guard replay with airborne
authority assumed over the retained interval; it is not a full flight replay,
proof of recovery, or a fabricated grade. No raw log or physical truth is
required by the ordinary unit tests. [Runtime identity](evidence/runtime.json)
records the source/packaged manifest identities and pending moving result.
