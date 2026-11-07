# NVIDIA split SITL height-reference correction (draft integration)

The previous range-height configuration treated a downward laser's distance to
raised course geometry as its height reference. At the window sill, the range
fell although launch-frame altitude did not. The reviewed simulation candidate
selects fixed launch-frame visual height (`EKF2_HGT_REF=3`, `EKF2_EV_CTRL=11`),
retains barometer (`EKF2_BARO_CTRL=1`) and conditional real range/terrain aiding
(`EKF2_RNG_CTRL=1`). It neither adds a synthetic position fix nor removes range
checks. These values apply only to the isolated simulated autopilot.

The draft currently packages a dependency-free profile/preflight module and
regression fixtures. The as-flown split runner is being packaged as a retrievable
prerequisite PR under `split_sim/`; its normal launcher/mission integration and
complete installation instructions will be linked here before review readiness.
This directory alone is **not a full-stack installer or an executable mission**.
Do not run or arm an aircraft from these instructions.

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
is separate and is not duplicated. Window-forward yaw remains a separately
staged optional candidate and is excluded until its normal entrypoint is
integrated and validated. Current controls, course geometry, integrity bounds,
recorders, authority/freshness checks and approval requirements stay in scope
for integration review.

Known draft issue identified during the first candidate trial: conditional
range fusion can legitimately suspend at a sill while fixed-frame EV/barometer
remain healthy. The initial extracted `fusion_problem` predicate still mirrors
the overly strict frozen guard and is **not ready for flight integration**.
The pilot owns the correction with actual range freshness, replay and fault
controls. This draft will replace that predicate with the final reviewed
implementation and record the trial's failure before review readiness.

Retrievable prerequisite: [split-runner PR #44](https://github.com/RaagaSundar/Navless/pull/44),
upstream branch `codex/split-sim-portable-install` at
`5da63f59dc78930c04e723dfd518485eb084554a`. This height PR stacks on it.
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
