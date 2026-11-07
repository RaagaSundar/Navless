# FlyX lab test-flight tools

Drive `planner/px4_bridge_node.py` test flights in the FlyX lab from a laptop agent, and check PX4's
position estimate against simulation truth. Simulation only: the truth recorder reads the
`dronehacks_sim` oracle, which flight code must never consume.

Needs gstack `browse` in headed mode, logged in to the lab Workbench, **holding editing control**.
The bridge runs with `--approve-flight`, so it arms and flies by itself; get the team's OK first.

| File | What it does |
|---|---|
| `labrun.sh "<cmd>" [wait_s]` | Types a shell command into the lab terminal and prints its output. |
| `labfly.sh <tag> ["<extra args>"]` | Reset world (`SKIP_RESET=1` to skip), start `tools/sim_truth_recorder.py`, fly the bridge from the lab clone (`LABDIR=` for another checkout), stream its log, save `est_<tag>.csv` locally. |
| `est_summary.py <csv[.gz]>` | 1 s timeline of truth vs PX4 estimate, cuVSLAM `vo_state`, EKF EV/mag flags, nav state, true/PX4/setpoint heading; first VIO loss and max error. |
| `flight_metrics.py <csv[.gz]>...` | Per flight: armed time, min wall clearance from the course mesh (and where), VIO dropouts, PX4-vs-truth error, touchdowns vs the pad and the launch mark. Run from the repo root. |
| `recordings/` | Recorder CSVs from lab flights (gzip, duplicate rows removed), named `<date>_bridge_<commit>_<run>`. |

Run from a Unix shell (Git Bash on Windows): `bash tools/flyx_lab/labfly.sh f3 "--fast-mode"`

## Results (2026-11-07, fresh Reset world each run)

Bridge args: `--sim-time --approve-flight --launch-origin -2.515 1.114 0.126 --prior gazebo_sim/models/dh_course/meshes/course_visual.obj` plus the extra args shown.
"Armed" is first arm to final disarm: course, pad landing, re-arm, hop over the wall, home landing.

| Run | Code | Extra args | Result | Armed | Min wall clearance | VIO dropouts | Max PX4-vs-truth | Pad / home touchdown error |
|---|---|---|---|---|---|---|---|---|
| f1 | `e4a26c8` | | DONE | 99.8 s | 0.22 m (wall hop) | 0 | 0.14 m | 0.11 / 0.31 m |
| f2 | `e4a26c8` | | DONE | 102.2 s | 0.20 m (wall hop) | 9 (first maze turn) | 0.10 m | 0.03 / 0.34 m |
| b1 | `c967e27` (main) | | **failsafe** | 102.0 s | **0.015 m** (west net) | 14 | 10 m after the crash | 0.45 m / — |
| r1 | `00622e9` | | DONE | 104.5 s | 0.17 m (wall hop) | 0 | 0.10 m | 0.07 / 0.05 m |
| r2 | `f59fd44` | `--speed 0.8` | DONE | 102.5 s | 0.26 m (maze) | 1 | 0.15 m | 0.05 / 0.03 m |
| r3 | `f59fd44` | `--fast-mode` | DONE | 88.1 s | 0.28 m | 0 | 0.19 m | 0.11 / 0.17 m |
| r4 | `f59fd44` | `--fast-mode` | DONE | 90.8 s | 0.25 m | 0 | 0.23 m | 0.09 / 0.07 m |

What changed:
- **b1:** NAV_LAND fired 0.30 m off the pad centre and the drone touched down 0.45 m off. The fly-home re-plan
  from there started sideways while still low, flew into the west net at 0.2 m, cuVSLAM lost the scene and
  PX4's estimate ran away. Fixed in `00622e9`: the fly-home leg climbs straight up to hop height first, and
  an ALIGN state centres within 0.08 m before every NAV_LAND.
- **r1:** the wall hop left only 0.17 m at the drone's centre (wall top 1.72 m; PX4 flew ~0.1 m under the
  hop setpoint). Fixed in `f59fd44`: 0.40 m over the wall (was 0.28).
- **Speed:** `--speed` alone does little. The uniform B-spline's time is stretched until its worst
  acceleration fits `a_max`, so the tightest corner sets the pace. `--fast-mode` scales speed, acceleration
  and jerk (1.65×), and gave about 14% off the whole sequence in two clean runs. PX4's estimate error grows
  with speed (~0.2 m).

Earlier builds lost tracking at the first maze corner and PX4's estimate ran 25–50 m off. With these
fixes no run since r1 has lost tracking there, but it is still the thinnest margin for real-drone VIO.

`tools/sim_px4_mission.py` now models PX4's position gain as 0.95 (`--kp-pos`, MPC_XY_P) instead of 3.4,
which reproduces the lab's ~100 s; its `--fast-mode`/`--speed` screen speed changes before a lab run.
