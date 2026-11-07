# FlyX lab test-flight tools

Drive `planner/px4_bridge_node.py` test flights in the FlyX lab from a laptop agent, and check PX4's
position estimate against simulation truth. Simulation only: the truth recorder reads the
`dronehacks_sim` oracle, which flight code must never consume.

Needs gstack `browse` in headed mode, logged in to the lab Workbench, **holding editing control**.
The bridge runs with `--approve-flight`, so it arms and flies by itself; get the team's OK first.

| File | What it does |
|---|---|
| `labrun.sh "<cmd>" [wait_s]` | Types a shell command into the lab terminal and prints its output. |
| `labfly.sh <tag> ["<extra args>"]` | Reset world (`SKIP_RESET=1` to skip), start `tools/sim_truth_recorder.py`, fly the bridge from the lab clone, stream its log, save `est_<tag>.csv` locally. |
| `est_summary.py <csv[.gz]>` | 1 s timeline of truth vs PX4 estimate, cuVSLAM `vo_state`, EKF EV/mag flags, nav state, true/PX4/setpoint heading; first VIO loss and max error. |
| `recordings/` | Recorder CSVs from lab flights (gzip, duplicate rows removed). |

Run from a Unix shell (Git Bash on Windows): `bash tools/flyx_lab/labfly.sh f3`

## Results so far (2026-11-07, `main` @ `e4a26c8`, fresh Reset world)

Bridge args: `--sim-time --approve-flight --launch-origin -2.515 1.114 0.126 --prior gazebo_sim/models/dh_course/meshes/course_visual.obj`

| Run | Sequence | cuVSLAM | PX4 vs truth (airborne) | Notes |
|---|---|---|---|---|
| f1 | DONE | never lost | max 0.14 m | clean |
| f2 | DONE | 9 dropouts < 0.3 s in the first maze turn (17–20 s after arm) | max 0.08 m | VIO grace hold rode through; commanded yaw led true heading by 35–45° in that turn. Drone yawed ~37° on the pad after touchdown; estimate wandered 0.42 m for ~4 s while disarmed, settled before re-arm. |

Earlier builds lost tracking at that same first maze corner and PX4's estimate ran 25–50 m off.
That corner is still the thinnest margin for real-drone VIO.
