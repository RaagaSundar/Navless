# master_sim: fly the drone's code against the course

The point: the flight code that goes on skybeta runs **unmodified** in Gazebo against the course,
through the same topics, limits and handover as on the drone. Edit it in this repo, re-run one
command, watch it fly, read the score.

```bash
# WSL Ubuntu-24.04 (ROS 2 Jazzy, Gazebo Harmonic, PX4 v1.16 in ~/PX4-Autopilot)
cd /mnt/c/.../Navless
master_sim/run_master.sh                            # navless_nav with the drone's launch defaults
master_sim/run_master.sh --gui                      # same, with the Gazebo window to watch
master_sim/run_master.sh --route route_gate_b.json  # another route
master_sim/run_master.sh -- -p blind_carry:=true    # override one of the drone's params
master_sim/run_master.sh -- -p fly_home:=true       # the pilot re-arms on the pad for the home leg
DH_TUBE_BLACKOUT=stop master_sim/run_master.sh      # the tube is dark: PX4's vision and the node go blind
master_sim/run_master.sh --brain planner            # Favour's live replanner instead
master_sim/run_master.sh --brain none               # bench: perception only, drone on the launch mark
DH_SIM_ARGS="--odom-noise 0.01 --vio-dropout 20:23" master_sim/run_master.sh   # harder odometry
master_sim/stop_master.sh                           # if a run was cut short
```

Each run boots a fresh sim (about 40 s), flies until the vehicle is back on the ground (or 300 s
after the takeover), prints a summary and stops everything. Code runs from the working tree, so an
edit takes effect on the next run with no build step. The first run builds a small sim-only
interface package (`ros_ws/`, the `VisualSlamStatus` stand-in) into `~/master_sim_ws`.

**Results:** `gazebo_sim/runs/<time>-master-<brain>/summary.json` and `log.csv`. The summary has:
- result (`LANDED`, `PILOT-LANDED`, `TIMEOUT`, `NO-TAKEOVER`, with a `COLLISION +` prefix on any
  contact or negative clearance);
- checkpoint times (including the window crossing) and touchdowns (cm from the pad and from the start);
- minimum wall clearance (props included) and max tilt;
- each blind pass through the tube;
- perception: point-cloud rate, localization rate and error against ground truth.

**Several laps, one table:** `master_sim/regress.py` runs named cases for N laps each and tabulates
pass rate, pad distance, minimum clearance and where it occurred, as median (min–max). One lap can
mislead: the planner was clean through the maze in one run and stalled against M2 in another, on the
same code.

```bash
master_sim/regress.py --list                          # the cases
master_sim/regress.py --laps 5                        # all of them (~4 min per lap)
master_sim/regress.py --laps 3 default gate_b         # before/after a change
```

Results go to `gazebo_sim/runs/regress-<time>/`: `results.json` after every lap, so a cut-short run
keeps what it has, and `table.md` at the end.

**While it runs:** http://localhost:8081 (stereo depth and cloud), http://localhost:8082 (localization).
Logs: `~/dh_sim_logs/master/*.log`.

## What runs, and how faithful it is

| On the drone | In the master sim |
|---|---|
| `navless_nav`, started by `navless_course.launch.py` → `navless.launch.py` | the same node with the same values: `launch_params.py` reads both launch files (route, bearing, speed, odometry topic, every mission param) |
| `planner/px4_bridge_node.py --skybeta-mode` | the same, with `--prior` = the official course mesh |
| `depthcloud/stereo_cloud.py`, `cloud_localize.py` | the same, unmodified |
| OAK-D-S2 on `/oak/{left,right}/{image_raw,camera_info}` | Gazebo cameras on a generated copy of the quad (`make_assets.py`), through `ros_gz_bridge` |
| cuVSLAM `/visual_slam/tracking/odometry`, `/visual_slam/status`, TF | `sim_core.py`: ground truth in the power-up frame, 30 Hz, 50 ms late; `vo_state` 1/2; optional noise, dropouts and the dark tube (`gazebo_sim/dh/tube.py`, the model `sensor_bridge.py` applies to PX4's vision) |
| flight_interface (`/skybeta/setpoint/*`, `/skybeta/state`, takeover rules) | `gazebo_sim/fly_navless.py`'s `FlightInterface`, with the launch file's altitude band and speed cap |
| the safety pilot | `fly_navless.py`'s `Pilot`: virtual sticks, Altitude → arm → Position, once PX4's height has settled and the brain streams; re-arms once with `fly_home:=true`; lands a vehicle that holds still in the air for 8 s |
| PX4 + EKF2 on vision + rangefinder | PX4 SITL, `DH_PX4_PROFILE=drone` (Raaga's real params and estimator setup) |

It reuses Raaga's harness classes, so it needs `gazebo_sim/` from PR #13 (`Pilot`, the takeover rules,
`dh/tube.py`). `gazebo_sim/` itself only gains two opt-in variables in `run_sim.sh` (`DH_MODEL_SDF`,
`DH_EXTRA_MODELS`), so Raaga's own runs behave exactly as before.

Nothing on skybeta publishes `/estimator/healthy`, so the master sim doesn't either. Add
`DH_SIM_ARGS=--estimator-health` to publish it, for a brain that still needs it.

**Fidelity limits.** These are guesses, so treat the numbers they affect accordingly:
- The course mesh is flat grey and stereo needs texture, so `make_assets.py` speckles every face.
  Real wall texture is unknown. The tube's inside is lit in Gazebo, so the stereo cloud keeps working
  there even when the blackout blinds odometry.
- Camera intrinsics and mount are datasheet guesses (80 deg HFOV, 1024x640, 7.5 cm baseline, no pitch).
  Two other sources disagree with each other, a skybeta bench unit (5 cm down, 15 deg pitch) and
  `hardware_slam/run_mapping.sh` (3 cm up, no pitch). Replace them with skybeta-19's own
  `/oak/left/camera_info` and `base_link -> oak_left_camera_optical_frame` TF.
- Cameras render at 5 Hz (`DH_CAMERA_HZ`) to keep the sim real-time on a laptop GPU. The stereo node
  only uses 5 Hz anyway.
- Odometry is perfect unless you add `--odom-noise`, `--vio-dropout` or the tube blackout.

## Results so far

On `main` + PRs #13, #14, #16, #17 and #18 together (2026-11-07), drone profile, pilot modes:

| Run | Result |
|---|---|
| navless_nav, defaults (centreline window route) | **LANDED** 6.1 cm from the pad, 43.4 s after the takeover. Window crossed 3.1 cm off-centre at 0.78 m. Min clearance 15.5 cm at C2a, no contacts |
| navless_nav, `DH_TUBE_BLACKOUT=stop`, defaults (carrot freezes) | **COLLISION**: blind three times around the tube (3.2 s, 2.0 s, 6.5 s), into the tube at −8.3 cm, 54 deg tilt, PX4 failsafe, down 3 m from the pad |
| navless_nav, `DH_TUBE_BLACKOUT=stop`, `blind_carry:=true` | **LANDED** 7.3 cm from the pad, 43.6 s. One clean 3.0 s blind pass, min clearance 16.0 cm at C2a, no contacts |
| planner, `--skybeta-mode` (real `vo_state`, no `/estimator/healthy`) | **COLLISION**: maze, Gate A and tube clean (22–56 cm). Leaving the tube for the chicane, its setpoint jumps 2.2 m (past the 0.55 m cap); it then flies the chicane low (0.7 m), hits C2a under the window at 42 s and flips |

These agree with Raaga's `fly_navless.py` laps (6–9 cm from the pad, ~43 s, 15–16 cm at C2a; freeze
fails in the tube, blind carry passes), with real ROS 2 and the stereo/localization stack running.

Earlier (on `main` before PR #13):
- **navless_nav, `route_gate_b.json`:** landed 1–5 cm from the pad, but drifted to 3 cm from Barrier 1
  on the climb-out. PR #16's takeoff lean fixes that.
- **planner before `b78096a`:** crashed on the first `/skybeta/state` (`import json` missing). Its
  setpoint ran ahead on spline time and flew it through M1. On `b78096a` it cut the M1 corner (−0.7 cm)
  and stalled against M2 in one run; it was clean through the maze in the run above.
- **cloud_localize never locks on.** On the bench the stereo cloud matches the course mesh to 1.7 cm
  median at the true pose, yet ICP reports fitness 0.13 ("poor fit"), so it stays on odometry. It also
  starts 12 cm off (camera put at the launch mark at 0.2 m; it is 11.5 cm forward at 0.08 m). In flight
  its error was 24–103 cm across runs. Reproduce with `--brain none`.
- **Heading:** flight_interface's pose setpoints carry no yaw, so the drone keeps its launch heading
  and the cameras face north for the whole course.

Object detection isn't in yet: add it as another `start` line in `run_master.sh` when it exists.
