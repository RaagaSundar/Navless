# master_sim: fly the drone's code against the course

The point: the flight code that goes on skybeta runs **unmodified** in Gazebo against the course,
through the same topics and limits as on the drone. Edit it in this repo, re-run one command, watch
it fly, read the score.

```bash
# WSL Ubuntu-24.04 (ROS 2 Jazzy, Gazebo Harmonic, PX4 v1.16 in ~/PX4-Autopilot)
cd /mnt/c/.../Navless
master_sim/run_master.sh                            # navless_nav flies route_gate_b.json
master_sim/run_master.sh --gui                      # same, with the Gazebo window to watch
master_sim/run_master.sh --route route_window.json
master_sim/run_master.sh -- -p cruise_speed_mps:=0.4      # override one of the drone's params
master_sim/run_master.sh --brain planner            # Favour's live replanner instead
master_sim/run_master.sh --brain none               # bench: perception only, drone on the launch mark
DH_SIM_ARGS="--odom-noise 0.01 --vio-dropout 20:23" master_sim/run_master.sh   # harder odometry
master_sim/stop_master.sh                           # if a run was cut short
```

Each run boots a fresh sim (about 40 s), flies until the vehicle is back on the ground (or 300 s
after arming), prints a summary and stops everything. Code is run from the working tree, so an edit
takes effect on the next run with no build step.

**Results:** `gazebo_sim/runs/<time>-master-<brain>/summary.json` and `log.csv`. The summary has:
- result (`LANDED`, `PILOT-LANDED`, `TIMEOUT`, with a `COLLISION +` prefix on any contact or negative
  clearance);
- checkpoint times and touchdowns (cm from the pad and from the start);
- minimum wall clearance (props included) and max tilt;
- perception: point-cloud rate, localization rate and error against ground truth.

**While it runs:** http://localhost:8081 (stereo depth and cloud), http://localhost:8082 (localization).
Logs: `~/dh_sim_logs/master/*.log`.

## What runs, and how faithful it is

| On the drone | In the master sim |
|---|---|
| `navless_nav` with `navless.launch.py` defaults | the same node and the same values: `launch_params.py` reads them from the launch file |
| `planner/px4_bridge_node.py --skybeta-mode` | the same, with `--prior` = the official course mesh |
| `depthcloud/stereo_cloud.py`, `cloud_localize.py` | the same, unmodified |
| OAK-D-S2 on `/oak/{left,right}/{image_raw,camera_info}` | Gazebo cameras on a generated copy of the quad (`make_assets.py`), through `ros_gz_bridge` |
| cuVSLAM `/visual_slam/tracking/odometry` + TF | `sim_core.py`: ground truth in the power-up frame, 30 Hz, 50 ms late; optional noise and dropouts |
| flight_interface (`/skybeta/setpoint/*`, `/skybeta/state`) | `sim_core.py` runs `gazebo_sim/fly_navless.py`'s `FlightInterface`, with the launch file's altitude band and speed cap |
| PX4 + EKF2 on vision + rangefinder | PX4 SITL, `DH_PX4_PROFILE=drone` (Raaga's real params and estimator setup) |
| the pilot arming / landing | arms when the brain streams while landed and disarmed (twice for navless, for the fly-home leg); lands the drone if it holds still in the air for 8 s |

`gazebo_sim/` is unchanged apart from two opt-in variables in `run_sim.sh` (`DH_MODEL_SDF`,
`DH_EXTRA_MODELS`). Raaga's own runs behave exactly as before.

**Fidelity limits.** These are guesses, so treat the numbers they affect accordingly:
- The course mesh is flat grey and stereo needs texture, so `make_assets.py` speckles every face.
  Real wall texture is unknown.
- Camera intrinsics and mount are datasheet guesses (80 deg HFOV, 1024x640, 7.5 cm baseline). Replace
  them with skybeta's `/oak/left/camera_info` and TF.
- Cameras render at 5 Hz (`DH_CAMERA_HZ`) to keep the sim real-time on a laptop GPU. The stereo node
  only uses 5 Hz anyway.
- Odometry is perfect unless you add `--odom-noise` / `--vio-dropout`. No tube blackout.

## What it has found so far (2026-11-07)

- **navless_nav, `route_gate_b.json`:** lands 1.1 cm from the pad centre, 48 s after arming, no
  contacts. Closest call is during the climb-out on the launch mark: it drifts 15 cm south to 3 cm from
  Barrier 1 while PX4 limits tilt at takeoff (`MPC_TILTMAX_LND` 12 deg).
- **planner, before `b78096a`:** it crashed on the first `/skybeta/state` message (`import json`
  missing). After that was fixed, the setpoint ran on spline time instead of the drone's progress: at
  arming it was 1.4 m up the route, and the drone flew through the M1 wall and flipped.
- **planner, on `b78096a` (`max_lead_m` carrot cap):** it clears the corridor (10.4 s). Then it grazes
  the M1 wall (-0.7 cm by the prop-disc clearance model, no Gazebo contact, 9.9 deg tilt) and stops in
  the maze, until the pilot stand-in lands it 40 s after arming. Its new pad landing
  (touch-and-go / `--stop-on-pad`) was not reached, so it is untested.
- **planner: `/estimator/healthy` has no publisher on skybeta.** The planner won't fly without it.
  `sim_core.py` publishes it here.
- **cloud_localize never locks on.** On the bench (drone still on the launch mark) the stereo cloud
  matches the course mesh to 1.7 cm median at the true pose, yet ICP reports fitness 0.13 ("poor
  fit"), so it stays on odometry. It also starts 12 cm off: it puts the camera at the launch mark at
  0.2 m, but the camera is 11.5 cm forward of it at 0.08 m. In flight, its error against ground truth
  was 45–103 cm across runs. That matters, because the planner prefers `/team/localize/pose` when it is
  fresh. Reproduce with `--brain none`.
- **Heading:** flight_interface's pose setpoints carry no yaw, so the drone keeps its launch heading
  and the cameras face north for the whole course.

Object detection isn't in yet: add it as another `start` line in `run_master.sh` when it exists.
