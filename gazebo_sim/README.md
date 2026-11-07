# DroneHacks course in Gazebo + PX4 SITL

A local stand-in for the FlyX Lab sim. It runs Gazebo Harmonic and PX4 SITL in WSL Ubuntu, with the official
course v3 and a quad built on the real airframe's rotor geometry. A Python mission flies the whole course
autonomously with one start: launch → corridor → maze → Gate A → tube → window → landing pad.

## Run it

Run everything from WSL (`wsl -d Ubuntu`), in this folder (`/mnt/c/Users/sunda/Navless/gazebo_sim`).

```bash
./run_sim.sh --gui                      # Gazebo + PX4, drone on the launch mark (drop --gui for headless)
python3 fly_course.py --preset race     # one autonomous run: arm, fly, land, score
./stop_sim.sh
```

Drone-like run: the real controller and estimator, and the real 10 Hz position-only interface.

```bash
DH_PX4_PROFILE=drone ./run_sim.sh
python3 fly_course.py --interface pos10 --v-max 1.5 --a-lat 1.5 --a-long 1.5 --v-window 0.8 --v-tube 1.5 --abort-error 0.6
```

Each run writes `runs/<time>-<tag>/` with `summary.json` (result, checkpoint times, touchdown time and pad offset,
minimum clearance, tracking error, contacts), `log.csv` and `run.png` (planned vs flown, top down).

Other tools:

- `python3 plan.py --preset race`: check the route offline (length, time, clearance per stage, plot).
- `./sweep.sh "tag|ENV=..|args" ...`: several runs back to back, each on a fresh sim, one summary line per run.
- `python3 tools/export_route.py`: writes the centreline route as `course/route_centreline_window.json`
  (`waypoints_enu` in world metres) for other flight code.
- `DH_INSTANCE=N ./run_sim.sh` with `python3 fly_course.py --instance N`: isolated parallel sims (own GZ_PARTITION,
  PX4 `-i N`, MAVLink port 14540+N).

## Profiles (`DH_PX4_PROFILE`)

| Profile | Controller | Estimator |
|---|---|---|
| `sim` (default) | tuned for the 50 Hz feedforward follower (`MPC_XY_P` 1.2, `MPC_XY_VEL_P_ACC` 4.5, short takeoff ramp, full tilt at takeoff) | ideal: true pose as vision (position, height, velocity, yaw) |
| `stock` | the real airframe's values (PX4 stock gains, 3 s takeoff ramp, 12° takeoff tilt, 0.7 m/s land, 2 s disarm) | ideal |
| `drone` | as `stock` | the real setup, via `sensor_bridge.py` (below) |

Under `drone`, the model's built-in pose feed is hidden from PX4, and `sensor_bridge.py` delivers the sensors the
way the drone does:

- **Vision:** 30 Hz in an FRD frame anchored at the start pose. Each sample arrives after a random latency
  (lognormal, p50 60 ms, p95 106 ms) with 5 mm noise. PX4 fuses horizontal position and yaw only
  (`EKF2_EV_CTRL` 9, `EKF2_EV_DELAY` 60). `DH_TUBE_BLACKOUT=stop` (or `1`) cuts vision inside the dark tube, below
  its roof, and bumps the reset counter when it returns. `freeze` repeats the last tracked pose instead, which is
  what PX4 would get if cuVSLAM republished it, since the drone's bridge forwards everything.
  `DH_TUBE_RECOVER_DELAY=s` brings tracking back late. `DH_TUBE_RESET_JUMP=dx,dy,dz` (odom metres) or `resume`
  (restart from the last tracked pose, as if the drone had not moved while blind) jumps the pose on recovery.
  `fly_navless.py` reads the same variables and blinds the node's odometry identically (`vo_state` 2), so export
  them once for both. Logic: `dh/tube.py`; checks: `dh/test_tube_blackout.py`.
- **Height:** a downward laser rangefinder, 2.7 cm left of and 3.4 cm below the body origin, 0.02–4 m. It is
  ray-cast against the course, so it sees boulders, cradles and the floor. It's the height reference while low
  and slow (`EKF2_HGT_REF` 2, `EKF2_RNG_CTRL` 1), with the barometer behind it.

## Follower interfaces (`fly_course.py --interface`)

- **`ff` (default):** position, velocity and acceleration setpoints at 50 Hz over MAVLink, timed by Gazebo's
  clock. Acceleration is sent 0.12 s early, because PX4 needs about 0.1 s to tilt. This is the fastest the model
  can fly; it only works in the sim.
- **`pos10`:** position-only setpoints at 10 Hz, as the drone's flight interface forwards them. It follows a
  carrot along the planned path, steering on PX4's own estimate. The lead is planned speed × `--carrot-time`,
  shortened so the line to the carrot never strays more than `--carrot-dev` (4 cm) from the path. Poses are
  floored at 0.3 m.
- **`vel10`:** velocity-only at 10 Hz. It overshoots turns on stock gains; not safe as written.

## Results (Gazebo, liftoff → touchdown on the pad)

| Run | Time | Pad offset | Min clearance | Mean tracking error |
|---|---|---|---|---|
| `--preset safe`, `sim` profile | 13.14 s | 5.0 cm | 13.0 cm | 6.7 cm |
| `--preset fast`, `sim` profile | 9.91 s | 5.1 cm | 12.6 cm | 9.0 cm |
| `--preset race`, `sim` profile (55° tilt) | **7.88 s** | 2.4 cm | 9.9 cm | 15.6 cm |
| `--interface pos10`, `stock` profile | 34.9 s | 6.9 cm | 12.3 cm | 4.8 cm |
| `--interface pos10`, `drone` profile | **34.4 s** (39.4 s from arm) | 3.0 cm | 12.1 cm | 6.5 cm |
| same with `DH_TUBE_BLACKOUT=1` | 34.4 s | 4.8 cm | 11.9 cm | 6.6 cm (vision lost for 1.4 s in the tube) |

All runs were clean (no contacts, every checkpoint). Under the stock controller, liftoff comes about 4.8 s after
arming (1 s spool-up, 3 s takeoff ramp, 12° takeoff tilt). The real flight node was also flown on this sim with
`tools/export_route.py`'s route, and landed both times.

**Window route: placement and the crossing shift.** The node's carrot cuts the turn before the window, so it
crosses off-centre towards C2a_a: 2.8 cm off at 0.5 m/s, 5.5 cm at the course launch's 0.7 m/s. Turning the
nose east of north adds to that offset; west takes it away.

The route therefore aims its crossing 4 cm the other way (`export_route.py --window-shift-m`, default 0.04; 0 is
the geometric centreline). Window-wall clearance, `drone` profile, `--pilot modes`, one lap each unless noted:

| Placement | 0.7 m/s, unshifted | 0.7 m/s, shifted | 0.5 m/s, unshifted | 0.5 m/s, shifted |
|---|---|---|---|---|
| 1° east | 7.4 cm | 11.5 cm (2 laps) | 10.0 cm | 14.3 cm |
| north | 12.9–13.6 cm (4 laps) | 16.2 cm | 15.4 cm | — |
| 1° west | maze-limited, 12.3 cm | 20.4 cm | 17.7 cm | 17.2 cm (C2a_b) |

With the shift, ±1° keeps at least 10.9 cm to every wall at both speeds. The limit is then the maze (M3, at
1° east and 0.7 m/s), not the window.
Boulders stay at least 18.7 cm away, and the planned minimum is still 16.6 cm (M1).

**Gate B route.** `tools/export_route.py --route gate_b` writes `course/route_centreline_gate_b.json` (18.0 m),
the chicane alternative to the window. Past the tube it turns on arcs centred on three wall ends: round C1's
south end, up the chicane, west through Gate B, then round A1's south end to the pad.
- **Planned nose-north clearance:** 17.4 cm to C1 and C2b, 17.7 cm to A1. The overall minimum is 16.6 cm, at
  M1 in the maze it shares with the window route.
- **One flight of the node** (`drone` profile, `--pilot modes`): it went through Gate B and landed 9.8 cm from
  the pad, 48.0 s after arming, with no contacts.
- **Flown clearance:** 10.0 cm from A1 and 12.2 cm from C2b. The node's carrot cuts the inside of those two turns.

**Camera blind in the tube, node included** (`fly_navless.py`, `drone` profile, centreline route, `--pilot modes`).
- **`DH_TUBE_BLACKOUT=stop`.** Today's node freezes its carrot when `vo_state` goes to 2, and PX4 dead-reckons.
  Its estimate was 7 cm off after 1 s blind, 0.41 m after 2 s and 0.80 m after 2.5 s, so the hold wandered:
  1. Out of the south end.
  2. Back in on recovery: blind twice, for 3.1 s and 2.3 s.
  3. Out of the north mouth, where an arm caught the tube's west rim. It sat there 1 m short of the carrot, with
     PX4's estimate within 1 cm of truth, until the 20 s stall hold and a pilot landing.
- **`freeze` with `DH_TUBE_RESET_JUMP=resume`.** It got through the tube: PX4, fed the frozen pose, kept flying
  towards the frozen carrot. But restarting from the last tracked pose moved the rest of the route 1.52 m south,
  into the south net.
- **Both runs.** The node's odometry and PX4's vision lost and regained tracking together, and jumped by the same
  amount: both processes computed the same 1.524 m.

**Takeoff drift (worth checking on the real drone).** With the measured rotor layout and the centre of mass at
the allocation origin, the front motors carry about 65% of the hover thrust. PX4's `THR_MDL_FAC` is 0 on the real
airframe: it treats thrust as linear in command. The simulated motors are quadratic, so the front/rear split comes
out wrong and the nose pitches up at liftoff. The drone drifts about 15 cm backward while it climbs, towards the
barrier behind the launch mark. That's the closest point on the course for a vertical takeoff.
`PX4_PARAM_THR_MDL_FAC=0.72` (the fit for this motor model) removes the drift: 13.7 cm clearance, 3.4 cm mean
error on the `stock` + `pos10` run. All profiles keep the real 0 by default.

## How it is built

- `models/dh_course/` is generated by `tools/build_course_model.py`. The visual is the official OBJ minus the net,
  with normals added. The collisions are primitives built from `course/course_v3.json`: walls, the window wall split
  around its opening, the tube as a ring of boxes on its 58 × 72 in oval, boulders as half-ellipsoids, and the net.
  The official OBJ has no normals, which crashes DART's mesh collision.
- `models/dh_quad/` uses the real airframe's measured rotor positions (front x +0.08, y ±0.16; rear x −0.15,
  y ±0.14; the same values go to PX4's `CA_ROTOR*`). It has 6.1 in props (0.385 × 0.475 m footprint), 1.5 kg mass
  and thrust-to-weight about 3. It uses x500 sensors and motor plugins so PX4's gz bridge attaches unchanged, with
  inertia scaled so PX4's x500 rate gains fit.
- `dh/route.py` builds the route from map features: corridor centrelines, maze openings, the tube axis, the window
  normal and the pad. Corners are rounded into arcs. The speed profile is limited by top speed, lateral and
  along-path acceleration (sharing one budget), vertical acceleration, and caps in the tube, window and landing zones.
  The result is Gaussian-smoothed in time so acceleration is continuous.
- `dh/geom.py` computes clearance between the drone's parts (props, body, feet) and every collision primitive, read
  from the generated model.
- Frames: world x = east, y = north, z = up, origin at the floor centre. PX4's local frame is the world frame under
  `sim`/`stock`, and the start pose under `drone`. `fly_course.py` aligns the two once, from the launch pose.
  Map inches: world = in × 0.0254 − 3.048.

## Setup on this PC (already done)

- WSL Ubuntu 24.04 with Gazebo Harmonic (gz sim 8.13) and `~/PX4-Autopilot`, built with `make px4_sitl_default`.
- `libopencv-dev` was installed, because PX4's optical-flow gz plugin needs it to build.

## Gotcha: PX4 SITL remembers parameters

PX4 SITL saves every parameter change (including `PX4_PARAM_*` and MAVLink `PARAM_SET`) to
`~/PX4-Autopilot/build/px4_sitl_default/rootfs/<N>/parameters.bson`. Without a reset, one profile's settings leak
into the next run; for example `EKF2_EV_DELAY` 60 from a `drone` run makes the `sim` estimator lag. `run_sim.sh`
deletes that file on every boot, so each run starts from the airframe defaults plus its profile.

## Limits of the sim

- Vision under `drone` is the true pose plus latency and white noise. There is no drift, no scale error, and no
  pose jump on re-initialisation.
- The airframe's dynamics derive from PX4's x500, scaled to the real geometry, not identified from flight data.
- `sim`-profile results use PX4 settings the real airframe doesn't run. Use `drone` with `pos10` to predict real
  flights.
