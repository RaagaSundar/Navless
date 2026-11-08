<p align="center">
  <img src="docs/media/hero_lap.gif" width="720" alt="Top-down view of the DroneHacks course with the new maze. A drone flies from the launch mark north-east beside the long diagonal wall, rounds its tip through the 1 m gap, flies the dark tunnel, crosses the chicane window and lands on the pad; the five checkpoints light up as it passes them, and inside the tunnel two orange side-sensor beams reach the walls.">
</p>

<h1 align="center">Navless</h1>

<p align="center">
  <b>A drone that flies the DroneHacks indoor course on its own: no GPS, no optical flow, one stereo camera.</b><br>
  Stereo SLAM to know where it is, a phone-LiDAR scan of the real course to plan on,<br>
  two infrared side sensors for the dark tunnel, and a flight program built around checks that refuse to fly on bad data.
</p>

<p align="center">
  <img alt="ROS 2 Humble" src="https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white">
  <img alt="PX4 v1.16" src="https://img.shields.io/badge/PX4-v1.16-2b6cb0">
  <img alt="NVIDIA Jetson Orin Nano" src="https://img.shields.io/badge/Jetson-Orin_Nano-76B900?logo=nvidia&logoColor=white">
  <img alt="cuVSLAM, stereo only" src="https://img.shields.io/badge/cuVSLAM-stereo_only-76B900">
  <img alt="Gazebo Harmonic" src="https://img.shields.io/badge/Gazebo-Harmonic-F58113">
  <br>
  <img alt="Gazebo gate: 3 of 3 laps on the new maze" src="https://img.shields.io/badge/Gazebo_gate-3%2F3_laps,_new_maze-3ddc97">
  <img alt="Real flights: hover, yaw, first course runs" src="https://img.shields.io/badge/real_flights-hover_·_yaw_·_course_runs-38bdf8">
</p>

<p align="center">
  <a href="#the-course">Course</a> ·
  <a href="#how-it-works">How it works</a> ·
  <a href="#how-we-test">Testing</a> ·
  <a href="#real-flights">Real flights</a> ·
  <a href="#results">Results</a> ·
  <a href="#run-it">Run it</a> ·
  <a href="#the-agent-channel">Agent channel</a> ·
  <a href="#team">Team</a>
</p>

<p align="center"><sub>
A recorded lap in Gazebo + PX4 SITL on the organisers' new maze, flying skybeta-19's flight code (<code>drone19/flight</code> <code>865db6b</code>) with PX4 reading 19 cm high, as on the real drone. Sped up 4×.
Position, height, speed, wall clearance and checkpoint times come from the log. The camera cone, the side-sensor beams and the dark tunnel are drawn on top.
</sub></p>

---

## At a glance

<table>
<tr>
<td align="center" width="25%"><h3>0</h3>GPS or optical-flow sensors</td>
<td align="center" width="25%"><h3>1 m</h3>gap in the maze for a 47.5 cm drone</td>
<td align="center" width="25%"><h3>31 Hz</h3>stereo odometry on the drone's Jetson</td>
<td align="center" width="25%"><h3>3 / 3</h3>Gazebo laps on the new maze, 0 contacts</td>
</tr>
<tr>
<td align="center"><h3>39.6 s</h3>lap, landing 6.4–7.6 cm from the pad centre</td>
<td align="center"><h3>≥ 17.9 cm</h3>planned prop room to every wall</td>
<td align="center"><h3>2</h3>infrared side sensors for the dark tunnel</td>
<td align="center"><h3>1</h3>Slack channel shared by 5 people and their AI agents</td>
</tr>
</table>

## The course

A 20 × 20 ft netted room with five checkpoints worth 35 points. Our drone, **skybeta-19**, is 47.5 cm (18.7 in) wide with props: 16 cm ahead of its centre, 23 cm behind, 24 cm to each side.

| | Checkpoint | Points | The hard part | What we do |
|:-:|---|:-:|---|---|
| 1 | First corridor | 4 | The camera only knows the heading it started with. 1° off at the start is 5 cm off 3 m later. | Start square on the launch mark. Hold the takeoff heading, guarded against PX4's compass resets. |
| 2 | Maze | 7 | Rebuilt the night before: one 1 m gap between the long diagonal wall (MD) and the north net | Fly north-east beside MD and round its tip through the gap, on a route planned from the walls themselves |
| 3 | Tunnel | 10 | It's dark, so the camera tracks nothing for about 3 s | Ride through on PX4's own estimate (`blind_carry`), centred by two side time-of-flight sensors |
| 4 | Chicane | 7 | The window is 36 in, leaving 8.6 in on each side | Cross at the window's centre, 0.85 m up (backup: a Gate B route) |
| 5 | Landing | 7 | Land on a bullseye scored by ring | Descend onto the pad until PX4 detects touchdown and disarms. Optionally, steer onto the rings by camera. |

### The maze changed the night before

<table>
<tr>
<td width="50%"><img src="docs/media/maze_change.gif" alt="The scanned course with walls M1, M2 and M3 and our gated route. M1 to M3 fade out and three new walls MN, MD and MT fade in. The old route turns red where it runs through MD and where its props clip MN. A new route draws in cyan: north-east beside MD, round its tip through a 1 m gap."></td>
<td width="50%"><img src="docs/media/plan_vs_asbuilt.gif" alt="The official course map morphs into the scanned course: five walls slide to new positions, the old route turns red where it meets M3, and a new route is drawn on the real layout."></td>
</tr>
<tr>
<td><sub><b>Nov 7, evening.</b> The organisers replaced M1–M3 with MN, MD and MT. Our gated route now ran straight through MD. A new route, re-exported from the new walls, passed the Gazebo gate the same night.</sub></td>
<td><sub><b>Nov 6.</b> The official map vs our phone-LiDAR scan. Five walls weren't where the map put them. M3 alone sat 76 cm east, and a route planned on the map passed 2.8 cm from it.</sub></td>
</tr>
</table>

<p align="center">
  <a href="docs/media/course_newmaze.stl"><img alt="Open the course in 3D" src="https://img.shields.io/badge/🧊_Spin_the_course_in_3D-new_maze_+_route-0b0f14?style=for-the-badge"></a>
</p>

<details>
<summary><b>How we built the as-built course</b></summary>

We scanned the real course with a phone's LiDAR: a Scaniverse scan with 596k points. We aligned it on the parts that match the official plan within 0.3–6 cm (the net, the barriers, the window wall and the tube), then fitted lines to the rest. After the organisers rebuilt the maze, the new walls went in from a measured sketch: MN along the north net, MD on a 49.9° diagonal ending 1.000 m from the north net, and MT running to the tube's north-west corner.

The course lives in one file, [`gazebo_sim/course/course_asbuilt.json`](gazebo_sim/course/course_asbuilt.json), with the mesh in [`dronehacks-course-asbuilt.obj`](dronehacks-course-asbuilt.obj). The Gazebo world, the route exporter, the localizer and the planner all read from it, so a wall moves in one place. The old floor tape is the old maze's, so the file marks it DO NOT FOLLOW: it runs under MD.

</details>

## How it works

```mermaid
flowchart LR
  subgraph SENSORS["Sensors"]
    CAM["OAK-D-S2<br/>stereo camera"]
    TOF["2 × VL53L1X<br/>side ToF"]
    RNG["VL53L1X<br/>downward range"]
  end
  subgraph JETSON["Jetson Orin Nano · ROS 2 Humble"]
    VS["cuVSLAM<br/>stereo odometry"]
    BR["vision bridge<br/>ENU → NED, FRD"]
    TN["tube_node<br/>offset = (L − R) / 2"]
    NAV["navless_nav<br/>course_mission"]
    FI["flight_interface<br/>(organisers)"]
    LOC["cloud_localize<br/>walls vs depth"]
    PAD["pad_observer<br/>bullseye rings"]
  end
  subgraph PX4["Pixhawk 6C · PX4 v1.16"]
    EKF["EKF2"]
    CTRL["position control"]
  end
  CAM --> VS --> BR -->|external vision| EKF
  RNG --> EKF
  VS -->|pose| NAV
  TOF --> TN -->|tube offset| NAV
  CAM -.-> LOC -.->|drift correction, off| NAV
  CAM -.-> PAD -.->|pad centre, off| NAV
  EKF -->|height, heading| NAV
  NAV -->|/skybeta/setpoint| FI -->|Offboard| CTRL
  EKF --> CTRL
```

Everything above the autopilot runs on the drone's Jetson in ROS 2. `navless_nav` **never arms the drone, never changes its mode and never writes to PX4 directly**. The safety pilot arms on RC and switches to Position. The organisers' `flight_interface` then hands control to Offboard, clamps altitude and speed, and keeps the 10 Hz heartbeat. The drone flies the lap and lands on its own, and any mode change on the transmitter takes control back at once.

### 1. Knowing where it is: stereo SLAM into PX4

NVIDIA cuVSLAM runs on the Jetson using the stereo camera alone (1024 × 640 at 30 Hz, IMU fusion off). On skybeta-19 it ran at **31.2 Hz with a 49 ms worst gap**. A bridge converts its pose to PX4's frame, and EKF2 fuses it as external vision for position and yaw. Height comes from a downward laser rangefinder.

<details>
<summary><b>Three things we found about this pipeline, and what we did</b></summary>

- **Stale poses look like a drone standing still.** When SLAM loses track it can keep publishing its last pose. Koji's `hardware_slam/odom_gate.py` passes a pose only while SLAM is tracking and the pose is under 0.25 s old. On 19 it passed **28.43 Hz**, with samples **59.89 ms** old at the median.
- **PX4 snaps to the compass in a vision gap.** About 0.5 s into a gap, EKF2 resets its heading to the magnetometer: a 76° jump in Gazebo and about 100° on 19's floor. `EKF2_MAG_TYPE=5` fixes it in simulation, but the organisers asked teams to keep default parameters. So we guard against it in our own code (the heading guard below) instead.
- **A busy Jetson starves the camera.** With a simulation container running next to the stack, odometry fell from 29.4 Hz (39 ms worst gap) to 25.6 Hz with gaps up to 300 ms, which is enough to trigger the compass reset. Rule: nothing else runs on 19's Jetson while it flies.

</details>

### 2. The route

The window route is **14.2 m and 286 waypoints**, exported from the course file by [`gazebo_sim/tools/export_route.py`](gazebo_sim/tools/export_route.py). It runs corridor centrelines with filleted corners and keeps **at least 17.9 cm between the props and every wall** (planned, nose north). It also keeps the downward rangefinder's beam at least 20.7 cm off every boulder, so PX4's height never jumps. The first leg heads north-east beside MD rather than due north. The takeoff leans 25 cm into the first leg, so that start buys 37–38 cm of prop room from the west net during the climb (Gazebo), instead of 28–30 cm. That matters against the drone's 25–40 cm liftoff slide.

A Gate B backup route (15.9 m) goes around the window. Every setpoint is also pulled 25 cm inside the net, because `flight_interface` clamps altitude only, never x/y.

### 3. The flight program: `course_mission`

```mermaid
stateDiagram-v2
  direction LR
  [*] --> WAIT
  WAIT --> TAKEOFF: Offboard, checks pass
  WAIT --> REFUSED: Offboard, checks fail
  REFUSED --> [*]: descend until disarm
  TAKEOFF --> ROUTE: held 2 s within 5 cm
  ROUTE --> LAND: route done
  ROUTE --> HOLD: no progress
  LAND --> DONE: touchdown, disarm
  TAKEOFF --> PAUSED: pilot takes over
  ROUTE --> PAUSED: pilot takes over
  PAUSED --> WAIT: disarmed
```

It walks the route with a speed-limited carrot: 0.5 m/s on straights, 0.6 m lead, 0.25 m/s² in turns. Every check below came from something we saw in a log:

| Check | What it stops | Where it came from |
|---|---|---|
| **Pre-arm height match** | PX4 can boot with its height 0.6 m off. Nothing streams (so no takeover) until PX4 and the camera agree within 30 cm for 2 s. | A PX4 boot fault |
| **Floor offset correction** | On 19, PX4's height on the floor reads **18.5–19.8 cm** above the camera's. It's measured on the floor at each arming (median of 2 s) and added to every height sent, so the route is flown at its planned heights. | All three of 19's flight-test logs |
| **Pre-arm heading match** | PX4 and the camera must agree within 10° before takeoff, so PX4 isn't on the compass | The magnetometer reset |
| **Heading hold + guard** | The takeoff heading is held for the whole lap. If PX4 and the camera disagree by more than 15°, or PX4 resets its heading, the node stops steering the nose instead of chasing a bad heading. | The magnetometer reset |
| **Takeoff settle** | The route starts only after the drone has held its takeoff point within 5 cm for 2 s | 19's liftoff slide |
| **Net clamp** | No setpoint ever lands within 25 cm of the net | `flight_interface` doesn't clamp x/y |
| **Freeze on lost tracking** | The carrot stops while cuVSLAM is lost or stale, and resumes when it's back | VIO dropouts |
| **Land until disarm** | LAND keeps sending descent setpoints until PX4 detects touchdown and disarms. An expired descent setpoint would make `flight_interface` climb back to the last position. | The interface's setpoint expiry |
| **Launch defaults test** | A unit test fails if any launch-file default differs from the node's own. A stale launch default once shipped a 15 cm pre-arm limit that would have refused every takeover on 19. | Found on 19 |

### 4. The dark tunnel

<p align="center">
  <img src="docs/media/tube_tof.gif" width="640" alt="Cross-section of the tunnel: the drone starts 12 cm left of centre, two orange beams measure the distance to each wall, and the drone moves back to within 2 cm of the centre.">
</p>

The tunnel is the 10-point checkpoint, and it's dark: for about 3 s the camera has nothing to track. Two things carry the drone through:

- **`blind_carry`** (on in the course launch): if odometry stops inside the tube, the drone keeps moving along the route on PX4's own estimate, for at most 4 s. In the master sim, freezing in place hit the tube 3/3 times. `blind_carry` landed 4/4 and made 6/6 clean passes at 0.5 m/s.
- **Side time-of-flight centring**: one VL53L1X on each side, each on its own I²C bus. They carry their own 940 nm light, so the dark doesn't matter. The offset from centre is simply (L − R) / 2. A reading is used only inside the tube, only when L + R adds up to the tube's width, only if it's under 0.3 s old, and never more than 15 cm off the planned route. Otherwise the drone flies the route.

The first ToF driver reached only about 30 cm, and each wall is about 65 cm away. The rewrite reaches 60–90 cm. On skybeta-19 it ran at **9.6 Hz**, with **99–99.6 %** of readings valid and steady to ±0.3 cm. In a closed-loop test with the sensors raycast off the course model, a route 10–12 cm off centre ends **within 2 cm** of the centreline.

### 5. Landing on the bullseye

Bryan's `pad_reference` finds the pad's printed rings (9 : 20 : 30 in radii) in the camera image. It projects edges onto the floor and accepts a fix only when at least two rings share a centre. `pad_observer` publishes the pad centre, and `course_mission` (`pad_vision`, off by default) lands on it instead of on the dead-reckoned pad. Offline results: median error 0.15 cm, 95th percentile 2.9 cm over 200 poses. In Gazebo with 32 cm of injected drift: **29.4 cm off without it, 4.6–6.4 cm with it**.

### 6. Path planner and live re-planning

Favour's [`planner/`](planner) turns the course into a 3D grid (0.1 m voxels), pads every wall by 0.28 m, finds a path with A\*, and smooths it into a B-spline the drone can follow. On `main`, `route_source` picks the planner's path (the default) or the exported route file. With `replan_on_obstacles` the drone drops live stereo depth into the grid and re-plans around anything the map didn't have. Points are placed with the drone's current heading, and points below 0.30 m are filtered out so the floor isn't treated as a wall.

### 7. The course-model localizer

Shining's `depthcloud/cloud_localize.py` matches stereo depth against the course model to say where the walls put the drone, independent of odometry drift. In simulation on the new route it holds about 4 cm median error when the camera faces the direction of travel. Facing north, it sees fewer walls and the error is larger. Its output can drive `wall_correction` in `course_mission`, which flies the route in the frame the walls agree with. That's built and tested in simulation, and stays off until it's validated on the drone.

## How we test

```mermaid
flowchart LR
  A["Unit tests<br/>pytest, ROS stubbed"] --> B["Gazebo gate<br/>3 laps, PX4 SITL"] --> C["Master sim<br/>drone's own launch"] --> D["Split sim<br/>real cuVSLAM on 19"] --> E["Props off<br/>on 19"] --> F["Real flight<br/>pilot on RC"]
```

| Tool | What it does |
|---|---|
| [`gazebo_sim/tools/gate_asbuilt.sh`](gazebo_sim/tools/gate_asbuilt.sh) | The gate a branch passes before it goes near the drone: three laps on the as-built course with the drone's estimator profile, 5 mm odometry noise and 50 ms delay. It reports minimum clearance per wall (props counted), pad distance, window height and contacts. `fly_navless.py --px4-z-bias 0.19` reproduces 19's height offset. |
| [`master_sim/`](master_sim) | One command flies the drone's launch unmodified in Gazebo + PX4 SITL, with a scorecard: checkpoint times, pad distance, closest wall, blind passes through the tube. `regress.py` runs several laps per change, because one lap can mislead. |
| [`split_sim/`](split_sim) | A laptop or Mac renders the stereo cameras and runs PX4 SITL. 19's Jetson runs the real cuVSLAM and our code in an isolated ROS domain (219, never 19). |
| [`tools/flyx_lab/`](tools/flyx_lab) | Runs in the organisers' FlyX Lab simulator, with `tools/sim_truth_recorder.py` checking PX4's estimate against simulation truth |
| [`tools/preflight_check.sh`](tools/preflight_check.sh) | Read-only check on the drone. It must report 0 FAIL before arming. |
| Flight recorders | `flight_tests/` logs every 20 Hz tick, and `course_mission` (on the latest branch) logs every 10 Hz tick plus each run's parameters. Every real flight leaves a CSV. |

## Real flights

### Nov 7: first autonomous flights on skybeta-19

<table>
<tr>
<td width="50%"><img src="flight_tests/evidence/2026-11-07-skybeta-19/images/hover.png" alt="Hover test: the position target steps up to about 1.19 m, PX4's height estimate follows, holds for about 10 seconds and descends to the ground; armed and Offboard flags below."></td>
<td width="50%"><img src="flight_tests/evidence/2026-11-07-skybeta-19/images/yaw.png" alt="Yaw test: requested and measured clockwise heading change over 35 seconds."></td>
</tr>
<tr>
<td><sub><b>Hover:</b> climb 1 m, hold 10 s, descend, land and disarm, all under our setpoints after the pilot's handover</sub></td>
<td><sub><b>Yaw:</b> a 180° clockwise turn that reached its target within tolerance, then descent</sub></td>
</tr>
</table>

Shining's `flight_tests` (hover, yaw, forward) send setpoints only, through the organisers' interface. These flights are where we measured the 19 cm height offset, a 25–31 cm liftoff slide that turns with the drone, and a steady 13 cm hold offset. The last one led to a drift correction that learns the push while the drone holds still. Details and raw logs: [`flight_tests/evidence/`](flight_tests/evidence/2026-11-07-skybeta-19).

### Nov 8: first course runs, and what the flight recorder showed

<p align="center">
  <img src="docs/media/flight_recorder_nov8.png" width="900" alt="Two top-down maps from skybeta-19's flight recorder. Run 1: the believed track follows the planned north-east leg and ends at a contact mark 1.5 m in, while the same route rotated 12 degrees runs into wall MD. Run 2: the track follows the route through the maze gap and ends at a contact mark beside the north net on the turn east, with the setpoints offset to the south. Below, a bar chart compares 17 to 25 cm of planned prop room with errors of 5 to 40 cm.">
</p>

19 flew the course twice, on a newer branch (nose facing the direction of travel, drift correction on, not yet gated), and hit a wall both times. Every tick was logged, so we know why:

- **Run 1:** by its own tracking the drone was on its line, with the props 32 cm clear of MD, when it hit MD 1.5 m in. The real drone was at least 32 cm off, which is what a ~12° error in the starting heading produces.
- **Run 2:** through the maze gap, then momentum carried it 19 cm wide on the turn east, side-on to the north net. Facing north it would have had 25 cm instead of 17. After the impact the camera's position jumped 2.9 m while still reporting "tracking", and the node kept flying on it.

The lesson is in the chart: 17–25 cm of planned room is less than any one real error. Before the next course run:
1. **Fly only a gated build**, with a gate that injects real errors: start heading ±5°, 3% drift, the liftoff slide and 19's measured corner lag.
2. **A placement jig** on the launch mark, so the drone starts within 1° and 2 cm.
3. **More room**: at least 35 cm at the tight spots, the nose north through the maze, slower into corners.
4. **Safety stops**: stop and descend on an odometry jump, a PX4–camera split, a large tracking error or a disarm.

## Results

| What | Where | Result |
|---|---|---|
| Full lap, 19's flight code (`drone19/flight` `865db6b`), PX4 19 cm high | Gazebo + PX4 SITL, new maze | **3/3 landed**, 39.6–39.9 s, **6.4–7.6 cm** from the pad centre, window crossed 1.2–1.6 cm off centre at 0.81–0.83 m, closest wall 16.0–16.4 cm (Barrier 1, behind the launch mark), **0 contacts** |
| Gate B backup route with heading hold (`993a967`) | same | 3/3 landed, 60.6–60.7 s, 1.8–2.0 cm from the pad, heading within 0.17° |
| Height-offset fix vs the old behaviour | same, PX4 19 cm high | Fixed: window at 0.81–0.84 m as planned. Old: refuses to take off. Limit widened without the fix: flies 19 cm low, 15.7 cm from a boulder. |
| Autonomous hover and 180° yaw | **skybeta-19, real** | Climbed, held and landed under our setpoints; yaw reached its target |
| First two course runs | **skybeta-19, real** | Both hit a wall (MD at 1.5 m, the north net at the maze exit). Fully logged and analysed above. |
| Stereo odometry | skybeta-19 | 31.2 Hz, 49 ms worst gap (Jetson quiet) |
| Side ToF sensors | skybeta-19 bench | 9.6 Hz, 99–99.6 % valid, ±0.3 cm |
| ToF centring | closed-loop test | 10–12 cm off centre → within 2 cm |
| Camera pad landing | Gazebo, 32 cm injected drift | 29.4 cm off → 4.6–6.4 cm |

## Run it

**Simulation.** Use WSL or Linux with Gazebo Harmonic and PX4 SITL, set up as in [`gazebo_sim/README.md`](gazebo_sim/README.md):

```bash
BRANCH=drone19/flight bash gazebo_sim/tools/gate_asbuilt.sh 3   # the gate for skybeta-19's branch
bash gazebo_sim/tools/gate_asbuilt.sh 3                         # the same gate for main
master_sim/run_master.sh --gui                                  # watch the drone's own launch fly the course
master_sim/regress.py --laps 3 default                          # several laps, one table
```

**The real drone.** Follow [FLIGHT_DAY.md](FLIGHT_DAY.md): setup, the preflight check (0 FAIL), placing the drone on the launch mark, and flying.

**This README's media.** `python docs/media/make_readme_media.py` renders every GIF, the flight-recorder figure and the 3D model from the course file, a recorded gate lap and the flight logs in [`docs/media/data/`](docs/media/data).

## Repo map

| Folder | What's in it |
|---|---|
| [`navless_nav/`](navless_nav) | The ROS 2 package that flies the course: `course_mission`, the route and carrot, tube guidance, pad vision, `mirror_relay` (live pose, ToF and camera to the laptop) and the launch files |
| [`gazebo_sim/`](gazebo_sim) | Gazebo + PX4 SITL of the course, the as-built course file, route export, the gate and the mirror dashboard |
| [`tube/`](tube) | Side ToF driver, `tube_node` centring, the bench view and experimental stereo centring |
| [`hardware_slam/`](hardware_slam) | cuVSLAM on the drone, the odometry gate and the SLAM map viewer |
| [`depthcloud/`](depthcloud) | Stereo depth, the course-model localizer, the phone-scan course and the walk-session recorder |
| [`planner/`](planner) | 3D grid, A\*, B-spline, the PX4 bridge, and the real-flight start and kill scripts |
| [`flight_tests/`](flight_tests), [`onboard/`](onboard) | Hover, yaw and forward tests for 19 with their recorded evidence, and the FlyX versions |
| [`master_sim/`](master_sim), [`split_sim/`](split_sim), [`sim_sessions/`](sim_sessions), [`local_sim/`](local_sim), [`local_course/`](local_course) | The one-command sim, Mac + Jetson split simulation, and shared-Jetson sessions |
| [`real_jetson/`](real_jetson) | A staged Jetson flight package with handover checks and a Mac telemetry dashboard (not yet flown) |
| [`tools/`](tools) | Preflight check, truth recorder, FlyX Lab helpers, magnetometer check |
| [`docs/`](docs) | The Jetson simulation runbook, the skybeta-19 snapshot, and this README's media |
| [`src/`](src), [`scripts/`](scripts) | The Navless MCP server for the agent channel (below) |

Saved maps, recordings and credentials stay out of Git.

---

## The agent channel

Navless is also the tool we built to work this way: a small MCP server that puts every teammate's coding agents in one Slack channel. Each agent posts under its own name, such as `lidar (Raaga)`, `slam-koji (Koji)`, `depthcloud (Shining)`, `pathplanner (Favour)` and `sim (Bryan)`. Agents say what they're touching before they start, ask the agent that owns a piece of code, and wait for the answer. The humans read the same channel and step in. The rules they follow live in [`AGENTS.md`](AGENTS.md).

```
 lidar (Raaga) ────────┐                        ┌── slam-koji (Koji)
 depthcloud (Shining) ─┼── navless ── Slack ────┤
 sim (Bryan) ──────────┘   #team-navless        └── pathplanner (Favour)
```

| Tool | What it does |
| --- | --- |
| `slack_post` | Post as this agent, top level or in a thread |
| `slack_read` | Catch up on the channel or a thread, oldest first, with `since` for only-new |
| `slack_wait` | Block until someone replies or mentions this agent |
| `slack_whoami` | Tells the agent its name and how others reach it |

Any agent that speaks MCP can use it: Claude Code, Cursor, Codex or Claude Desktop.

<details>
<summary><b>Setup (one person, once): about 5 minutes</b></summary>

### 1. Make a channel

Slack apps can't join group DMs. Either convert your group DM (click the names at the top, then **Convert to private channel**) or make a new channel like `#navless-agents`.

Get the channel ID: open the channel, click its name at the top, scroll to the bottom of the About tab. It looks like `C0ABC123XYZ`.

### 2. Create the Slack app

1. Go to https://api.slack.com/apps and click **Create New App**, then **From a manifest**.
2. Pick your workspace.
3. Paste the contents of [`slack-manifest.json`](slack-manifest.json) (JSON tab), then **Create**.
4. Click **Install to Workspace** and **Allow**.
5. Go to **OAuth & Permissions** and copy the **Bot User OAuth Token** (starts with `xoxb-`).

### 3. Add the bot to the channel

In the channel, type `/invite @navless` and send it.

### 4. Share the two values with the team

Send teammates the `xoxb-` token and the channel ID **privately** (not in the channel itself, and never commit them).

</details>

<details>
<summary><b>Setup (every teammate, each machine)</b></summary>

### 1. Get the code

```bash
git clone https://github.com/RaagaSundar/Navless.git
cd Navless
npm install
```

### 2. Connect your agent

**Fastest:** run `npm run setup`. It asks for the bot token, your agent name, and your project folder, then registers the agent with Claude Code and adds the rules to your project's `CLAUDE.md`. Run it again with a different name for a second agent. Our channel is `#team-navless` (`C0CDARV3UQG`).

**By hand:** pick a short, unique agent name per agent (`frontend`, `backend`, `infra`, `research`...). If you run two agents, register Navless twice with different names.

**Claude Code** (run from inside the Navless folder so `$(pwd)` is right):

```bash
claude mcp add navless --scope user \
  -e SLACK_BOT_TOKEN=xoxb-your-token \
  -e SLACK_CHANNEL_ID=C0ABC123XYZ \
  -e AGENT_NAME=frontend \
  -e AGENT_OWNER=Raaga \
  -e AGENT_EMOJI=:rocket: \
  -- node "$(pwd)/src/server.js"
```

Then teach it the protocol. In each project you work on, add this line to that project's `CLAUDE.md`:

```
@/full/path/to/Navless/AGENTS.md
```

**Cursor, Claude Desktop, or anything with an `mcp.json`:**

```json
{
  "mcpServers": {
    "navless": {
      "command": "node",
      "args": ["/full/path/to/Navless/src/server.js"],
      "env": {
        "SLACK_BOT_TOKEN": "xoxb-your-token",
        "SLACK_CHANNEL_ID": "C0ABC123XYZ",
        "AGENT_NAME": "backend",
        "AGENT_OWNER": "Koji",
        "AGENT_EMOJI": ":gear:"
      }
    }
  }
}
```

Paste `AGENTS.md` into that tool's rules file (`.cursor/rules`, project instructions, etc.).

### 3. Test it

Start your agent and say:

> Read the navless channel, then post a hello saying who you are and what you're working on.

The message should show up in Slack under your agent's name.

</details>

<details>
<summary><b>Environment variables and troubleshooting</b></summary>

| Variable | Required | Meaning |
| --- | --- | --- |
| `SLACK_BOT_TOKEN` | yes | `xoxb-` token from the Slack app |
| `SLACK_CHANNEL_ID` | yes | Channel the agents share |
| `AGENT_NAME` | yes | How other agents @mention this one |
| `AGENT_OWNER` | no | Human who runs it, shown in parentheses |
| `AGENT_EMOJI` | no | Avatar, default `:robot_face:` |
| `NAVLESS_POLL_SECONDS` | no | How often `slack_wait` checks, default 4 |

| Error | Fix |
| --- | --- |
| `not_in_channel` | `/invite @navless` in the channel |
| `channel_not_found` | Wrong channel ID, or the bot isn't in that private channel |
| `missing_scope` | Re-create the app from the manifest, or add the scope and reinstall |
| Name shows as `navless` instead of your agent name | `chat:write.customize` scope is missing; add it and reinstall |
| Humans show as `U0123...` | `users:read` scope is missing |

</details>

## Team

- **Raaga:** the flight program (`navless_nav`): routes, heading hold and guards, the height-offset fix, the tunnel and side ToF, the Gazebo sim and gate, the live dashboard, and the agent channel
- **Koji:** positioning on the drone: cuVSLAM, the odometry gate, the split sim on the Jetson, and the Jetson's networking
- **Shining:** the phone-scan course, stereo depth and the localizer, the first flight tests, drift correction and the flight recorder
- **Favour:** the path planner and live re-planning
- **Bryan:** testing: the master sim, the FlyX Lab, camera pad landing and flight-day checks
