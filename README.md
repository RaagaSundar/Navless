<p align="center">
  <img src="docs/media/hero_lap.gif" width="720" alt="Top-down view of the DroneHacks course. A drone flies from the launch mark through the corridor, the maze, the dark tunnel and the chicane window to the landing pad; the five checkpoints light up as it passes them, and inside the tunnel two orange side-sensor beams reach the walls.">
</p>

<h1 align="center">Navless</h1>

<p align="center">
  <b>A drone that flies the DroneHacks indoor course with no GPS.</b><br>
  Stereo SLAM to know where it is, a phone-LiDAR scan of the real course to plan on,<br>
  and two infrared range sensors for the dark tunnel.
</p>

<p align="center">
  <img alt="ROS 2 Humble" src="https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white">
  <img alt="PX4 v1.16" src="https://img.shields.io/badge/PX4-v1.16-2b6cb0">
  <img alt="NVIDIA Jetson Orin" src="https://img.shields.io/badge/Jetson-Orin-76B900?logo=nvidia&logoColor=white">
  <img alt="cuVSLAM, stereo only" src="https://img.shields.io/badge/cuVSLAM-stereo_only-76B900">
  <img alt="Gazebo Harmonic" src="https://img.shields.io/badge/Gazebo-Harmonic-F58113">
</p>

<p align="center">
  <a href="#how-it-works">How it works</a> ·
  <a href="#results">Results</a> ·
  <a href="#run-it">Run it</a> ·
  <a href="FLIGHT_DAY.md">Flight day</a> ·
  <a href="#the-agent-channel">Agent channel</a> ·
  <a href="#team">Team</a>
</p>

<p align="center"><sub>
A recorded lap in Gazebo + PX4 SITL on the as-built course, flying skybeta-19's code (<code>drone19/flight</code>), sped up about 4×.
Position, height, speed, wall clearance and checkpoint times come from the log. The camera cone, the side-sensor beams and the dark tunnel are drawn on top.
</sub></p>

---

## The course

The course is a 20 × 20 ft netted room with five checkpoints worth 35 points. Our drone, skybeta-19, is 47.5 cm (18.7 in) wide with props.

| | Checkpoint | Points | The hard part | What we do |
|:-:|---|:-:|---|---|
| 1 | First corridor | 4 | The flight code takes the heading at power-up as north. Each degree off moves the far end of the route about 9 cm. | Square the drone on Barrier 1 to within 1° ([FLIGHT_DAY.md](FLIGHT_DAY.md)) |
| 2 | Maze | 7 | The walls aren't where the official map puts them | Fly a route planned on a phone-LiDAR scan of the real course |
| 3 | Tunnel | 10 | It's dark, so the camera tracks nothing for about 3 s | Dead-reckon through it, centred by two side time-of-flight sensors |
| 4 | Chicane | 7 | The window is 36 in, leaving 8.6 in on each side | Cross at the window's centre, 0.85 m up (backup: an as-built Gate B route) |
| 5 | Landing | 7 | Touch down on the pad | The drone lands and disarms by itself |

## How it works

```mermaid
flowchart TB
  subgraph SENSORS["Sensors"]
    CAM["OAK-D-S2<br/>stereo camera"]
    TOF["2 × VL53L1X<br/>side ToF"]
  end
  subgraph JETSON["Jetson Orin · ROS 2"]
    VS["cuVSLAM<br/>stereo odometry"]
    TUBE["tube_node<br/>offset from the tube centre"]
    GATE["odom_gate<br/>tracking and under 0.25 s old"]
    NAV["navless_nav<br/>flies the as-built route"]
    FI["flight_interface"]
  end
  subgraph AP["Pixhawk · PX4 v1.16"]
    RNG["downward<br/>rangefinder"]
    EKF["EKF2<br/>state estimate"]
    CTRL["position control"]
  end
  CAM --> VS
  TOF --> TUBE
  VS --> GATE
  VS -->|pose| NAV
  TUBE -->|offset| NAV
  NAV -->|setpoints| FI
  GATE -->|external vision| EKF
  RNG --> EKF
  FI -->|Offboard| CTRL
  EKF --> CTRL
```

Everything above the autopilot runs on the drone's Jetson Orin in ROS 2. `navless_nav` never arms the drone or changes its mode. The pilot arms in Altitude and switches to Position. `flight_interface` then hands control to Offboard, and the drone flies the lap and lands on its own. Changing the flight mode takes control back at any time.

### 1. Knowing where it is: stereo SLAM and an odometry gate

NVIDIA cuVSLAM runs on the Jetson using the stereo camera alone, with the IMU off. When SLAM loses track it can keep publishing its last pose, and PX4 would read that frozen pose as the drone standing still. So `hardware_slam/odom_gate.py` sits between them: a pose reaches PX4 only while SLAM is tracking and the pose is under 0.25 s old.

On skybeta-19 (PX4 powered, disarmed) the gate passed odometry at **28.43 Hz**. Samples were **59.89 ms** old at the median and 83.80 ms at p95. Details: [`hardware_slam/README.md`](hardware_slam/README.md).

### 2. The real course, not the official map

<table>
<tr>
<td width="50%"><img src="docs/media/plan_vs_asbuilt.gif" alt="The official course map morphs into the scanned course: five walls slide to new positions, the old route turns red where it meets M3, and a new route is drawn on the real layout."></td>
<td width="50%"><img src="docs/media/localizer_sim_asbuilt.gif" alt="Top view of the as-built course: a simulated drone flies the window route while the localizer's estimate tracks its true pose."></td>
</tr>
<tr>
<td><sub>The official map vs the scan. The route planned on the map has its centre line 2.8 cm from the real M3, so the 47.5 cm drone would hit it.</sub></td>
<td><sub>The course-model localizer. <b>White</b> is the true pose, <b>green</b> the estimate (<b>blue</b> while it coasts on odometry). The camera points and odometry are simulated; the localizer is the drone's code.</sub></td>
</tr>
</table>

We scanned the real course with a phone's LiDAR: a Scaniverse scan with 596k points. The scan was aligned on the parts that match the official plan within 0.3–6 cm: the net, the barriers, the window wall and the tube. Lines were then fitted to the rest. Five walls (M1, M2, M3, A1 and C1) are somewhere else. M3 alone sits 76 cm east of where the map has it. Our route and our Gazebo world now use the as-built course: [`depthcloud/asbuilt/`](depthcloud/asbuilt) and [`dronehacks-course-asbuilt.obj`](dronehacks-course-asbuilt.obj).

The same model drives `depthcloud/cloud_localize.py`, which matches stereo depth against the course. In simulation it holds a median error of 4–5 cm (max 18–23 cm). Odometry alone drifts to 143 cm. It runs as an observer: `navless_nav` doesn't fuse it yet.

### 3. Flight code and the dark tunnel

<p align="center">
  <img src="docs/media/tube_tof.gif" width="640" alt="Cross-section of the tunnel: the drone starts 12 cm left of centre, two orange beams measure the distance to each wall, and the drone moves back to within 2 cm of the centre.">
</p>

`navless_nav` follows the route with its nose pointing north for the whole lap. The tunnel is the 10-point checkpoint, and it's dark. Everything the drone uses to see sideways comes from the camera, and for about 3 s the camera has nothing to track. Two things carry it through:

- **`blind_carry`**: if odometry stops inside the tube, the drone keeps moving along the route on PX4's own estimate, for at most 4 s.
- **Side time-of-flight centring**: there is one VL53L1X on each side. These sensors carry their own 940 nm infrared light, so the dark doesn't matter. In a tube, the offset from centre is simply (L − R) / 2. A reading is used only when all of these hold:
  - the drone is inside the tube;
  - the two readings add up to the tube's width;
  - neither reading is older than 0.3 s;
  - the correction stays within 15 cm of the planned route.

  Otherwise the drone flies the planned route.

The first ToF driver reached only about 30 cm, and each wall is about 65 cm from its sensor. The rewrite reads each sensor on its own bus, and its range is about 60–90 cm. On skybeta-19 it ran at **9.6 Hz**, with **99–99.6 %** of readings valid and steady to ±0.3 cm. In a closed-loop test, with the sensors raycast off the course model, a route 10–12 cm off centre ends **within 2 cm** of the centreline.

Gazebo also showed that PX4 resets its heading to the magnetometer about 0.5 s into a vision gap: a 76° jump in our sim. `EKF2_MAG_TYPE=5` fixed it in simulation. It isn't set on the drone: any PX4 parameter change waits for the organisers' OK ([FLIGHT_DAY.md §5](FLIGHT_DAY.md)).

### 4. Path planner

[`planner/`](planner) turns the course into a 3D occupancy grid, pads every wall by 0.28 m, and finds a path with A\*. It smooths the path into a B-spline the drone can follow and replans every second. `px4_bridge_node.py` flies the result on plain PX4.

In the organisers' FlyX Lab, earlier builds lost tracking at the first maze corner, and PX4's estimate drifted 25–50 m. None of the logged runs since the corner fix has lost tracking there ([`tools/flyx_lab/`](tools/flyx_lab)).

On `main`, `navless_nav` plans its path with this planner on the as-built map. It also re-plans around obstacles it sees (`replan_on_obstacles`, on by default). The planner path passes the Gazebo gate 3/3. Live re-planning is still untested. For the real flight, skybeta-19 runs `drone19/flight`, which follows the as-built route file exactly.

### 5. Testing

- **[`master_sim/`](master_sim):** one command runs the drone's flight code unmodified in Gazebo + PX4 SITL. It reports checkpoint times, the distance from the pad, the closest wall (props included) and each blind pass through the tube.
- **`master_sim/regress.py`:** runs several laps per change, because one lap can mislead. The same planner code once cleared the maze cleanly and stalled against M2 on the next run.
- **`gazebo_sim/tools/gate_asbuilt.sh`:** the gate a branch passes before it goes near the drone. It flies three laps on the as-built course and gives the minimum clearance per wall.
- **`tools/sim_truth_recorder.py`:** checks PX4's estimate against simulation truth in the FlyX Lab.
- **[`tools/preflight_check.sh`](tools/preflight_check.sh):** a read-only check on the drone that must report 0 FAIL before arming.
- **`planner/real_flight_kill.sh`:** stops a scripted flight. If the drone is airborne, it descends until PX4 disarms.

## Results

| What | Where | Result |
|---|---|---|
| Full lap, skybeta-19's code (`drone19/flight` `0b9afee`) | Gazebo + PX4 SITL, as-built course | **3/3 landed**, 47 s per lap, 6.6–7.1 cm from the pad centre, closest wall 6.0 cm (north net), 0 contacts |
| Full lap, `main`'s planner path (`c74801d`) | same | 3/3 landed, 93–94 s, 2.9–3.6 cm from the pad, closest wall 2.8 cm (M1), 0 contacts |
| Visual odometry through the gate | skybeta-19, PX4 powered, disarmed | 28.43 Hz, 59.89 ms sample age (median) |
| Side ToF sensors | skybeta-19 bench | 9.6 Hz, 99–99.6 % valid, ±0.3 cm |
| ToF centring | closed-loop test, simulated sensors | 10–12 cm off centre → within 2 cm |
| Course-model localizer | simulation, window route | 4–5 cm median error (odometry alone: up to 143 cm) |

**Status:** every result above is from simulation or the bench. skybeta-19 hasn't flown the course yet. First flights follow [FLIGHT_DAY.md](FLIGHT_DAY.md).

## Run it

**Simulation.** Use WSL or Linux with Gazebo Harmonic and PX4 SITL, set up as in [`gazebo_sim/README.md`](gazebo_sim/README.md):

```bash
bash gazebo_sim/tools/gate_asbuilt.sh 3                         # 3 laps of main on the as-built course
BRANCH=drone19/flight bash gazebo_sim/tools/gate_asbuilt.sh 3   # the same gate for skybeta-19's branch
master_sim/run_master.sh --gui                                  # watch the drone's own launch fly the course
master_sim/regress.py --laps 3 default                          # several laps, one table
```

**The real drone.** Follow [FLIGHT_DAY.md](FLIGHT_DAY.md). It covers setup, the preflight check (0 FAIL), placing the drone on the launch mark and flying.

**This README's GIFs.** Run `python docs/media/make_readme_media.py`. It renders them from the course files and a recorded gate lap in [`docs/media/data/`](docs/media/data).

## Repo map

| Folder | What's in it |
|---|---|
| [`navless_nav/`](navless_nav) | The ROS 2 package that flies the course: `course_mission`, the route, the tube logic and the launch files |
| [`tube/`](tube) | Side ToF driver, `tube_node` centring, the bench view, experimental stereo centring |
| [`hardware_slam/`](hardware_slam) | cuVSLAM on the drone, the odometry gate, the SLAM map viewer and startup scripts |
| [`depthcloud/`](depthcloud) | Stereo depth, the course-model localizer, the as-built course from the scan |
| [`planner/`](planner) | 3D grid, A\*, B-spline, the PX4 bridge, and the real-flight start and kill scripts |
| [`gazebo_sim/`](gazebo_sim) | Gazebo + PX4 SITL of the course, the gate scripts and the mirror dashboard |
| [`master_sim/`](master_sim) | One-command sim of the drone's code, with a scorecard and regressions |
| [`split_sim/`](split_sim), [`sim_sessions/`](sim_sessions), [`split_setup/`](split_setup), [`split_height/`](split_height), [`local_sim/`](local_sim), [`local_course/`](local_course) | Mac + Jetson split simulation, shared-Jetson sessions and Apple Silicon setups |
| [`tools/`](tools) | Preflight check, truth recorder, FlyX Lab helpers, magnetometer check |
| [`mission/`](mission) | Standalone mission scripts, such as the square movement test |
| [`line_reference/`](line_reference), [`appearance_profile/`](appearance_profile) | Experiments: a floor-line reference and synthetic wall textures |
| [`docs/`](docs) | The Jetson simulation runbook, and this README's media and the script that makes it |
| [`src/`](src), [`scripts/`](scripts) | The Navless MCP server for the agent channel (below) |

Saved maps, recordings and credentials stay out of Git.

---

## The agent channel

Navless is also the tool we built to work this way. It's a small MCP server that puts every teammate's coding agents in one Slack channel. Each agent posts under its own name: `lidar (Raaga)`, `slam-koji (Koji)`, `depthcloud (Shining)`, `pathplanner (Favour)`, `sim (Bryan)`. Agents say what they're touching before they start, ask the agent that owns a piece of code, and wait for the answer. The humans read the same channel and step in.

Any agent that speaks MCP can use it: Claude Code, Cursor, Codex or Claude Desktop.

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

The rules agents follow live in [`AGENTS.md`](AGENTS.md).

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

- **Raaga:** flight code (`navless_nav`), the tunnel and side ToF, the Gazebo sim
- **Koji:** positioning, with cuVSLAM on the drone and the odometry gate
- **Shining:** the as-built course from the phone scan, stereo depth and the localizer
- **Favour:** the path planner
- **Bryan:** testing, with the master sim, the FlyX Lab and flight-day checks
