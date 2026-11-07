# Flight day: skybeta-19

`navless_nav` is the only commander. Everything below is for the first live flights on `main`.

## 1. Set up the drone (once, on the ground, disarmed)

```bash
ssh dronehacks@skybeta-19.local
sudo systemctl disable --now handheld-slam.service   # else the competitor window is 'sleep infinity'
cd ~/team_ws/src/Navless && git pull                 # main; then rebuild the team workspace as usual
sudoedit /etc/skybeta/drone.env                      # COMPETITOR_PKG=navless_nav
                                                     # COMPETITOR_LAUNCH=navless_course_slow.launch.py
docker rm -f skybeta-stack                           # the stack restarts with the new launch
```

Use the slow launch (0.5 m/s) until real laps look like the sim. The course launches now use
`gazebo_sim/course/route_centreline_window_asbuilt.json`, `net_margin_m:=0.25`, and the checked side
ToF sensors on 19. The old plan route is for FlyX comparisons only; pass that route explicitly there.
The as-built route is loaded as recorded, never replaced with the dynamic planner's old plan prior.
`navless_course.launch.py` is the fast variant (0.7 m/s). `fly_home` remains off; tube-only
`blind_carry` is on, but acts only when odometry stops, not when frozen poses continue. Qualify the
PX4 odometry gate and heading behaviour before relying on it (section 5). Never run the planner
alongside `navless_nav`: both command flight. Its launch defaults to `approve_flight:=false`.

The exact pre-consolidation flight snapshot is tagged `archive/drone19-flight-20261107` (8c3efb2).
Merging branches does not deploy or flight-qualify the combined code; review and deploy separately.

## 2. Run the check: it must say 0 FAIL

```bash
bash ~/team_ws/src/Navless/tools/preflight_check.sh
```

It's read-only. It checks the service, `drone.env`, that `navless_nav` finds its route (the
`REPO_ROOT` fix), that there's no second commander, that `map_server.py`/stereo aren't eating CPU,
cuVSLAM tracking, the odometry rate, and the PX4 link. Each FAIL prints its fix.

## 3. Place the drone on the launch mark: aim exactly north

The drone has no compass: `navless_nav` assumes the nose points exactly north at power-up, and every
degree off moves the far end of the route about 9 cm sideways. The old plan route's placement
results do not qualify the as-built route; use the as-built Gazebo gate and pilot review.
- Hold a straight edge of at least 1 m along the airframe's centreline. Measure from both of its ends
  to the M1 wall face (the east side of corridor 1, which runs north–south).
- The two readings must agree within **17 mm per metre of straight edge for 1 deg**, or **9 mm for 0.5 deg**.
- First check that M1 is square to the course on the day: the layout may differ from the model.
- After any handling or placement, restart the stack and keep the drone still: the odometry frame
  is fixed at startup. Never restart it in flight.

## 4. Fly

1. Wait for `pre-arm height check passed` in the competitor log. If it keeps warning, power-cycle the
   Pixhawk.
2. Pilot: select **Altitude**, arm, then switch to **Position**. Arming already in Position gives no
   autonomy.
3. To take over at any time, change the flight mode: PX4 leaves Offboard and `navless_nav` pauses.
4. It lands on the pad by itself and disarms (`COM_DISARM_LAND`).

## 4b. Flying another script (the planner, the square test) instead of `navless_nav`

Only if the team decides to. Sections 1–3 still apply (run the preflight check first). Then, on the host:

```bash
bash ~/team_ws/src/Navless/planner/real_flight_start.sh mission/square_test.py   # movement test (below)
bash ~/team_ws/src/Navless/planner/real_flight_start.sh                          # px4_bridge_node
bash ~/team_ws/src/Navless/planner/real_flight_start.sh --stop-on-pad            # ... with bridge args
docker exec skybeta-stack tail -f /tmp/flight.log
```

It refuses if the drone is armed or a script is already running. Then it replaces the stack's `competitor`
window (`navless_nav`) with the script, so only one program commands `/skybeta/setpoint/*`. Fly as in
section 4: Altitude, arm, Position.

**Movement test** (`mission/square_test.py`):
- Takes off to 0.7 m and flies a 1 ft square: forward, right, back, left.
- At each corner it turns in place so the camera faces the next side, then lands where it took off.
- Options: `--side`, `--alt`, `--left` to mirror it.
- Check it offline with `python3 mission/square_test.py --selftest`.
- In the FlyX lab, `--px4` mode arms by itself.

Rules for any script started this way:
- Command only through `/skybeta/setpoint/*`.
- Never wait for keyboard input.
- Treat SIGINT/SIGTERM as "land, then exit" (copy `square_test.py`'s handler).
- Exit once disarmed.

To stop it: `bash ~/team_ws/src/Navless/planner/real_flight_kill.sh`.
- **Flying:** it descends where it is until PX4 disarms, then exits.
- **On the ground or after a pilot takeover:** it exits at once.
- `--now` stops publishing immediately, so the pilot must take over.
- To get `navless_nav` back, on the ground only: `docker rm -f skybeta-stack`.

Caveats: flight_interface ignores pose yaw, so the bridge keeps the launch heading (square_test.py turns
with velocity setpoints instead). Only plain-PX4 runs
in the FlyX lab have flown the bridge so far.

## 5. Open decisions (humans)

| Decision | Sim evidence (master sim / Raaga's harness) | What settles it |
|---|---|---|
| `blind_carry` in the dark tube | On for course launches; stopped-feed tests passed at 0.5 m/s. At 0.7 m/s, late recovery with magnetometer fusion left 3.9 cm; `EKF2_MAG_TYPE=5` gave 4/4 passes with >=40 cm (plan-course simulation) | Off-course covered-camera check and review of the as-built gate |
| PX4 vision through Koji's gate | With the stock bridge, a frozen pose is fused and PX4 coasts blind | Same test: does odometry stop or freeze while `vo_state` is 2? |
| Window vs Gate B route | Window: about 15 cm worst wall (slow). Gate B: about 10–11 cm at A1 | Organisers: does the window earn the chicane's 7 points? |
| Slow vs fast launch | As-built slow-launch Gazebo gate: 3/3 landed; north-net clearance 5.9–6.5 cm. Old plan-course speed margins do not apply | Practice flight with the pilot ready to take over |

**Off-course dark bench check**: drone stationary off the course, VIO running, props off. Cover both
lenses for 3–5 s or use a dark box. Nobody walks through the course. Watch
`ros2 topic echo /visual_slam/status --field vo_state` (1 tracking, 2 lost) and
`ros2 topic hz /visual_slam/tracking/odometry`. Note: did it drop, for how long, did odometry stop or
keep coming (frozen), and did the pose jump on recovery? Raaga's dashboard records it all:
`python3 gazebo_sim/mirror/mirror.py --drone <ip> --record dark_bench.jsonl`, then
`python3 gazebo_sim/mirror/tube_test.py dark_bench.jsonl`. Use `tools/mag_check.py` to observe
magnetometer takeover. `EKF2_MAG_TYPE=5` still requires the team's/organisers' approval; do not
change aircraft parameters as part of a code merge.

## Not blocking

Side ToF does steer on 19's course-launch defaults: re-check each sensor independently after mounting.
`cloud_localize` is separate (`navless_nav` doesn't fuse it). Camera calibration dump (`/oak/left/camera_info`, `tf2_echo base_link
oak_left_camera_optical_frame`) only matters for the sim.
