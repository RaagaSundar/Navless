# Flight day: skybeta-11

`navless_nav` is the only commander. Everything below is for the first live flights on `main`.

## 1. Set up the drone (once, on the ground, disarmed)

```bash
ssh dronehacks@skybeta-11.local
sudo systemctl disable --now handheld-slam.service   # else the competitor window is 'sleep infinity'
cd ~/team_ws/src/Navless && git pull                 # main; then rebuild the team workspace as usual
sudoedit /etc/skybeta/drone.env                      # COMPETITOR_PKG=navless_nav
                                                     # COMPETITOR_LAUNCH=navless_course_slow.launch.py
docker rm -f skybeta-stack                           # the stack restarts with the new launch
```

Use the slow launch (0.5 m/s) until real laps look like the sim. `navless_course.launch.py` is the fast
one (0.7 m/s, about 11 s quicker) and needs tighter placement. Keep `fly_home` and `blind_carry` off
unless you decide otherwise (section 5). Don't install or start the planner (`px4_bridge_node`): its
launch file defaults to `approve_flight:=true` until PR #22 merges, and in the sim it still hits walls.

## 2. Run the check: it must say 0 FAIL

```bash
bash ~/team_ws/src/Navless/tools/preflight_check.sh
```

It's read-only. It checks the service, `drone.env`, that `navless_nav` finds its route (the
`REPO_ROOT` fix), that there's no second commander, that `map_server.py`/stereo aren't eating CPU,
cuVSLAM tracking, the odometry rate, and the PX4 link. Each FAIL prints its fix.

## 3. Place the drone on the launch mark: heading within 1 deg (0.5 deg on the fast launch)

The drone has no compass: `navless_nav` assumes the nose points exactly north at power-up, and every
degree off moves the far end of the route about 9 cm sideways. In the sim, −2 deg left 5 cm at C2a.
- Hold a straight edge of at least 1 m along the airframe's centreline. Measure from both of its ends
  to the M1 wall face (the east side of corridor 1, which runs north–south).
- The two readings must agree within **17 mm per metre of straight edge for 1 deg**, or **9 mm for 0.5 deg**.
- First check that M1 is square to the course on the day: the layout may differ from the model.
- Power up (or restart cuVSLAM) only after placing it: the odometry frame is fixed at power-up.

## 4. Fly

1. Wait for `pre-arm height check passed` in the competitor log. If it keeps warning, power-cycle the
   Pixhawk.
2. Pilot: select **Altitude**, arm, then switch to **Position**. Arming already in Position gives no
   autonomy.
3. To take over at any time, change the flight mode: PX4 leaves Offboard and `navless_nav` pauses.
4. It lands on the pad by itself and disarms (`COM_DISARM_LAND`).

## 4b. Flying the planner (`px4_bridge_node`) instead of `navless_nav`

Only if the team decides to. Sections 1–3 still apply (run the preflight check first). Then, on the host:

```bash
bash ~/team_ws/src/Navless/planner/real_flight_start.sh     # refuses if armed; add --stop-on-pad etc.
docker exec skybeta-stack tail -f /tmp/px4_bridge_flight.log
```

It replaces the stack's `competitor` window (`navless_nav`) with the bridge in `--skybeta-mode`, so only
one node commands `/skybeta/setpoint/*`. Fly as in section 4: Altitude, arm, Position.

To stop it: `bash ~/team_ws/src/Navless/planner/real_flight_kill.sh`.
- **Flying:** it descends where it is until PX4 disarms, then exits.
- **On the ground or after a pilot takeover:** it exits at once.
- `--now` stops publishing immediately, so the pilot must take over.
- To get `navless_nav` back, on the ground only: `docker rm -f skybeta-stack`.

Caveats: flight_interface ignores pose yaw, so the cameras keep the launch heading. Only plain-PX4 runs
in the FlyX lab have flown the bridge so far.

## 5. Open decisions (humans)

| Decision | Sim evidence (master sim / Raaga's harness) | What settles it |
|---|---|---|
| `blind_carry` in the dark tube | Default (carrot freezes): crashed 3/3 (the tube twice, M3 once). `blind_carry`: landed 4/4, including late recovery and a 15 cm exit jump (old speeds; untested at 0.7 m/s) | The props-off tube carry (below) |
| PX4 vision through Koji's gate | With the stock bridge, a frozen pose is fused and PX4 coasts blind | Same test: does odometry stop or freeze while `vo_state` is 2? |
| Window vs Gate B route | Window: about 15 cm worst wall (slow). Gate B: about 10–11 cm at A1 | Organisers: does the window earn the chicane's 7 points? |
| Slow vs fast launch | Fast: 32.6 s, ≥ 11.9 cm, needs ±0.5 deg placement | Two clean slow laps first |

**Props-off tube carry** (about 2 minutes, decides the first two rows): drone powered, VIO running, props
off. Carry it through the tube at walking pace, nose north, north mouth to south mouth. Watch
`ros2 topic echo /visual_slam/status --field vo_state` (1 tracking, 2 lost) and
`ros2 topic hz /visual_slam/tracking/odometry`. Note: did it drop, for how long, did odometry stop or
keep coming (frozen), and did the pose jump at the exit? Raaga's dashboard records it all:
`python3 gazebo_sim/mirror/mirror.py --drone <ip> --record tube_carry.jsonl`, then
`python3 gazebo_sim/mirror/tube_test.py tube_carry.jsonl`.

## Not blocking

Side ToF silent on I2C (check XSHUT; it doesn't steer). `cloud_localize` is being fixed (`navless_nav`
doesn't use it). Camera calibration dump (`/oak/left/camera_info`, `tf2_echo base_link
oak_left_camera_optical_frame`) only matters for the sim.
