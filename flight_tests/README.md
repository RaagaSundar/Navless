# First-flight tests (`navless_flight_tests`)

These are three simple tests to find out whether skybeta-19 flies. They follow the organisers' rules: the node sends setpoints only, and the safety pilot arms.

| Test | What it does |
|---|---|
| `hover` | climb 1 m, hold 10 s, land |
| `yaw` | climb 1 m, hold 2 s, yaw 180° clockwise at 30°/s, hold 2 s, land |
| `forward` | climb 1 m, hold 2 s, fly 2 m along the starting heading at 0.4 m/s, hold 10 s, fly back (heading unchanged), hold 2 s, land |

**How it talks to the drone.** The node publishes only `/skybeta/setpoint/pose` and `/skybeta/setpoint/twist`. It reads `/skybeta/state`, `/visual_slam/tracking/odometry`, and, read-only, `/fmu/out/vehicle_local_position`, `/fmu/out/estimator_status_flags` and `/visual_slam/status`. The organisers' `skybeta_flight_interface` runs next to it, unchanged. It owns Offboard, the heartbeat and the safety envelope, which is tightened here to a 2.0 m ceiling and a 1.0 m/s speed cap. The node:
- never publishes to `/fmu/in/*` (forbidden by the skybeta README);
- never arms;
- never changes mode.

**How it moves.** Motion is a setpoint that moves at the test's speed, so nothing jumps. Landing is straight down with twist setpoints until PX4 detects the touchdown and disarms; the interface has no land command. If odometry goes stale, the setpoint freezes where it is.

**It holds in PX4's frame, not cuVSLAM's.** `flight_interface` passes our position setpoints to PX4 almost unchanged. But PX4 holds position in EKF2's frame, which the organisers' guide says differs from cuVSLAM's by "a constant yaw offset and a constant translation". Commanding "stay at cuVSLAM's position" would send PX4 to a different spot the moment Offboard starts: the drone lifts off and slides sideways. So the node takes its position **and** heading from PX4's own estimate for everything:
- the "stay here" stream before takeoff;
- the hold point;
- the forward direction and the yaw target;
- the "have I arrived" checks.

PX4 is only ever told to stay where it already thinks it is, or to move relative to that. The offset (PX4 minus cuVSLAM, position and heading) is logged while waiting and at takeoff. No setpoints are streamed until both PX4's position and cuVSLAM's odometry are fresh, so without healthy vision `flight_interface` can't take over.

**Drift correction** (`drift_correction`, on by default). In the 2026-11-07 yaw flight, PX4 sat a steady 13 cm off its hold point: it leaves a fixed offset when something keeps pushing the drone. The node slowly moves its commanded point the opposite way, so the drone itself ends up on the target. That's integral action on our setpoint only; PX4's parameters are untouched.
- It only builds while the target stands still (holds and the yaw turn), once the drone is at least 0.4 m up, from fresh data, outside a 2 cm deadband, and while the error is under 0.4 m. A bigger error is a tracking problem or the pilot, not a steady push.
- It's capped at 0.2 m, with a warning if it hits the cap.
- The "have I arrived" checks still use the true target.
- It's logged as `corr_x` / `corr_y`.
- Fake-vehicle test with a steady push: hold error 12.9 cm without it, 1.7 cm with it.
- It doesn't fix the drift during the takeoff climb itself (25 cm back-left in both real flights). That points to the airframe: battery position or centre of gravity, props.

**Flight log.** Every run writes `~/team_ws/flight_logs/<time>_<test>.csv`, flushed every second, so a crash keeps the data. It has one row per control tick (20 Hz) from the moment the test is loaded, including the wait before arming:
- **PX4:** position, velocity, heading, age, reset counters and the size of the last position reset;
- **cuVSLAM:** position, heading, age and tracking state;
- **the offset** between the two;
- **our command:** the setpoint (or descend velocity) and how far the drone is from it;
- **EKF2's fusion flags:** vision position, vision yaw, magnetometer heading and so on;
- **status and events:** Offboard, armed, landed, clamps, and every event in a `note` column.

Commands:
- `bash fly_test.sh offset` prints the live offset, and whether EKF2 is fusing vision, from the newest log.
- `bash fly_test.sh logs` lists the logs.

Send the CSV for any flight that misbehaves. Better still, add the PX4 flight log (ULog) from the Pixhawk's SD card.

**One test per arming.** After landing and disarming, or after the pilot takes over, the node stops publishing for good, so a later arming can't hand the drone back to it.

## On skybeta-19

From the Jetson host, in the repo checkout (or wherever this folder is):

```bash
bash flight_tests/fly_test.sh install      # once: builds into ~/team_ws/flight_tests_ws
bash flight_tests/fly_test.sh hover        # disarmed only: hover test in the competitor window
bash flight_tests/fly_test.sh status       # what's running, last log lines
bash flight_tests/fly_test.sh restore      # disarmed only: back to navless_nav
```

`~/team_ws/flight_tests_ws` has a `COLCON_IGNORE`, so the stack's boot build (which scans all of `~/team_ws`) skips it. These tests can't break the navless_nav build. `drone.env` is not touched. `fly_test.sh` refuses to change the competitor window unless PX4 reports the drone **disarmed**.

## Flying a test

Rehearse the whole sequence **with the props off** first.

1. Drone on the floor and **still**. Restart the stack after it was moved (`docker rm -f skybeta-stack`), so cuVSLAM's origin is where it sits.
2. Run `bash flight_tests/fly_test.sh hover` (or `yaw` / `forward`). The log ends with "Waiting for the pilot to arm, then switch to Position".
   - Then run `bash flight_tests/fly_test.sh offset`.
   - The offset should be **steady**, not drifting between readings.
   - `ekf_ev_pos` should be `1` (EKF2 fusing vision) and `vo_state` should be `1` (cuVSLAM tracking).
   - **If either is wrong, don't arm.**
3. Pilot **arms** with the mode switch on **Altitude**, throttle low.
4. Pilot switches to **Position** within a few seconds. Offboard starts and the test flies itself. It must leave the ground within 10 s of arming, or PX4 disarms it.
5. **To take over at any time:** throttle towards centre (Position), or flip the mode switch, or use the kill switch. The node stops and never takes the drone back.
6. After it lands and disarms, run `fly_test.sh <next test>` for another one. When you're done, run `fly_test.sh restore`.

Make sure there's room: about 2.5 m ahead of the drone along its heading for `forward`, and 2 m of height.

## Tested

`test/run_fake_tests.sh` builds the package in a scratch workspace and flies each test against `test/fake_vehicle.py`, a pretend PX4 and pilot, on an isolated localhost-only ROS domain. Results on skybeta-19, 2026-11-07:
- hover reached 1.10 m and landed;
- yaw turned 180° clockwise (90° → −90°);
- forward flew 2 m along a 30° heading and back to the start;
- when the pilot took over mid-hover, the node stopped and didn't take back control.

**Not yet flown on a real drone.**
