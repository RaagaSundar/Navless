# First-flight tests (`navless_flight_tests`)

These are three simple tests to find out whether skybeta-19 flies. They follow the organisers' rules: the node sends setpoints only, and the safety pilot arms.

| Test | What it does |
|---|---|
| `hover` | climb 1 m, hold 10 s, land |
| `yaw` | climb 1 m, hold 2 s, yaw 180° clockwise at 30°/s, hold 2 s, land |
| `forward` | climb 1 m, hold 2 s, fly 2 m along the starting heading at 0.4 m/s, hold 10 s, fly back (heading unchanged), hold 2 s, land |

**How it talks to the drone.** The node publishes only `/skybeta/setpoint/pose` and `/skybeta/setpoint/twist`, and reads `/skybeta/state` and `/visual_slam/tracking/odometry`. The organisers' `skybeta_flight_interface` runs next to it, unchanged. It owns Offboard, the heartbeat and the safety envelope, which is tightened here to a 2.0 m ceiling and a 1.0 m/s speed cap. The node:
- never publishes to `/fmu/in/*` (forbidden by the skybeta README);
- never arms;
- never changes mode.

**How it moves.** Motion is a setpoint that moves at the test's speed, so nothing jumps. Landing is straight down with twist setpoints until PX4 detects the touchdown and disarms; the interface has no land command. If odometry goes stale, the setpoint freezes where it is.

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
