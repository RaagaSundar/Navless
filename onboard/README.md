# SLAM-required test flights on FlyX and a real Jetson

These standalone ROS 2/PX4 scripts use **PX4's fused local position**, with
mandatory monitoring of the existing SLAM -> external-vision bridge -> PX4 path.
They never generate odometry, start another XRCE agent, or change PX4 parameters.

| Script in `examples/` | Requested plan |
| --- | --- |
| `hover_land.py` | Climb 1 m, hold initial XY/heading for 10 continuous seconds, land |
| `hover_yaw_180_land.py` | Climb 1 m, hover 2 s, yaw clockwise 180 degrees at a 10 degree/s setpoint slew, settle 2 s, land |
| `forward_2m_return_land.py` | Climb 1 m, move 2 m along initial heading at a 0.4 m/s setpoint slew, settle, hold 10 s, return to initial XY without turning, settle, land |

Positions are local NED; negative Z is up. Height is relative to initial estimated
Z. Arrival tolerances are 0.20 m XY, 0.15 m Z and 6 degrees yaw. Hold timers restart
when the estimated pose leaves tolerance. There is no obstacle avoidance.

**Running a flight script can request Offboard and arm automatically after
preflight.** The physical pilot must approve the exact plan, clear the flight
volume, and have a tested RC takeover/kill path. Software tests and simulator
imports are not hardware qualification. No physical aircraft was run for this PR.

## Requirements and unchanged estimator path

Use the Jetson's existing, reviewed ARM ROS environment, matching `px4_msgs`,
calibrated OAK driver/CameraInfo/TF, cuVSLAM, independent odometry gate, original
PX4 bridge, and exactly one reviewed XRCE transport. Do not copy x86 FlyX GPU
binaries, camera assumptions or simulator parameters onto the aircraft.

The hardware stack documented in [`hardware_slam`](../hardware_slam/README.md) is:

```
/visual_slam/tracking/odometry
  -> existing odom_gate.py
  -> /mapping/validated_odometry
  -> existing px4_odom_bridge
  -> /fmu/in/vehicle_visual_odometry
  -> existing XRCE link -> PX4 estimator
  -> /fmu/out/vehicle_local_position[_vN] -> flight script
```

Use continuous tracking odometry, not loop-closed map poses. The bridge owns
FLU/ENU to FRD/NED conversion, covariance validation and source reset handling.
The scripts resolve PX4 output topic suffixes from installed message versions.
Confirm the installed firmware/messages actually provide the fields they use.
The existing examples address MAVLink system/component **1/1**. Confirm the
actual `VehicleStatus.system_id` and `component_id` match before running; a
different vehicle address needs a separately reviewed command-address change.

FlyX supplies `/estimator/healthy` (`std_msgs/Bool`). The real Jetson gate instead
supplies `/mapping/gate_status` (`std_msgs/String` JSON). The included
`jetson_slam_health.py` adapter publishes `/onboard/vision_health` from **fresh
validated odometry and actual gate reports**. It publishes no position or flight
commands and does not change the gate or bridge. Do not replace it with a
constant `true` publisher. A second publisher on its output makes it fail closed.

## Install on the Jetson without starting a flight

On the Jetson host, using the existing login and Docker installation:

```bash
git clone --branch codex/slam-test-flight-examples --single-branch \
  https://github.com/RaagaSundar/Navless.git Navless-test-flights
cd Navless-test-flights
git rev-parse HEAD
# Copy only Python/config/docs into the already running aircraft container.
docker exec skybeta-stack mkdir -p /root/team_ws/test_flights
docker cp onboard/. skybeta-stack:/root/team_ws/test_flights/
docker exec -it skybeta-stack bash
```

Inside that container:

```bash
export SKYBETA_UNDERLAY_ONLY=1
source skybeta_env.sh
cd /root/team_ws/test_flights
# Physical aircraft domain only; fail rather than silently changing domains.
test "${ROS_DOMAIN_ID:-}" = 19
export DRONE_PARAMS_FILE="$PWD/config/jetson_test_flights.yaml"
python3 -c 'import rclpy, nav_msgs.msg, std_msgs.msg, px4_msgs.msg'
```

`config/jetson_test_flights.yaml` explicitly selects `use_sim_time: false` and
`/onboard/vision_health`. Use it for every command below. Keep the existing
hardware clock/bridge timestamp relationship; do not adjust timestamps to make
a failed freshness check pass.

No build, firmware flash, service restart, PX4 parameter write or new agent is
required by these files. The existing stack must already be commissioned.
Inspect its current owners before running anything: mapping-mode services can
suppress the competitor process, while flight-mode services may start it.
Arrange an operator-controlled state with the estimator/gate/bridge/transport
running and **all other mission/keyboard controllers inactive**. Do not restart
`handheld-slam.service` or the whole stack just to switch missions: that can
reset SLAM or restore a competing flight process. This PR does not switch those
services automatically.

## Disarmed checks first

With props removed for bench checks, inspect the current processes and actual
messages. Do not run a flight script as a probe.

```bash
ps -ef
ros2 topic list
ros2 param get /px4_odom_bridge input_topic
# Expected: /mapping/validated_odometry; otherwise stop for bridge-owner review.
ros2 topic info /fmu/in/vehicle_visual_odometry --verbose
ros2 topic info /onboard/vision_health --verbose
```

Confirm one actual vision publisher and no existing health adapter before
starting the adapter in a dedicated foreground terminal with the same ROS
underlay, domain and working directory:

```bash
python3 jetson_slam_health.py --ros-args --params-file "$DRONE_PARAMS_FILE"
```

Leave it running during preflight and the approved flight. The adapter requires
validated capture and receipt ages <= 0.25 s, valid odom/base_link pose/covariance,
a fresh tracking-healthy gate report (1.5 s allowance for its 1 Hz report rate),
and an advancing accepted count. Gate quarantine/bootstrap failures, when those
fields exist, also make health false. It does not certify the vehicle.

In another equally configured terminal, record a named, passive check:

```bash
check_id="$(date -u +%Y%m%dT%H%M%SZ)-jetson-slam-preflight"
mkdir -p "evidence/$check_id"
python3 check_odometry.py --seconds 30 --ros-args \
  --params-file "$DRONE_PARAMS_FILE" > "evidence/$check_id/preflight.jsonl" 2>&1
check_result=$?
cat "evidence/$check_id/preflight.jsonl"
printf 'Checker exit status: %s\n' "$check_result"
```

`check_odometry.py` creates subscriptions and inert publisher objects; it never
creates DDS control writers or calls the flight state machine. It observes for
at least 30 wall seconds and requires the final 5 seconds continuously ready.
It checks fresh landed/disarmed PX4, no failsafe, valid position/heading, raw
SLAM, vision input, health and external-vision position/yaw fusion. A stopped
ROS clock is bounded by a wall-time timeout. A pass is a readiness observation,
not permission to fly. Exit nonzero means stop and inspect the reported reason.
Archive the exact code hash/settings with this evidence. For project bench or
flight reports, create `run-comparison.json` with a human name, kind `bench` or
`flight attempted`, explicit baseline (or null), changes/confounds and local
evidence using [`split_sim/run_comparison.py`](../split_sim/run_comparison.py).

## Run one approved plan

Only after bench checks, hardware commissioning, props/flight-area preparation
and explicit pilot approval, run **one** of these commands in the configured
container. The hover-only plan is the least complex starting point.

```bash
python3 examples/hover_land.py --ros-args --params-file "$DRONE_PARAMS_FILE"
# Or, separately approved after the first test:
# python3 examples/hover_yaw_180_land.py --ros-args --params-file "$DRONE_PARAMS_FILE"
# python3 examples/forward_2m_return_land.py --ros-args --params-file "$DRONE_PARAMS_FILE"
```

Keep the health adapter and existing SLAM/gate/bridge/transport running. Preserve
PX4 ULogs, script output and source/settings identity. Completion requires a
fresh landed report plus disarmed status. Do not reset or start another mission
until the physical pilot confirms the state.

The scripts require raw SLAM/vision capture and receipt ages <= 0.5 ROS seconds,
fresh bridge health, fresh compatible `EstimatorStatusFlags`, `cs_ev_pos` and
`cs_ev_yaw`, and no yaw fault, horizontal-position/yaw rejection or inertial
dead reckoning. EV height and velocity fusion are not required; height remains
PX4's separately configured estimate. Preflight failure sends no arm command;
recovery must earn a new full 2 s prime. Frame/reset changes during priming
recapture the start; during active control they yield.

**Loss of SLAM/telemetry or control authority stops the Offboard stream and the
script exits unsuccessfully. It does not guarantee a landing.** PX4's already
configured Offboard-loss behavior and the pilot's RC recovery path must be
reviewed and tested independently. Ctrl-C/process exit is also not a Land
command. Do not increase timeouts, publish dummy health/odometry, or force arm
to get past a failed check.

## Reported yaw failure and verification limits

A user-provided FlyX log reached YAW and then reported
`SLAM odometry missing, invalid or stale; stopping Offboard stream`. The visible
run report marked program/flight failed and said landing was not confirmed.
The generic error alone did not distinguish source rejection from clock skew.
Retained tracker/bridge logs were subsequently inspected; see the correction
and evidence below. This PR preserves the checks and adds capture/arrival age
and sample-validity diagnostics to future failures.
The ROS_LOCALHOST_ONLY deprecation warnings did not produce that abort branch.
Do not describe the yaw plan as a passing flight or as hardware-qualified.

Software validation: the original three scripts passed 207 inert-ROS unit tests.
This PR additionally tests diagnostics, passive-checker publication prevention,
and gate-health rejection cases; the corrected full suite has **256 tests**. Python compilation and
Git whitespace checks passed. No real flight or new simulator flight was
performed while preparing the PR. Syntax/import checks of the original saved
scripts passed in the FlyX companion before the user's subsequent yaw attempt.

Official interface/safety references:
[PX4 ROS 2 Offboard example](https://docs.px4.io/main/en/ros2/offboard_control),
[PX4 external position estimation](https://docs.px4.io/main/en/ros/external_position_estimation).
These explain the Offboard heartbeat, manual recovery requirement and frame
conventions; use documentation matching the installed PX4 release.

## Timing correction for the reported forward/yaw aborts

The first monitor incorrectly gave `EstimatorStatusFlags` a 1.0 s lifetime,
exactly its nominal publication period. PX4's `EKF2::PublishStatusFlags` publishes
at about 1 Hz and immediately on status/fault changes. This made ordinary DDS
scheduling jitter sufficient to abort an otherwise fresh flight. The monitor now
uses a bounded **2.5 s** receipt/source-age budget for that slow status stream.
The high-rate PX4 telemetry and 0.5 s SLAM/vision/bridge-health limits remain.
Repeated status packets cannot renew freshness, and actual local-position
`dead_reckoning` is now explicitly rejected as well as the existing fusion faults.

Raw SLAM and vision-input capture checks now share the established FlyX bridge's
**5 ms** future-clock tolerance (`onboard/vio/validation.py: CaptureGate`, default
`future_tol_s=0.005`). This fixes a second mismatch: the bridge could accept a
measurement that the mission rejected for tiny callback/clock ordering skew.
Samples beyond 5 ms ahead and stale/frozen/invalid data still fail. This is a
bounded timestamp-compatibility fix, not permission to ignore tracking loss.

Regression evidence: nine added timing/skew cases fail against commit `6d63535`
and pass after the correction; the full suite has **256 passing tests**. Source:
[PX4 EKF2 implementation](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/ekf2/EKF2.cpp),
`EKF2::PublishStatusFlags` (also checked in the installed simulator source).
The user's original failures remain failures; this software correction alone
does not establish a successful rerun or prove the yaw abort's exact cause.

The retained FlyX logs subsequently became readable. A cuVSLAM warning at
`1794097573.815511960` withheld a pose for unavailable/non-finite covariance,
immediately before the reported yaw abort; the bridge health later became false.
At `1794097884.271982676`, just before the reported forward abort, the bridge
reported healthy=true with no stale/future captures. This supports correcting
the forward monitor cadence without blaming the SLAM source. The yaw command
rate is reduced from 30 to **10 degrees/s** (18 s for 180 degrees) as a conservative
tracking-demand change. The covariance, health and freshness requirements are
retained. Only a new recorded flight can establish whether this rate resolves
the yaw maneuver; the old failure is not reclassified.

Live passive checks also exposed the same 1 Hz publication issue for PX4
`VehicleLandDetected` (confirmed in `LandDetector.cpp`). It now has a 2.5 s
receipt-age budget for preflight and ground confirmation. False landed reports
still block startup; expired reports still block arming/confirmation. The passive
checker records readiness transitions and finishes using wall time while
rejecting a stopped ROS clock. It permits ROS parameter events and suppresses
all `/fmu/in/` publishers. No airborne test result is implied by these bench fixes.
