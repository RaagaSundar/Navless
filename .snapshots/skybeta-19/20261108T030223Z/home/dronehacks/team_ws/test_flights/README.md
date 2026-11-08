# Skybeta-19: pilot-handover test flights

These three entry points replace the direct-PX4 tests on 19. The only flight outputs are
`/skybeta/setpoint/twist` (zero velocity on the ground) and `/skybeta/setpoint/pose`
(ENU in the continuous cuVSLAM `odom` frame). The flight interface owns the PX4 connection.
The tests cannot arm, switch mode, disarm or send a Land command.

| Launcher plan | After pilot handover |
| --- | --- |
| `hover` | Climb 1 m above the launch pose; hold launch XY and heading for 10 continuous seconds. |
| `yaw` | Climb 1 m, hover for 2 s, turn clockwise 180° at a 10°/s setpoint slew; settle for 2 s. |
| `forward` | Climb 1 m, hover for 2 s, travel 2 m along launch heading at a 0.4 m/s setpoint slew; settle for 1 s, hold for 10 s, return without turning and settle for 1 s. |

All three hold their final pose until the pilot takes back control and lands. The node never
commands a descent below the envelope. Arrival/hold use measured odometry (0.20 m XY,
0.15 m Z, 6° yaw); leaving tolerance resets the timer. These tests have no collision avoidance.
The yaw slew was reduced from 19's old 30°/s. Numbers are setpoint rates, not measured aircraft speeds.

**Before a pilot-approved rehearsal:** physically remove the propellers. On the launch mark,
with the aircraft disarmed, confirm the course mission and other controllers are stopped.
The original `stop-navless` stops the entire competitor pane, including its flight interface.
The test launcher provides a separate foreground `bash flight.sh interface` action which uses
the organisers' unchanged launch with `competitor:=false`, pilot handover enabled, manual
mode requests disabled, and limits 0.3–2.5 m and 1 m/s. Use it only if there is no existing
interface. Keep that terminal open. Exactly one existing health adapter should be running;
if two are present, coordinate with their owners to stop the duplicate while disarmed.
Use `bash flight.sh health` only if none is running. The node reads the existing SLAM,
validated bridge health, external-vision input and PX4 estimator flags; it does not supply any of them.

In `~/team_ws/test_flights` on the Jetson host, `bash flight.sh check` is passive: no
mission timer or setpoint writer. It requires 30 seconds of observation and at least the last
5 seconds continuously ready. It also checks the interface is subscribed to both setpoint topics.
Keep any failing diagnostic. Follow the team run-comparison procedure for bench/flight evidence.

**With the pilot's approval of one exact maneuver:** `bash flight.sh start hover` (or `yaw`,
`forward`). Wait for **READY** before the safety pilot arms on the ground in Altitude and
switches to Position. The organiser interface performs the handover. The node needs fresh
agreement from both PX4 and the interface before publishing a climb pose. At **HANDOFF** the
pilot takes control and lands; success requires fresh landed and disarmed reports. Early
takeover, invalid/stale telemetry, competing writers, movement before handover or a maneuver
timeout stops publication and fails the run. Stopping the process is not a landing command:
the interface can retain the last target. Follow the organiser's RC takeover procedure.
After disarm, close the standalone interface before restoring the course mission.

Reference: aircraft source and [competitor guide](https://github.com/Artash-N/skybeta/blob/3c42933f5adc7b22997614946f6719b2c2f89f20/docs/competitor_guide.md).
The example and interface source hashes matched the copies in 19's container. No change to
either package, PX4 parameters, the active course route or calibration is required.
