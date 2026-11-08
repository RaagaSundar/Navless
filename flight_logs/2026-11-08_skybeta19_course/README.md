# skybeta-19 course runs, 2026-11-08 ~01:14-01:24 (PST)

Code on 19: `nav-stabilize-travel-heading` `2ffac5d` (main + travel heading, drift correction, flight recorder).
Launch: `navless_course_slow.launch.py route_source:=file` (heading_mode travel, drift_correction on,
tof_tube_centering on, cruise 0.5 m/s). Files: course_mission's flight recorder (one row per 0.1 s tick + every
log line) and the parameters each run used. 19 rebooted afterwards, so the tmux output is gone; the last
~second of each crash file may be missing (power lost before the flush).

| file | what happened |
|---|---|
| `20261108-091401_course.csv` | **Run 1.** Takeoff slid ~40 cm left, drift correction pulled it back, settled after 14 s. Flew the NE first leg ON its planned line by odometry (vo_state 1); hit a wall 1.6 m in (t=58.6 s, Offboard lost). Likely the route vs the real course: start pose (nose not exactly north / not exactly on the mark): ~25 cm planned room there. |
| `20261108-091952_course.csv`, `20261108-092004_course.csv` | Program restarts on the ground; no flight. |
| `20261108-092321_course.csv` | **Run 2**, armed further back than the launch mark (the route is placed relative to the arming pose, so it shifted back). Through the 1 m gap at MD's tip; at the turn east the drone swung 18 cm outside the setpoint (north) while lagging ~25 cm, side-on to the north net (travel heading), and hit it (~47 s). cuVSLAM then jumped 2.7 m and the node kept flying the route for 20 s (PX4 up to ~2.8 m) until the 20 s no-progress HOLD. |

## Later attempts (2026-11-08 ~03:43–05:36 PST)

Code: `main-localizer` (Favour's threaded planner + our localizer launch). 19's PX4 had `EKF2_MAG_TYPE` = 0 for
`114307`/`114602` and **5 (no magnetometer) for `133310`**.

| file | setup | what happened |
|---|---|---|
| `20261108-114307_course.csv` | planner + localizer (free mode), wall correction on | **Refused takeoff.** Pre-arm heading check: PX4 heading on the compass, ~100°+ off the camera. Offboard came anyway → REFUSED: descend in place until PX4 disarmed. Also: Favour's planner re-planned on the ground (phantom obstacles from the stereo cloud). |
| `20261108-114602_course.csv` | same | **Slid 34 cm left (west) during the climb into the west net** (z 0–0.5 m, 6–8 s after arming). The drift correction only learns above 0.4 m; the planner's takeoff leans straight north. PX4 heading reset counter was already 158 (compass fallbacks). Never reached ROUTE, so the wall correction never applied. |
| `20261108-133310_course.csv` | planner, **no localizer**, re-planning off, takeoff lean 0.15 m, compass off | Takeoff slid 24 cm left despite the lean (passed ~4 cm from the west net by odometry), settled in 7 s. Flew the planner path NE, nose along travel; **by odometry 40 cm clear of every wall, then hit the front wall ~1.9 m in** (t≈116 s) → the route did not line up with the real course (start pose ~13° or ~40 cm off). PX4 reset heading 9× in 16 s (vision-gap resets, small with the compass off). |

Fixes since (on `main-localizer` `15a2c71`): takeoff lean 0.30 m in the `tools/flight_day/fly_course.sh` profiles;
localizer `rot_mode yaw`; `fly_course.sh prep` checks cuVSLAM starts at ~0 and the localizer's pose vs the mark
before printing GO.

Columns: see `navless_nav/navless_nav/flight_log.py` (positions in odom: x = north, y = west at bearing 0).
