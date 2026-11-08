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

Columns: see `navless_nav/navless_nav/flight_log.py` (positions in odom: x = north, y = west at bearing 0).
