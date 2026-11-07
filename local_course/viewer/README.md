# Local run room

Run `python3 local_course/viewer/server.py` from the repository and open
http://127.0.0.1:8774. Standard-library Python only; loopback only. Stop with Ctrl-C.
This read-only viewer has no ROS, simulator, reset, process-start or flight controls.

The archive discovers every `local_course/state/runs/<id>/` folder on Refresh,
including preflight and incomplete attempts. It reads `flight/track.json`
(`samples` with world ENU `t,x,y,z,armed,landed`), `flight/grade.json`, and
optional `review.json`:

```json
{"outcome":"Course failed: corridor not reached", "worked":["Takeoff observed"], "failed":["Route did not progress"], "next":["Inspect mission and estimator logs"]}
```

Reviews are edited on disk, never through the viewer. Only an explicit independent
grade with `passed: true` appears as passed. A missing track or grade remains
unqualified. Each run uses its archived `flight/planned-route.json` (`waypoints_enu`, optional
`name`, `route_name`, `route_type` or `planner`) when present. Comparison runs
show their own archived plan in dotted purple. Missing legacy plans use the
repository's `route_centreline_window.json` and are explicitly labelled as a
legacy fallback. Invalid archived plans are labelled unavailable and not drawn.
Planned paths are never presented as flight evidence. Map geometry comes from `course_v3.json`.
Actual truth is for offline simulation review only and is never fed to navigation.

Evidence links allow only JSON/JSONL/log/text/YAML/PNG files inside a selected run,
excluding symlinks, secret-like filenames, binaries and files above 16 MiB.
Other repository files are not served. No network hosting or publishing configured.

The prominent run debrief reads optional `incident_timeline` entries with `at`,
`label` and run-local `evidence` filename(s), `cause` with `status` (`verified` or
`hypothesis`), `summary` and `evidence`, and `open_questions`. Only an explicit
`cause.status: "verified"` is labelled verified; strings are unverified hypotheses.
Missing timelines remain explicitly unknown. Passing and incomplete runs receive
corresponding debrief text. `flight/camera/montage.png` appears as recorded stereo
evidence when present; all evidence links must match the safe run-local catalog.

Archive counts distinguish simulator session folders, observed flights and course
passes. A flight requires a recorded sample with `armed: true`, `landed: false`
and finite `z > 0.25 m`; a grade file alone is not flight evidence. Course passes
require both an observed flight and independent `passed: true`. The Flights
filter defaults to observed flights; All sessions retains setup/interrupted
attempts. Track classification is cached in memory by file size and nanosecond
mtime, so unchanged tracks are not reparsed on archive refresh.

The optional OBJ observer overlay uses only `kind: base_pose`, `frame: course`
records from the selected run's `map-observer/stream.jsonl`, clipped to actual
track simulation timestamps and replay time. Camera/status positions are never
used as aircraft poses. Gaps over 0.6 s are not joined or extrapolated. Preceding
status records mark tracking versus weak/odometry-gap segments; all accuracy
remains unqualified. Actual independent truth stays the primary green trajectory.
Run-local `flight/imu-review.json` and `flight/range-review.json` supply concise
recorded sensor summaries with evidence links; no live estimation is performed.
