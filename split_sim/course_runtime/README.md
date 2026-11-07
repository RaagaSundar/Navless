# Split A* companion snapshot

This is a simulation-only snapshot of `local_course/mission.py`, with the
existing team's complete `navless_nav` Python package and unchanged generated
A* route. It executes **on Jetson**, ROS domain219. It does not replace the
NVIDIA tracker, validity gate or arbitrary-heading FRD external-vision bridge.
Mac supplies Gazebo and PX4 SITL. Physical domain19 is excluded.

The snapshot uses domain219, with validated odometry input on
`/mapping/validated_odometry`, isolated fused-navigation output name,
package lookup, and immutable fixture lookup. Mission speed/controller,
estimator checks, source timestamp freshness, command authority, bounded
loss behavior and arm approval remain unchanged. Default navigation is
actual NVIDIA visual odometry; optional PX4 fusion is not enabled implicitly.

The existing split spawn points the nose north (ENU yaw90deg). fixture.json
records that documented fixture, not an observed position fix. The geographic route uses bearing 0. The mission independently registers PX4's
world-fixed control axes against synchronized launch odometry and requires
fresh EV position/yaw aiding, a stationary landed/disarmed interval, and no
frame reset after registration. The original FRD bridge is unchanged. The
recorded failed run registers at about 90 degrees: a forward setpoint is +Y
in the PX4 local frame, while the geographic route remains north. An independently changed launch requires
reviewing this fixture and route transform before use.

After copying this directory into the isolated simulation container:

```bash
export SKYBETA_UNDERLAY_ONLY=1
source /usr/local/bin/skybeta_env.sh
[[ "$ROS_DOMAIN_ID" == 219 ]]
python3 /root/split_state/course_runtime/mission.py --preflight-only \
  --qualified-astar --route /root/split_state/course_runtime/astar-route.json \
  --preflight-observe 30 --wall-timeout 90
```

This command never publishes flight/mode commands. A moving run requires
explicit `--approve-flight`, fresh verified PX4 landed/disarmed/position,
actual NVIDIA tracking and original gate/bridge, a separate native Gazebo
truth/contact recorder plus Jetson PX4 telemetry recorder. It must be the only
mission publisher. Preserve source hashes, all logs and ULogs, including
failed preflights. Merge recorder files offline with `../split_record.py`;
stale/missing telemetry remains a scoring failure. No grade is created merely
from successful process exit. Simulator results are not hardware qualification.

## Post-failure corrections (offline tested, not flown)

The first split flight used the geographic bearing for its PX4 control transform.
Actual preflight PX4 heading was90deg because the original FRD external-vision
frame has an arbitrary heading reference. The current snapshot independently
calibrates control-frame bearing from fresh landed/disarmed PX4 heading plus
VO orientation while preserving geographic route bearing0. It does not change
the vetted bridge or claim geographic heading from stereo.

`integrity.py` additionally latches rejection for source step speed exceeding
the approved1m/s envelope or VO/PX4 vertical disagreement exceeding the existing
0.15m prearm limit for0.2simulation seconds while armed. The companion stops
control publication on rejection; it does not synthesize a replacement fix.
Four pure tests pass. Actual failed-run ULog replay rejects at525.3simulation
seconds, compared afterward with first independent contact529.288. This is
an offline rejection result, not evidence of successful recovery or flight.
The original flown snapshot is retained in that run's `flown-runtime.tgz`.

The next domain219 trial uses the team's checkpoint-interior A* candidate
with .40 m/s cruise, .18 m lead, .12 m/s² lateral acceleration and .015 m
sagitta, from the offline screen in local_course/astar_candidate.README.md.
The course duration limit is 180 s. The first split flight used the previous
route and .30 m/s cruise with .10 m lead.

Following run 202103Z, the staged team A* route raises open-segment waypoints
to .95 m and keeps the window and pad approach at .85 m and .40 m. The
first waypoint sets the team's actual takeoff target. See
local_course/state/astar-nvidia-higher-screen.json for the clearance screen.
