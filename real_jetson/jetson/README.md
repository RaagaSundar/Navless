# Skybeta 19 course payload

Copy the contents of this directory as one unit to the team's own `~/team_ws/real19/` on the assigned host when the team schedules deployment. The existing physical container mounts that location at `/root/team_ws/real19`. The host entry point is `bash run.sh`. The code runs from source with the organiser’s ROS 2 Humble underlay. The directory contains a colcon ignore marker so an unrelated workspace build cannot pick it up.

## Runtime contract

The aircraft supplies the organiser’s running flight interface, the real camera and cuVSLAM, validated odometry and gate status, the flight controller, and matching ROS messages. The passive checker expects exactly one interface, both setpoint subscriptions, one vision-health adapter and zero competing setpoint publishers. If the team has the validated odometry and gate status but needs the health adapter, `bash run.sh health` runs the supplied adapter in the foreground. The independent odometry gate and PX4 fusion checks must both be functioning.

Use the organisers’ current rules, competitor guide and hardware checklist. Confirm physical domain 19, the launch mark, measured nose bearing and origin height. This profile assumes the odometry origin was established on the mark with nose north, origin 0.07 m above the floor, and the organiser’s normal unanchored pose interface. The controller consumes continuous odometry in `odom → base_link`. Its existing floor-height correction calibrates the PX4–odometry Z offset on the ground. The bundled route climbs, crosses the window toward the primary pad and ends on the pad. It holds launch heading, uses a cruise setpoint speed of 0.4 m/s and can use an existing `/tube/offset` for side ToF centering. There is no return leg.

## Entry points for the team's scheduled rehearsal

1. On the Mac, or later inside the staged directory, `python3 verify.py` checks the packaged file hashes. `python3 plan.py` prints the window route from the included map and planner. It requires NumPy. Flight reads the reviewed `window-route.json` directly.
2. On the Jetson host with props removed and the controller slot available, `bash run.sh check` runs the passive 30-second input check. It requires five continuous passing seconds at the end. If the FC link or required estimator data is absent, read the reason before proceeding.
3. With one reviewed mission, team approval, the usual safety pilot on RC and exactly one organiser flight interface, `bash run.sh course` starts the competitor in the foreground. Wait for READY while landed and disarmed. The pilot arms and hands over using the organiser procedure. The node requires agreement from PX4 and the interface and sends only a stationary twist while it waits. It sends ENU poses and twists through the same competitor endpoints used by the recorded hover controller. An early takeover, stale required stream or estimator fault ends publication. The pilot retains takeover and recovery responsibility. A stopped stream can leave the interface holding its last pose.

First rehearse with props off. A later low tethered test needs its own approved plan and recorder. Use a new named archive and run comparison for each physical attempt. The staged software has only local software checks.
