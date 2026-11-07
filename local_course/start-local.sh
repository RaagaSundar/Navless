#!/usr/bin/env bash
# Local-only stationary fixture with experimental estimators. Never arms.
set -eo pipefail
cd "$(dirname "$0")"
profile=${VISION_PROFILE:-opencv-measured-stereo}
case "$profile" in opencv-measured-stereo|temporal-flow|baseline-descriptor) ;; *) echo "Unknown VISION_PROFILE: $profile" >&2; exit 2;; esac
case "${1:-}" in
  --fresh) docker compose -f sim-compose.yaml up -d --force-recreate ;;
  '') ;;
  *) echo 'Usage: VISION_PROFILE=opencv-measured-stereo bash start-local.sh [--fresh]' >&2; exit 2;;
esac
bash sim-start.sh
docker exec -i -e VISION_PROFILE="$profile" local-course-sim bash -s <<'INNER'
set -eo pipefail
exec 9>/tmp/course-perception-start.lock
flock 9
source /local_course/env.sh
[[ "$ROS_DOMAIN_ID" == 220 && "$GZ_PARTITION" == local_course ]]
for i in {1..60}; do
  [[ -d /course-state/current && -s /course-state/current/px4.log ]] && break
  sleep 1
done
[[ -s /course-state/current/px4.log ]] || { echo 'SITL startup not ready' >&2; exit 2; }
cd /course-state/current
# The physical spawn and EV calibration share this per-run immutable fixture.
start_bearing=$(python3 -c 'import json; print(json.load(open("fixture.json"))["start_bearing_deg"])')
if [[ -f vision-profile.txt && "$(cat vision-profile.txt)" != "$VISION_PROFILE" ]]; then
  echo 'Profile changes require --fresh; preserve the prior run.' >&2; exit 2
fi
if pgrep -f '^python3 (-u )?/local_course/mission.py' >/dev/null; then
  echo 'Mission process active; refuse perception startup.' >&2; exit 2
fi
if [[ "$VISION_PROFILE" == opencv-measured-stereo ]]; then
  if pgrep -f '/opt/ros/jazzy/bin/ros2 launch /local_course/vision.launch.py' >/dev/null; then
    echo 'RTAB estimator already active; use --fresh for OpenCV.' >&2; exit 2
  fi
  if ! pgrep -f '/opt/ros/jazzy/bin/ros2 launch /local_course/vo.launch.py' >/dev/null; then
    echo "$VISION_PROFILE" > vision-profile.txt
    VO_SIM_FIXTURE_TF=1 nohup ros2 launch /local_course/vo.launch.py > vision.log 2>&1 </dev/null 9>&- &
  fi
  remaps=(-r /local_course/vo/odom:=/local_course/opencv/odom -r /local_course/vo/odom_info:=/local_course/opencv/odom_info)
else
  if pgrep -f '/opt/ros/jazzy/bin/ros2 launch /local_course/vo.launch.py' >/dev/null; then
    echo 'OpenCV estimator already active; use --fresh for RTAB.' >&2; exit 2
  fi
  if ! pgrep -f '/opt/ros/jazzy/bin/ros2 launch /local_course/vision.launch.py' >/dev/null; then
    echo "$VISION_PROFILE" > vision-profile.txt
    nohup bash /local_course/vision.sh > vision.log 2>&1 </dev/null 9>&- &
  fi
  remaps=()
fi
if ! pgrep -f '^python3 (-u )?/local_course/bridge_node.py' >/dev/null; then
  nohup python3 -u /local_course/bridge_node.py --ros-args -p use_sim_time:=true -p calibrated_start:=true -p "start_bearing_deg:=$start_bearing" "${remaps[@]}" > ev-bridge.log 2>&1 </dev/null 9>&- &
fi
INNER
