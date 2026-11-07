#!/usr/bin/env bash
# CPU-only, domain 220, observer outputs only. No PX4/EV publisher.
set -eo pipefail
source /local_course/env.sh
export ROS_DOMAIN_ID=220 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
TEAM=/workspace/Navless
RUN=$(readlink -f /course-state/current)
python3 - "$RUN/fixture.json" <<'CHECK'
import json,sys
assert json.load(open(sys.argv[1]))['start_bearing_deg'] == 90, 'OBJ observer requires reviewed physical east fixture'
CHECK
mkdir -p "$RUN/map-observer"
printf '%s\n' 'Current checkout stereo_cloud + cloud_localize; CPU observer; fixed east launch heading ENU 0deg, geographic bearing90deg; calibrated mount; validated alternative camera odom only; exact-capture-nanosecond cloud join, bounded100, 3wall-second expiry, 100ms delivery settle, original timestamps unchanged; missing odom_frame disables raw TF fallback.' > "$RUN/map-observer/settings.txt"
pids=()
python3 /local_course/map-observer-sync.py > "$RUN/map-observer/sync.log" 2>&1 & pids+=("$!")
cleanup() { for p in "${pids[@]}"; do kill "$p" 2>/dev/null || true; done; wait || true; }
trap cleanup EXIT TERM INT
python3 "$TEAM/depthcloud/stereo_cloud.py" --ros-args -p use_sim_time:=true -p scale:=1.0 -p rate_hz:=3.0 -p require_calibration_tf:=true -p http_port:=8081 > "$RUN/map-observer/stereo.log" 2>&1 & pids+=("$!")
python3 "$TEAM/depthcloud/cloud_localize.py" --ros-args -p use_sim_time:=true -p obj:=/workspace/Navless/gazebo_sim/models/dh_course/meshes/course_visual.obj -p init_x:=-2.5146 -p init_y:=0.9144 -p init_base_z:=0.07 -p init_yaw_deg:=0.0 -p mount_fallback:="0.1053,0.0375,0.0304" -p cloud_topic:=/local_course/map_observer/paired_points -p validated_odom_topic:=/local_course/validated_odometry -p odom_frame:=validated_only_no_raw_tf -p rate_hz:=3.0 -p http_port:=8082 > "$RUN/map-observer/localize.log" 2>&1 & pids+=("$!")
python3 /local_course/map-observer-record.py --output "$RUN/map-observer/stream.jsonl" > "$RUN/map-observer/recorder.log" 2>&1 & pids+=("$!")
wait
