#!/usr/bin/env bash
# Mac orchestrator. Companion/controller remains on Jetson, Gazebo/PX4 on Mac.
# The approved run includes continuous passive observation before control.
set -euo pipefail
cd "$(dirname "$0")"
source ./prepare_split.sh
if [[ "${1:-}" == --approve-prepared && $# == 2 ]]; then approve_prepared "$2"; exit; fi
[[ $# == 1 && ( "$1" == --approve-flight || "$1" == --prepare-flight ) ]] || { echo 'Usage: run_split_course.sh --prepare-flight | --approve-prepared RUN_ID | --approve-flight'; exit 2; }
launch_mode="$1"
# Kernel-owned lock is released on exit, including interruption.
if [[ "${SPLIT_FLIGHT_LOCK_HELD:-}" != 1 ]]; then
  exec python3 -c 'import fcntl,os,sys
lock=os.open("/tmp/drone19-domain219-flight.lock",os.O_CREAT|os.O_RDWR,0o600)
try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
except BlockingIOError: raise SystemExit("another split flight launcher is active")
os.set_inheritable(lock,True)
os.environ["SPLIT_FLIGHT_LOCK_HELD"]="1"
os.execvp("bash",["bash",*sys.argv[1:]])' "$PWD/run_split_course.sh" "$@"
fi
source ./split_network.sh
split_ensure_ssh
SSH=("${SSH[@]}" -o BatchMode=yes "$TARGET")
mkdir -p ../local_course/state/nvidia-runs
container=drone19-split-sim-laptop-1
[[ "$(docker inspect -f '{{.State.Running}}' "$container")" == true ]]
[[ "$(docker exec "$container" printenv ROS_DOMAIN_ID)" == 219 ]]
if "${SSH[@]}" 'docker exec drone19-split-sim pgrep -f "^python3 .*course_runtime/mission.py"'; then
  echo "existing domain219 companion mission detected" >&2
  exit 1
fi
"${SSH[@]}" 'set -e; test "$(docker inspect -f "{{.State.Running}}" drone19-gazebo-bench)" = false; test "$(docker exec drone19-split-sim printenv ROS_DOMAIN_ID)" = 219'
runid="$(date -u +%Y%m%dT%H%M%SZ)-split-astar-flight"
run="$(cd ../local_course/state/nvidia-runs && pwd)/$runid"
mkdir "$run"
mkdir "$run/flight"
printf '%s\n' "$run" > /tmp/nvidia-current-run
cp course_runtime/astar-route.json "$run/flight/planned-route.json"
cp course_runtime/sources.sha256 "$run/sources.sha256"
height_profile="$(docker exec "$container" printenv SPLIT_HEIGHT_PROFILE 2>/dev/null || echo range)"
case "$height_profile" in range|ev-terrain) ;; *) echo 'unreviewed height configuration' >&2; exit 2;; esac
printf '%s\n' "$height_profile" > "$run/height-profile"
cp state/assets/dh_quad_stereo.sdf "$run/simulated-vehicle.sdf"
if [[ -f state/render-profile.json ]]; then
  cp state/render-profile.json "$run/renderer-source.json"
  docker exec "$container" pgrep -af '^gz sim -s -r ' > "$run/renderer-process.log"
  chosen_world="$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["world"])' "$run/renderer-source.json")"
  grep -Fq -- "$chosen_world" "$run/renderer-process.log"
  if [[ "$chosen_world" == /split-appearance/worlds/dronehacks-synthetic.sdf ]]; then
    docker cp "$container:/split-appearance/appearance.json" "$run/appearance.json"
  fi
fi
mkdir "$run/pre-ekf-source"
cp viewer/{odom_gate.py,source_admission.py} "$run/pre-ekf-source/"
"${SSH[@]}" 'cd ~/team_ws/split_sim/viewer && sha256sum odom_gate.py source_admission.py' > "$run/pre-ekf-source/deployed.sha256"
(cd "$run/pre-ekf-source" && shasum -a256 -c deployed.sha256) > "$run/pre-ekf-source/verified.log"
profile="$("${SSH[@]}" 'cat ~/team_ws/split_sim_state/estimator-profile 2>/dev/null || echo stereo')"
case "$profile" in
  stereo-imu) expected=True ;;
  stereo) expected=False ;;
  *) echo "unrecognized Jetson estimator profile: $profile" >&2; exit 2 ;;
esac
if [[ "$launch_mode" == --approve-flight ]]; then
"${SSH[@]}" "docker exec drone19-split-sim bash -lc 'export SKYBETA_UNDERLAY_ONLY=1; source /usr/local/bin/skybeta_env.sh; ros2 param get /visual_slam_node enable_imu_fusion'" > "$run/nvidia-estimator-mode.log"
grep -Fxq "Boolean value is: $expected" "$run/nvidia-estimator-mode.log"
else
  readlink state/px4-current > "$run/prepared-world"
  cp prepare_split.sh check_stereo_rate.py check_imu_stereo.py "$run/"
  echo "Preparing $runid; authorize this exact run with --approve-prepared $runid"
fi
python3 - "$run" "$profile" "$launch_mode" <<'PY'
import json,sys
from pathlib import Path
p=Path(sys.argv[1]); estimator='NVIDIA cuVSLAM stereo + 250Hz body IMU GPU' if sys.argv[2]=='stereo-imu' else 'NVIDIA cuVSLAM stereo-only GPU'
(p/'platform.json').write_text(json.dumps(dict(renderer_host='Mac',px4_host='Mac',estimator_host='Jetson',onboard_host='Jetson',estimator=estimator,setup_status='preparing' if sys.argv[3]=='--prepare-flight' else 'ready',verified=sys.argv[3]!='--prepare-flight'),indent=2)+'\n')
(p/'settings.json').write_text(json.dumps(dict(label='NVIDIA SIM',estimator=estimator,height_profile=(p/'height-profile').read_text().strip(),ros_domain=219,gazebo_partition='drone19_split',route=json.loads((p/'flight/planned-route.json').read_text())['route_type'],start_bearing_deg=0,takeoff_height_m=json.loads((p/'flight/planned-route.json').read_text())['waypoints_enu'][0][2],cruise_m_s=.40,max_lead_m=.18,takeoff_vertical_lead_m=None,max_lat_accel_m_s2=.12,max_sagitta_m=.015,yaw_policy='first-exterior',yaw_rate_deg_s=10,navigation_source='PX4 fused local position from NVIDIA stereo VO, body IMU and range',hardware_qualification=False),indent=2)+'\n')
PY
# Freeze exactly the reviewed manifest, excluding later staged planner work.
python3 - "$run/sources.sha256" > "$run/archive-files.txt" <<'PYFILES'
from pathlib import Path
import sys
for line in Path(sys.argv[1]).read_text().splitlines():
    _, name = line.split('  ', 1)
    member = Path(name)
    if member.is_absolute() or '..' in member.parts:
        raise SystemExit('invalid runtime archive path')
    print('course_runtime/' + name)
print('course_runtime/sources.sha256')
PYFILES
(cd course_runtime && shasum -a256 -c sources.sha256) > "$run/local-verified-sources.log"
COPYFILE_DISABLE=1 tar -czf /tmp/nvidia-course-runtime.tgz -T "$run/archive-files.txt" 
cp /tmp/nvidia-course-runtime.tgz "$run/flown-runtime.tgz"
cp capture_flight_sensors.py seal_sensor_capture.py "$run/"
cp run_split_course.sh prepare_split.sh check_startup_recording.py split_record.py wait_settled.py "$run/"
scp -o ControlPath="$SOCKET" -o BatchMode=yes /tmp/nvidia-course-runtime.tgz "$TARGET":team_ws/split_sim_state/course-runtime.tgz
"${SSH[@]}" 'set -e; tar -xzf ~/team_ws/split_sim_state/course-runtime.tgz -C ~/team_ws/split_sim_state/course_stage; docker cp ~/team_ws/split_sim_state/course_stage/course_runtime drone19-split-sim:/root/split_state/; docker exec drone19-split-sim bash -lc "cd /root/split_state/course_runtime; sha256sum -c sources.sha256 && python3 -m unittest discover -v"' > "$run/deployed-sources.log" 2>&1
scp -o ControlPath="$SOCKET" -o BatchMode=yes split_record.py "$TARGET":team_ws/split_sim_state/split_record.py
"${SSH[@]}" 'docker cp ~/team_ws/split_sim_state/split_record.py drone19-split-sim:/root/split_state/split_record.py'
check_px4_sensors() {
docker exec "$container" bash -c 'set -e; cd /root/px4-current; b=/root/PX4-Autopilot/build/px4_sitl_default/bin; for topic in distance_sensor vehicle_imu estimator_aid_src_rng_hgt estimator_status_flags; do "$b/px4-listener" "$topic" -n 1; done' > "$run/px4-sensor-preflight.log" || return 1
docker exec "$container" bash -c 'cd /root/px4-current; b=/root/PX4-Autopilot/build/px4_sitl_default/bin; for key in EKF2_HGT_REF EKF2_EV_CTRL EKF2_RNG_CTRL EKF2_BARO_CTRL; do echo "$key"; "$b/px4-param" show -q "$key"; done' > "$run/px4-height-parameters.log"
python3 - "$run/px4-sensor-preflight.log" "$height_profile" "$run/px4-height-parameters.log" <<'PYRANGE'
from pathlib import Path
import re,sys
record = Path(sys.argv[1]).read_text()
if sys.argv[2]=='ev-terrain':
    numbers={k:int(v) for k,v in re.findall(r'(EKF2_[A-Z_]+)\n\s*(\d+)\s*(?:\n|$)',Path(sys.argv[3]).read_text())}
    expected={'EKF2_HGT_REF':3,'EKF2_EV_CTRL':11,'EKF2_RNG_CTRL':1,'EKF2_BARO_CTRL':1}
    if numbers!=expected or any(field not in record for field in ('cs_ev_hgt: True','cs_baro_hgt: True','cs_rng_terrain: True')):
        sys.exit('actual fixed EV height, baro and conditional range/terrain profile not confirmed')

for topic in ('distance_sensor', 'vehicle_imu', 'estimator_aid_src_rng_hgt'):
    data = record.split('TOPIC: '+topic+'\n',1)[1].split('TOPIC:',1)[0]
    age = re.search(r'timestamp: \d+ \(([0-9.]+) seconds ago\)', data)
    if not age or float(age[1]) > 0.5:
        sys.exit('missing or stale '+topic)
if any(field not in record for field in ('fused: True','cs_rng_fault: False','cs_rng_stuck: False','cs_rng_hgt: True')):
    sys.exit('PX4 range height not qualified')
PYRANGE
}
if [[ "$launch_mode" == --approve-flight ]]; then check_px4_sensors; fi
sensor_path="/root/$runid-camera-imu.sqlite"
sensor_stop="/root/$runid-camera-imu.stop"
docker exec "$container" python3 /split/capture_flight_sensors.py "$sensor_path" --stop-file "$sensor_stop" > "$run/native-camera-imu.log" 2>&1 & sensor_pid=$!
for i in {1..50}; do
  grep -q '^subscribed to original calibrated stereo and body IMU' "$run/native-camera-imu.log" && break
  kill -0 "$sensor_pid" || { cat "$run/native-camera-imu.log" >&2; exit 1; }
  sleep .2
done
grep -q '^subscribed to original calibrated stereo and body IMU' "$run/native-camera-imu.log"
# The following passive observers start before the companion can request a mode.
docker exec "$container" python3 /split/split_record.py truth --output "/root/$runid-truth.json" --wall-timeout 900 > "$run/truth-recorder.log" 2>&1 & truth_pid=$!
"${SSH[@]}" "docker exec drone19-split-sim bash -lc 'export SKYBETA_UNDERLAY_ONLY=1; source /usr/local/bin/skybeta_env.sh; python3 /root/split_state/split_record.py telemetry --output /root/split_state/$runid-telemetry.jsonl --wall-timeout 900'" > "$run/telemetry-recorder.log" 2>&1 & telemetry_pid=$!
# Publish measured track snapshots to the viewer while this flight is running.
live_track() {
  while :; do
    if cp "state/$runid-truth.json" "$run/flight/live-truth.json" 2>/dev/null &&
       "${SSH[@]}" "cat ~/team_ws/split_sim_state/$runid-telemetry.jsonl" > "$run/flight/live-telemetry.jsonl" &&
       python3 split_record.py merge --truth "$run/flight/live-truth.json" --telemetry "$run/flight/live-telemetry.jsonl" --output "$run/flight/.live-track.json"; then
      mv "$run/flight/.live-track.json" "$run/flight/track.json"
    fi
    sleep 5
  done
}
live_track > "$run/live-viewer.log" 2>&1 & live_pid=$!
cleanup() {
  docker exec "$container" touch "$sensor_stop" || true
  wait "$sensor_pid" || true
  kill "$live_pid" 2>/dev/null || true
  wait "$live_pid" 2>/dev/null || true
  if [[ -n "${recorder_gate_pid:-}" ]]; then kill "$recorder_gate_pid" 2>/dev/null || true; fi
  docker exec "$container" pkill -TERM -f "split_record.py truth --output /root/$runid-truth.json" || true
  "${SSH[@]}" "docker exec drone19-split-sim pkill -TERM -f 'split_record.py telemetry --output /root/split_state/$runid-telemetry.jsonl'" || true
  wait "$truth_pid" || true;wait "$telemetry_pid" || true
}
trap cleanup EXIT
# Start one passive-to-active node alongside the recorders.
token_path="/root/split_state/$runid-recorder-ready.json"
"${SSH[@]}" "docker exec drone19-split-sim test ! -e '$token_path'"
refresh_recorder() {
  if [[ "$launch_mode" == --prepare-flight ]]; then
    if ! qualify_prepared; then
      echo 'Prepared sensor qualification failed; no activation sent.' >&2
      return 1
    fi
    while [[ ! -s "$run/prepared-approval" ]]; do sleep .2; done
    [[ "$(cat "$run/prepared-world")" == "$(readlink state/px4-current)" ]] || return 1
    "${SSH[@]}" "cd ~/team_ws/split_sim_state && cat > $runid-activation.tmp && mv $runid-activation.tmp $runid-activation" < "$run/prepared-approval" || return 1
  fi
  while ! grep -q 'continuous startup observation complete' "$run/flight/mission.log" 2>/dev/null; do sleep .25; done
  while ! grep -q 'single arm request' "$run/flight/mission.log"; do
    kill -0 "$truth_pid" "$telemetry_pid" || return 1
    if [[ "$launch_mode" == --prepare-flight ]]; then
      [[ "$(cat "$run/prepared-world")" == "$(readlink state/px4-current)" ]] || return 1
    fi
    if cp "state/$runid-truth.json" "$run/flight/startup-truth.json" &&
       "${SSH[@]}" "cat ~/team_ws/split_sim_state/$runid-telemetry.jsonl" > "$run/flight/startup-telemetry.jsonl" &&
       python3 check_startup_recording.py "$runid" "$run/flight/startup-truth.json" "$run/flight/startup-telemetry.jsonl" > "$run/recorder-token.json"; then
      "${SSH[@]}" "cd ~/team_ws/split_sim_state && cat > $runid-recorder-ready.tmp && mv $runid-recorder-ready.tmp $runid-recorder-ready.json" < "$run/recorder-token.json" || return 1
    fi
    sleep .15
  done
}
refresh_recorder > "$run/recorder-readiness.log" 2>&1 & recorder_gate_pid=$!
result=0
"${SSH[@]}" "docker exec drone19-split-sim bash -lc 'export SKYBETA_UNDERLAY_ONLY=1; source /usr/local/bin/skybeta_env.sh; python3 /root/split_state/course_runtime/mission.py $launch_mode --px4-localization --height-profile $height_profile --qualified-astar --yaw-policy first-exterior --route /root/split_state/course_runtime/astar-route.json --preflight-observe 30 --recorder-token $token_path --run-id $runid --duration 180 --wall-timeout 600'" > "$run/flight/mission.log" 2>&1 || result=$?
kill "$recorder_gate_pid" 2>/dev/null || true
wait "$recorder_gate_pid" 2>/dev/null || true
sed -n '1,/requesting Offboard/p' "$run/flight/mission.log" > "$run/preflight.log"
# Remain a passive observer through touchdown and normal PX4 disarm.
# A timeout records an incomplete outcome; it never authorizes a reset.
"${SSH[@]}" "docker exec -i drone19-split-sim python3 - /root/split_state/$runid-telemetry.jsonl" < "$run/wait_settled.py" > "$run/settled.log" || result=1
cleanup;trap - EXIT
docker cp "$container:/root/$runid-truth.json" "$run/flight/truth.json"
"${SSH[@]}" "cat ~/team_ws/split_sim_state/$runid-telemetry.jsonl" > "$run/flight/telemetry.jsonl"
python3 split_record.py merge --truth "$run/flight/truth.json" --telemetry "$run/flight/telemetry.jsonl" --output "$run/flight/track.json"
sensor_archive_ok=0
python3 "$run/seal_sensor_capture.py" "$run" "state/$runid-camera-imu.sqlite" || sensor_archive_ok=$?
# The independent grade remains available even when a separate archive failed.
if [[ "$sensor_archive_ok" != 0 ]]; then result=1; fi
# Actual independent grader, using unchanged local course geometry.
docker cp "$run/flight/track.json" "$container:/root/$runid-track.json"
docker exec "$container" python3 /split/course_runtime/score_course.py "/root/$runid-track.json" --repo /workspace/Navless --output "/root/$runid-grade.json" > "$run/grader.log" 2>&1 || true
docker cp "$container:/root/$runid-grade.json" "$run/flight/grade.json" || true
px4path="$(docker exec "$container" readlink -f /root/px4-current)"
docker cp "$container:$px4path" "$run/px4"
cp state/split-station.log "$run/station.log"
"${SSH[@]}" 'docker exec drone19-split-sim tar -czf - -C /root/split_state logs' > "$run/jetson-node-logs.tgz"
python3 course_runtime/review_run.py "$run"
python3 export_run_videos.py --enqueue "$run" || echo 'Unable to start offline video export' >&2
printf 'Evidence: %s\n' "$run"
exit "$result"
