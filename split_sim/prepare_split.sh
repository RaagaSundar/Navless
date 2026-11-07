#!/usr/bin/env bash
# Sourced by the split launcher; the run-specific marker records operator approval.
approve_prepared() {
  local id=$1 run socket manifest world
  [[ "$id" =~ ^[0-9]{8}T[0-9]{6}Z-split-astar-flight$ ]] || return 2
  run="$(cd ../local_course/state/nvidia-runs && pwd)/$id"
  [[ "$(cat /tmp/nvidia-current-run)" == "$run" && -f "$run/prepared-world" && ! -e "$run/settled.log" ]] || return 1
  world="$(readlink state/px4-current)"
  [[ "$(cat "$run/prepared-world")" == "$world" ]] || return 1
  cmp "$run/sources.sha256" course_runtime/sources.sha256 || return 1
  (cd course_runtime && shasum -a256 -c sources.sha256 >/dev/null) || return 1
  source ./split_network.sh
  socket="$SOCKET"
  ssh -S "$socket" -o BatchMode=yes "$TARGET" "docker exec drone19-split-sim pgrep -f '^python3 .*/(mission|mission_m3_capture)\.py .*--prepare-flight .*--run-id $id( |$)'" >/dev/null || return 1
  manifest="$(shasum -a256 "$run/sources.sha256" | awk '{print $1}')"
  printf '%s %s\n' "$id" "$manifest" > "$run/.prepared-approval.tmp" && mv "$run/.prepared-approval.tmp" "$run/prepared-approval"
  echo "Approved prepared run $id; live observation, sensor qualification and recording gates remain active."
}

# These passive probes use actual ROS endpoints, acquisition stamps and PX4 output.
# A failing probe leaves the mission without an activation file.
qualify_prepared() {
  local env_prefix='export SKYBETA_UNDERLAY_ONLY=1; source /usr/local/bin/skybeta_env.sh;'
  local camera_rate
  camera_rate="$(python3 - "$run/simulated-vehicle.sdf" <<'PYRATE'
import sys
import xml.etree.ElementTree as ET
model = ET.parse(sys.argv[1])
rates = [float(model.find(f".//sensor[@name='oak_{side}']/update_rate").text) for side in ('left','right')]
if rates[0] != rates[1] or rates[0] not in (10, 15, 31.25):
    sys.exit('unexpected stereo camera rates in the archived vehicle model')
print(rates[0])
PYRATE
)" || return 1
  "${SSH[@]}" "docker exec -i drone19-split-sim bash -lc '$env_prefix python3 - --camera-hz $camera_rate'" < "$run/check_stereo_rate.py" > "$run/prepared-stereo-rates.json" || return 1
  local attempt mode_ok=0 graph_ok=0
  for attempt in 1 2 3; do
    local mode_log="$run/prepared-estimator-mode-$attempt.log"
    if "${SSH[@]}" "docker exec drone19-split-sim bash -lc '$env_prefix ros2 param get --no-daemon --spin-time 3 /visual_slam_node enable_imu_fusion'" > "$mode_log" 2>&1; then
      if grep -Fxq "Boolean value is: $expected" "$mode_log"; then
        cp "$mode_log" "$run/nvidia-estimator-mode.log"; mode_ok=1; break
      fi
      if grep -Eq '^Boolean value is: (True|False)$' "$mode_log"; then
        echo "Live NVIDIA estimator mode disagrees with selected $profile" >&2; return 1
      fi
    fi
  done
  [[ "$mode_ok" == 1 ]] || return 1
  if [[ "$profile" == stereo-imu ]]; then
    "${SSH[@]}" "docker exec -i drone19-split-sim bash -lc '$env_prefix python3 -'" < "$run/check_imu_stereo.py" > "$run/prepared-source-rates.json" || return 1
    for attempt in 1 2 3; do
      local graph="$run/prepared-imu-graph-$attempt.log"
      "${SSH[@]}" "docker exec drone19-split-sim bash -lc '$env_prefix ros2 topic info --no-daemon --spin-time 3 /split_camera/body_imu --verbose'" > "$graph" 2>&1 || continue
      if python3 - "$graph" <<'PYGRAPH'
import sys
from pathlib import Path
blocks = Path(sys.argv[1]).read_text().split('Node name: ')
for name, role in [('visual_slam_node', 'SUBSCRIPTION'), ('split_ordered_body_imu', 'PUBLISHER')]:
    if not any(b.startswith(name+'\n') and 'Endpoint type: '+role+'\n' in b for b in blocks):
        sys.exit('missing IMU endpoint '+name)
PYGRAPH
      then cp "$graph" "$run/prepared-imu-graph.log"; graph_ok=1; break; fi
    done
    [[ "$graph_ok" == 1 ]] || return 1
  fi
  check_px4_sensors || return 1
  [[ "$(cat "$run/prepared-world")" == "$(readlink state/px4-current)" ]] || return 1
  python3 - "$run/platform.json" <<'PYMETA' || return 1
import json,sys
from pathlib import Path
p=Path(sys.argv[1]); v=json.loads(p.read_text())
v.update(setup_status='ready',verified=True)
tmp=p.with_suffix('.partial'); tmp.write_text(json.dumps(v,indent=2)+'\n'); tmp.replace(p)
PYMETA
  touch "$run/prepared-sources-qualified"
}
