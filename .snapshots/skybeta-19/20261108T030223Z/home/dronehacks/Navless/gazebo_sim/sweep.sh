#!/usr/bin/env bash
# Run several full course flights (fresh sim each time) and print one summary line per run.
#   ./sweep.sh "TAG|ENV=VAL ENV2=VAL|--fly-course-args" ...
# Example:
#   ./sweep.sh "base||" "fast|PX4_PARAM_MPC_XY_P=1.2|--v-max 3 --a-lat 5"
SIM_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SIM_DIR"
for spec in "$@"; do
  IFS='|' read -r tag envs args <<< "$spec"
  echo "=== $tag  [$envs]  $args"
  env $envs bash ./run_sim.sh > /dev/null
  sleep 2
  env $envs timeout 150 python3 fly_course.py --tag "$tag" $args > "/tmp/dh_fly${DH_INSTANCE:-0}.log" 2>&1
  run=$(ls -td runs/*-"$tag"/ | head -1)
  python3 - "$run" <<'EOF'
import json, sys
s = json.load(open(sys.argv[1] + "/summary.json"))
cp = s["checkpoints_s"]
print(f'  {s["result"]:<12s} [{s.get("interface")}/{s.get("px4_profile")}] touchdown {s["touchdown_time_s"]} s (plan {s["planned_time_s"]}), pad offset {s["touchdown_offset_from_pad_cm"]} cm, '
      f'min clearance {s["min_clearance_cm"]} cm ({s["min_clearance_at"]}), err mean/p95/max {s["tracking_error_cm"]["mean"]}/{s["tracking_error_cm"]["p95"]}/{s["tracking_error_cm"]["max"]} cm, '
      f'window offset {cp.get("window_offset_along_cm")} cm, tilt {s["max_tilt_deg"]}, vmax {s["max_speed_mps"]}')
EOF
done
bash ./stop_sim.sh
