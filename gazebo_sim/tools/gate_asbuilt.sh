#!/usr/bin/env bash
# Gazebo gate on the as-built course: N laps of a branch's course_mission, then a verdict.
#
#   bash gazebo_sim/tools/gate_asbuilt.sh [LAPS]                     # main, cloned from GitHub
#   BRANCH=drone19/flight SRC=/path/to/local/Navless bash ...        # another branch, cloned offline
#
# Each lap uses the drone estimator profile, the as-built window route and course_mission's default speeds
# (= navless_course_slow's 0.5 / 0.6 / 0.25), with net_margin_m 0.25 (the as-built route crosses M1's and
# M3's north ends 0.256 m from the net) and 5 mm odometry noise. The clone's Gazebo course model is rebuilt
# from the as-built .obj + course_asbuilt.json, so the walls, boulders and the rangefinder's raycast are the
# real course. Needs the Linux Gazebo + PX4 SITL setup run_sim.sh uses. One line per lap, a verdict, then the
# minimum clearance per wall (tools/gate_per_wall.py).
N="${1:-3}"
BRANCH="${BRANCH:-main}"
SRC="${SRC:-https://github.com/RaagaSundar/Navless.git}"
DIR="${DIR:-$HOME/navless-asbuilt-gate}"
OUT="${OUT:-/tmp/dh_gate_asbuilt}"
mkdir -p "$OUT"; : > "$OUT/results.txt"
rm -rf "$DIR" && git -c safe.directory='*' clone -q -b "$BRANCH" "$SRC" "$DIR" || exit 1
OBJ="$DIR/dronehacks-course-asbuilt.obj"
if [[ ! -f "$OBJ" ]]; then                       # branches older than the committed .obj: take main's
  git -C "$DIR" fetch -q origin main && git -C "$DIR" show origin/main:dronehacks-course-asbuilt.obj > "$OBJ" || exit 1
fi
echo "code: $(git -C "$DIR" log -1 --format='%h %s')" | tee -a "$OUT/results.txt"
cd "$DIR/gazebo_sim" || exit 1
python3 tools/build_course_model.py "$OBJ" course/course_asbuilt.json | tee -a "$OUT/results.txt"
ROUTE="$DIR/gazebo_sim/course/route_centreline_window_asbuilt.json"
for lap in $(seq 1 "$N"); do
  DH_INSTANCE=1 DH_PX4_PROFILE=drone ./run_sim.sh > "$OUT/sim_$lap.log" 2>&1
  sleep 3
  DH_PX4_PROFILE=drone timeout 300 python3 fly_navless.py --instance 1 --route "$ROUTE" -p net_margin_m=0.25 \
      --noise 0.005 --tag asb$lap > "$OUT/fly_$lap.log" 2>&1
  run=$(ls -d runs/*navless-asb$lap 2>/dev/null | tail -1)
  if [[ -z "$run" ]]; then
    echo "lap $lap: NO RUN ($(grep -E 'SystemExit|Error|no takeover' "$OUT/fly_$lap.log" | tail -1))" | tee -a "$OUT/results.txt"
    continue
  fi
  python3 - "$run" "$lap" <<'PY' | tee -a "$OUT/results.txt"
import json, sys
s = json.load(open(sys.argv[1] + "/summary.json"))
cp = s.get("checkpoints_s", {})
print(f"lap {sys.argv[2]}: {s['result']:<9} touchdown {s.get('touchdown_time_from_arm_s')} s, pad {s.get('touchdown_offset_from_pad_cm')} cm, "
      f"min clearance {s.get('min_clearance_cm')} cm at {s.get('min_clearance_at')}, contacts {s.get('contacts')}, "
      f"window {cp.get('window_offset_along_cm')} cm, errors {len(s.get('node_errors', []))}")
PY
done
DH_INSTANCE=1 ./stop_sim.sh > /dev/null 2>&1
landed=$(grep -c "LANDED" "$OUT/results.txt")
contacts=$(grep -c "COLLISION" "$OUT/results.txt")
echo "VERDICT: $landed/$N landed, $contacts with contacts" | tee -a "$OUT/results.txt"
python3 tools/gate_per_wall.py "runs/*navless-asb[0-9]*" | tee -a "$OUT/results.txt"
