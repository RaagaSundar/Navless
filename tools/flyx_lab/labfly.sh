#!/usr/bin/env bash
# usage: labfly.sh <tag> ["<extra px4_bridge_node args>"]
# Resets the world (skip with SKIP_RESET=1 if you just reset it yourself), starts tools/sim_truth_recorder.py
# (/tmp/est_<tag>.csv), runs planner.px4_bridge_node from the lab clone (/tmp/bridge_<tag>.log), streams the
# log here, then stops the recorder and saves the CSV to ${OUT_DIR:-$TMPDIR}/est_<tag>.csv.
# Needs: gstack browse in headed mode, logged in to the FlyX lab Workbench, holding editing control.
# The bridge ARMS AND FLIES by itself (--approve-flight): get the team's OK before running it.
D=$(cd "$(dirname "$0")" && pwd)
B="$HOME/.claude/skills/gstack/browse/dist/browse"; tag=$1; extra=$2
LABDIR=${LABDIR:-/var/lib/flyx-lab/workspace/navless}; OUT=${OUT_DIR:-${TMPDIR:-${TEMP:-/tmp}}}
lab_state() { "$B" --headed js "document.body.innerText.match(/\n(RESETTING|READY|RUNNING|STARTING)\n/)?.[1]||'?'" 2>&1 | tail -1; }

"$B" --headed frame main >/dev/null 2>&1
seen=1
if [ -z "$SKIP_RESET" ]; then
  r=$("$B" --headed snapshot -i -c 2>&1 | grep '"Reset world"' | grep -o '@e[0-9]*' | tail -1)
  "$B" --headed click "$r" >/dev/null 2>&1; seen=0
fi
for i in $(seq 1 60); do
  s=$(lab_state); [ "$s" = RESETTING ] && seen=1; [ "$s" = READY ] && [ $seen = 1 ] && break; sleep 3
done
echo "world: $s"; [ "$s" = READY ] || { echo "world not READY, not flying"; exit 1; }

"$D/labrun.sh" "pkill -INT -f _recorder.py; (nohup python3 $LABDIR/tools/sim_truth_recorder.py > /tmp/est_$tag.csv 2> /tmp/est_$tag.err &); sleep 6; echo; tail -1 /tmp/est_$tag.csv | cut -c1-200" 30 | tail -1
"$B" --headed frame main >/dev/null 2>&1; "$B" --headed frame 'iframe[src*="/code/"]' >/dev/null 2>&1
"$B" --headed js "(()=>{const x=[...document.querySelectorAll('.xterm')].find(e=>e.offsetWidth>0);x.querySelector('textarea').focus();return 1})()" >/dev/null 2>&1
"$B" --headed type "cd $LABDIR && python3 -u -m planner.px4_bridge_node --sim-time --approve-flight --launch-origin -2.515 1.114 0.126 --prior gazebo_sim/models/dh_course/meshes/course_visual.obj $extra 2>&1 | tee /tmp/bridge_$tag.log" >/dev/null
"$B" --headed press Enter >/dev/null

prev=""
for i in $(seq 1 420); do
  out=$("$B" --headed js "new Promise(r=>setTimeout(r,1000)).then(()=>fetch('/code/vscode-remote-resource?path=/tmp/bridge_$tag.log',{cache:'no-store'})).then(r=>r.ok?r.text():'')" 2>&1 | grep -v -e UNTRUSTED -e "rcl\]")
  new=$(diff <(echo "$prev") <(echo "$out") | grep '^>' | sed 's/^> //' | sed -E 's/\[(INFO|WARN|ERROR)\] \[[0-9.]+\] \[ego_px4_planner_bridge\]: //')
  [ -n "$new" ] && echo "[t+${i}s] $new"; prev="$out"
  echo "$out" | grep -qE "\[DONE\]|\[ABORT\]|Traceback|Ctrl-C|Mission complete|Emergency land complete" && break
done

sleep 8; "$D/labrun.sh" "pkill -INT -f _recorder.py; echo; echo recorder_stopped" 20 | tail -1
"$B" --headed js "fetch('/code/vscode-remote-resource?path=/tmp/est_$tag.csv',{cache:'no-store'}).then(r=>r.text())" 2>&1 | grep -v UNTRUSTED > "$OUT/est_$tag.csv"
echo "saved $OUT/est_$tag.csv ($(wc -l < "$OUT/est_$tag.csv") rows)  ->  python3 $D/est_summary.py $OUT/est_$tag.csv"
