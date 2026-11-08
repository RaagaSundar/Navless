#!/usr/bin/env bash
# One command to get skybeta-19 ready for a course run. Run on the Jetson HOST (ssh dronehacks@skybeta-19.local).
#
#   bash fly_course.sh prep loc        # localizer + wall correction, planner path, nose along travel
#   bash fly_course.sh prep planner    # planner path, no localizer
#   bash fly_course.sh prep file       # SAFEST: Raaga's gated route, nose north, 0.4 m/s, nothing extra
#   bash fly_course.sh prep loc -- drift_seed_body_y_m:=-0.25   # extra launch settings after --
#   bash fly_course.sh check           # re-run the go/no-go checks only (no restart)
#   bash fly_course.sh stop            # competitor window idle (after landing)
#   bash fly_course.sh logs            # newest flight log path + its last events
#
# YOU do the physical steps first: (1) power-cycle the Pixhawk, (2) place the drone on the mark +-3 cm,
# nose north +-1 deg, then hands off. `prep` then: idles the course program, restarts the stack, waits for
# cuVSLAM and checks it reads ~0 and holds still, waits for PX4, starts the course program, waits for the
# pre-arm checks, checks the localizer (loc) and cuVSLAM's timing, and prints GO / NO-GO with the reason.
# Never arms anything: the pilot does (Altitude -> arm -> Position).
set -uo pipefail
STACK=skybeta-stack
REPO_C=/root/team_ws/src/Navless
LOGS="$HOME/team_ws/flight_logs"
ENV='export SKYBETA_UNDERLAY_ONLY=1; source skybeta_env.sh >/dev/null 2>&1'
LEAN='drift_seed_body_y_m:=-0.30'
declare -A PROFILE=(
  [loc]="replan_on_obstacles:=false $LEAN"
  [planner]="localizer:=false wall_correction:=false replan_on_obstacles:=false $LEAN"
  [file]="route_source:=file heading_mode:=hold cruise_speed_mps:=0.4 localizer:=false wall_correction:=false stereo_cloud:=false $LEAN"
)
MARK_X=-2.51; MARK_Y=0.91         # launch mark, course metres
G=$'\e[32m'; R=$'\e[31m'; Y=$'\e[33m'; N=$'\e[0m'
say() { echo "${Y}==${N} $*"; }
ok() { echo "  ${G}OK${N}  $*"; }
bad() { echo "  ${R}NO${N}  $*"; NOGO+=("$*"); }
NOGO=()

[ "$(hostname)" = skybeta-19 ] || { echo "not skybeta-19 ($(hostname)): stop"; exit 1; }

chk() {   # chk WHAT [arg] -> one JSON line from fly_check.py inside the stack
  docker exec "$STACK" bash -lc "$ENV; python3 $REPO_C/tools/flight_day/fly_check.py $*" 2>/dev/null | tail -1
}
jv() {    # jv JSON KEY -> value ('' if missing)
  python3 -c 'import json,sys
try: d=json.loads(sys.argv[1])
except Exception: d={}
v=d
for k in sys.argv[2].split("."):
    v=v.get(k) if isinstance(v,dict) else None
print("" if v is None else v)' "$1" "$2"
}
idle() { docker exec "$STACK" tmux respawn-pane -k -t skybeta:competitor "sleep infinity" 2>/dev/null; }
newest_log() { ls -t "$LOGS"/*_course.csv 2>/dev/null | head -1; }

checks() {   # checks PROFILE START_EPOCH
  local prof="$1" t0="$2" f=""
  say "pre-arm checks (waiting up to 90 s)"
  local passed="" last="" i
  for i in $(seq 1 45); do
    f=$(newest_log)
    if [ -n "$f" ] && [ "$(stat -c %Y "$f")" -ge "$t0" ]; then
      last=$(grep -a "pre-arm" "$f" | tail -1 | cut -d, -f5 | tr -d '"' | cut -c1-150)
      case "$last" in "pre-arm height & zero check passed"*) passed=1; break ;; esac
    fi
    sleep 2
  done
  if [ -n "$passed" ]; then ok "pre-arm passed (PX4 and the camera agree)"
  else bad "pre-arm: ${last:-no pre-arm line yet} (heading: PX4 vs camera; height: cuVSLAM lost? -> redo prep)"; fi

  if [ "$prof" = loc ]; then
    say "localizer"
    local s st x y yaw why
    s=$(chk status 8)
    st=$(jv "$s" status); why=$(jv "$s" reason)
    x=$(python3 -c "import json,sys; d=json.loads(sys.argv[1] or '{}'); p=d.get('position') or [None,None]; print(p[0] if p[0] is not None else '')" "$s" 2>/dev/null)
    y=$(python3 -c "import json,sys; d=json.loads(sys.argv[1] or '{}'); p=d.get('position') or [None,None]; print(p[1] if p[1] is not None else '')" "$s" 2>/dev/null)
    yaw=$(jv "$s" yaw)
    if [ "$st" = tracking ] || [ "$st" = weak ]; then ok "status $st, camera at ($x, $y), yaw $(printf '%.1f' "${yaw:-0}") (mark ~ ($MARK_X, $MARK_Y+0.1), north = 90)"
    else bad "localizer status '${st:-none}' ${why:+($why)}: it can't correct anything -> use 'prep planner' or 'prep file'"; fi
    if [ -n "$x" ] && [ -n "$yaw" ]; then
      python3 -c "import sys,math; x,y,yw=map(float,sys.argv[1:4]); d=math.hypot(x-$MARK_X,y-($MARK_Y+0.10)); e=abs((yw-90+180)%360-180)
print('  pose vs mark: %.2f m, %.1f deg' % (d,e)); sys.exit(0 if d<0.25 and e<6 else 1)" "$x" "$y" "$yaw" \
        || bad "localizer pose disagrees with the mark (> 25 cm or > 6 deg): drone misplaced, or the localizer is wrong -> check, re-place, redo prep"
    fi
    local c; c=$(chk corr 6)
    if [ "$(jv "$c" ok)" = True ]; then
      echo "  wall correction it will apply: dx $(jv "$c" dx) m, dy $(jv "$c" dy) m, dyaw $(jv "$c" dyaw_deg) deg"
    else echo "  wall correction: none published yet (stays zero until the localizer is confident)"; fi
  fi

  say "cuVSLAM timing (12 s)"
  local h r g
  h=$(chk hz 12); r=$(jv "$h" rate_hz); g=$(jv "$h" max_gap_s)
  if [ -z "$g" ]; then bad "no cuVSLAM odometry"
  elif python3 -c "import sys; sys.exit(0 if float(sys.argv[1]) < 0.25 else 1)" "$g"; then ok "${r} Hz, worst gap ${g} s"
  else bad "${r} Hz, worst gap ${g} s (> 0.25): Jetson overloaded -> close viewers, or 'prep planner'/'prep file'"; fi

  echo
  if [ ${#NOGO[@]} -eq 0 ]; then
    echo "${G}==== GO ====${N}  pilot: Altitude -> arm -> Position. Take over early. After landing: bash $0 stop"
  else
    echo "${R}==== NO-GO ====${N}"; for m in "${NOGO[@]}"; do echo "  - $m"; done
  fi
  echo "flight log: $(newest_log)"
}

cmd="${1:-}"; shift || true
case "$cmd" in
  prep)
    prof="${1:-loc}"; shift || true
    [ -n "${PROFILE[$prof]:-}" ] || { echo "profile must be loc | planner | file"; exit 1; }
    [ "${1:-}" = "--" ] && shift
    extra="$*"
    echo "Before this: Pixhawk power-cycled, drone on the mark (+-3 cm, nose north +-1 deg), hands off."
    read -r -p "Ready? [Enter] " _ || true
    say "course program idle"; idle; ok "idle"
    say "restarting the stack (keep hands off)"
    docker rm -f "$STACK" >/dev/null 2>&1
    for i in $(seq 1 60); do docker ps --format '{{.Names}}' | grep -qx "$STACK" && break; sleep 2; done
    docker ps --format '{{.Names}}' | grep -qx "$STACK" || { echo "${R}stack did not come back${N}"; exit 1; }
    say "waiting for cuVSLAM (up to 2 min)"
    o=""; for i in $(seq 1 40); do o=$(chk odom 3); [ "$(jv "$o" ok)" = True ] && break; sleep 2; done
    [ "$(jv "$o" ok)" = True ] || { echo "${R}no cuVSLAM odometry after 2 min${N} (camera? stack logs: docker exec $STACK tmux capture-pane -p -t skybeta:slam)"; exit 1; }
    sleep 4; o1=$(chk odom 4); sleep 5; o2=$(chk odom 4)
    if python3 -c "import json,sys,math
a=json.loads(sys.argv[1]); b=json.loads(sys.argv[2])
far=max(abs(b['x']),abs(b['y']),abs(b['z'])); moved=math.dist((a['x'],a['y'],a['z']),(b['x'],b['y'],b['z']))
print('  odometry (%.3f, %.3f, %.3f), moved %.3f m in 5 s' % (b['x'],b['y'],b['z'],moved))
sys.exit(0 if far<0.05 and moved<0.02 else 1)" "$o1" "$o2" 2>/dev/null; then ok "cuVSLAM started at the origin and holds still"
    else
      echo "${R}cuVSLAM is not steady at ~0${N}: was the drone touched, or is the view blank/dark? Fix, then run prep again."; exit 1
    fi
    say "waiting for PX4 (up to 60 s)"
    p=""; for i in $(seq 1 20); do p=$(chk px4 3); [ "$(jv "$p" ok)" = True ] && break; sleep 1; done
    [ "$(jv "$p" ok)" = True ] && ok "PX4 linked (heading resets so far: $(jv "$p" heading_resets))" \
      || { echo "${R}PX4 not linked${N}: power-cycle the Pixhawk (unplug QGC/USB), then run prep again."; exit 1; }
    say "starting the course program: profile $prof ${extra:+(+ $extra)}"
    t0=$(date +%s)
    docker exec "$STACK" tmux respawn-pane -k -t skybeta:competitor \
      "bash -lc 'source skybeta_env.sh; source /root/team_ws/install/setup.bash; ros2 launch navless_nav navless_course_slow.launch.py ${PROFILE[$prof]} $extra'"
    [ "$prof" = loc ] && { say "giving the localizer 40 s on the mark"; sleep 40; }
    checks "$prof" "$t0"
    ;;
  check)
    prof="loc"; docker exec "$STACK" bash -c "pgrep -af cloud_localize >/dev/null" || prof="planner"
    checks "$prof" 0
    ;;
  stop) idle && echo "course program idle" ;;
  logs)
    f=$(newest_log); echo "$f"; grep -av ",tick," "$f" | cut -d, -f3-5 | cut -c1-160 | tail -15 ;;
  *) sed -n '2,17p' "$0"; exit 1 ;;
esac
