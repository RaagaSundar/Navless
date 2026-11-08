#!/usr/bin/env bash
# One command to get skybeta-19 ready for a course run. Run on the Jetson HOST (ssh dronehacks@skybeta-19.local).
#
#   bash fly_course.sh quick loc       # FAST (~25 s): NO stack restart. Use when cuVSLAM is healthy and the drone
#                                      # sits on the mark nose north (it checks cuVSLAM yaw within 2 deg of 0)
#   bash fly_course.sh prep loc        # full: restarts the stack first (cuVSLAM lost / yaw off / first run)
#   bash fly_course.sh quick planner   # planner path, no localizer      (also: prep planner)
#   bash fly_course.sh quick file      # Raaga's gated route, nose north, 0.4 m/s, nothing extra
#   bash fly_course.sh quick loc -- prearm_height_tol_m:=1.0   # extra launch settings after --
#   bash fly_course.sh check           # re-run the go/no-go checks only (no restart)
#   bash fly_course.sh stop            # competitor window idle (after landing)
#   bash fly_course.sh logs            # newest flight log path + its last events
#
# YOU place the drone on the mark +-3 cm, nose north +-1 deg, then hands off. The course program zeroes x/y/z
# where the drone sits, so a stack restart is only needed when cuVSLAM's HEADING is off or it lost tracking.
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
START=$(date +%s)

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
  local prof="$1" t0="$2" f="" hzf hzpid
  hzf=$(mktemp); ( chk hz 5 > "$hzf" ) &         # cuVSLAM timing runs alongside the other checks
  hzpid=$!
  say "pre-arm checks (up to 60 s)"
  local passed="" last="" i
  for i in $(seq 1 60); do
    f=$(newest_log)
    if [ -n "$f" ] && [ "$(stat -c %Y "$f")" -ge "$t0" ]; then
      last=$(grep -a "pre-arm" "$f" | tail -1 | cut -d, -f5 | tr -d '"' | cut -c1-150)
      case "$last" in "pre-arm height & zero check passed"*|"pre-arm height check is OFF"*) passed=1; break ;; esac
      case "$last" in *"height check"*"apart"*) [ "$i" -ge 15 ] && break ;; esac   # a steady offset won't fix itself
    fi
    sleep 1
  done
  if [ -n "$passed" ]; then ok "pre-arm passed (PX4 and the camera agree)"
  else
    bad "pre-arm: ${last:-no pre-arm line yet}"
    case "$last" in *"height check"*"apart"*)
      echo "     fix: reboot the Pixhawk with the drone ON THE FLOOR (QGC Reboot Vehicle), wait 45 s, then: bash $0 quick $prof"
      echo "     or override (risk: PX4 may correct its height mid-flight): bash $0 quick $prof -- prearm_height_tol_m:=1.0" ;;
    esac
  fi

  if [ "$prof" = loc ]; then
    say "localizer (up to 30 s to lock on the mark)"
    local s="" st="" x="" y="" yaw="" why="" near=""
    for i in $(seq 1 12); do
      s=$(chk status 3)
      st=$(jv "$s" status); why=$(jv "$s" reason); yaw=$(jv "$s" yaw)
      x=$(python3 -c "import json,sys; d=json.loads(sys.argv[1] or '{}'); p=d.get('position') or [None,None]; print(p[0] if p[0] is not None else '')" "$s" 2>/dev/null)
      y=$(python3 -c "import json,sys; d=json.loads(sys.argv[1] or '{}'); p=d.get('position') or [None,None]; print(p[1] if p[1] is not None else '')" "$s" 2>/dev/null)
      near=""
      if { [ "$st" = tracking ] || [ "$st" = weak ]; } && [ -n "$x" ] && [ -n "$yaw" ]; then
        near=$(python3 -c "import sys,math; x,y,yw=map(float,sys.argv[1:4]); d=math.hypot(x-$MARK_X,y-($MARK_Y+0.10)); e=abs((yw-90+180)%360-180)
print(('ok ' if d<0.25 and e<6 else '')+'%.2f m, %.1f deg' % (d,e))" "$x" "$y" "$yaw")
        case "$near" in ok*) break ;; esac
      fi
      sleep 1
    done
    if [ "$st" = tracking ] || [ "$st" = weak ]; then ok "status $st, camera at ($x, $y), yaw $(printf '%.1f' "${yaw:-0}") (mark ~ ($MARK_X, $MARK_Y+0.1), north = 90)"
    else bad "localizer status '${st:-none}' ${why:+($why)}: it can't correct anything -> 'quick planner'"; fi
    case "$near" in
      ok*) ok "pose vs mark: ${near#ok }" ;;
      "") ;;
      *) bad "localizer pose vs mark: $near (> 25 cm or > 6 deg): misplaced, or the localizer is wrong -> re-place + 'quick loc', or 'quick planner'" ;;
    esac
    local c; c=$(chk corr 3)
    if [ "$(jv "$c" ok)" = True ]; then
      echo "  wall correction it will apply: dx $(jv "$c" dx) m, dy $(jv "$c" dy) m, dyaw $(jv "$c" dyaw_deg) deg"
    else echo "  wall correction: none published yet (stays zero until the localizer is confident)"; fi
  fi

  say "cuVSLAM timing"
  wait "$hzpid" 2>/dev/null
  local h r g
  h=$(cat "$hzf"); rm -f "$hzf"; r=$(jv "$h" rate_hz); g=$(jv "$h" max_gap_s)
  if [ -z "$g" ]; then bad "no cuVSLAM odometry"
  elif python3 -c "import sys; sys.exit(0 if float(sys.argv[1]) < 0.25 else 1)" "$g"; then ok "${r} Hz, worst gap ${g} s"
  else bad "${r} Hz, worst gap ${g} s (> 0.25): Jetson overloaded -> close viewers, or 'quick planner'/'quick file'"; fi

  echo
  if [ ${#NOGO[@]} -eq 0 ]; then
    echo "${G}==== GO ====${N}  pilot: Altitude -> arm -> Position. Take over early. After landing: bash $0 stop"
  else
    echo "${R}==== NO-GO ====${N}"; for m in "${NOGO[@]}"; do echo "  - $m"; done
  fi
  echo "flight log: $(newest_log)   (took $(( $(date +%s) - START )) s)"
}

odom_pair() {   # two odometry samples 1 s apart -> "$o1" "$o2"
  o1=$(chk odom 3); sleep 1; o2=$(chk odom 3)
}

cmd="${1:-}"; shift || true
case "$cmd" in
  prep|quick)
    prof="${1:-loc}"; shift || true
    [ -n "${PROFILE[$prof]:-}" ] || { echo "profile must be loc | planner | file"; exit 1; }
    [ "${1:-}" = "--" ] && shift
    extra="$*"
    for a in $extra; do case "$a" in *:=*) ;; *) echo "${R}'$a' is not a launch setting${N} (name:=value). Usage: bash $0 $cmd $prof [-- name:=value ...]"; exit 1 ;; esac; done
    say "course program idle"; idle; ok "idle"
    if [ "$cmd" = quick ]; then
      docker ps --format '{{.Names}}' | grep -qx "$STACK" || { echo "${R}stack not running${N}: bash $0 prep $prof"; exit 1; }
      odom_pair
      if python3 -c "import json,sys,math
a=json.loads(sys.argv[1]); b=json.loads(sys.argv[2])
moved=math.dist((a['x'],a['y'],a['z']),(b['x'],b['y'],b['z'])); yw=b['yaw_deg']
print('  odometry (%.2f, %.2f, %.2f) yaw %.1f deg, moved %.3f m in 1 s' % (b['x'],b['y'],b['z'],yw,moved))
sys.exit(0 if abs(yw)<2.0 and moved<0.02 else 1)" "$o1" "$o2" 2>/dev/null; then
        ok "cuVSLAM healthy, heading matches the mark: no restart needed"
      else
        echo "${R}cuVSLAM yaw not within 2 deg of 0, moving, or lost${N}: re-place nose north and run quick again, or: bash $0 prep $prof"; exit 1
      fi
    else
      echo "Drone on the mark (+-3 cm, nose north +-1 deg), hands off."
      say "restarting the stack (keep hands off)"
      docker rm -f "$STACK" >/dev/null 2>&1
      for i in $(seq 1 120); do docker ps --format '{{.Names}}' | grep -qx "$STACK" && break; sleep 1; done
      docker ps --format '{{.Names}}' | grep -qx "$STACK" || { echo "${R}stack did not come back${N}"; exit 1; }
      say "waiting for cuVSLAM (up to 2 min)"
      o=""; for i in $(seq 1 60); do o=$(chk odom 2); [ "$(jv "$o" ok)" = True ] && break; sleep 0.5; done
      [ "$(jv "$o" ok)" = True ] || { echo "${R}no cuVSLAM odometry after 2 min${N} (camera? docker exec $STACK tmux capture-pane -p -t skybeta:slam)"; exit 1; }
      sleep 1; odom_pair
      if python3 -c "import json,sys,math
a=json.loads(sys.argv[1]); b=json.loads(sys.argv[2])
far=max(abs(b['x']),abs(b['y']),abs(b['z'])); moved=math.dist((a['x'],a['y'],a['z']),(b['x'],b['y'],b['z']))
print('  odometry (%.3f, %.3f, %.3f) yaw %.1f, moved %.3f m in 1 s' % (b['x'],b['y'],b['z'],b['yaw_deg'],moved))
sys.exit(0 if far<0.10 and abs(b['yaw_deg'])<2.0 and moved<0.02 else 1)" "$o1" "$o2" 2>/dev/null; then
        ok "cuVSLAM started at the origin and holds still"
      else
        echo "${R}cuVSLAM is not steady at ~0${N}: drone touched, or view blank/dark? Fix, then run prep again."; exit 1
      fi
    fi
    say "PX4"
    p=""; for i in $(seq 1 40); do p=$(chk px4 2); [ "$(jv "$p" ok)" = True ] && break; sleep 0.5; done
    [ "$(jv "$p" ok)" = True ] && ok "PX4 linked (heading resets so far: $(jv "$p" heading_resets))" \
      || { echo "${R}PX4 not linked${N}: power-cycle the Pixhawk ON THE FLOOR, wait 45 s, then: bash $0 quick $prof"; exit 1; }
    say "starting the course program: profile $prof ${extra:+(+ $extra)}"
    t0=$(date +%s)
    docker exec "$STACK" tmux respawn-pane -k -t skybeta:competitor \
      "bash -lc 'source skybeta_env.sh; source /root/team_ws/install/setup.bash; ros2 launch navless_nav navless_course_slow.launch.py ${PROFILE[$prof]} $extra'"
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
