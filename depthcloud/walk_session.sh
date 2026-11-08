#!/usr/bin/env bash
# Walk session: carry the drone (PROPS OFF, disarmed) through the course and watch / record where the localizer
# thinks it is, live. Run on skybeta-19's Jetson HOST. Only if walking the drone on the course is allowed.
#
#   bash walk_session.sh start         start stereo_cloud + cloud_localize + walk_recorder into a new session folder
#   bash walk_session.sh checkpoints   (optional, 2nd terminal) walk_test: Enter at known spots = ground truth
#   bash walk_session.sh status        what's running, the live localizer status, last recorder row
#   bash walk_session.sh stop          stop exactly what `start` started, confirm it's gone, write the summary
#   bash walk_session.sh summary [dir] (re)write summary.txt + plan.svg for a session (default: the newest)
#
# Live view while walking (laptop browser, same network):  http://<drone-ip>:8082  (best estimate on the course model,
# trail, the live point cloud: green = matched the model)  and  http://<drone-ip>:8081  (camera + depth).
#
# Before `start`: drone on the launch mark, nose north, STILL; restart the stack first (docker rm -f skybeta-stack,
# wait ~60 s) so cuVSLAM's origin is the mark. Then walk slowly (~0.3 m/s), nose along the direction of travel,
# camera unobstructed (no hand in front of it), roughly along the route.
# Never run this while the drone is being flown or prepped for a flight: it uses Jetson CPU.
set -uo pipefail
STACK=skybeta-stack
HERE_HOST="$(cd "$(dirname "$0")" && pwd)"
DIR_C=/root/team_ws/src/dronehacks                     # scripts + course files, as the container sees them
SESS_HOST="$HOME/team_ws/walk_sessions"
SESS_C=/root/team_ws/walk_sessions
ENV='export SKYBETA_UNDERLAY_ONLY=1; source skybeta_env.sh >/dev/null 2>&1'
[ "$(hostname)" = skybeta-19 ] || { echo "not skybeta-19 ($(hostname)): stop"; exit 1; }

newest() { ls -1dt "$SESS_HOST"/*/ 2>/dev/null | head -1 | sed 's#/$##'; }
alive() { docker exec "$STACK" kill -0 "$1" 2>/dev/null; }

start_one() {   # name, command (run in the container); records the pid in the session
  local name="$1" cmd="$2"
  docker exec -d "$STACK" bash -lc "$ENV; echo \$\$ > $SC/$name.pid; exec $cmd > $SC/$name.log 2>&1"
}

case "${1:-}" in
  start)
    if [ -n "$(newest)" ] && [ ! -f "$(newest)/stopped" ] && ls "$(newest)"/*.pid >/dev/null 2>&1; then
      echo "a session is still running: $(newest). Run: bash $0 stop"; exit 1
    fi
    for f in stereo_cloud.py cloud_localize.py walk_recorder.py dronehacks-course-asbuilt.obj course_asbuilt.json; do
      [ -f "$HOME/team_ws/src/dronehacks/$f" ] || { echo "missing ~/team_ws/src/dronehacks/$f"; exit 1; }
    done
    if docker exec "$STACK" pgrep -f "cloud_localize.py|stereo_cloud.py" >/dev/null; then
      echo "stereo_cloud / cloud_localize already running (not from this script?):"
      docker exec "$STACK" pgrep -af "cloud_localize.py|stereo_cloud.py" | cut -c1-120; exit 1
    fi
    name="$(date +%Y%m%d-%H%M%S)"
    mkdir -p "$SESS_HOST/$name/frames"
    SC="$SESS_C/$name"
    start_one stereo   "python3 -u $DIR_C/stereo_cloud.py --ros-args -p rate_hz:=3.0"
    start_one localize "python3 -u $DIR_C/cloud_localize.py --ros-args -p obj:=$DIR_C/dronehacks-course-asbuilt.obj -p dump_dir:=$SC/frames -p dump_every:=5"
    start_one recorder "python3 -u $DIR_C/walk_recorder.py --ros-args -p out:=$SC/track.csv"
    echo "session $SESS_HOST/$name started; waiting for the localizer to load the course (~10 s)..."
    sleep 15
    for n in stereo localize recorder; do
      p=$(cat "$SESS_HOST/$name/$n.pid" 2>/dev/null); alive "$p" && echo "  $n: running (pid $p)" || { echo "  $n: NOT running:"; tail -5 "$SESS_HOST/$name/$n.log"; }
    done
    ip=$(ip -4 -br addr show wlP1p1s0 2>/dev/null | awk '{print $3}' | cut -d/ -f1)
    echo "live view: http://${ip:-<drone-ip>}:8082   camera: http://${ip:-<drone-ip>}:8081"
    echo "keep the drone still on the mark until the view shows it there, then walk. Stop with: bash $0 stop"
    ;;
  checkpoints)
    d=$(newest); [ -n "$d" ] || { echo "no session"; exit 1; }
    docker exec -it "$STACK" bash -lc "$ENV; python3 $DIR_C/walk_test.py --course asbuilt --out $SESS_C/$(basename "$d")/checkpoints"
    ;;
  status)
    d=$(newest); [ -n "$d" ] || { echo "no session"; exit 1; }
    echo "$d $( [ -f "$d/stopped" ] && echo '(stopped)' )"
    for n in stereo localize recorder; do p=$(cat "$d/$n.pid" 2>/dev/null); alive "$p" && echo "  $n running" || echo "  $n not running"; done
    tail -1 "$d/track.csv" 2>/dev/null | cut -c1-200
    ;;
  stop)
    d=$(newest); [ -n "$d" ] || { echo "no session"; exit 1; }
    for n in recorder localize stereo; do                # recorder first, with SIGINT so it flushes
      p=$(cat "$d/$n.pid" 2>/dev/null) || continue
      docker exec "$STACK" kill -INT "$p" 2>/dev/null
    done
    sleep 3
    for n in recorder localize stereo; do
      p=$(cat "$d/$n.pid" 2>/dev/null) || continue
      alive "$p" && docker exec "$STACK" kill -TERM "$p" 2>/dev/null
      sleep 1; alive "$p" && docker exec "$STACK" kill -KILL "$p" 2>/dev/null
      alive "$p" && echo "  $n (pid $p) STILL RUNNING: check by hand" || echo "  $n stopped"
    done
    left=$(docker exec "$STACK" pgrep -af "walk_recorder.py|$SESS_C/$(basename "$d")" | grep -v pgrep)
    [ -z "$left" ] && echo "  nothing of this session left running" || { echo "  LEFT RUNNING:"; echo "$left"; }
    touch "$d/stopped"
    bash "$0" summary "$d"
    ;;
  summary)
    d="${2:-$(newest)}"; [ -n "$d" ] || { echo "no session"; exit 1; }
    docker exec "$STACK" python3 "$DIR_C/walk_summary.py" "$SESS_C/$(basename "$d")" "$DIR_C/course_asbuilt.json"
    echo "copy to a laptop:  scp -r dronehacks@<drone>:team_ws/walk_sessions/$(basename "$d") ."
    ;;
  *)
    sed -n '2,17p' "$0"; exit 1 ;;
esac
