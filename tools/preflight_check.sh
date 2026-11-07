#!/usr/bin/env bash
# Pre-flight check for skybeta-11. Read-only: it changes nothing, and prints the fix for each FAIL.
#
#   ssh dronehacks@skybeta-11.local
#   bash ~/team_ws/src/Navless/tools/preflight_check.sh
#
# Run it on the Jetson host (not inside the container), on the ground, disarmed, with the stack up
# and the drone sitting on the launch mark. Exit code = number of FAILs.
set -u
STACK="${SKYBETA_CONTAINER:-skybeta-stack}"
ENV_FILE="${DRONE_ENV:-/etc/skybeta/drone.env}"
REPO_HOST="$HOME/team_ws/src/Navless"
REPO_CTR="/root/team_ws/src/Navless"
FAILS=0 WARNS=0

pass() { printf '  PASS  %s\n' "$1"; }
warn() { printf '  WARN  %s\n' "$1"; WARNS=$((WARNS + 1)); }
fail() { printf '  FAIL  %s\n        fix: %s\n' "$1" "$2"; FAILS=$((FAILS + 1)); }
# Inside the stack container, with the stack's environment and the team overlay sourced.
ctr() {
  docker exec "$STACK" bash -lc "source skybeta_env.sh >/dev/null 2>&1; \
    [ -f /root/team_ws/install/setup.bash ] && source /root/team_ws/install/setup.bash; $1" 2>/dev/null
}

echo "== host"
if systemctl is-enabled --quiet handheld-slam.service 2>/dev/null || systemctl is-active --quiet handheld-slam.service 2>/dev/null; then
  fail "handheld-slam.service is enabled or running: it replaces the competitor window with 'sleep infinity'" \
       "sudo systemctl disable --now handheld-slam.service, then restart the stack"
else
  pass "handheld-slam.service is off"
fi
if [[ -r "$ENV_FILE" ]]; then
  pkg=$(sed -n 's/^COMPETITOR_PKG=//p' "$ENV_FILE" | tail -1)
  launch=$(sed -n 's/^COMPETITOR_LAUNCH=//p' "$ENV_FILE" | tail -1)
  [[ "$pkg" == navless_nav ]] && pass "COMPETITOR_PKG=navless_nav" \
    || fail "COMPETITOR_PKG is '$pkg'" "set COMPETITOR_PKG=navless_nav in $ENV_FILE, then: docker rm -f $STACK"
  case "$launch" in
    navless_course_slow.launch.py) pass "COMPETITOR_LAUNCH=navless_course_slow.launch.py (first-flight speeds)" ;;
    navless_course.launch.py) warn "COMPETITOR_LAUNCH=navless_course.launch.py: the fast defaults (0.7 m/s). First flights: navless_course_slow.launch.py, placement within 0.5 deg here" ;;
    *) fail "COMPETITOR_LAUNCH is '$launch'" "set COMPETITOR_LAUNCH=navless_course_slow.launch.py in $ENV_FILE, then: docker rm -f $STACK" ;;
  esac
else
  fail "can't read $ENV_FILE" "check the path (DRONE_ENV=...) or run with sudo"
fi
if [[ -e "$REPO_HOST/.git" ]]; then          # a directory, or a file in a worktree/submodule
  rev=$(git -C "$REPO_HOST" log --oneline -1 2>/dev/null)
  dirty=$(git -C "$REPO_HOST" status --porcelain --untracked-files=no 2>/dev/null | head -3)
  pass "team repo at $REPO_HOST: $rev"
  [[ -z "$dirty" ]] || warn "the team repo has uncommitted changes (the drone flies them): $(echo $dirty)"
else
  fail "no Navless clone at $REPO_HOST" "git clone the team repo into ~/team_ws/src/Navless and rebuild the team workspace"
fi

echo "== stack container ($STACK)"
if ! docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$STACK"; then
  fail "container $STACK is not running" "start the stack (SKYBETA_DETACH=1 host/run.sh stack in ~/skybeta)"
  echo "$FAILS FAIL, $WARNS WARN"; exit "$FAILS"
fi
root=$(ctr "python3 -c 'from navless_nav import route; print(route.REPO_ROOT)'")
if [[ "$root" == "$REPO_CTR" ]]; then
  pass "navless_nav finds the repo ($root)"
else
  fail "navless_nav REPO_ROOT is '${root:-<import failed>}': the route won't load and the drone only hovers" \
       "rebuild the team workspace from main (route realpath fix, PR #19) and source its overlay"
fi
# Both course launches fly navless_course.launch.py's route (the slow one includes it).
route=$(sed -n "s/.*('route_file', '\([^']*\)'.*/\1/p" "$REPO_HOST/navless_nav/launch/navless_course.launch.py" 2>/dev/null | head -1)
if [[ -n "$route" ]] && ctr "test -f $REPO_CTR/$route"; then
  pass "route file $route is there"
else
  fail "route file '${route:-?}' not found under $REPO_CTR" "pull main into the team repo"
fi
procs=$(ctr "ps -eo args")
if grep -q "px4_bridge_node" <<<"$procs" || grep -q "mission/mission.py" <<<"$procs"; then
  fail "a second commander is running (planner px4_bridge_node or mission/mission.py)" \
       "stop it: navless_nav must be the only node on /skybeta/setpoint/*"
else
  pass "no second commander (planner, mission.py) running"
fi
grep -q "course_mission" <<<"$procs" && pass "navless_nav course_mission is running" \
  || fail "course_mission isn't running (competitor window empty or 'sleep infinity'?)" \
          "docker exec $STACK tmux capture-pane -p -t skybeta:competitor | tail; check drone.env and handheld-slam"
grep -q "map_server.py" <<<"$procs" && fail "map_server.py is running (~50% CPU; mapping mode)" \
  "disable handheld-slam.service and restart the stack" || pass "no map_server.py"
grep -q "stereo_cloud.py" <<<"$procs" && warn "stereo_cloud.py is running: use rate_hz 3 or stop it for flights (CPU; Koji's gate drops samples > 250 ms)" \
  || pass "stereo_cloud.py is off"
grep -q "mirror_relay" <<<"$procs" && warn "mirror_relay is running: on shared Wi-Fi bind it to the USB address (bind:=192.168.55.1) or turn it off"
pubs=$(ctr "ros2 topic info /skybeta/setpoint/pose" | sed -n 's/^Publisher count: //p')
[[ "${pubs:-0}" -le 1 ]] && pass "/skybeta/setpoint/pose publishers: ${pubs:-0}" \
  || fail "/skybeta/setpoint/pose has $pubs publishers: two commanders" "find and stop the extra node (ros2 topic info -v)"

echo "== live state (drone on the launch mark)"
vo=$(ctr "timeout 5 ros2 topic echo --once /visual_slam/status --field vo_state" | head -1)
[[ "$vo" == 1 ]] && pass "cuVSLAM tracking (vo_state 1)" \
  || fail "cuVSLAM vo_state is '${vo:-no message}'" "check the camera and the slam window; don't arm without tracking"
hz=$(ctr "timeout 12 ros2 topic hz /visual_slam/tracking/odometry" | sed -n 's/average rate: //p' | tail -1)
awk -v h="${hz:-0}" 'BEGIN { exit !(h >= 20) }' && pass "odometry at ${hz} Hz" \
  || fail "odometry at '${hz:-0}' Hz (want ~30)" "check cuVSLAM and CPU load"
state=$(ctr "timeout 4 ros2 topic echo --once /skybeta/state --field data" | head -1)
grep -q '"px4_link": true' <<<"$state" && pass "flight_interface has the PX4 link" \
  || fail "flight_interface reports no PX4 link" "check the Pixhawk power and cable"
grep -q '"armed": false' <<<"$state" && pass "disarmed" || warn "the vehicle reports armed"

echo
echo "$FAILS FAIL, $WARNS WARN"
cat <<'EOF'
Before arming: drone on the launch mark within 1 deg of north (0.5 deg on the fast launch), area
clear. The competitor log must show "pre-arm height check passed" (if it keeps warning, power-cycle
the Pixhawk). Pilot: select Altitude, arm, then switch to Position. Arming already in Position gives
no autonomy. To take over, change the flight mode: PX4 leaves Offboard and navless_nav pauses.
EOF
exit "$FAILS"
