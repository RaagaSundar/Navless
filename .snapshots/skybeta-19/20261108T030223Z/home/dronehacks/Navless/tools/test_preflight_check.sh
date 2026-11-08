#!/usr/bin/env bash
# Off-drone test of preflight_check.sh with stub docker/systemctl:  bash tools/test_preflight_check.sh
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
T="$(mktemp -d)"
trap 'rm -rf "$T"' EXIT
mkdir -p "$T/bin" "$T/home/team_ws/src"
ln -s "$(dirname "$HERE")" "$T/home/team_ws/src/Navless"

cat > "$T/bin/systemctl" <<'EOF'
#!/usr/bin/env bash
[[ "${STUB_SLAM:-off}" == on ]]
EOF
cat > "$T/bin/docker" <<'EOF'
#!/usr/bin/env bash
if [[ "$1" == ps ]]; then echo skybeta-stack; exit 0; fi
cmd="${*: -1}"
case "$cmd" in
  *route.REPO_ROOT*) echo "${STUB_ROOT:-/root/team_ws/src/Navless}" ;;
  *"test -f"*) exit 0 ;;
  *"ps -eo args"*) printf '%b\n' "${STUB_PROCS:-python3 course_mission}" ;;
  *"topic info"*) echo "Publisher count: ${STUB_PUBS:-1}" ;;
  *vo_state*) echo "${STUB_VO:-1}" ;;
  *"topic hz"*) echo "average rate: 30.1" ;;
  *skybeta/state*) echo '{"px4_link": true, "armed": false}' ;;
esac
EOF
chmod +x "$T/bin/"*

run() {   # run NAME expected_fails [VAR=value ...]
  local name="$1" want="$2"; shift 2
  printf 'COMPETITOR_PKG=navless_nav\nCOMPETITOR_LAUNCH=%s\n' "${LAUNCH:-navless_course_slow.launch.py}" > "$T/drone.env"
  out=$(env HOME="$T/home" PATH="$T/bin:$PATH" DRONE_ENV="$T/drone.env" "$@" bash "$HERE/preflight_check.sh")
  got=$?
  if [[ "$got" == "$want" ]]; then echo "ok    $name ($got FAIL)"; else echo "FAIL  $name: $got FAIL, want $want"; echo "$out"; BAD=1; fi
}
BAD=0
run "healthy drone" 0
run "handheld-slam on, mapping mode" 2 STUB_SLAM=on STUB_PROCS='python3 course_mission\npython3 map_server.py'
run "route lookup broken (no realpath fix)" 1 STUB_ROOT=/root/team_ws/build
run "planner running beside navless_nav" 2 STUB_PROCS='python3 course_mission\npython3 -m planner.px4_bridge_node' STUB_PUBS=2
run "competitor window is sleep infinity" 1 STUB_PROCS='sleep infinity'
run "VIO lost" 1 STUB_VO=2
exit "$BAD"
