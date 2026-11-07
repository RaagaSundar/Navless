#!/usr/bin/env bash
# Off-drone test of real_flight_start.sh / real_flight_kill.sh with a stub docker:
#   bash planner/test_real_flight_scripts.sh
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
T="$(mktemp -d)"
trap 'rm -rf "$T"' EXIT
mkdir -p "$T/bin"
cat > "$T/bin/docker" <<'EOF'
#!/usr/bin/env bash
# Stub of the stack container. $STUB_DIR/running = the bridge process exists.
echo "$*" >> "$STUB_DIR/calls"
[[ "$1" == ps ]] && { echo skybeta-stack; exit 0; }
shift                                                     # exec
if [[ "$1" == -i ]]; then cat >/dev/null; echo "${STUB_ARMED:-False}"; exit 0; fi   # /skybeta/state check
shift                                                     # container name
case "$1" in
  tmux) [[ "$2" == list-windows ]] && echo competitor || touch "$STUB_DIR/running" ;;
  pgrep|grep) [[ -f "$STUB_DIR/running" ]] ;;
  pkill) [[ "$2" == -KILL || -z "${STUB_STUCK:-}" ]] && rm -f "$STUB_DIR/running"; exit 0 ;;
  *) exit 0 ;;                                            # test -f, tail, bash -c ': > log'
esac
EOF
chmod +x "$T/bin/docker"

BAD=0
check() {   # check NAME want_exit script [args] [VAR=value ...] ; sets $out
  local name="$1" want="$2" script="$3"; shift 3
  local args=() envs=()
  for a in "$@"; do [[ "$a" == *=* ]] && envs+=("$a") || args+=("$a"); done
  out=$(env STUB_DIR="$T" PATH="$T/bin:$PATH" KILL_WAIT_S=2 "${envs[@]}" bash "$HERE/$script" "${args[@]}" 2>&1)
  local got=$?
  if [[ "$got" == "$want" ]]; then echo "ok    $name"; else echo "FAIL  $name: exit $got, want $want"; echo "$out"; BAD=1; fi
}
fresh() { rm -f "$T/running" "$T/calls"; touch "$T/calls"; }
expect_call() { grep -q -- "$1" "$T/calls" || { echo "FAIL  $2: no docker call matching '$1'"; BAD=1; }; }
no_call() { ! grep -q -- "$1" "$T/calls" || { echo "FAIL  $2: unexpected docker call '$1'"; BAD=1; }; }

fresh; check "start: disarmed -> starts" 0 real_flight_start.sh --stop-on-pad
expect_call "respawn-pane -k -t skybeta:competitor" "start"
expect_call "--skybeta-mode --approve-flight --planner ego --prior .*course_visual.obj --stop-on-pad" "start args"
fresh; check "start: armed -> refuses" 1 real_flight_start.sh STUB_ARMED=True
no_call respawn-pane "start while armed"
fresh; check "start: state unreadable -> refuses" 1 real_flight_start.sh STUB_ARMED=unknown
fresh; check "start: state unreadable + FORCE=1 -> starts" 0 real_flight_start.sh STUB_ARMED=unknown FORCE=1
fresh; touch "$T/running"; check "start: already running -> refuses" 1 real_flight_start.sh
no_call respawn-pane "start while running"

fresh; check "kill: nothing running" 0 real_flight_kill.sh
fresh; touch "$T/running"; check "kill: controlled stop" 0 real_flight_kill.sh
expect_call "pkill -INT" "kill"; no_call "pkill -KILL" "kill"
fresh; touch "$T/running"; check "kill: still landing after timeout" 1 real_flight_kill.sh STUB_STUCK=1
no_call "pkill -KILL" "kill timeout must not hard-kill a landing drone"
fresh; touch "$T/running"; check "kill --now: INT, INT, then KILL" 0 real_flight_kill.sh --now STUB_STUCK=1
[[ $(grep -c "pkill -INT" "$T/calls") == 2 ]] || { echo "FAIL  kill --now: want 2 SIGINTs"; BAD=1; }
expect_call "pkill -KILL" "kill --now"

[[ $BAD == 0 ]] && echo "all passed"
exit $BAD
