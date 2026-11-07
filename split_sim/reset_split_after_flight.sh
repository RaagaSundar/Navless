#!/usr/bin/env bash
# Isolated domain219 simulator reset following a recorder-confirmed safe stop.
set -euo pipefail
cd "$(dirname "$0")"
source ./split_network.sh
split_ensure_ssh
run="$(cat /tmp/nvidia-current-run)"
[[ "$(docker compose exec -T laptop printenv ROS_DOMAIN_ID)" == 219 ]]
"${SSH[@]}" -o BatchMode=yes "$TARGET" 'set -e
docker inspect -f "{{range .Config.Env}}{{println .}}{{end}}" drone19-split-sim | grep -qx ROS_DOMAIN_ID=219
if test "$(docker inspect -f "{{.State.Running}}" drone19-split-sim)" = true; then
  ! docker exec drone19-split-sim pgrep -f "^python3 .*course_runtime/mission.py"
fi'
python3 - "$run/settled.log" state/split-station.log <<'PY'
import json,sys,time
from pathlib import Path
settled=json.loads(Path(sys.argv[1]).read_text())
assert settled['landed_disarmed'] and settled['to_sim']-settled['from_sim']>=2
lines=Path(sys.argv[2]).read_bytes().splitlines()
observed={}
for line in reversed(lines):
    try: row=json.loads(line)
    except ValueError: continue
    msg=row.get('message',{})
    if row.get('system')!=1 or row.get('component')!=1: continue
    kind=msg.get('mavpackettype')
    if kind in ('HEARTBEAT','EXTENDED_SYS_STATE') and kind not in observed:
        assert 0 <= time.time()-row['wall_time'] < 4, 'stale station telemetry'
        observed[kind]=msg
    if len(observed)==2: break
assert not observed['HEARTBEAT']['base_mode'] & 128 and observed['EXTENDED_SYS_STATE']['landed_state']==1
PY
if [[ -d "$run/mac-world-logs" ]]; then
  bench="evidence/stationary-world-$(date -u +%Y%m%dT%H%M%SZ)"
  mkdir -p "$bench"
  cp -R state/logs "$bench/"
  cp state/split-station.log "$bench/station.log"
else
  cp -R state/logs "$run/mac-world-logs"
fi
"${SSH[@]}" -o BatchMode=yes "$TARGET" 'docker stop -t 5 drone19-split-sim'
docker compose up -d --force-recreate laptop
for i in {1..30}; do docker compose exec -T laptop bash -c 'xdpyinfo >/dev/null 2>&1' && break; sleep 1; done
docker compose exec -T laptop bash -c 'xdpyinfo >/dev/null 2>&1'
bash start.sh --split
for i in {1..35}; do docker compose exec -T laptop bash -c 'gz topic -l | grep -qx /model/dh_quad_0/odometry' && break; sleep .5; done
bash start_px4_from_mac.sh
# start_px4_from_mac.sh also starts the sole heartbeat-only station.
