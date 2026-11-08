#!/usr/bin/env bash
# One approved attempt on a previously staged, fresh SITL instance.
set -eo pipefail
cd "$(dirname "$0")"
id=$1
[[ "$id" =~ ^[0-9]{8}T[0-9]{6}Z$ && "$2" == --team-approved ]] || exit 2
out="runs/$id"
python3 wind_fixture.py verify "$out"
source ../split_network.sh
grep -q 'from traverse import FunnelTraverse' "$out/pilot/mission.py"
mkdir "$out/flight/approved-attempt"
touch "$out/flight/mission.log"
bash recorder_token.sh "$id" > "$out/flight/token.log" 2>&1 &
refresher=$!
trap 'kill "$refresher" 2>/dev/null || true' EXIT
result=0
"${SSH[@]}" "$TARGET" "docker exec funnel-jetson-$id bash -c 'set -e; source /usr/local/bin/skybeta_env.sh; cd /run-data; exec python3 -u /funnel/pilot/mission.py --approve-flight --run-id $id --recorder-token /funnel/token.json --start-bearing 96 --duration 68 --wall-timeout 300'" > "$out/flight/mission.log" 2>&1 || result=$?
echo "$result" > "$out/flight/finished"
wait "$refresher" || true
exit "$result"
