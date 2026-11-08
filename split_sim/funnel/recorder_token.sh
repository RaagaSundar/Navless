#!/usr/bin/env bash
# Transfer only actual independent recorder timestamps to the admission check.
set -eu
cd "$(dirname "$0")"
id=$1
[[ "$id" =~ ^[0-9]{8}T[0-9]{6}Z$ ]] || exit 2
source ../split_network.sh
remote="team_ws/funnel_sitl/$id"
out="runs/$id"
while [[ ! -f "$out/flight/finished" ]] && ! grep -q 'single arm request' "$out/flight/mission.log"; do
  "${SSH[@]}" "$TARGET" "tail -n 3000 $remote/output/telemetry.jsonl | grep -E '\"kind\": \"(land|status)\"' | tail -n 20" > "$out/flight/startup-states.jsonl"
  if python3 ../check_startup_recording.py "$id" "$out/truth.json" "$out/flight/startup-states.jsonl" > "$out/flight/token.json" 2>/dev/null; then
    "${SSH[@]}" "$TARGET" "cat > $remote/token.tmp && mv $remote/token.tmp $remote/token.json" < "$out/flight/token.json"
  fi
  sleep .15
done
