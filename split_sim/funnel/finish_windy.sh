#!/usr/bin/env bash
# Observe a completed attempt through disarm and collect both independent recorders.
set -euo pipefail
cd "$(dirname "$0")"
id=$1
index=$2
[[ "$id" =~ ^[0-9]{8}T[0-9]{6}Z$ && "$index" =~ ^[0-9]+$ ]] || exit 2
out="runs/$id"
source ../split_network.sh
status="$out/flight/final-status.log"
ground=0
for i in {1..75}; do
  docker exec "funnel-laptop-$id" bash -c 'cd /root/px4-current; b=/root/PX4-Autopilot/build/px4_sitl_default/bin; "$b/px4-listener" vehicle_status -n 1; "$b/px4-listener" vehicle_land_detected -n 1' > "$status"
  if grep -q 'arming_state: 1' "$status" && grep -q '    landed: True' "$status"; then ground=1; break; fi
  sleep 2
done
[[ "$ground" == 1 ]] || { echo "Landing not confirmed for $id"; exit 1; }
# Allow the independent DDS recorder to receive the final disarm status.
sleep 2
docker exec "funnel-laptop-$id" pkill -TERM -f '^python3 /run-data/split_record.py truth' || true
docker exec "funnel-laptop-$id" pkill -TERM -f '^python3 /run-data/record_wind.py' || true
"${SSH[@]}" "$TARGET" "docker exec funnel-jetson-$id pkill -TERM -f '^python3 /funnel/split_record.py telemetry' || true"
"${SSH[@]}" "$TARGET" "docker stop -t 5 funnel-jetson-$id" >/dev/null
docker stop -t 5 "funnel-laptop-$id" >/dev/null
"${SSH[@]}" -O cancel -R 127.0.0.1:17457:127.0.0.1:17457 "$TARGET"
mkdir -p "$out/jetson"
"${SSH[@]}" "$TARGET" "tar -C team_ws/funnel_sitl/$id/output -czf - ." | tar -C "$out/jetson" -xzf -
python3 "$out/split_record.py" merge --truth "$out/truth.json" --telemetry "$out/jetson/telemetry.jsonl" --output "$out/flight/track.json"
python3 wind_result.py "$out" "$index"
