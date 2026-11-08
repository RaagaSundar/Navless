#!/usr/bin/env bash
# Continue an approved set; run IDs in the batch ledger count toward the cap.
set -euo pipefail
cd "$(dirname "$0")"
first=$1
count=$2
[[ "$first" =~ ^[0-9]{8}T[0-9]{6}Z$ && "$count" =~ ^[1-9][0-9]*$ && "$3" == --team-approved ]] || exit 2
batch="runs/$first/batch"
mkdir "$batch"
id=$first
for ((index=1; index<=count; index++)); do
  if ((index > 1)); then
    prepared=$(bash prepare_sitl.sh windy-traversal)
    id=$(tail -1 <<<"$prepared")
    [[ "$id" =~ ^[0-9]{8}T[0-9]{6}Z$ ]] || exit 1
  fi
  echo "$index $id" | tee -a "$batch/run-ids.txt"
  out="runs/$id"
  if ((index > 1)); then
    bash stage_traversal.sh "$id"
    cp "runs/$first/tests.log" "$out/tests.log"
    # Verify wind coupling and independent recorder presence before starting pilot.
    ready=0
    for i in {1..90}; do
      if [[ -s "$out/truth.json" && -s "$out/wind.jsonl" ]] && grep -qm1 '"wind_enabled": true' "$out/wind.jsonl"; then ready=1; break; fi
      sleep 1
    done
    [[ "$ready" == 1 ]] || { echo "Missing disturbance or recording evidence $id"; exit 1; }
    bash fly_traversal.sh "$id" --team-approved || test -s "$out/flight/finished"
  else
    while [[ ! -s "$out/flight/finished" ]]; do sleep 1; done
  fi
  bash finish_windy.sh "$id" "$index" | tee -a "$batch/outcomes.jsonl"
done
