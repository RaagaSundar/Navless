#!/usr/bin/env bash
# Run on Jetson; collect passive performance evidence without stopping anything.
set -euo pipefail
container="${1:?container}"; label="${2:?evidence label}"; seconds="${3:-35}"
[[ "$container" == drone19-split-sim || "$container" == drone19-gazebo-bench ]] || exit 2
[[ "$label" =~ ^[a-zA-Z0-9_-]+$ && "$seconds" =~ ^[0-9]+$ ]] || exit 2
out="$HOME/team_ws/gazebo_bench_evidence/$label"
mkdir -p "$out"
date -u +%FT%TZ > "$out/start-utc.txt"
docker inspect "$container" --format 'Image={{.Image}} ConfigImage={{.Config.Image}} Devices={{json .HostConfig.Devices}} Mounts={{json .Mounts}}' > "$out/container.txt"
(
  for ((i=0;i<seconds;i+=2)); do
    date -u +%FT%TZ
    docker stats --no-stream --format '{{.Name}} CPU={{.CPUPerc}} MEM={{.MemUsage}}' "$container" skybeta-stack
  done
) > "$out/resources.txt" 2>&1 & resources=$!
timeout "$seconds" tegrastats --interval 1000 > "$out/tegrastats.txt" 2>&1 & tegra=$!
docker exec "$container" bash -lc "export SKYBETA_UNDERLAY_ONLY=1; source /usr/local/bin/skybeta_env.sh; python3 /split/jetson_gazebo/measure.py --seconds $seconds" > "$out/measurement.json"
wait "$resources" || true
wait "$tegra" || true
