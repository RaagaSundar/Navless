#!/usr/bin/env bash
# Fresh, disarmed local SITL only. No aircraft or SSH access.
set -euo pipefail
cd "$(dirname "$0")"
docker compose -f sim-compose.yaml up -d
docker exec local-course-sim bash -lc 'pgrep -f "^bash /local_course/sim-inside.sh$"' >/dev/null && exit 0
docker exec -d local-course-sim bash /local_course/sim-inside.sh
