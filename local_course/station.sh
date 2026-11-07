#!/usr/bin/env bash
# Telemetry only; run inside local-course-sim. Stop with TERM to this process.
set -eo pipefail
run=$(readlink -f /course-state/current)
exec python3 -u /local_course/station.py "$@" >> "$run/station.jsonl" 2>&1
