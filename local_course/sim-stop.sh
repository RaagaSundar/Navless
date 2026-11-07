#!/usr/bin/env bash
# Stops this isolated SITL only; logs and generated assets are preserved.
set -euo pipefail
cd "$(dirname "$0")"
docker compose -f sim-compose.yaml down
