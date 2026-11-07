#!/usr/bin/env bash
# Build and start the desktop only. Opening a scene is a separate command.
set -euo pipefail
cd "$(dirname "$0")"
if ! command -v docker >/dev/null && [[ "$(uname -s)" == Darwin ]]; then
  export PATH="/Applications/Docker.app/Contents/Resources/bin:$PATH"
fi
command -v docker >/dev/null || { echo 'Docker is not installed.' >&2; exit 1; }
if ! docker info >/dev/null 2>&1; then
  if [[ "$(uname -s)" == Darwin ]]; then open -a Docker; fi
  for i in {1..60}; do
    docker info >/dev/null 2>&1 && break
    sleep 1
  done
fi
docker info >/dev/null
repo_source="${NAVLESS_SOURCE_DIR:-..}"
[[ "$repo_source" = /* ]] || repo_source="$PWD/$repo_source"
[[ -f "$repo_source/gazebo_sim/worlds/dronehacks.sdf" ]] || {
  echo "Course source not found at $repo_source" >&2; exit 1;
}
export NAVLESS_SOURCE_DIR="$repo_source"
mkdir -p evidence state/home state/runs "$repo_source/gazebo_sim/runs"
docker compose build 2>&1 | tee "evidence/build-$(date -u +%Y%m%dT%H%M%SZ).log"
docker compose up -d
echo 'Desktop: http://localhost:6080/vnc.html?autoconnect=true&resize=scale'
echo 'Open the paused course: docker compose exec gazebo sim-course'
