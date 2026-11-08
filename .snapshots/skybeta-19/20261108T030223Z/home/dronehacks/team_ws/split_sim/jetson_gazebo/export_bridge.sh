#!/usr/bin/env bash
# OPTIONAL: export only after the capped ARM64 Jammy/Humble compile succeeds.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
container=drone19-bridge-compile
[[ "$(docker inspect "$container" --format '{{.State.Running}} {{.State.ExitCode}}')" == 'false 0' ]] || exit 2
image="$(docker inspect "$container" --format '{{.Image}}')"
[[ "$(docker image inspect "$image" --format '{{.Architecture}}')" == arm64 ]] || exit 2
[[ ! -e "$root/bridge_install" ]] || { echo 'Preserve previous bridge_install before exporting again.' >&2; exit 2; }
mkdir "$root/bridge_install"
docker cp "$container:/opt/jetson_gazebo_ws/install/." "$root/bridge_install/"
docker cp "$container:/opt/jetson_gazebo_ws/build-package-versions.txt" "$root/build-package-versions.txt"
docker cp "$container:/opt/jetson_gazebo_ws/source-commit.txt" "$root/source-commit.txt"
docker image inspect "$image" --format '{{.Id}} {{.Architecture}}' > "$root/builder-image.txt"
COPYFILE_DISABLE=1 tar -czf "$root/bridge-install.tar.gz" -C "$root/bridge_install" .
(cd "$root" && shasum -a 256 bridge-install.tar.gz > bridge-install.sha256)
