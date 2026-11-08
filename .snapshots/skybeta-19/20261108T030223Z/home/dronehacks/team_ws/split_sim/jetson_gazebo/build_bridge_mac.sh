#!/usr/bin/env bash
# OPTIONAL official C++ bridge. Run only after coordinator releases Mac builds.
# Native sensor adapter is already operational; no compilation is needed for it.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
image=drone19-bridge-dependencies:humble-arm64
container=drone19-bridge-compile
docker build --platform linux/arm64 --progress=plain -f "$root/BridgeBuilder.Dockerfile" -t "$image" "$root"
if docker container inspect "$container" >/dev/null 2>&1; then
  [[ "$(docker inspect "$container" --format '{{index .Config.Labels "drone19.experiment"}}')" == bridge-compile ]] || exit 2
  [[ "$(docker inspect "$container" --format '{{.State.Running}}')" == false ]] || { echo 'Compiler already running.' >&2; exit 3; }
  docker start -a "$container"
else
  docker run --name "$container" --label drone19.experiment=bridge-compile \
    --platform linux/arm64 --memory=2g --memory-swap=2g --cpus=2 --pids-limit=256 \
    --ulimit as=1610612736:1610612736 \
    -v drone19-bridge-build:/opt/jetson_gazebo_ws/build \
    -v drone19-bridge-install:/opt/jetson_gazebo_ws/install \
    --entrypoint /bin/bash "$image" -lc '
      set -eo pipefail
      source /opt/ros/humble/setup.bash
      cd /opt/jetson_gazebo_ws
      GZ_VERSION=harmonic MAKEFLAGS=-j1 colcon build --executor sequential \
        --packages-select ros_gz_interfaces ros_gz_bridge \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
      dpkg-query -W > build-package-versions.txt
      git -C src/ros_gz rev-parse HEAD > source-commit.txt'
fi
# To cancel safely: docker stop -t 5 drone19-bridge-compile (this container only).
