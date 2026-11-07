#!/usr/bin/env bash
# Run inside the container. Builds PX4 only; never starts it.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
export PATH="/opt/sim-venv/bin:$PATH"
PX4_DIR="${PX4_DIR:-/root/PX4-Autopilot}"
if [[ ! -d "$PX4_DIR/.git" ]]; then
  git clone --branch v1.16.1 --depth 1 --filter=blob:none --no-checkout \
    https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
fi
git -C "$PX4_DIR" sparse-checkout set --cone boards cmake msg platforms src Tools ROMFS srv test posix-configs
git -C "$PX4_DIR" checkout v1.16.1
python3 -m pip install -r "$PX4_DIR/Tools/setup/requirements.txt" 'numpy<2'
# Keep new submodule checkouts shallow on this download-limited laptop.
# Local Git configuration only; the PX4 checkout and its pinned commits stay intact.
while IFS= read -r key; do
  git -C "$PX4_DIR" config --local "${key%.path}.shallow" true
done < <(git config -f "$PX4_DIR/.gitmodules" --name-only --get-regexp '^submodule\..*\.path$')
# PX4 pulls only submodules needed by this target. Two jobs fit Docker's 8 GB VM.
make -C "$PX4_DIR" px4_sitl_default j=2
git -C "$PX4_DIR" rev-parse HEAD
if [[ "${1:-}" != --with-messages ]]; then
  echo 'PX4 build complete. px4_msgs is optional and was not built.'
  exit 0
fi
mkdir -p /root/ros2_ws/src
if [[ ! -d /root/ros2_ws/src/px4_msgs/.git ]]; then
  git clone --branch release/1.16 --depth 1 \
    https://github.com/PX4/px4_msgs.git /root/ros2_ws/src/px4_msgs
fi
cd /root/ros2_ws
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --packages-select px4_msgs --executor sequential
git -C /root/ros2_ws/src/px4_msgs rev-parse HEAD
