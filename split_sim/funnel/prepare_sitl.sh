#!/usr/bin/env bash
# Start an unarmed, fresh split simulation using the shared split ports/domain.
set -eo pipefail
cd "$(dirname "$0")"
here=$PWD
source ../split_network.sh
split_ensure_ssh
ssh_run() { "${SSH[@]}" "$TARGET" "$@"; }
[[ -z "$(docker ps -q -f name=drone19-split-sim-laptop -f name=funnel-laptop)" ]] || exit 2
ssh_run 'test -z "$(docker ps -q -f name=drone19-split-sim -f name=drone19-gazebo-bench -f name=funnel-jetson)" && ! ss -uln | grep -q ":8889 "'
ip=$(split_select_xrce_ip)
id=$(date -u +%Y%m%dT%H%M%SZ)
out="$here/runs/$id"
mkdir -p "$here/runs"
mkdir "$out"
case "${1:-hover}" in
  hover) world=funnel.sdf ;;
  traversal) world=funnel_traversal.sdf ;;
  windy-traversal) world=''
    if [[ -n "${2:-}" ]]; then
      python3 wind_fixture.py create "$out" --seed "$2"
    else
      python3 wind_fixture.py create "$out"
    fi ;;
  *) echo 'Expected hover, traversal or windy-traversal' >&2; exit 2 ;;
esac
if [[ -n "$world" ]]; then cp "$world" "$out/funnel.sdf"; fi
cp split_{jetson,laptop}.sh observe.py opening.py "$out/"
mkdir "$out/px4-runs"
remote="team_ws/funnel_sitl/$id"
ssh_run "mkdir -p $remote/output"
COPYFILE_DISABLE=1 tar -C "$out" -cf - split_jetson.sh observe.py opening.py | ssh_run "tar -C $remote -xf -"
# The existing downward laser model reads boxes with world-frame poses.
# Flatten only the two panel collisions. Its own ground-plane test stays enabled.
python3 - "$out" <<'PY'
from xml.etree import ElementTree as ET
from pathlib import Path
import sys
p = Path(sys.argv[1])
world = ET.parse(p/'funnel.sdf').find('world')
root = ET.Element('model', name='funnel_range_geometry')
for name in ('left','right'):
    model = world.find(f"model[@name='{name}']")
    c = model.find('link/collision')
    assert c.find('pose') is None and model.find('link/pose') is None
    c.set('name',name)
    c.append(model.find('pose'))
    root.append(c)
ET.ElementTree(root).write(p/'range-geometry.xml')
PY
"${SSH[@]}" -O forward -o ExitOnForwardFailure=yes -R 127.0.0.1:17457:127.0.0.1:17457 "$TARGET"
ssh_run "docker run -d --rm --name funnel-jetson-$id --runtime nvidia --network host --cpus=4 --memory=3g -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e ROS_DOMAIN_ID=219 -e ROS_LOCALHOST_ONLY=1 -e SKYBETA_UNDERLAY_ONLY=1 -v \$HOME/team_ws/split_sim:/split:ro -v \$HOME/$remote:/funnel:ro -v \$HOME/$remote/output:/run-data --entrypoint bash skybeta:0.3.3 /funnel/split_jetson.sh"
docker run -d --name "funnel-laptop-$id" --init --shm-size=1g \
  -p 127.0.0.1:17457:7447 -e ROS_DOMAIN_ID=219 -e ROS_LOCALHOST_ONLY=1 -e JETSON_IP="$ip" \
  -v "$here/../..:/workspace/Navless:ro" \
  -v "$out/range-geometry.xml:/workspace/Navless/gazebo_sim/models/dh_course/model.sdf:ro" \
  -v "$here/..:/split:ro" -v "$out:/funnel:ro" -v "$out:/run-data" \
  -v "$here/../state/PX4-Autopilot:/root/PX4-Autopilot:ro" \
  -v "$out/px4-runs:/root/px4-runs" --entrypoint bash dronehacks-local:jazzy-harmonic /funnel/split_laptop.sh
echo "$id"
