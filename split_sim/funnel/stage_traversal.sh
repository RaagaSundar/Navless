#!/usr/bin/env bash
# Use after prepare_sitl.sh traversal. Starts passive recorders and a telemetry station.
set -eo pipefail
cd "$(dirname "$0")"
id=$1
[[ "$id" =~ ^[0-9]{8}T[0-9]{6}Z$ ]] || exit 2
out="runs/$id"
python3 wind_fixture.py verify "$out"
source ../split_network.sh
mkdir "$out/pilot" "$out/flight"
cp ../course_runtime/{mission,frames,startup,integrity,launch_datum,px4_navigation,split_frame}.py "$out/pilot/"
cp {hover,observe,opening,passage,traverse}.py "$out/pilot/"
(cd "$out/pilot" && patch -s mission.py < ../../../hover.patch)
python3 - "$out/pilot/mission.py" <<'PY'
from pathlib import Path
import sys
p=Path(sys.argv[1])
s=p.read_text().replace('from hover import FunnelHover','from traverse import FunnelTraverse').replace('mission = FunnelHover(', 'mission = FunnelTraverse(')
s=s.replace("route endpoint verified on fused navigation: requesting PX4 autonomous landing", "traversal controller requested PX4 autonomous landing; see passage phase record")
p.write_text(s)
PY
if [[ ! -f "$out/wind-fixture.json" ]]; then cp dh_quad_stereo.sdf "$out/simulated-quad.sdf"; fi
cp wind_fixture.py "$out/"
cp {prepare_sitl,stage_traversal,recorder_token,fly_traversal}.sh build_world.py test_passage.py "$out/"
cp ../{start_px4.sh,range_sensor.py,check_startup_recording.py,sensor_link.py} "$out/"
cp ../station.py "$out/"
sed 's@/world/dronehacks/model@/world/funnel/model@g' ../split_record.py > "$out/split_record.py"
COPYFILE_DISABLE=1 tar --format ustar -C "$out" -cf - pilot split_record.py | "${SSH[@]}" "$TARGET" "tar -C team_ws/funnel_sitl/$id -xf -"
docker exec -d funnel-laptop-$id bash -c 'exec python3 /split/station.py >/run-data/station.jsonl 2>&1'
docker exec -d -e GZ_PARTITION=drone19_split -e GZ_IP=127.0.0.1 funnel-laptop-$id bash -c 'source /opt/ros/jazzy/setup.bash; exec python3 /run-data/split_record.py truth --output /run-data/truth.json --wall-timeout 600 >/run-data/truth.log 2>&1'
"${SSH[@]}" "$TARGET" "docker exec -d funnel-jetson-$id bash -c 'source /usr/local/bin/skybeta_env.sh; exec python3 /funnel/split_record.py telemetry --output /run-data/telemetry.jsonl --wall-timeout 600 >/run-data/record.log 2>&1'"

if [[ -f "$out/wind-fixture.json" ]]; then
  cp record_wind.py "$out/"
  docker exec -d -e GZ_PARTITION=drone19_split -e GZ_IP=127.0.0.1 funnel-laptop-$id bash -c 'source /opt/ros/jazzy/setup.bash; exec python3 /run-data/record_wind.py >/run-data/wind.jsonl 2>/run-data/wind-record.log'
fi
