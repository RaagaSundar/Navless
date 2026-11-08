#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
source ../split_network.sh
split_ensure_ssh
run=$(date -u +%Y%m%dT%H%M%SZ)
remote="team_ws/funnel_bench/$run"
out="runs/$run"
mkdir -p runs
mkdir "$out"
cp bench.sh observe.py opening.py funnel.sdf "$out/"
ssh_run() { "${SSH[@]}" "$TARGET" "$@"; }
ssh_run "mkdir -p team_ws/funnel_bench && mkdir $remote && mkdir $remote/output"
tar -C "$out" -cf - bench.sh observe.py opening.py funnel.sdf | ssh_run "tar -C $remote -xf -"
status=0
ssh_run "docker run --rm --name funnel-$run --network none --runtime nvidia --memory=3g --cpus=4 --pids-limit=512 --device /dev/dri/renderD128 --device /dev/dri/renderD129 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e SKYBETA_UNDERLAY_ONLY=1 -e ROS_DOMAIN_ID=219 -e ROS_LOCALHOST_ONLY=1 -v \$HOME/team_ws/split_sim:/split:ro -v \$HOME/$remote:/funnel:ro -v \$HOME/$remote/output:/run-data --entrypoint /bin/bash drone19-gazebo-deps:humble-harmonic /funnel/bench.sh" >"$out/container.log" 2>&1 || status=$?
ssh_run "tar -C $remote/output -cf - ." | tar -C "$out" -xf -
python3 - "$out" "$status" <<'PY'
import hashlib, json, pathlib, sys
p = pathlib.Path(sys.argv[1])
(p/'renderer-source.json').write_text(json.dumps(dict(world_sha256=hashlib.sha256((p/'funnel.sdf').read_bytes()).hexdigest(), container_exit=int(sys.argv[2]))))
PY
python3 ../run_comparison.py "$out" --kind bench --name "Jetson funnel bearing at fixed yaw" "${@}"
exit "$status"
