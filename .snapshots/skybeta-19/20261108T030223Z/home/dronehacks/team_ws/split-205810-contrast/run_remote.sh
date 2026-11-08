#!/usr/bin/env bash
# Run on Jetson after pilot hands over GPU, with sealed sensor.sqlite in this directory.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
[[ $(docker inspect -f '{{.State.Running}}' drone19-split-sim) == false ]]
name=drone19-contrast-offline
for profile in native gamma-half; do
    docker run -d --rm --name "$name" --network none --cpus 2 --runtime nvidia --shm-size 512m \
        --cap-drop ALL --cap-add DAC_OVERRIDE --security-opt no-new-privileges \
        -e ROS_DOMAIN_ID=218 -e ROS_LOCALHOST_ONLY=1 -e SKYBETA_UNDERLAY_ONLY=1 \
        -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
        -e REPLAY_STEREO_ONLY=1 -e REPLAY_PIXELS="$profile" -e REPLAY_START_S=19 -e REPLAY_END_S=90 -e REPLAY_RTF=1 \
        -v "$here":/replay --entrypoint bash skybeta:0.3.3 \
        -lc 'source /usr/local/bin/skybeta_env.sh; exec ros2 launch /replay/vio.launch.py' >/dev/null
    trap 'docker stop -t 3 "$name" >/dev/null 2>&1 || true' EXIT
    docker exec "$name" bash -lc "source /usr/local/bin/skybeta_env.sh; python3 /replay/play.py /replay/sensor.sqlite /replay/$profile.json" > "$here/$profile-play.log" 2>&1
    docker logs "$name" > "$here/$profile-launch.log" 2>&1
    docker stop -t 3 "$name" >/dev/null
    trap - EXIT
done
