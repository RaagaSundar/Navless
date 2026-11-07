#!/usr/bin/env bash
# Separate finite optical renders only. No ROS/PX4/flight or existing-world calls.
set -euo pipefail
[[ $# == 7 ]] || { echo 'Usage: run_static_ab.sh IMAGE BASE_ASSETS BASE_WORLD CAMERA POSE_JSON ATTITUDE_JSON OUTPUT' >&2; exit 2; }
HERE=$(cd -- "$(dirname -- "$0")" && pwd)
IMAGE=$1
BASE=$(cd -- "$2" && pwd)
WORLD=$(python3 -c 'import pathlib,sys;print(pathlib.Path(sys.argv[1]).resolve(strict=True))' "$3")
CAMERA=$(python3 -c 'import pathlib,sys;print(pathlib.Path(sys.argv[1]).resolve(strict=True))' "$4")
POSE=$(python3 -c 'import pathlib,sys;print(pathlib.Path(sys.argv[1]).resolve(strict=True))' "$5")
ATTITUDE=$(python3 -c 'import pathlib,sys;print(pathlib.Path(sys.argv[1]).resolve(strict=True))' "$6")
OUTPUT=$(python3 -c 'import pathlib,sys;print(pathlib.Path(sys.argv[1]).resolve())' "$7")
: "${APPEARANCE_OVERLAY:?Set an explicitly generated APPEARANCE_OVERLAY directory}"
OVERLAY=$(cd -- "$APPEARANCE_OVERLAY" && pwd)
[[ ! -e "$OUTPUT" ]] || { echo 'Output already exists; choose a new directory.' >&2; exit 2; }
mkdir -p "$OUTPUT"
# An operator-supplied image must already have gz transport13/msgs10, OpenCV,
# NumPy, Gazebo Harmonic and Xvfb. Never builds/pulls dependencies here.
docker image inspect "$IMAGE" >/dev/null
for variant in original synthetic; do
  assets="$BASE"; world="$WORLD"
  if [[ "$variant" == synthetic ]]; then
    assets="$OVERLAY"; world="$OVERLAY/worlds/dronehacks-synthetic.sdf"
  fi
  output="$OUTPUT/$variant"
  python3 "$HERE/static_bench.py" prepare --output "$output" --world "$world" \
    --camera "$CAMERA" --pose "$POSE" --attitude "$ATTITUDE"
  docker run --pull=never --rm --init --cpus=1.5 --memory=2g --pids-limit=192 \
    --network=none --shm-size=256m \
    -e ROS_DOMAIN_ID=223 -e "GZ_PARTITION=appearance_ab_${RANDOM}_${variant}" -e GZ_IP=127.0.0.1 \
    -e LIBGL_ALWAYS_SOFTWARE=1 -e GALLIUM_DRIVER=llvmpipe -e QT_QPA_PLATFORM=offscreen \
    -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 \
    -v "$HERE:/bench:ro" -v "$assets:/assets:ro" -v "$output:/out" \
    "$IMAGE" bash -c \
    'Xvfb :91 -screen 0 640x480x24 -nolisten tcp >/out/xvfb.log 2>&1 & export DISPLAY=:91; python3 /bench/static_bench.py capture --output /out --assets /assets'
done
