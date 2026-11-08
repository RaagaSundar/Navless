#!/usr/bin/env bash
# Run only INSIDE a network-none simulation container. No PX4 or aircraft I/O.
set -uo pipefail
[[ "${ADAPTIVE_BENCH_ONLY:-}" == 1 && "${ROS_DOMAIN_ID:-}" == 227 ]] || exit 2
[[ ! -e /evidence/ros-v4 ]] || exit 3
mkdir /evidence/ros-v4
source /usr/local/bin/skybeta_env.sh
cases=(nominal M1-3cm M1-5cm M1-10cm depth-2hz depth-loss tracking-loss authority-loss missing-tf stale-depth odom-reset heading-6deg)
for scenario in "${cases[@]}"; do
  duration=9
  [[ "$scenario" == nominal ]] && duration=32
  [[ "$scenario" == M1-* ]] && duration=24
  printf '%s starting %s\n' "$(date -u +%FT%TZ)" "$scenario"
  nice -n 19 timeout 100 python3 /src/tools/jetson_adaptive_ros_bench.py \
    --case "$scenario" --duration "$duration" --output "/evidence/ros-v4/$scenario" \
    > "/evidence/ros-v4/$scenario.log" 2>&1
  code=$?
  printf '%s finished %s exit=%s\n' "$(date -u +%FT%TZ)" "$scenario" "$code"
done
