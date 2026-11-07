#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source ./split_network.sh
JETSON_IP="$(split_select_xrce_ip)"
if ! docker compose exec -T laptop pgrep -f '^gz sim -s' >/dev/null; then
  echo 'Start the Mac Gazebo bench before attaching Mac PX4 SITL.' >&2
  exit 2
fi
if docker compose exec -T laptop pgrep -x px4 >/dev/null; then
  echo 'PX4 SITL already runs in the laptop simulation container.'
  exit 0
fi
docker compose exec -d -e "JETSON_IP=$JETSON_IP" laptop bash /split/start_px4.sh
echo "PX4 SITL started on the Mac; XRCE target $JETSON_IP:8889, ROS domain 219."
echo 'No arm or mode command sent. Logs: state/px4-runs/<UTC timestamp>/.'

# Heartbeat-only local station; never sends mode, arm, RC or parameter commands.
if ! docker compose exec -T laptop pgrep -f '^python3 /split/station.py$' >/dev/null; then
  docker compose exec -d laptop bash -c 'python3 /split/station.py > /root/split-station.log 2>&1'
fi
