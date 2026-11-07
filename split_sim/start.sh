#!/usr/bin/env bash
# Password-only SSH. No password or private key is stored by this setup.
set -euo pipefail
cd "$(dirname "$0")"
MODE="${1:-auto}"
[[ "$MODE" == auto || "$MODE" == --split ]] || { echo 'Usage: bash start.sh [--split]' >&2; exit 2; }
source ./split_network.sh
split_ensure_ssh
# Forwarding stays on loopback. A reverse SSH tunnel carries camera ROS messages.
for spec in 'R 127.0.0.1:17447:127.0.0.1:17447' \
            'R 127.0.0.1:17448:127.0.0.1:17448' \
            'L 127.0.0.1:18765:127.0.0.1:8765' \
            'L 127.0.0.1:18081:127.0.0.1:18081' \
            'L 127.0.0.1:18082:127.0.0.1:18082'; do
  read -r direction address <<< "$spec"
  "${SSH[@]}" -O forward -o ExitOnForwardFailure=yes "-$direction" "$address" "$TARGET" \
    || echo "Forward $address may already exist; verify the viewer below."
done
if [[ "$MODE" == auto ]] && "${SSH[@]}" "$TARGET" 'docker inspect drone19-gazebo-bench >/dev/null 2>&1'; then
  "${SSH[@]}" "$TARGET" '
    if [ "$(docker inspect -f "{{.State.Running}}" drone19-split-sim 2>/dev/null || true)" = true ]; then
      docker stop -t 10 drone19-split-sim >/dev/null
    fi
    docker start drone19-gazebo-bench >/dev/null'
  echo 'Jetson-local Gazebo is running; the Mac renderer stays stopped.'
  echo 'Simulation SLAM: http://127.0.0.1:18765/'
  echo 'Simulation stereo depth: http://127.0.0.1:18081/'
  echo 'Course localization: http://127.0.0.1:18082/'
  exit 0
fi
if [[ "$MODE" == --split ]]; then
  "${SSH[@]}" "$TARGET" 'if docker inspect drone19-gazebo-bench >/dev/null 2>&1; then docker stop -t 10 drone19-gazebo-bench >/dev/null; fi'
fi
docker compose up -d
"${SSH[@]}" "$TARGET" 'docker start drone19-split-sim >/dev/null'
if ! docker compose exec -T laptop pgrep -f '^bash /split/run_laptop.sh$' >/dev/null; then
  docker compose exec -d laptop bash /split/run_laptop.sh
fi
echo 'Gazebo desktop: http://localhost:6081/vnc.html?autoconnect=true&resize=scale'
echo 'Simulation SLAM: http://127.0.0.1:18765/'
echo 'Simulation stereo depth: http://127.0.0.1:18081/'
echo 'Starts stationary sensor bench only; PX4 SITL has a separate start step.'
