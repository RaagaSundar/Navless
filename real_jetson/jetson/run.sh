#!/usr/bin/env bash
# Run on the Jetson host after this directory is copied to ~/team_ws/real19.
set -eu
case "${1:-}" in
  check) program=check.py ;;
  cloud) program=stereo_cloud.py ;;
  course) program=course.py ;;
  health) program=jetson_slam_health.py ;;
  *) echo 'Usage: bash run.sh {check|cloud|course|health}' >&2; exit 2 ;;
esac
[[ "$(hostname -s)" == skybeta-19 ]] || { echo 'Requires assigned skybeta-19 host' >&2; exit 1; }
exec docker exec -i skybeta-stack bash -c '
  set -e
  export SKYBETA_UNDERLAY_ONLY=1
  source /usr/local/bin/skybeta_env.sh
  test "${ROS_DOMAIN_ID:-}" = 19 || { echo "Physical domain 19 required" >&2; exit 1; }
  cd /root/team_ws/real19
  python3 verify.py
  exec python3 "$1"
' bash "$program"
