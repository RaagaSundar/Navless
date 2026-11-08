#!/usr/bin/env bash
# skybeta-dashboard: remove the status agent + ROS sidecar from THIS drone.
#
#   uninstall.sh [--dry-run]        (also: install.sh --uninstall)
#
# Stops and disables both units, removes the sidecar container, the units, the
# tmpfiles.d entry, the sudoers drop-in, /etc/skybeta/status-agent.env,
# /run/skybeta-status and /opt/skybeta-status. Leaves /etc/skybeta/drone.env and
# the skybeta repo alone. One sudo prompt.
set -Eeuo pipefail

log()  { printf '[uninstall] %s\n' "$*"; }
die()  { printf '[uninstall] ERROR: %s\n' "$*" >&2; exit 1; }

DRY_RUN=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done
[[ "$(id -u)" -ne 0 ]] || die "run as the drone's login user, not root (it uses sudo where needed)"

run() { if [[ "${DRY_RUN}" -eq 1 ]]; then printf '    would run: %s\n' "$*"; else "$@"; fi; }
UNITS=(skybeta-status.service skybeta-ros-status.service)
FILES=(
  /etc/systemd/system/skybeta-status.service
  /etc/systemd/system/skybeta-ros-status.service
  /etc/tmpfiles.d/skybeta-status.conf
  /etc/sudoers.d/skybeta-status
  /etc/sudoers.d/.skybeta-status.new
  /etc/skybeta/status-agent.env
)

if [[ "${DRY_RUN}" -eq 1 ]]; then
  log "DRY RUN: nothing is changed, no sudo is used"
else
  log "sudo is needed once to remove the units and /etc files:"
  sudo -v || die "sudo failed"
fi
for u in "${UNITS[@]}"; do
  run sudo systemctl disable --now "${u}" 2>/dev/null || true
done
run docker rm -f skybeta-ros-status >/dev/null 2>&1 || true
run sudo rm -f "${FILES[@]}"
run sudo systemctl daemon-reload
run sudo systemctl reset-failed "${UNITS[@]}" 2>/dev/null || true
run sudo rm -rf /run/skybeta-status /opt/skybeta-status
log "removed. Left in place: /etc/skybeta/drone.env and the skybeta repo."
