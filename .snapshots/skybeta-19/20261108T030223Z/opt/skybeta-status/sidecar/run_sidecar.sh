#!/usr/bin/env bash
# skybeta ROS status sidecar -- host-side wrapper (what skybeta-ros-status.service runs).
#
# Starts the container `skybeta-ros-status` from the STOCK skybeta:<tag> image
# (tag = the skybeta repo's VERSION, re-read on every start, so it follows the
# stack's image), with ros_status_node.py bind-mounted read-only. It joins the
# stack's DDS exactly the way host/run.sh does (same domain, same scope, same
# loopback profile) but gets NO devices, NO --privileged, NO GPU runtime: it can
# never take the UART or the camera.
#
#   run_sidecar.sh [--repo-dir DIR] [--out-dir DIR] [--dry-run]
#
#   --repo-dir DIR  skybeta checkout (default: $SKYBETA_REPO_DIR, ~/skybeta, ~/Downloads/skybeta)
#   --out-dir DIR   where ros_summary.json goes (default: $SKYBETA_STATUS_RUNDIR or
#                   /run/skybeta-status; use a dir under $HOME for a dev run)
#   --dry-run       print the docker command instead of running it
#
# Exit 1 (systemd retries in 30 s) if the image is absent, the DDS profile does
# not parse, or drone.env's domain is unusable.
set -Eeuo pipefail

log()  { printf '[sidecar] %s\n' "$*"; }
die()  { printf '[sidecar] ERROR: %s\n' "$*" >&2; exit 1; }
usage() { sed -n '2,19p' "$0" | sed 's/^# \{0,1\}//'; }

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME=skybeta-ros-status
REPO="${SKYBETA_REPO_DIR:-}"
OUT_DIR="${SKYBETA_STATUS_RUNDIR:-/run/skybeta-status}"
DRY_RUN=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo-dir) REPO="${2:-}"; shift 2 ;;
    --out-dir)  OUT_DIR="${2:-}"; shift 2 ;;
    --dry-run)  DRY_RUN=1; shift ;;
    -h|--help)  usage; exit 0 ;;
    *) usage >&2; die "unknown argument: $1" ;;
  esac
done

if [[ -z "${REPO}" ]]; then
  for cand in "${HOME}/skybeta" "${HOME}/Downloads/skybeta"; do
    if [[ -f "${cand}/host/drone_env.sh" ]]; then REPO="${cand}"; break; fi
  done
fi
[[ -n "${REPO}" && -f "${REPO}/host/drone_env.sh" ]] \
  || die "skybeta repo not found (set SKYBETA_REPO_DIR in /etc/skybeta/status-agent.env or pass --repo-dir)"
[[ -f "${HERE}/ros_status_node.py" ]] || die "${HERE}/ros_status_node.py missing"

# Identity exactly as host/run.sh reads it: parsed (never sourced) drone.env.
# shellcheck source=/dev/null
source "${REPO}/host/drone_env.sh"
skybeta_load_drone_env
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
SKYBETA_DDS_SCOPE="${SKYBETA_DDS_SCOPE:-local}"
if ! derr="$(skybeta_check_domain)"; then die "${derr}"; fi
case "${SKYBETA_DDS_SCOPE}" in
  local|network) ;;
  *) die "SKYBETA_DDS_SCOPE='${SKYBETA_DDS_SCOPE}' -- expected 'local' or 'network'" ;;
esac
export DRONE_ID="${DRONE_ID:-}" TEAM="${TEAM:-}"

TAG=""
[[ -f "${REPO}/VERSION" ]] && TAG="$(tr -d '[:space:]' < "${REPO}/VERSION")"
[[ -n "${TAG}" ]] || die "${REPO}/VERSION missing or empty"
IMAGE="skybeta:${TAG}"
STATUS_VERSION=unknown
for v in "${HERE}/../VERSION" "${HERE}/../../VERSION"; do   # installed layout, then the repo layout
  if [[ -f "${v}" ]]; then STATUS_VERSION="$(tr -d '[:space:]' < "${v}")"; break; fi
done

# Same guard as host/run.sh: a malformed profile makes Fast DDS fall back to its
# built-in transports -- straight onto the WiFi. (Probe the interpreter first: a
# python3 that exists but cannot run must not read as "malformed XML".)
PY=""
for p in python3 python; do
  if command -v "${p}" >/dev/null 2>&1 && "${p}" -c 'print()' >/dev/null 2>&1; then PY="${p}"; break; fi
done
xml_ok() {
  [[ -z "${PY}" ]] || "${PY}" -c 'import sys, xml.etree.ElementTree as E; E.parse(sys.argv[1])' "$1" 2>/dev/null
}
DDS_PROFILE="${REPO}/host/dds/fastdds_localhost.xml"
DDS_ARG=()
if [[ "${SKYBETA_DDS_SCOPE}" == "local" ]]; then
  [[ -f "${DDS_PROFILE}" ]] || die "${DDS_PROFILE} missing -- cannot keep DDS on this drone"
  xml_ok "${DDS_PROFILE}" || die "${DDS_PROFILE} is not well-formed XML"
  DDS_ARG=(
    -v "${DDS_PROFILE}:/etc/skybeta/fastdds.xml:ro"
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml
    -e FASTDDS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml
  )
fi

CMD=(docker run --rm --name "${NAME}"
  --network host --ipc host
  --label io.skybeta.role=status-sidecar
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" -e "SKYBETA_DDS_SCOPE=${SKYBETA_DDS_SCOPE}" -e DRONE_ID -e TEAM
  -e SKYBETA_UNDERLAY_ONLY=1 -e PYTHONUNBUFFERED=1 -e "SKYBETA_STATUS_VERSION=${STATUS_VERSION}"
  "${DDS_ARG[@]}"
  -v "${HERE}:/opt/skybeta-status:ro"
  -v "${OUT_DIR}:/out"
  "${IMAGE}" bash -c 'source skybeta_env.sh && exec python3 /opt/skybeta-status/ros_status_node.py --out /out/ros_summary.json')

if [[ "${DRY_RUN}" -eq 1 ]]; then
  printf 'would run:'; printf ' %q' "${CMD[@]}"; printf '\n'
  exit 0
fi

docker image inspect "${IMAGE}" >/dev/null 2>&1 \
  || die "image ${IMAGE} not present (skybeta host/pull_image.sh or host/provision.sh --from pull); retrying later"
[[ -d "${OUT_DIR}" ]] || mkdir -p "${OUT_DIR}" 2>/dev/null \
  || die "${OUT_DIR} missing and cannot be created (installed drones get it from tmpfiles.d)"
[[ -w "${OUT_DIR}" ]] || die "${OUT_DIR} not writable by $(id -un)"

docker rm -f "${NAME}" >/dev/null 2>&1 || true
log "starting ${NAME} from ${IMAGE} (drone ${DRONE_ID:-?}, domain ${ROS_DOMAIN_ID}, DDS ${SKYBETA_DDS_SCOPE}) -> ${OUT_DIR}/ros_summary.json"
exec "${CMD[@]}"
