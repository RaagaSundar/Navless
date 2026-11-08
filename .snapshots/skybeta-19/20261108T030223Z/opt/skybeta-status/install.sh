#!/usr/bin/env bash
# skybeta-dashboard: install / update the status agent + ROS sidecar on THIS drone.
#
# Runs ON THE DRONE as the login user (not root). Normally driven by
# tools/install_remote.sh from the admin machine. Idempotent: a re-run with
# nothing changed prints "up to date" and needs no sudo at all.
#
#   install.sh [--repo-dir DIR] [--port N] [--also-localhost] [--no-sudoers]
#              [--no-selfcheck] [--dry-run] [--uninstall]
#
#   --repo-dir DIR    skybeta checkout (default: ~/skybeta, else ~/Downloads/skybeta).
#                     A relative path is taken relative to $HOME.
#   --port N          /status port on the Tailscale IPv4 (default 8765)
#   --also-localhost  agent also serves 127.0.0.1 (default: Tailscale IPv4 only)
#   --no-sudoers      do not install /etc/sudoers.d/skybeta-status (then every
#                     update that restarts the units asks for the sudo password)
#   --no-selfcheck    skip the final GET /status check
#   --dry-run         print every action; change nothing; never uses sudo
#   --uninstall       remove everything this script installed (runs uninstall.sh)
#
# Installs (root-owned, only when they differ; one sudo prompt):
#   /etc/systemd/system/skybeta-status.service, skybeta-ros-status.service
#   /etc/tmpfiles.d/skybeta-status.conf      (/run/skybeta-status, owned by you)
#   /etc/skybeta/status-agent.env            (repo dir, port, ...)
#   /etc/sudoers.d/skybeta-status            (NOPASSWD systemctl start/stop/restart/status
#                                             of the two units only; --no-sudoers skips)
# and, owned by you: /opt/skybeta-status/{agent,sidecar,systemd,VERSION,install.sh,uninstall.sh}
set -Eeuo pipefail

log()  { printf '[install] %s\n' "$*"; }
warn() { printf '[install] WARNING: %s\n' "$*" >&2; }
die()  { printf '[install] ERROR: %s\n' "$*" >&2; exit 1; }
usage() { sed -n '2,29p' "$0" | sed 's/^# \{0,1\}//'; }

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST=/opt/skybeta-status
UNITS=(skybeta-status.service skybeta-ros-status.service)
REPO=""
PORT=8765
ALSO_LOCALHOST=0
SUDOERS=1
SELFCHECK=1
DRY_RUN=0
UNINSTALL=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo-dir)       REPO="${2:-}"; shift 2 ;;
    --port)           PORT="${2:-}"; shift 2 ;;
    --also-localhost) ALSO_LOCALHOST=1; shift ;;
    --no-sudoers)     SUDOERS=0; shift ;;
    --no-selfcheck)   SELFCHECK=0; shift ;;
    --dry-run)        DRY_RUN=1; shift ;;
    --uninstall)      UNINSTALL=1; shift ;;
    -h|--help)        usage; exit 0 ;;
    *) usage >&2; die "unknown argument: $1" ;;
  esac
done

[[ "$(id -u)" -ne 0 ]] || die "run as the drone's login user, not root (it uses sudo where needed)"
ME="$(id -un)"
MYGROUP="$(id -gn)"

if [[ "${UNINSTALL}" -eq 1 ]]; then
  args=()
  [[ "${DRY_RUN}" -eq 1 ]] && args+=(--dry-run)
  exec bash "${SRC}/uninstall.sh" "${args[@]}"
fi

if ! [[ "${PORT}" =~ ^[0-9]+$ ]] || (( PORT < 1 || PORT > 65535 )); then
  die "--port must be 1..65535, got '${PORT}'"
fi
[[ "${DRY_RUN}" -eq 1 ]] && log "DRY RUN: nothing is changed, no sudo is used"

run() { if [[ "${DRY_RUN}" -eq 1 ]]; then printf '    would run: %s\n' "$*"; else "$@"; fi; }

# --- preflight ------------------------------------------------------------------------
VERSION_FILE=""
for cand in "${SRC}/VERSION" "${SRC}/../VERSION"; do
  if [[ -f "${cand}" ]]; then VERSION_FILE="$(cd "$(dirname "${cand}")" && pwd)/VERSION"; break; fi
done
[[ -n "${VERSION_FILE}" ]] || die "VERSION not found next to ${SRC} (copy drone/ together with VERSION)"
NEW_VERSION="$(tr -d '[:space:]' < "${VERSION_FILE}")"
for d in agent sidecar systemd; do
  [[ -d "${SRC}/${d}" ]] || die "${SRC}/${d} missing -- incomplete copy of drone/"
done

command -v python3 >/dev/null 2>&1 || die "python3 not found"
python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3, 8) else 1)' \
  || die "python3 >= 3.8 needed, have $(python3 --version 2>&1)"
[[ " $(id -nG) " == *" docker "* ]] || die "${ME} is not in the docker group (sudo usermod -aG docker ${ME}, then re-login)"
docker info >/dev/null 2>&1 || die "cannot talk to the docker daemon as ${ME}"

expand_repo() {   # ~/x and relative paths are relative to $HOME
  local p="$1"
  # shellcheck disable=SC2088  # matching a literal tilde on purpose
  case "${p}" in
    "~")    p="${HOME}" ;;
    "~/"*)  p="${HOME}/${p#"~/"}" ;;
    /*)     ;;
    *)      p="${HOME}/${p}" ;;
  esac
  printf '%s' "${p%/}"
}
if [[ -n "${REPO}" ]]; then
  REPO="$(expand_repo "${REPO}")"
else
  for cand in "${HOME}/skybeta" "${HOME}/Downloads/skybeta"; do
    if [[ -f "${cand}/host/doctor.sh" ]]; then REPO="${cand}"; break; fi
  done
  [[ -n "${REPO}" ]] || die "no skybeta repo at ~/skybeta or ~/Downloads/skybeta; pass --repo-dir"
fi
[[ "${REPO}" != *"|"* && "${REPO}" != *$'\n'* ]] || die "--repo-dir must not contain '|' or newlines"
for f in host/doctor.sh host/drone_env.sh host/dds/fastdds_localhost.xml VERSION; do
  [[ -f "${REPO}/${f}" ]] || die "${REPO} is not a skybeta checkout (missing ${f})"
done
TAG="$(tr -d '[:space:]' < "${REPO}/VERSION")"
command -v tegrastats >/dev/null 2>&1 || warn "tegrastats not found -- the agent falls back to /proc and /sys"
[[ -r /etc/skybeta/drone.env ]] || warn "/etc/skybeta/drone.env missing -- the dashboard will flag this drone as anonymous"
docker image inspect "skybeta:${TAG}" >/dev/null 2>&1 \
  || warn "image skybeta:${TAG} not present yet -- the sidecar will retry every 30 s until it is"
TSIP="$(tailscale ip -4 2>/dev/null | head -1 || true)"
[[ -n "${TSIP}" ]] || warn "no Tailscale IPv4 right now -- the agent waits for one (it never binds anything else)"
log "agent ${NEW_VERSION} for ${ME}; skybeta repo ${REPO} (image tag ${TAG}); port ${PORT}"

# --- render the root-owned files --------------------------------------------------------
TMP="$(mktemp -d)"
SUDO_KEEPALIVE=""
cleanup() {
  rm -rf "${TMP}"
  if [[ -n "${SUDO_KEEPALIVE}" ]]; then kill "${SUDO_KEEPALIVE}" 2>/dev/null || true; fi
}
trap cleanup EXIT

SYSTEMCTL="$(command -v systemctl || echo /usr/bin/systemctl)"
render() {
  sed -e "s|__USER__|${ME}|g" -e "s|__REPO__|${REPO}|g" -e "s|__PORT__|${PORT}|g" \
      -e "s|__ALSO_LOCALHOST__|${ALSO_LOCALHOST}|g" -e "s|__SYSTEMCTL__|${SYSTEMCTL}|g" "$1" > "$2"
}
render "${SRC}/systemd/skybeta-status.service.in"         "${TMP}/skybeta-status.service"
render "${SRC}/systemd/skybeta-ros-status.service.in"     "${TMP}/skybeta-ros-status.service"
render "${SRC}/systemd/skybeta-status.tmpfiles.conf.in"   "${TMP}/skybeta-status.conf"
render "${SRC}/systemd/status-agent.env.example"          "${TMP}/status-agent.env"
render "${SRC}/systemd/skybeta-status.sudoers.in"         "${TMP}/skybeta-status.sudoers"

# name | rendered file | target | mode.  /etc/sudoers.d is not readable by us, so
# every target is compared against the copy kept in ${DEST}/.installed/ at the
# last install (and against the target itself where it is readable).
TARGETS=(
  "skybeta-status.service|${TMP}/skybeta-status.service|/etc/systemd/system/skybeta-status.service|0644"
  "skybeta-ros-status.service|${TMP}/skybeta-ros-status.service|/etc/systemd/system/skybeta-ros-status.service|0644"
  "skybeta-status.conf|${TMP}/skybeta-status.conf|/etc/tmpfiles.d/skybeta-status.conf|0644"
  "status-agent.env|${TMP}/status-agent.env|/etc/skybeta/status-agent.env|0644"
)
[[ "${SUDOERS}" -eq 1 ]] && TARGETS+=("skybeta-status.sudoers|${TMP}/skybeta-status.sudoers|/etc/sudoers.d/skybeta-status|0440")

ROOT_CHANGES=()
for t in "${TARGETS[@]}"; do
  IFS='|' read -r name new target _mode <<< "${t}"
  if [[ "${name}" == *.sudoers ]]; then
    cmp -s "${new}" "${DEST}/.installed/${name}" || ROOT_CHANGES+=("${target}")
  elif ! cmp -s "${new}" "${target}" 2>/dev/null; then
    ROOT_CHANGES+=("${target}")
  fi
done
REMOVE_SUDOERS=0
if [[ "${SUDOERS}" -eq 0 && -f "${DEST}/.installed/skybeta-status.sudoers" ]]; then
  REMOVE_SUDOERS=1
  ROOT_CHANGES+=("/etc/sudoers.d/skybeta-status (remove: --no-sudoers)")
fi
DEST_OK=0
if [[ -d "${DEST}" && -O "${DEST}" ]]; then DEST_OK=1; else ROOT_CHANGES+=("${DEST}/ (create, owned by ${ME})"); fi
UNITS_ENABLED=1
for u in "${UNITS[@]}"; do
  systemctl is-enabled --quiet "${u}" 2>/dev/null || UNITS_ENABLED=0
done
[[ "${UNITS_ENABLED}" -eq 1 ]] || ROOT_CHANGES+=("systemctl enable ${UNITS[*]}")

# --- root step (only if something root-owned differs) -----------------------------------
if [[ ${#ROOT_CHANGES[@]} -gt 0 ]]; then
  log "root-owned changes needed:"
  for c in "${ROOT_CHANGES[@]}"; do printf '    %s\n' "${c}"; done
  if [[ "${SUDOERS}" -eq 1 ]]; then
    log "the sudoers drop-in being installed is exactly:"
    sed 's/^/    | /' "${TMP}/skybeta-status.sudoers"
  fi
  if [[ "${DRY_RUN}" -eq 0 ]]; then
    log "sudo is needed once for these:"
    sudo -v || die "sudo failed"
    ( while sudo -n true 2>/dev/null; do sleep 50; done ) &
    SUDO_KEEPALIVE=$!
  fi
  [[ "${DEST_OK}" -eq 1 ]] || run sudo install -d -o "${ME}" -g "${MYGROUP}" -m 0755 "${DEST}"
  run sudo install -d -m 0755 /etc/skybeta
  for t in "${TARGETS[@]}"; do
    IFS='|' read -r name new target mode <<< "${t}"
    if [[ "${name}" == *.sudoers ]]; then
      # Validate in place under a name sudo's includedir ignores (it contains a dot).
      run sudo install -m 0440 -o root -g root "${new}" /etc/sudoers.d/.skybeta-status.new
      if [[ "${DRY_RUN}" -eq 1 ]] || sudo visudo -cf /etc/sudoers.d/.skybeta-status.new >/dev/null; then
        run sudo mv -f /etc/sudoers.d/.skybeta-status.new "${target}"
      else
        sudo rm -f /etc/sudoers.d/.skybeta-status.new
        die "rendered sudoers drop-in failed visudo -c; nothing installed for sudoers"
      fi
    else
      run sudo install -D -m "${mode}" -o root -g root "${new}" "${target}"
    fi
  done
  if [[ "${REMOVE_SUDOERS}" -eq 1 ]]; then
    run sudo rm -f /etc/sudoers.d/skybeta-status
    run rm -f "${DEST}/.installed/skybeta-status.sudoers"
  fi
  run sudo systemd-tmpfiles --create /etc/tmpfiles.d/skybeta-status.conf
  run sudo systemctl daemon-reload
  run sudo systemctl enable "${UNITS[@]}"
  run mkdir -p "${DEST}/.installed"
  for t in "${TARGETS[@]}"; do
    IFS='|' read -r name new _target _mode <<< "${t}"
    run cp "${new}" "${DEST}/.installed/${name}"
  done
  ROOT_CHANGED=1
else
  log "root-owned files: up to date (no sudo needed)"
  ROOT_CHANGED=0
fi

# --- user-owned files under /opt/skybeta-status ---------------------------------------------
USER_CHANGED=0
same_tree() { diff -rq --exclude=__pycache__ "$1" "$2" >/dev/null 2>&1; }
for d in agent sidecar systemd; do
  if [[ -d "${DEST}/${d}" ]] && same_tree "${SRC}/${d}" "${DEST}/${d}"; then continue; fi
  log "updating ${DEST}/${d}/"
  USER_CHANGED=1
  run rm -rf "${DEST}/${d}.new"
  run cp -a "${SRC}/${d}" "${DEST}/${d}.new"
  run find "${DEST}/${d}.new" -name __pycache__ -prune -exec rm -rf {} +
  run rm -rf "${DEST}/${d}"
  run mv "${DEST}/${d}.new" "${DEST}/${d}"
done
for pair in "${VERSION_FILE}|VERSION" "${SRC}/install.sh|install.sh" "${SRC}/uninstall.sh|uninstall.sh"; do
  IFS='|' read -r from to <<< "${pair}"
  if ! cmp -s "${from}" "${DEST}/${to}" 2>/dev/null; then
    log "updating ${DEST}/${to}"
    USER_CHANGED=1
    run cp "${from}" "${DEST}/${to}"
  fi
done
run chmod 0755 "${DEST}/install.sh" "${DEST}/uninstall.sh" "${DEST}/sidecar/run_sidecar.sh"

# --- (re)start ------------------------------------------------------------------------------
sysctl_unit() {   # sysctl_unit <verb> <unit>: passwordless via the drop-in or the cached sudo timestamp
  if [[ "${DRY_RUN}" -eq 1 ]]; then printf '    would run: sudo -n %s %s %s\n' "${SYSTEMCTL}" "$1" "$2"; return 0; fi
  sudo -n "${SYSTEMCTL}" "$1" "$2" 2>/dev/null && return 0
  if [[ -t 0 ]]; then   # no drop-in (--no-sudoers) and no cached timestamp: ask
    sudo "${SYSTEMCTL}" "$1" "$2" && return 0
  fi
  warn "could not '$1 $2' without a password; run: sudo systemctl $1 $2"
  return 1
}
if [[ "${ROOT_CHANGED}" -eq 1 || "${USER_CHANGED}" -eq 1 ]]; then
  log "restarting ${UNITS[*]}"
  for u in "${UNITS[@]}"; do sysctl_unit restart "${u}" || true; done
else
  log "up to date (agent ${NEW_VERSION})"
  for u in "${UNITS[@]}"; do
    systemctl is-active --quiet "${u}" 2>/dev/null || { log "${u} not active: starting it"; sysctl_unit start "${u}" || true; }
  done
fi

# --- self-check -----------------------------------------------------------------------------
if [[ "${SELFCHECK}" -eq 0 || "${DRY_RUN}" -eq 1 ]]; then
  log "done (self-check skipped)"
  exit 0
fi
HOSTS="${TSIP}"
[[ "${ALSO_LOCALHOST}" -eq 1 ]] && HOSTS="${HOSTS} 127.0.0.1"
[[ -n "${HOSTS// /}" ]] || die "no Tailscale IPv4 and no --also-localhost: cannot self-check (the agent is installed and waiting for tailscale0)"
# shellcheck disable=SC2086  # HOSTS is a space-separated list on purpose
python3 - "${PORT}" ${HOSTS} <<'PY'
import json, sys, time, urllib.request
port, hosts = sys.argv[1], sys.argv[2:]
deadline = time.time() + 15
doc = None
while time.time() < deadline and doc is None:
    for h in hosts:
        try:
            with urllib.request.urlopen("http://%s:%s/healthz" % (h, port), timeout=2) as r:
                if r.status == 200:
                    url = "http://%s:%s/status" % (h, port)
                    doc = True
                    break
        except Exception:
            pass
    else:
        time.sleep(1)
if doc is None:
    print("[install] ERROR: agent not answering on %s port %s after 15 s -- journalctl -u skybeta-status -n 50"
          % (" or ".join(hosts), port), file=sys.stderr)
    sys.exit(1)
# give the first doctor run (a few seconds) a chance to land
for _ in range(20):
    with urllib.request.urlopen(url, timeout=3) as r:
        doc = json.load(r)
    if doc["doctor"].get("pass") is not None and doc["sidecar"].get("updated_at") is not None:
        break
    time.sleep(1)
de = (doc["identity"].get("drone_env") or {})
d = doc["doctor"]
pwf = "%s/%s/%s" % (d.get("pass"), d.get("warn"), d.get("fail")) if d.get("pass") is not None else "pending"
print("[install] self-check %s: drone %s team %s agent %s image %s stack %s sidecar %s doctor %s" % (
    url, de.get("drone_id"), de.get("team"), doc["agent"]["version"], doc["image"].get("id_short"),
    "up" if doc["stack"].get("running") else "down", "up" if doc["sidecar"].get("running") else "down", pwf))
PY
