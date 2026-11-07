#!/usr/bin/env bash
# Source from installed split launchers. No network operation occurs on source.
TARGET="${JETSON_SSH_TARGET:-dronehacks@skybeta-19.local}"
SOCKET="${JETSON_SSH_SOCKET:-/tmp/drone19-sim-ssh}"
SPLIT_HOST_ALIAS="${JETSON_HOST_KEY_ALIAS:-skybeta-19.local}"
SSH=(ssh -S "$SOCKET" -o "HostKeyAlias=$SPLIT_HOST_ALIAS"
     -o StrictHostKeyChecking=yes -o ConnectTimeout=8
     -o PubkeyAuthentication=no -o PreferredAuthentications=password)

split_ensure_ssh() {
  if "${SSH[@]}" -O check "$TARGET" >/dev/null 2>&1; then
    return 0
  fi
  # The organizer's verified host identity is stable across its LAN addresses.
  local askpass
  askpass="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/keychain_askpass.sh"
  if [[ "$SPLIT_HOST_ALIAS" == skybeta-19.local && "$TARGET" == dronehacks@* &&
        -x "$askpass" && -x /usr/bin/security ]] &&
     /usr/bin/security find-internet-password -a dronehacks -s skybeta-19.local -P 22 -r 'ssh ' >/dev/null 2>&1; then
    local -x SSH_ASKPASS="$askpass" SSH_ASKPASS_REQUIRE=force
  fi
  if "${SSH[@]}" -M -fN -o ControlPersist=1800 -o ServerAliveInterval=10 \
      -o ServerAliveCountMax=3 "$TARGET"; then
    return 0
  fi
  # Never scan addresses or weaken host checking. Fallback is operator supplied.
  if [[ -n "${JETSON_SSH_TARGET:-}" || -z "${JETSON_SSH_FALLBACK:-}" ]]; then
    echo 'SSH unavailable. Set a reviewed JETSON_SSH_TARGET or JETSON_SSH_FALLBACK.' >&2
    return 1
  fi
  TARGET="$JETSON_SSH_FALLBACK"
  "${SSH[@]}" -M -fN -o ControlPersist=1800 -o ServerAliveInterval=10 \
    -o ServerAliveCountMax=3 "$TARGET"
}

split_select_xrce_ip() {
  local candidate connection
  if [[ -n "${JETSON_IP:-}" ]]; then
    candidate="$JETSON_IP"
  else
    # SSH_CONNECTION is supplied by the authenticated server, not mDNS.
    # BatchMode prevents PX4 startup from hanging on an authentication prompt.
    connection="$("${SSH[@]}" -o BatchMode=yes "$TARGET" 'printf "%s\n" "$SSH_CONNECTION"')" || {
      echo 'Cannot inspect authenticated SSH server address; run start.sh or set JETSON_IP.' >&2
      return 1
    }
    candidate="$(printf '%s\n' "$connection" | python3 -c '
import sys
parts = sys.stdin.read().split()
if len(parts) != 4:
    raise SystemExit("Malformed SSH_CONNECTION")
print(parts[2])')" || return 1
  fi
  # PX4 XRCE uses IPv4. Reject malformed, IPv6, wildcard, multicast and loopback.
  python3 -c '
import ipaddress, sys
try:
    address = ipaddress.IPv4Address(sys.argv[1])
except ipaddress.AddressValueError:
    raise SystemExit("XRCE target must be a literal IPv4 address")
if address.is_unspecified or address.is_multicast or address.is_loopback:
    raise SystemExit("XRCE target must be a remote unicast IPv4 address")
print(address)' "$candidate"
}
