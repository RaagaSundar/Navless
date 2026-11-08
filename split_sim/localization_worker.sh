#!/usr/bin/env bash
# Receive-only local data product worker; no SSH, ROS, Docker, GPU or flight commands.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
[[ $# -ge 1 ]] || { echo 'Usage: localization_worker.sh RUN_DIR [--watch|--once] [builder options]' >&2; exit 2; }
run="$1"; shift
mode="${1:---once}"; [[ $# == 0 ]] || shift
case "$mode" in
  --watch) set -- --watch --interval 10 "$@" ;;
  --once) ;;
  *) echo 'Expected --watch or --once' >&2; exit 2 ;;
esac
# PEP723 installs the pinned sealed ULog reader in a uv-managed script environment.
# Live mode does not import pyulog or parse an active ULog.
exec nice -n 10 uv run --script "$here/build_localization_poses.py" "$run" --hz 5 "$@"
