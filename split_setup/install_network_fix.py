#!/usr/bin/env python3
"""Patch an existing reviewed split deployment. Files only; no runtime commands."""
import argparse
from pathlib import Path

OLD_SSH = '''TARGET=dronehacks@skybeta-19.local
SOCKET=/tmp/drone19-sim-ssh
SSH=(ssh -S "$SOCKET" -o PubkeyAuthentication=no -o PreferredAuthentications=password)
if ! "${SSH[@]}" -O check "$TARGET" >/dev/null 2>&1; then
  "${SSH[@]}" -M -fN -o ControlPersist=1800 -o ServerAliveInterval=10 \\
    -o ServerAliveCountMax=3 "$TARGET"
fi'''
NEW_SSH = '''source ./split_network.sh
split_ensure_ssh'''
OLD_IP = '''JETSON_IP="${JETSON_IP:-$(python3 -c 'import socket; print(socket.gethostbyname("skybeta-19.local"))')}"'''
NEW_IP = '''source ./split_network.sh
JETSON_IP="$(split_select_xrce_ip)"'''


def patched(text, old, new, filename):
    if text.count(new) == 1 and old not in text:
        return text
    if text.count(old) != 1 or new in text:
        raise ValueError('Unrecognized launcher; refusing to patch ' + filename)
    return text.replace(old, new, 1)


def install(directory):
    directory = Path(directory)
    changes = {}
    for name, old, new in (('start.sh', OLD_SSH, NEW_SSH),
                           ('start_px4_from_mac.sh', OLD_IP, NEW_IP)):
        path = directory / name
        changes[path] = patched(path.read_text(), old, new, name)
    helper = Path(__file__).with_name('split_network.sh').read_text()
    # Validate both launchers before writing either one. Re-running is idempotent.
    for path, content in changes.items():
        if path.read_text() != content:
            path.write_text(content)
    (directory / 'split_network.sh').write_text(helper)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('deployment', type=Path)
    args = parser.parse_args()
    try:
        install(args.deployment)
    except (OSError, ValueError) as error:
        parser.exit(2, str(error) + '\n')
    print('Updated launcher files only; no process was started or restarted.')
