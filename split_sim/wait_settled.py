"""Passively require two seconds of new landed/disarmed source samples."""
import json
import sys
import time


def wait(stream):
    stream.seek(0, 2)
    states = {}
    since = None
    deadline = time.monotonic() + 180
    while time.monotonic() < deadline:
        line = stream.readline()
        if not line:
            time.sleep(.1)
            continue
        row = json.loads(line)
        kind = row['kind']
        stamp = row['timestamp'] / 1e6
        if kind in states and stamp <= states[kind][0]:
            continue
        states[kind] = (stamp, row)
        if 'land' not in states or 'status' not in states:
            continue
        land, status = states['land'], states['status']
        if abs(land[0] - status[0]) > 1.5 or not land[1]['landed'] or status[1]['armed']:
            since = None
            continue
        common = min(land[0], status[0])
        if since is None:
            # The older of a fresh land/status pair can predate final disarm.
            # Begin only after *both* signals first attest settled state.
            since = max(land[0], status[0])
        if common - since >= 2.5:
            print(json.dumps({'landed_disarmed': True, 'from_sim': since, 'to_sim': common}))
            return 0
    print(json.dumps({'landed_disarmed': False, 'last': states}))
    return 1


if __name__ == '__main__':
    with open(sys.argv[1]) as records:
        sys.exit(wait(records))
