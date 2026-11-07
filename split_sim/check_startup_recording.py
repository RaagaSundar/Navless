"""Independent recorder attestation for isolated split SITL."""
from course_runtime.startup import fresh_recorder_token


def recorder_token(run_id, truth, rows):
    if not truth.get('contact_topics_verified') or not truth.get('samples'):
        raise ValueError('independent contact publishers or truth samples missing')
    stamp = truth['samples'][-1]['t']
    latest = {}
    newest = {}
    for row in rows:
        kind = row['kind']
        if row['timestamp'] > newest.get(kind, {}).get('timestamp', -1):
            newest[kind] = row
        if row['timestamp'] / 1e6 <= stamp and row['timestamp'] > latest.get(kind, {}).get('timestamp', -1):
            latest[kind] = row
    status, land = latest.get('status', {}), latest.get('land', {})
    if status.get('armed') is not False or status.get('failsafe') is not False or land.get('landed') is not True:
        raise ValueError('independent landed/disarmed status unavailable')
    token = dict(run_id=run_id, truth_s=stamp, status_s=status['timestamp']/1e6, land_s=land['timestamp']/1e6)
    if not fresh_recorder_token(token, run_id, stamp) or stamp-token['land_s'] >= 1.0:
        raise ValueError('independent source stamps stale')
    status, land = newest['status'], newest['land']
    if status.get('armed') is not False or status.get('failsafe') is not False or land.get('landed') is not True:
        raise ValueError('independent telemetry changed after truth snapshot')
    token.update(status_s=status['timestamp']/1e6, land_s=land['timestamp']/1e6)
    return token


if __name__ == '__main__':
    import json
    import sys
    from pathlib import Path
    run_id, truth_path, telemetry_path = sys.argv[1:]
    truth = json.loads(Path(truth_path).read_text())
    rows = [json.loads(row) for row in Path(telemetry_path).read_text().splitlines()]
    print(json.dumps(recorder_token(run_id, truth, rows), allow_nan=False))
