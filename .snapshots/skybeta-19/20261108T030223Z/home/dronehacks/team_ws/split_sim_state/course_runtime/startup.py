"""Run-scoped evidence for the split SITL recorder handshake."""
import math
from pathlib import Path


def fresh_recorder_token(token, run_id, sim_now):
    if not isinstance(token, dict) or token.get('run_id') != run_id:
        return False
    for field, limit in [('truth_s', 1.0), ('status_s', 0.75), ('land_s', 1.5)]:
        stamp = token.get(field)
        if type(stamp) not in (int, float) or not math.isfinite(stamp):
            return False
        if not 0 <= sim_now - stamp <= limit:
            return False
    return True


class RecorderReadiness:
    """Require current proof for commands. Brief delivery gaps allow disarmed streaming."""
    def __init__(self, run_id):
        self.run_id = run_id
        self.last_token = None
        self.last_verified = None

    def check(self, token, now):
        if fresh_recorder_token(token, self.run_id, now):
            self.last_token = token
        # The latest snapshot can precede our next /clock callback. Its predecessor
        # remains usable only while every original acquisition stamp is still fresh.
        current = (isinstance(token, dict) and token.get('run_id') == self.run_id and
                   fresh_recorder_token(self.last_token, self.run_id, now))
        if current:
            self.last_verified = now
        stream = current or (self.last_verified is not None and 0 <= now - self.last_verified <= 0.5)
        return stream, current


def prepared_approved(path, run_id, manifest_digest):
    """Approve only this run and the exact locally verified mission manifest."""
    try:
        approval = Path(path).read_text().split()
    except (OSError, UnicodeError):
        return False
    return approval == [run_id, manifest_digest]
