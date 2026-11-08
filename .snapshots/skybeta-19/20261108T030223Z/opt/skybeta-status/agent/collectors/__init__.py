"""Collector base class and the subprocess helper every collector uses.

A collector gathers one section (or a few related sections) of the /status
document on its own period, in its own daemon thread. Rules shared by all of
them:

* ``collect()`` returns a plain dict. It may raise; the base class catches
  everything, records ``error`` and keeps the LAST GOOD value (or ``None`` if
  there never was one). Nothing ever propagates out of a collector thread.
* Every subprocess has a fixed argv (no shell) and a hard timeout.
* Ages are computed from ``time.monotonic()`` so a wall-clock jump on the
  drone cannot make a section look fresh or stale.
* The first failure and the recovery are logged, not every failure.
"""
import logging
import os
import re
import subprocess
import threading
import time
from datetime import datetime, timedelta, timezone

log = logging.getLogger("skybeta-status")


class CommandError(Exception):
    """A subprocess could not be run, timed out, or (when checked) failed."""


class Degraded(Exception):
    """Raised by ``collect()`` to publish ``data`` AND an error at the same time.

    For failures whose correct reading is "known bad" rather than "last good"
    (the docker daemon is down: the stack is NOT known to be running).
    """

    def __init__(self, message, data):
        super().__init__(message)
        self.data = data


def run_cmd(argv, timeout, env=None, cwd=None, check=True):
    """Run ``argv`` (a list, never a shell string) with a hard timeout.

    Returns ``(returncode, stdout, stderr)`` as text. Raises CommandError when
    the binary is missing, on timeout, or -- with ``check`` -- on a non-zero
    exit status (the message carries the first line of stderr).
    """
    try:
        proc = subprocess.run(
            argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, timeout=timeout, env=env, cwd=cwd,
            check=False)
    except FileNotFoundError:
        raise CommandError("%s: not found" % argv[0])
    except subprocess.TimeoutExpired:
        raise CommandError("%s: timeout after %gs" % (os.path.basename(argv[0]), timeout))
    except OSError as e:
        raise CommandError("%s: %s" % (argv[0], e))
    out = proc.stdout.decode("utf-8", "replace")
    err = proc.stderr.decode("utf-8", "replace")
    if check and proc.returncode != 0:
        first = (err.strip().splitlines() or [""])[0]
        raise CommandError("%s exited %d%s" % (
            " ".join(os.path.basename(a) if i == 0 else a for i, a in enumerate(argv[:3])),
            proc.returncode, (": " + first) if first else ""))
    return proc.returncode, out, err


def read_text(path, default=None):
    """File contents as text, or ``default`` if unreadable.

    The Orin's cv0/cv1/cv2 thermal zones fail reads with EAGAIN, which Python reports as a
    ``None`` read rather than an OSError (in text mode the decoder then raises TypeError).
    Bench-measured 2026-10-16, so read bytes and treat ``None`` as unreadable.
    """
    try:
        with open(path, "rb") as f:
            data = f.read()
    except OSError:
        return default
    if data is None:
        return default
    return data.decode("utf-8", "replace")


_RFC3339_RE = re.compile(
    r"^(\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d)(?:\.(\d+))?(Z|[+-]\d\d:\d\d)?$")


def parse_rfc3339(text):
    """Docker/Go timestamps ('2026-10-15T20:00:00.123456789Z') -> unix seconds.

    Python 3.10's ``fromisoformat`` handles neither 'Z' nor nanoseconds, hence
    this. Returns None for the zero time ('0001-01-01T00:00:00Z') or garbage.
    """
    if not text:
        return None
    m = _RFC3339_RE.match(text.strip())
    if not m:
        return None
    base, frac, tz = m.groups()
    if base.startswith("0001-"):
        return None
    dt = datetime.strptime(base, "%Y-%m-%dT%H:%M:%S")
    if tz in (None, "Z"):
        offset = timedelta(0)
    else:
        sign = 1 if tz[0] == "+" else -1
        offset = sign * timedelta(hours=int(tz[1:3]), minutes=int(tz[4:6]))
    dt = dt.replace(tzinfo=timezone(offset))
    value = dt.timestamp()
    if frac:
        value += float("0." + frac[:9])
    return value


class Collector:
    """Base class. Subclasses set ``name``/``period_s`` and implement ``collect``."""

    name = "base"
    period_s = 60.0
    #: Keys of the section(s) with null values: what /status shows before the
    #: first successful collection, so the schema never loses a key.
    EMPTY = {}

    def __init__(self, ctx):
        self.ctx = ctx
        self._lock = threading.Lock()
        self.data = None
        self.error = None
        self.error_at = None
        self.updated_at = None
        self.updated_mono = None
        self.runs = 0
        self._failing = False

    def collect(self):  # pragma: no cover - abstract
        raise NotImplementedError

    def run_once(self):
        try:
            data = self.collect()
        except Degraded as d:
            msg = str(d)
            with self._lock:
                self.data = d.data
                self.error = msg
                self.error_at = self.updated_at = time.time()
                self.updated_mono = time.monotonic()
                self.runs += 1
            if not self._failing:
                log.warning("collector %s degraded: %s", self.name, msg)
                self._failing = True
            self.ctx.record_error(self.name, msg)
            return False
        except Exception as e:  # noqa: BLE001 - a collector must never raise
            msg = str(e) or e.__class__.__name__
            with self._lock:
                self.error = msg
                self.error_at = time.time()
                self.runs += 1
            if not self._failing:
                log.warning("collector %s failing: %s", self.name, msg)
                self._failing = True
            self.ctx.record_error(self.name, msg)
            return False
        with self._lock:
            self.data = data
            self.error = None
            self.updated_at = time.time()
            self.updated_mono = time.monotonic()
            self.runs += 1
        if self._failing:
            log.info("collector %s recovered", self.name)
            self._failing = False
        return True

    def snapshot(self):
        """(data, updated_at, updated_mono, error) under the lock."""
        with self._lock:
            return self.data, self.updated_at, self.updated_mono, self.error

    def loop(self, stop):
        while not stop.is_set():
            self.run_once()
            stop.wait(self.period_s)
