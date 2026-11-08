"""Agent: owns the collectors, their threads and the tegrastats reader.

Nothing here needs root, the UART or the camera. Each collector runs in its own
daemon thread on its own period; ``build_doc()`` assembles /status on demand.
"""
import collections
import logging
import os
import platform
import threading
import time

from . import status_doc
from .collectors.docker_state import DockerStateCollector
from .collectors.doctor import DoctorCollector
from .collectors.identity import IdentityCollector
from .collectors.image import ImageCollector
from .collectors.repo import RepoCollector
from .collectors.ros_summary import RosSummaryCollector
from .collectors.system import SystemCollector
from .collectors.tailscale_self import TailscaleSelfCollector
from .collectors.team_ws import TeamWsCollector
from .collectors.tegrastats import TegrastatsReader

log = logging.getLogger("skybeta-status")

MAX_ERRORS = 20


def read_version(base_dir):
    """The agent's own VERSION (/opt/skybeta-status/VERSION or the repo's)."""
    for cand in (os.path.join(base_dir, "VERSION"), os.path.join(base_dir, "..", "VERSION"),
                 os.path.join(base_dir, "..", "..", "VERSION")):
        try:
            with open(cand, "r", encoding="utf-8") as f:
                v = f.read().strip()
            if v:
                return v
        except OSError:
            continue
    return "unknown"


class AgentContext:
    """What collectors share: paths/settings, a cross-collector getter, errors."""

    def __init__(self, repo_dir, drone_env_path, rundir, doctor_period_s):
        self.repo_dir = repo_dir
        self.drone_env_path = drone_env_path
        self.rundir = rundir
        self.doctor_period_s = doctor_period_s
        self.collectors = {}
        self._errors = collections.OrderedDict()
        self._lock = threading.Lock()

    def get(self, name):
        c = self.collectors.get(name)
        return c.snapshot()[0] if c else None

    def record_error(self, source, message):
        with self._lock:
            self._errors.pop(source, None)
            self._errors[source] = {"source": source, "error": message, "at": round(time.time(), 3)}
            while len(self._errors) > MAX_ERRORS:
                self._errors.popitem(last=False)

    def errors(self):
        with self._lock:
            return list(self._errors.values())


class Agent:
    COLLECTOR_CLASSES = (IdentityCollector, TailscaleSelfCollector, RepoCollector,
                         DockerStateCollector, ImageCollector, DoctorCollector,
                         SystemCollector, TeamWsCollector, RosSummaryCollector)

    def __init__(self, version, repo_dir, drone_env_path, rundir, doctor_period_s=60.0,
                 tegrastats_bin=None):
        self.version = version
        self.ctx = AgentContext(repo_dir, drone_env_path, rundir, doctor_period_s)
        self.collectors = {}
        for cls in self.COLLECTOR_CLASSES:
            c = cls(self.ctx)
            self.collectors[c.name] = c
        self.ctx.collectors = self.collectors
        self.tegrastats = TegrastatsReader(tegrastats_bin)
        self.started_at = time.time()
        self.started_mono = time.monotonic()
        self.bind = {"ip": None, "port": None, "also_localhost": False}
        self.stop_event = threading.Event()
        self._threads = []

    def start(self):
        # identity first and synchronously: system and team_ws read TEAM_WS from it.
        self.collectors["identity"].run_once()
        self.tegrastats.start(self.stop_event)
        for c in self.collectors.values():
            t = threading.Thread(target=c.loop, args=(self.stop_event,), name="col-" + c.name,
                                 daemon=True)
            t.start()
            self._threads.append(t)
        log.info("agent %s started: repo=%s drone_env=%s rundir=%s", self.version,
                 self.ctx.repo_dir, self.ctx.drone_env_path, self.ctx.rundir)

    def stop(self):
        self.stop_event.set()
        self.tegrastats.stop()

    def info(self, now, now_mono):
        return {
            "version": self.version,
            "pid": os.getpid(),
            "python": platform.python_version(),
            "started_at": round(self.started_at, 3),
            "uptime_s": round(now_mono - self.started_mono, 1),
            "bind": dict(self.bind),
            "fake": False,
            "errors": self.ctx.errors(),
        }

    def build_doc(self):
        return status_doc.build(self)
