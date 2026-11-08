#!/usr/bin/env python3
"""skybeta status agent -- read-only GET /status on the drone's Tailscale IPv4.

  python3 -E -s /opt/skybeta-status/agent/main.py              # what systemd runs
  python3 -E -s agent/main.py --once | python3 -m json.tool     # one document, no server
  python drone/agent/main.py --fake --profile hot --port 8766 --drone-id 2   # off-drone dev

Every option also has an environment variable (set in /etc/skybeta/status-agent.env
for the systemd unit); the command line wins.
"""
import argparse
import json
import logging
import os
import signal
import sys
import threading

# Run as a script, sys.path[0] is .../agent, where agent/agent.py would shadow the
# package. Put the PARENT on the path instead and import the package properly.
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:] = [p for p in sys.path if os.path.abspath(p or os.curdir) != _HERE]
sys.path.insert(0, os.path.dirname(_HERE))

from agent.agent import Agent, read_version  # noqa: E402
from agent.collectors.repo import detect_repo  # noqa: E402
from agent.fake import PROFILES, FakeAgent  # noqa: E402
from agent.httpd import BindManager, check_bind_ip, make_handler  # noqa: E402

log = logging.getLogger("skybeta-status")


def env_bool(name, default=False):
    v = os.environ.get(name)
    if v is None or v == "":
        return default
    return v.strip().lower() in ("1", "true", "yes", "on")


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bind", default=os.environ.get("SKYBETA_STATUS_BIND", "tailscale"),
                   help="'tailscale' (default: the node's 100.x IPv4, rebinds on change) or an IP "
                        "(loopback / 100.64.0.0/10 only, for --fake and tests)")
    p.add_argument("--port", type=int, default=int(os.environ.get("SKYBETA_STATUS_PORT", "8765")))
    p.add_argument("--also-localhost", action="store_true",
                   default=env_bool("SKYBETA_STATUS_ALSO_LOCALHOST"),
                   help="additionally serve on 127.0.0.1 (off by default: Tailscale IPv4 only)")
    p.add_argument("--i-know", action="store_true", help="allow --bind on a non-loopback, non-Tailscale IP")
    p.add_argument("--repo-dir", default=os.environ.get("SKYBETA_REPO_DIR") or None,
                   help="skybeta checkout (default: ~/skybeta, else ~/Downloads/skybeta)")
    p.add_argument("--drone-env", default=os.environ.get("SKYBETA_DRONE_ENV", "/etc/skybeta/drone.env"))
    p.add_argument("--rundir", default=os.environ.get("SKYBETA_STATUS_RUNDIR", "/run/skybeta-status"),
                   help="where the ROS sidecar writes ros_summary.json")
    p.add_argument("--doctor-period", type=float,
                   default=float(os.environ.get("SKYBETA_DOCTOR_PERIOD_S", "60")),
                   help="seconds between doctor runs (min 20)")
    p.add_argument("--once", action="store_true",
                   help="collect once, print the /status JSON to stdout and exit (no server)")
    p.add_argument("--fake", action="store_true", help="serve a synthetic document (no collectors)")
    p.add_argument("--profile", default="healthy", choices=PROFILES, help="--fake profile")
    p.add_argument("--drone-id", type=int, default=1, help="--fake drone id")
    p.add_argument("--fixtures", default=None, help="--fake fixtures dir (default: repo tests/fixtures)")
    p.add_argument("--version", action="store_true", help="print the agent version and exit")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args(argv)
    if args.fake and args.bind == "tailscale" and "SKYBETA_STATUS_BIND" not in os.environ:
        args.bind = "127.0.0.1"
    return args


def main(argv=None):
    args = parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, stream=sys.stderr,
                        format="%(levelname)s %(threadName)s: %(message)s")
    base = os.path.dirname(_HERE)
    version = read_version(base)
    if args.version:
        print(version)
        return 0
    if args.bind != "tailscale":
        try:
            check_bind_ip(args.bind, args.i_know)
        except ValueError as e:
            log.error("%s", e)
            return 2

    if args.fake:
        source = FakeAgent(args.profile, args.drone_id, version, args.fixtures)
    else:
        source = Agent(version, detect_repo(args.repo_dir), args.drone_env, args.rundir,
                       args.doctor_period)

    if args.once:
        if not args.fake:
            # One pass of every collector, plus up to ~2.5 s for a tegrastats line.
            source.tegrastats.start(source.stop_event)
            for c in source.collectors.values():
                c.run_once()
            for _ in range(25):
                if source.tegrastats.latest()["raw"] or source.tegrastats.available is False:
                    break
                source.stop_event.wait(0.1)
            source.stop()
        json.dump(source.build_doc(), sys.stdout, indent=2, default=str)
        sys.stdout.write("\n")
        return 0

    stop = threading.Event()

    def _stop(signum, _frame):
        log.info("signal %d: shutting down", signum)
        stop.set()

    signal.signal(signal.SIGINT, _stop)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, _stop)

    def on_change(ip, port):
        source.bind.update(ip=ip, port=port, also_localhost=bool(args.also_localhost))

    source.bind.update(port=args.port, also_localhost=bool(args.also_localhost))
    source.start()
    mgr = BindManager(make_handler(source.build_doc, version), args.port, args.bind,
                      args.also_localhost, on_change)
    t = threading.Thread(target=mgr.run, args=(stop,), name="bind", daemon=True)
    t.start()
    try:
        while not stop.is_set():
            stop.wait(0.5)       # short waits keep Ctrl-C responsive on Windows
    finally:
        stop.set()
        source.stop()
        t.join(timeout=5)
    return 0


if __name__ == "__main__":
    sys.exit(main())
