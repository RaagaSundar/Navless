"""--fake: serve a synthetic but schema-exact /status, for dashboard work off-drone.

The base document is ``tests/fixtures/status_healthy.json``; each profile is a
small, documented mutation of it (the FC/ROS ones splice in the
``ros_summary_*.json`` fixtures). Timestamps, ages, ``seq``, temperatures and
rates are jittered on every request so the dashboard sees a live drone.

  python drone/agent/main.py --fake --profile hot --bind 127.0.0.1 --port 8766 --drone-id 2
"""
import copy
import json
import os
import platform
import random
import time

PROFILES = ("healthy", "fc-down", "stack-down", "hot", "lowbat", "doctor-fail", "stale-ros",
            "old-image", "scope-network", "agent-old")

OLD_IMAGE_ID = "sha256:9267c55c27810c7a4b6f2a1f0c3e9d8b7a6f5e4d3c2b1a09f8e7d6c5b4a39281"


def default_fixtures_dir():
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", "..", "tests", "fixtures"))


def load_fixtures(fixtures_dir=None):
    d = fixtures_dir or default_fixtures_dir()
    names = {"healthy": "status_healthy.json", "ros_full": "ros_summary_full.json",
             "ros_no_fc": "ros_summary_no_fc.json", "ros_down": "ros_summary_stack_down.json"}
    out = {}
    for key, name in names.items():
        path = os.path.join(d, name)
        try:
            with open(path, "r", encoding="utf-8") as f:
                out[key] = json.load(f)
        except OSError as e:
            raise SystemExit("--fake needs %s (%s); pass --fixtures DIR" % (path, e.strerror))
    return out


def _sections(doc):
    return [k for k, v in doc.items() if isinstance(v, dict) and "age_s" in v]


def _splice_ros(doc, summary, now, age):
    ros = {"updated_at": now - 0.2, "age_s": None, "error": None, "file_present": True,
           "file_mtime": now - age}
    ros.update(copy.deepcopy(summary))
    ros["written_at"] = now - age
    ros["age_s"] = round(age, 2)
    doc["ros"] = ros


def apply_profile(doc, profile, fx, now):
    """Mutate ``doc`` in place for ``profile`` (see PROFILES)."""
    if profile not in PROFILES:
        raise ValueError("unknown profile %r (one of: %s)" % (profile, ", ".join(PROFILES)))
    if profile == "fc-down":
        _splice_ros(doc, fx["ros_no_fc"], now, doc["ros"]["age_s"] or 0.5)
    elif profile == "stack-down":
        _splice_ros(doc, fx["ros_down"], now, doc["ros"]["age_s"] or 0.5)
        doc["stack"].update(running=False, container=None, uart_holder=None, image_ref=None,
                            image_id=None, uptime_s=None, ros_domain_id=None, dds_scope=None,
                            tmux=None)
        doc["docker"]["containers"] = [c for c in doc["docker"]["containers"]
                                       if c["name"] != "skybeta-stack"]
    elif profile == "hot":
        ts = doc["system"]["tegrastats"]
        ts["temps_c"] = {k: round(v + 31.0, 3) for k, v in ts["temps_c"].items()}
        ts["temp_max_c"] = max(ts["temps_c"].values())
        doc["system"]["fallback"]["thermal_zones_c"] = {
            k: round(v + 31.0, 1) for k, v in doc["system"]["fallback"]["thermal_zones_c"].items()}
    elif profile == "lowbat":
        bat = doc["ros"]["fc"]["battery"]
        bat.update(remaining=0.12, voltage_v=13.6, warning=2, warning_name="CRITICAL")
    elif profile == "doctor-fail":
        d = doc["doctor"]
        for c in d["checks"]:
            if c["name"] == "uart-devicetree":
                c["result"] = "FAIL"
                c["detail"] = ("missing iommus dmas -- run host/provision.sh --from uart_patch, "
                               "then reboot")
        d.update(exit_code=1, ok=False, **{"pass": d["pass"] - 1, "fail": d["fail"] + 1})
    elif profile == "stale-ros":
        _splice_ros(doc, fx["ros_full"], now, 30.0)
    elif profile == "old-image":
        # The whole drone is consistently on an older build: only the fleet
        # comparison (expected id / majority) can catch it.
        doc["stack"]["image_id"] = doc["sidecar"]["image_id"] = OLD_IMAGE_ID
        for c in doc["docker"]["containers"]:
            c["image_id"] = OLD_IMAGE_ID
        doc["image"].update(id=OLD_IMAGE_ID, id_short=OLD_IMAGE_ID[7:19])
    elif profile == "scope-network":
        doc["identity"]["drone_env"]["dds_scope"] = "network"
        doc["stack"]["dds_scope"] = "network"
        doc["ros"]["env"]["dds_scope"] = "network"
    elif profile == "agent-old":
        doc["agent"]["version"] = "0.0.9"
    return doc


def repo_version():
    """The repo's version, for a fake agent that should look up to date to the real dashboard."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "VERSION")
    with open(path, encoding="utf-8") as f:
        return f.read().strip()


class FakeAgent:
    """Stands in for ``Agent``: ``build_doc()`` returns a fresh synthetic doc."""

    def __init__(self, profile="healthy", drone_id=1, version="0.1.0", fixtures_dir=None, seed=None):
        if profile not in PROFILES:
            raise ValueError("unknown profile %r (one of: %s)" % (profile, ", ".join(PROFILES)))
        self.profile = profile
        self.drone_id = int(drone_id)
        self.version = version
        self.fx = load_fixtures(fixtures_dir)
        self.rng = random.Random(seed if seed is not None else self.drone_id)
        self.started_at = time.time()
        self.started_mono = time.monotonic()
        self.bind = {"ip": None, "port": None, "also_localhost": False}
        self._seq0 = self.rng.randint(100, 5000)

    def start(self):
        pass

    def stop(self):
        pass

    def build_doc(self, now=None):
        now = time.time() if now is None else now
        rng = self.rng
        n = self.drone_id
        doc = copy.deepcopy(self.fx["healthy"])
        uptime = time.monotonic() - self.started_mono
        doc["generated_at"] = round(now, 3)
        doc["agent"].update(version=self.version, pid=os.getpid(), python=platform.python_version(), started_at=round(self.started_at, 3),
                            uptime_s=round(uptime, 1), bind=dict(self.bind), fake=True, errors=[])
        for name in _sections(doc):
            sec = doc[name]
            sec["age_s"] = round(sec["age_s"] * (0.5 + rng.random()), 2)
            sec["updated_at"] = round(now - sec["age_s"], 3)
        # identity: one drone per --drone-id, domain = id (the fleet convention)
        ident = doc["identity"]
        ident["hostname"] = "skybeta-%02d" % n
        ident["user"] = "skybeta"
        de = ident["drone_env"]
        de.update(drone_id=n, ros_domain_id=n, team="team-%02d" % n,
                  team_ws="/home/skybeta/team_ws")
        ident["tailscale"].update(ipv4=self.bind.get("ip") or "127.0.0.1",
                                  dns_name="fake-%02d.tail9e2d8c.ts.net" % n,
                                  hostname="skybeta-%02d" % n, tags=["tag:drone"])
        doc["repo"]["path"] = "/home/skybeta/skybeta"
        doc["team_ws"]["path"] = "/home/skybeta/team_ws"
        for c in doc["docker"]["containers"]:
            c["env"].update(ROS_DOMAIN_ID=str(n), DRONE_ID=str(n), TEAM="team-%02d" % n)
        doc["stack"]["ros_domain_id"] = n
        doc["stack"]["uptime_s"] = round(1500.0 + uptime, 1)
        doc["sidecar"]["uptime_s"] = round(4000.0 + uptime, 1)
        # live-ish values
        ts = doc["system"]["tegrastats"]
        ts["temps_c"] = {k: round(v + rng.uniform(-1.5, 1.5), 3) for k, v in ts["temps_c"].items()}
        ts["temp_max_zone"] = max(ts["temps_c"], key=ts["temps_c"].get)
        ts["temp_max_c"] = ts["temps_c"][ts["temp_max_zone"]]
        ts["age_s"] = round(rng.uniform(0.0, 1.0), 2)
        ros = doc["ros"]
        age = round(rng.uniform(0.05, 1.0), 2)
        ros.update(written_at=round(now - age, 3), age_s=age, file_mtime=round(now - age, 3),
                   seq=self._seq0 + int(uptime))
        ros["env"]["ros_domain_id"] = n
        vio = ros["vio"]
        for k, nominal in (("odom_rate_hz", 30.0), ("bridge_rate_hz", 30.0),
                           ("cam_left_rate_hz", 30.0), ("imu_rate_hz", 199.5)):
            vio[k] = round(nominal + rng.uniform(-0.5, 0.5), 1)
        if ros.get("skybeta_state"):
            ros["skybeta_state"]["stamp"] = round(now - 0.1, 3)
        doc["doctor"]["ran_at"] = round(now - doc["doctor"]["age_s"] - 2.6, 3)
        apply_profile(doc, self.profile, self.fx, now)
        return doc
