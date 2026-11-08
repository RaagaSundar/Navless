"""docker_state: running containers, the stack (and its tmux windows), the sidecar.

One collector feeds three /status sections -- ``docker``, ``stack`` and
``sidecar`` -- because all three come from the same ``docker ps`` +
``docker inspect`` pair.

Stack detection follows host/run.sh: the container named ``skybeta-stack``,
else whichever running container has ``/dev/ttyTHS1`` passed through (run.sh's
``uart_holder`` rule; this also catches ``run.sh shell``'s ``skybeta``). The
tmux probe is the only ``docker exec`` the agent ever runs, and it only lists
windows.
"""
import copy
import json
import time

from . import Collector, CommandError, Degraded, parse_rfc3339, run_cmd

STACK_NAME = "skybeta-stack"
STACK_SERVICE = "skybeta-stack.service"
SIDECAR_NAME = "skybeta-ros-status"
UART_DEV = "/dev/ttyTHS1"
TMUX_SESSION = "skybeta"
ENV_WHITELIST = ("ROS_DOMAIN_ID", "SKYBETA_DDS_SCOPE", "DRONE_ID", "TEAM",
                 "FASTRTPS_DEFAULT_PROFILES_FILE", "XRCE_DEV", "XRCE_BAUD")
# A real tab: tmux does not interpret "\t" escapes in -F formats.
TMUX_FORMAT = "\t".join(["#{window_index}", "#{window_name}", "#{pane_dead}", "#{pane_dead_status}"])

EMPTY_STACK = {"running": False, "container": None, "uart_holder": None, "image_ref": None,
               "image_id": None, "uptime_s": None, "ros_domain_id": None, "dds_scope": None,
               "tmux": None, "service": None}
EMPTY_SIDECAR = {"running": False, "container": None, "image_ref": None, "image_id": None,
                 "uptime_s": None}


def _int_or_none(v):
    try:
        return int(str(v).strip(), 10)
    except (TypeError, ValueError):
        return None


def container_from_inspect(item, now=None):
    """One ``docker inspect`` element -> the compact /status container dict."""
    now = time.time() if now is None else now
    state = item.get("State") or {}
    cfg = item.get("Config") or {}
    host = item.get("HostConfig") or {}
    env = {}
    for entry in cfg.get("Env") or []:
        k, _, v = entry.partition("=")
        if k in ENV_WHITELIST:
            env[k] = v
    started = parse_rfc3339(state.get("StartedAt"))
    running = bool(state.get("Running"))
    return {
        "name": (item.get("Name") or "").lstrip("/"),
        "id": (item.get("Id") or "")[:12],
        "image_ref": cfg.get("Image"),
        "image_id": item.get("Image"),
        "status": state.get("Status"),
        "started_at": state.get("StartedAt"),
        "uptime_s": round(now - started, 1) if (running and started) else None,
        "devices": [d.get("PathOnHost") for d in (host.get("Devices") or []) if d.get("PathOnHost")],
        "network_mode": host.get("NetworkMode"),
        "privileged": bool(host.get("Privileged")),
        "env": env,
    }


def pick_stack(containers):
    """(stack container dict or None, uart holder name or None)."""
    holder = next((c for c in containers if UART_DEV in c["devices"]), None)
    named = next((c for c in containers if c["name"] == STACK_NAME), None)
    return (named or holder), (holder["name"] if holder else None)


def parse_tmux_windows(text):
    """``tmux list-windows -F TMUX_FORMAT`` output -> list of window dicts."""
    windows = []
    for line in text.splitlines():
        parts = line.split("\t")
        if len(parts) < 2:
            continue
        idx = _int_or_none(parts[0])
        dead = len(parts) > 2 and parts[2].strip() == "1"
        status = _int_or_none(parts[3]) if (dead and len(parts) > 3) else None
        windows.append({"index": idx, "name": parts[1], "dead": dead, "exit_status": status})
    return windows


def stack_section(stack, holder, tmux, service=None):
    if stack is None:
        out = dict(EMPTY_STACK)
        out["uart_holder"] = holder
        out["service"] = service
        return out
    env = stack["env"]
    return {
        "running": True,
        "container": stack["name"],
        "uart_holder": holder,
        "image_ref": stack["image_ref"],
        "image_id": stack["image_id"],
        "uptime_s": stack["uptime_s"],
        "ros_domain_id": _int_or_none(env.get("ROS_DOMAIN_ID", "0")),
        "dds_scope": env.get("SKYBETA_DDS_SCOPE"),
        "tmux": tmux,
        "service": service,
    }


def sidecar_section(containers):
    sc = next((c for c in containers if c["name"] == SIDECAR_NAME), None)
    if sc is None:
        return dict(EMPTY_SIDECAR)
    return {"running": True, "container": sc["name"], "image_ref": sc["image_ref"],
            "image_id": sc["image_id"], "uptime_s": sc["uptime_s"]}


class DockerStateCollector(Collector):
    name = "docker_state"
    period_s = 5.0
    EMPTY = {
        "docker": {"daemon_ok": None, "server_version": None, "containers": []},
        "stack": EMPTY_STACK,
        "sidecar": EMPTY_SIDECAR,
    }

    def __init__(self, ctx):
        super().__init__(ctx)
        self._server_version = None
        self._version_mono = 0.0

    def _tmux(self, container):
        try:
            rc, out, _ = run_cmd(["docker", "exec", container, "tmux", "list-windows",
                                  "-t", TMUX_SESSION, "-F", TMUX_FORMAT], 3.0, check=False)
        except CommandError:
            return {"session": TMUX_SESSION, "ok": False, "windows": [], "dead_count": 0}
        windows = parse_tmux_windows(out) if rc == 0 else []
        return {"session": TMUX_SESSION, "ok": rc == 0, "windows": windows,
                "dead_count": sum(1 for w in windows if w["dead"])}

    def _service(self):
        """skybeta-stack.service (host/autostart.sh): None where the drone cannot say."""
        try:
            _, enabled, _ = run_cmd(["systemctl", "is-enabled", STACK_SERVICE], 3.0, check=False)
            _, active, _ = run_cmd(["systemctl", "is-active", STACK_SERVICE], 3.0, check=False)
        except CommandError:
            return None
        return {"enabled": enabled.strip() == "enabled", "active": active.strip() == "active"}

    def collect(self):
        try:
            return self._collect()
        except CommandError as e:
            down = copy.deepcopy(self.EMPTY)
            down["docker"]["daemon_ok"] = False
            raise Degraded(str(e), down)

    def _collect(self):
        now_mono = time.monotonic()
        if self._server_version is None or now_mono - self._version_mono > 300:
            _, v, _ = run_cmd(["docker", "version", "--format", "{{.Server.Version}}"], 5.0)
            self._server_version = v.strip() or None
            self._version_mono = now_mono
        _, ps, _ = run_cmd(["docker", "ps", "--no-trunc", "--format", "{{.ID}}"], 5.0)
        ids = [line.strip() for line in ps.splitlines() if line.strip()]
        containers = []
        if ids:
            # A container can exit between ps and inspect: inspect then exits 1 but
            # still prints the others, so do not treat rc != 0 as fatal.
            _, out, err = run_cmd(["docker", "inspect"] + ids, 5.0, check=False)
            try:
                items = json.loads(out) if out.strip() else []
            except ValueError:
                raise CommandError("docker inspect: bad JSON (%s)" % err.strip()[:200])
            now = time.time()
            containers = [container_from_inspect(i, now) for i in items
                          if (i.get("State") or {}).get("Running")]
        stack, holder = pick_stack(containers)
        tmux = self._tmux(stack["name"]) if stack else None
        return {
            "docker": {"daemon_ok": True, "server_version": self._server_version,
                       "containers": containers},
            "stack": stack_section(stack, holder, tmux, self._service()),
            "sidecar": sidecar_section(containers),
        }
