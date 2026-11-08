"""identity: drone.env, hostname, L4T release, board model, uptime, kernel.

``parse_drone_env`` is a pure-Python port of skybeta's ``host/drone_env.sh``
(``skybeta_load_drone_env`` + ``skybeta_check_domain``), with the same
whitelist, the same quote / ``export`` / ``~/`` / CRLF rules and the same
0..101 domain check. One difference, by design: the shell loader lets the
CALLER'S environment override the file; the agent reports what the FILE says,
so "caller-env precedence" does not apply here (what the stack really runs with
is reported separately, from the stack container's env, in ``stack``).
"""
import getpass
import os
import re
import socket
import time

from . import Collector, read_text

DRONE_ENV_KEYS = (
    "DRONE_ID", "TEAM",
    "ROS_DOMAIN_ID", "SKYBETA_DDS_SCOPE",
    "CAMERA_X", "CAMERA_Y", "CAMERA_Z", "CAMERA_ROLL", "CAMERA_PITCH", "CAMERA_YAW",
    "COMPETITOR_PKG", "COMPETITOR_LAUNCH",
    "TEAM_WS",
)
DOMAIN_MAX = 101
DEFAULT_DRONE_ENV = "/etc/skybeta/drone.env"

_DQ = re.compile(r'^"(.*)"$', re.S)
_SQ = re.compile(r"^'(.*)'$", re.S)


def parse_drone_env(text, home):
    """Parse drone.env text exactly like ``skybeta_load_drone_env``.

    Returns ``(values, ignored)``: ``values`` maps whitelisted keys to their
    literal string value (first occurrence wins -- in the shell loader a key
    already set is never overwritten); ``ignored`` lists unknown keys in file
    order.
    """
    values = {}
    ignored = []
    # bash `read -r` splits on \n only; a trailing \r is stripped per line.
    for line in text.split("\n"):
        if line.endswith("\r"):
            line = line[:-1]
        line = line.lstrip(" \t\v\f\r")
        if line.startswith("export "):
            line = line[len("export "):]
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        value = value.rstrip(" \t\v\f\r\n")
        m = _DQ.match(value) or _SQ.match(value)
        if m:
            value = m.group(1)
        if key not in DRONE_ENV_KEYS:
            ignored.append(key)
            continue
        if value.startswith("~/"):
            value = home + "/" + value[2:]          # "${HOME}/${value#"~/"}", verbatim
        if key in values:
            continue
        values[key] = value
    return values, ignored


def check_domain(value):
    """``skybeta_check_domain``: None if usable, else the one-line reason.

    ``value`` None/"" counts as 0 (unset). Decimal, never octal ("008" is 8).
    """
    d = "0" if value in (None, "") else value
    if not re.fullmatch(r"[0-9]+", d):
        return "ROS_DOMAIN_ID='%s' is not a non-negative integer" % d
    if int(d, 10) > DOMAIN_MAX:
        return ("ROS_DOMAIN_ID=%s is above %d; DDS ports would collide with the "
                "ephemeral range" % (d, DOMAIN_MAX))
    return None


def _int_or_none(v):
    if v is None or not re.fullmatch(r"[0-9]+", v.strip()):
        return None
    return int(v.strip(), 10)


def _float_or_none(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def drone_env_section(path, home):
    """The ``identity.drone_env`` sub-document for the file at ``path``."""
    out = {
        "path": path, "present": False, "mtime": None, "ignored_keys": [],
        "drone_id": None, "team": None, "ros_domain_id": 0, "dds_scope": "local",
        "camera": {k: 0.0 for k in ("x", "y", "z", "roll", "pitch", "yaw")},
        "team_ws": home.rstrip("/") + "/team_ws",
        "competitor_pkg": None, "competitor_launch": None,
        "domain_error": None,
    }
    try:
        st = os.stat(path)
        with open(path, "r", encoding="utf-8", errors="replace", newline="") as f:
            text = f.read()
    except OSError:
        return out
    values, ignored = parse_drone_env(text, home)
    out["present"] = True
    out["mtime"] = st.st_mtime
    out["ignored_keys"] = ignored
    out["drone_id"] = _int_or_none(values.get("DRONE_ID"))
    out["team"] = values.get("TEAM")
    dom = values.get("ROS_DOMAIN_ID")
    out["domain_error"] = check_domain(dom)
    # Effective domain, as host/run.sh computes it (${ROS_DOMAIN_ID:-0}); null if unusable.
    out["ros_domain_id"] = None if out["domain_error"] else int(dom or "0", 10)
    out["dds_scope"] = values.get("SKYBETA_DDS_SCOPE") or "local"
    for k in ("x", "y", "z", "roll", "pitch", "yaw"):
        if ("CAMERA_" + k.upper()) in values:
            out["camera"][k] = _float_or_none(values["CAMERA_" + k.upper()])
    if values.get("TEAM_WS"):
        out["team_ws"] = values["TEAM_WS"]
    out["competitor_pkg"] = values.get("COMPETITOR_PKG")
    out["competitor_launch"] = values.get("COMPETITOR_LAUNCH")
    return out


_L4T_RE = re.compile(r"R(\d+).*REVISION: ([\d.]+)")


def parse_l4t(text):
    """'# R36 (release), REVISION: 5.0, ...' -> ('R36', '5.0')."""
    if not text:
        return None, None
    m = _L4T_RE.search(text.splitlines()[0] if text.splitlines() else "")
    if not m:
        return None, None
    return "R" + m.group(1), m.group(2).rstrip(".")


def home_dir():
    home = os.environ.get("HOME")
    if home:
        return home
    try:
        import pwd  # noqa: PLC0415 - not on Windows
        return pwd.getpwuid(os.getuid()).pw_dir
    except Exception:  # noqa: BLE001
        return os.path.expanduser("~")


class IdentityCollector(Collector):
    name = "identity"
    period_s = 30.0
    EMPTY = {
        "hostname": None, "user": None, "model": None, "l4t_release": None,
        "l4t_revision": None, "kernel": None, "boot_time": None, "uptime_s": None,
        "drone_env": None,
    }

    def collect(self):
        home = home_dir()
        hostname = (read_text("/etc/hostname") or "").strip() or socket.gethostname()
        try:
            user = getpass.getuser()
        except Exception:  # noqa: BLE001
            user = None
        model = read_text("/proc/device-tree/model")
        model = model.replace("\x00", "").strip() if model else None
        rel, rev = parse_l4t(read_text("/etc/nv_tegra_release"))
        uptime = None
        up = read_text("/proc/uptime")
        if up:
            try:
                uptime = float(up.split()[0])
            except (ValueError, IndexError):
                uptime = None
        kernel = os.uname().release if hasattr(os, "uname") else None
        return {
            "hostname": hostname,
            "user": user,
            "model": model,
            "l4t_release": rel,
            "l4t_revision": rev,
            "kernel": kernel,
            "boot_time": (time.time() - uptime) if uptime is not None else None,
            "uptime_s": uptime,
            "drone_env": drone_env_section(self.ctx.drone_env_path, home),
        }
