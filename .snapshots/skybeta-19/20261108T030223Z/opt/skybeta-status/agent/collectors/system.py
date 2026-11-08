"""system: load, memory, disk, power mode, thermal zones, GPU load (/proc, /sys).

Merged with the tegrastats reader's latest line into the /status ``system``
section. Some thermal zones on the Orin (cv0/cv1/cv2) return EAGAIN on read;
those are skipped, never reported as a number.
"""
import glob
import os
import re
import shutil

from . import Collector, read_text

GPU_LOAD_FILE = "/sys/devices/platform/bus@0/17000000.gpu/load"
NVP_STATUS = "/var/lib/nvpmodel/status"
NVP_CONF_PATH = "/var/lib/nvpmodel/conf_file_path"
PM_ID_RE = re.compile(r"pmode:0*(\d+)")


def parse_meminfo(text):
    vals = {}
    for line in (text or "").splitlines():
        k, _, rest = line.partition(":")
        parts = rest.split()
        if parts and parts[0].isdigit():
            vals[k.strip()] = int(parts[0])  # kB
    return vals


def parse_power_mode(status_text, conf_text):
    """('pmode:0002', nvpmodel.conf text) -> {'id': 2, 'name': 'MAXN_SUPER'}."""
    out = {"id": None, "name": None}
    m = PM_ID_RE.search(status_text or "")
    if not m:
        digits = re.sub(r"[^0-9]", "", status_text or "").lstrip("0")
        if not digits and (status_text or "").strip():
            digits = "0"
        if not digits:
            return out
        out["id"] = int(digits)
    else:
        out["id"] = int(m.group(1))
    m = re.search(r"<\s*POWER_MODEL\s+ID=%d\s+NAME=([^\s>]+)" % out["id"], conf_text or "")
    if m:
        out["name"] = m.group(1)
    return out


def disk(path):
    try:
        du = shutil.disk_usage(path)
    except OSError:
        return None
    return {"path": path, "total_gb": round(du.total / 1e9, 1), "free_gb": round(du.free / 1e9, 1),
            "used_pct": int(round(100.0 * du.used / du.total)) if du.total else None}


def thermal_zones(base="/sys/class/thermal"):
    """{type: degrees C} for every zone whose temp file reads (EAGAIN ones omitted)."""
    zones = {}
    for zdir in sorted(glob.glob(os.path.join(base, "thermal_zone*"))):
        ztype = (read_text(os.path.join(zdir, "type")) or "").strip()
        temp = read_text(os.path.join(zdir, "temp"))
        if not ztype or temp is None:
            continue
        try:
            zones[ztype] = round(int(temp.strip()) / 1000.0, 1)
        except ValueError:
            continue
    return zones


class SystemCollector(Collector):
    name = "system"
    period_s = 5.0
    EMPTY = {"fallback": {"loadavg": None, "mem_available_mb": None, "gpu_load_pct": None,
                          "thermal_zones_c": {}},
             "disk": {"root": None, "team_ws": None},
             "power_mode": {"id": None, "name": None}}

    def collect(self):
        load = read_text("/proc/loadavg")
        loadavg = None
        if load:
            try:
                loadavg = [float(x) for x in load.split()[:3]]
            except ValueError:
                loadavg = None
        mem = parse_meminfo(read_text("/proc/meminfo"))
        gpu = read_text(GPU_LOAD_FILE)
        try:
            gpu_pct = round(int(gpu.strip()) / 10.0, 1) if gpu is not None else None
        except ValueError:
            gpu_pct = None
        root = disk("/")
        team_ws = None
        ident = self.ctx.get("identity") or {}
        ws = (ident.get("drone_env") or {}).get("team_ws")
        if ws and os.path.isdir(ws):
            try:
                if os.stat(ws).st_dev != os.stat("/").st_dev:
                    team_ws = disk(ws)
            except OSError:
                team_ws = None
        conf_path = (read_text(NVP_CONF_PATH) or "").strip() or "/etc/nvpmodel.conf"
        return {
            "fallback": {
                "loadavg": loadavg,
                "mem_available_mb": round(mem["MemAvailable"] / 1024.0) if "MemAvailable" in mem else None,
                "gpu_load_pct": gpu_pct,
                "thermal_zones_c": thermal_zones(),
            },
            "disk": {"root": root, "team_ws": team_ws},
            "power_mode": parse_power_mode(read_text(NVP_STATUS), read_text(conf_path)),
        }
