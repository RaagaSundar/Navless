"""tegrastats: one long-lived ``tegrastats --interval 1000`` and a line parser.

tegrastats runs unprivileged on the fleet image and prints one line a second:

  10-15-2026 20:24:29 RAM 873/7607MB (lfb 7x4MB) SWAP 0/3804MB (cached 0MB)
  CPU [5%@729,1%@729,...] GR3D_FREQ 0% soc1@48.968C tj@50.125C ...
  VDD_IN 4712mW/4712mW VDD_CPU_GPU_CV 677mW/677mW VDD_SOC 1435mW/1435mW

If the process exits it is restarted with a 5 -> 60 s backoff; if the binary is
missing the section says ``available: false`` and the system collector's /sys
fallback is what the dashboard uses.
"""
import logging
import re
import shutil
import subprocess
import threading
import time

log = logging.getLogger("skybeta-status")

RAM_RE = re.compile(r"\bRAM (\d+)/(\d+)MB")
SWAP_RE = re.compile(r"\bSWAP (\d+)/(\d+)MB")
CPU_RE = re.compile(r"\bCPU \[([^\]]+)\]")
CPU_ITEM_RE = re.compile(r"^(\d+)%@(\d+)$")
GR3D_RE = re.compile(r"\bGR3D_FREQ (\d+)%(?:@\[?(\d+))?")
TEMP_RE = re.compile(r"\b([A-Za-z][\w]*)@(-?[\d.]+)C\b")
POWER_RE = re.compile(r"\b([A-Z][A-Z0-9_]*) (\d+)mW/(\d+)mW")
#: tegrastats reports an absent/off sensor as -256C (older L4T) -- not a temperature.
MIN_VALID_TEMP_C = -40.0


def parse_line(line):
    """One tegrastats line -> dict (the /status ``system.tegrastats`` fields).

    Missing groups come back as None, never as a sentinel number.
    """
    out = {"raw": line.strip(), "ram_used_mb": None, "ram_total_mb": None, "swap_used_mb": None,
           "swap_total_mb": None, "cpu": [], "cpu_load_pct_avg": None, "gpu_load_pct": None,
           "gpu_freq_mhz": None, "temps_c": {}, "temp_max_c": None, "temp_max_zone": None,
           "power_mw": {}}
    m = RAM_RE.search(line)
    if m:
        out["ram_used_mb"], out["ram_total_mb"] = int(m.group(1)), int(m.group(2))
    m = SWAP_RE.search(line)
    if m:
        out["swap_used_mb"], out["swap_total_mb"] = int(m.group(1)), int(m.group(2))
    m = CPU_RE.search(line)
    if m:
        for item in m.group(1).split(","):
            im = CPU_ITEM_RE.match(item.strip())
            if im:
                out["cpu"].append({"load_pct": int(im.group(1)), "freq_mhz": int(im.group(2))})
            else:  # "off" (core parked)
                out["cpu"].append({"load_pct": None, "freq_mhz": None})
        loads = [c["load_pct"] for c in out["cpu"] if c["load_pct"] is not None]
        if loads:
            out["cpu_load_pct_avg"] = round(sum(loads) / float(len(loads)), 1)
    m = GR3D_RE.search(line)
    if m:
        out["gpu_load_pct"] = int(m.group(1))
        out["gpu_freq_mhz"] = int(m.group(2)) if m.group(2) else None
    for name, val in TEMP_RE.findall(line):
        t = float(val)
        if t > MIN_VALID_TEMP_C:
            out["temps_c"][name] = t
    if out["temps_c"]:
        zone = max(out["temps_c"], key=lambda k: out["temps_c"][k])
        out["temp_max_c"], out["temp_max_zone"] = out["temps_c"][zone], zone
    for name, now, avg in POWER_RE.findall(line):
        out["power_mw"][name] = {"now": int(now), "avg": int(avg)}
    return out


class TegrastatsReader:
    """Owns the tegrastats subprocess; ``latest()`` is safe from any thread."""

    def __init__(self, binary=None, interval_ms=1000):
        self.binary = binary or shutil.which("tegrastats") or "/usr/bin/tegrastats"
        self.interval_ms = interval_ms
        self._lock = threading.Lock()
        self._parsed = None
        self._mono = None
        self.available = None   # None = not tried yet
        self.error = None
        self._proc = None
        self._thread = None

    def start(self, stop):
        self._thread = threading.Thread(target=self._run, args=(stop,), name="tegrastats", daemon=True)
        self._thread.start()

    def _run(self, stop):
        backoff = 5.0
        while not stop.is_set():
            try:
                self._proc = subprocess.Popen(
                    [self.binary, "--interval", str(self.interval_ms)],
                    stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
            except (FileNotFoundError, PermissionError) as e:
                self.available = False
                self.error = "%s: %s" % (self.binary, e.__class__.__name__)
                stop.wait(300)
                continue
            self.available = True
            got_line = False
            for raw in self._proc.stdout:
                if stop.is_set():
                    break
                line = raw.decode("utf-8", "replace").strip()
                if not line:
                    continue
                parsed = parse_line(line)
                with self._lock:
                    self._parsed, self._mono = parsed, time.monotonic()
                self.error = None
                got_line = True
                backoff = 5.0
            self._kill()
            if stop.is_set():
                break
            self.error = "tegrastats exited (rc=%s)" % (self._proc.returncode,)
            log.warning("%s; restarting in %.0fs", self.error, backoff)
            stop.wait(backoff)
            if not got_line:
                backoff = min(60.0, backoff * 2)

    def _kill(self):
        p = self._proc
        if p and p.poll() is None:
            p.terminate()
            try:
                p.wait(timeout=2)
            except subprocess.TimeoutExpired:
                p.kill()

    def stop(self):
        self._kill()

    def latest(self):
        """The ``system.tegrastats`` dict: available, age_s and the parsed fields."""
        with self._lock:
            parsed, mono = self._parsed, self._mono
        out = {"available": bool(self.available) if self.available is not None else None,
               "age_s": round(time.monotonic() - mono, 2) if mono else None,
               "error": self.error}
        out.update(parsed or parse_line(""))
        if parsed is None:
            out["raw"] = None
        return out
