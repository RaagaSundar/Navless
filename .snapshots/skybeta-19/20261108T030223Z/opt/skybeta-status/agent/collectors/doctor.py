"""doctor: run skybeta's ``host/doctor.sh --no-uart`` and parse its output.

This is the "small adapter" between the dashboard and the public repo's
doctor. ``--json`` (skybeta fleet-ready, 2026-10-16) is tried first; a checkout
older than that answers "unknown flag: --json" with exit 2, and the text table
is parsed instead. doctor.sh prints colours unconditionally, so ANSI is
stripped first.

ALWAYS ``--no-uart``: the agent must never open the serial port (the stack
holds it when running, and a sniff when it does not would still be a side
effect on the team's device). NEVER ``--boot-camera`` (it starts a container).
"""
import json
import os
import re
import time

from . import Collector, run_cmd

DOCTOR_ARGS = ("--no-uart",)
FORBIDDEN_ARGS = ("--boot-camera",)

ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")
HEADER_RE = re.compile(r"^skybeta doctor -- (.+?)\s+host=(\S+)\s+image=(\S+)$")
CHECK_RE = re.compile(r"^\s+(PASS|WARN|FAIL)\s+(\S+)\s*(.*)$")
TRAILER_RE = re.compile(r"^doctor: (\d+) PASS\s+(\d+) WARN\s+(\d+) FAIL$")


def parse_doctor(text, exit_code=None):
    """doctor.sh stdout (ANSI or not) -> the /status ``doctor`` fields (pure)."""
    out = {"exit_code": exit_code, "header": None, "date": None, "host": None, "image": None,
           "pass": None, "warn": None, "fail": None, "ok": None, "checks": [],
           "unparsed_lines": []}
    trailer = None
    for raw in ANSI_RE.sub("", text).splitlines():
        line = raw.rstrip()
        if not line.strip():
            continue
        m = HEADER_RE.match(line)
        if m:
            out["header"] = line
            out["date"], out["host"], out["image"] = m.groups()
            continue
        m = CHECK_RE.match(line)
        if m:
            out["checks"].append({"name": m.group(2), "result": m.group(1),
                                  "detail": m.group(3).strip()})
            continue
        m = TRAILER_RE.match(line)
        if m:
            trailer = tuple(int(x) for x in m.groups())
            continue
        out["unparsed_lines"].append(line)
    counted = tuple(sum(1 for c in out["checks"] if c["result"] == r) for r in ("PASS", "WARN", "FAIL"))
    if trailer is None:
        out["unparsed_lines"].append("(no 'doctor: N PASS ...' trailer; counts are from the parsed checks)")
        trailer = counted
    elif trailer != counted:
        out["unparsed_lines"].append("(trailer says %d/%d/%d but %d/%d/%d checks parsed)" % (trailer + counted))
    out["pass"], out["warn"], out["fail"] = trailer
    if exit_code is not None:
        out["ok"] = exit_code == 0
    else:
        out["ok"] = out["fail"] == 0
    return out


def parse_doctor_json(text, exit_code=None):
    """``doctor.sh --json`` stdout -> the same fields as parse_doctor, or None if it is not
    doctor JSON (an older doctor.sh that rejects the flag)."""
    try:
        d = json.loads(text)
    except ValueError:
        return None
    if not isinstance(d, dict) or d.get("schema") != 1 or not isinstance(d.get("checks"), list):
        return None
    out = {"exit_code": exit_code, "header": None, "date": d.get("date"), "host": d.get("host"),
           "image": d.get("image"), "pass": d.get("pass"), "warn": d.get("warn"),
           "fail": d.get("fail"), "ok": None, "checks": [], "unparsed_lines": []}
    out["header"] = "skybeta doctor -- %s  host=%s  image=%s" % (out["date"], out["host"], out["image"])
    for c in d["checks"]:
        if isinstance(c, dict) and c.get("result") in ("PASS", "WARN", "FAIL"):
            out["checks"].append({"name": str(c.get("name")), "result": c["result"],
                                  "detail": str(c.get("detail") or "")})
        else:
            out["unparsed_lines"].append("(bad check entry: %r)" % (c,))
    counted = tuple(sum(1 for c in out["checks"] if c["result"] == r) for r in ("PASS", "WARN", "FAIL"))
    if (out["pass"], out["warn"], out["fail"]) != counted:
        out["unparsed_lines"].append("(json says %s/%s/%s but %d/%d/%d checks parsed)"
                                     % ((out["pass"], out["warn"], out["fail"]) + counted))
    out["ok"] = (exit_code == 0) if exit_code is not None else bool(d.get("ok"))
    return out


def doctor_env():
    """Minimal environment for the doctor: no team variables leak in."""
    env = {"LANG": "C", "TERM": "dumb"}
    for k in ("HOME", "PATH", "USER", "LOGNAME"):
        if os.environ.get(k):
            env[k] = os.environ[k]
    env.setdefault("PATH", "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin")
    # Which drone.env to read is the agent's decision, not the team's shell's.
    if os.environ.get("SKYBETA_DRONE_ENV"):
        env["SKYBETA_DRONE_ENV"] = os.environ["SKYBETA_DRONE_ENV"]
    return env


def run_doctor(repo, args=DOCTOR_ARGS, timeout=45.0):
    """Run doctor.sh once; returns the parsed dict plus args/duration/ran_at."""
    for a in args:
        if a in FORBIDDEN_ARGS:
            raise ValueError("refusing to run doctor with %s" % a)
    script = os.path.join(repo, "host", "doctor.sh")
    if not os.path.isfile(script):
        raise FileNotFoundError("%s not found" % script)
    t0 = time.monotonic()
    ran_at = time.time()
    rc, out, _ = run_cmd(["bash", script, "--json"] + list(args), timeout, env=doctor_env(),
                         cwd=repo, check=False)
    result, fmt = parse_doctor_json(out, rc), "json"
    if result is None:  # older doctor.sh: "unknown flag: --json", exit 2
        rc, out, _ = run_cmd(["bash", script] + list(args), timeout, env=doctor_env(), cwd=repo,
                             check=False)
        result, fmt = parse_doctor(out, rc), "text"
    result.update({"args": list(args), "format": fmt,
                   "duration_s": round(time.monotonic() - t0, 2), "ran_at": ran_at})
    return result


class DoctorCollector(Collector):
    name = "doctor"
    MIN_PERIOD_S = 20.0
    EMPTY = {"args": list(DOCTOR_ARGS), "exit_code": None, "duration_s": None, "ran_at": None,
             "header": None, "pass": None, "warn": None, "fail": None, "ok": None,
             "checks": [], "unparsed_lines": []}

    def __init__(self, ctx):
        super().__init__(ctx)
        self.period_s = max(self.MIN_PERIOD_S, float(ctx.doctor_period_s))

    def collect(self):
        res = run_doctor(self.ctx.repo_dir)
        for k in ("date", "host", "image"):
            res.pop(k, None)
        return res
