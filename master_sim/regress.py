#!/usr/bin/env python3
"""Run master-sim cases for several laps each and tabulate them: a regression check, or the numbers
behind a decision.

  master_sim/regress.py                         # every case below, 3 laps each
  master_sim/regress.py --laps 5 default gate_b tube_carry
  master_sim/regress.py --list

Each lap is one master_sim/run_master.sh run (a fresh sim), about 4 minutes. Results go to
gazebo_sim/runs/regress-<time>/: results.json after every lap, and table.md at the end (also printed).
A lap counts as a pass when the drone landed on the pad (first touchdown within 20 cm of its centre)
with no collision.
"""
import argparse
import json
import os
import re
import statistics
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(os.path.dirname(HERE), "gazebo_sim", "runs")
GATE_B = "gazebo_sim/course/route_centreline_gate_b.json"

# name: (what it answers, extra environment, run_master.sh arguments)
CASES = {
    "default": ("navless_nav as the drone launches it (centreline window route)", {}, []),
    "gate_b": ("the centreline Gate B route instead of the window", {}, ["--route", GATE_B]),
    "tube_freeze": ("dark tube, VIO stops, the node's default (carrot freezes)",
                    {"DH_TUBE_BLACKOUT": "stop"}, []),
    "tube_carry": ("dark tube, VIO stops, blind_carry on",
                   {"DH_TUBE_BLACKOUT": "stop"}, ["--", "-p", "blind_carry:=true"]),
    "tube_carry_late": ("as tube_carry, tracking back 1 s after the exit",
                        {"DH_TUBE_BLACKOUT": "stop", "DH_TUBE_RECOVER_DELAY": "1.0"},
                        ["--", "-p", "blind_carry:=true"]),
    "tube_carry_jump": ("as tube_carry, odometry jumps 15 cm sideways when tracking comes back",
                        {"DH_TUBE_BLACKOUT": "stop", "DH_TUBE_RESET_JUMP": "0,0.15,0"},
                        ["--", "-p", "blind_carry:=true"]),
    "bearing_+1": ("the node believes the nose is 1 deg clockwise of where it is (placement error)",
                   {}, ["--", "-p", "start_bearing_deg:=1.0"]),
    "bearing_-1": ("... 1 deg anticlockwise", {}, ["--", "-p", "start_bearing_deg:=-1.0"]),
    "bearing_+2": ("... 2 deg clockwise", {}, ["--", "-p", "start_bearing_deg:=2.0"]),
    "bearing_-2": ("... 2 deg anticlockwise", {}, ["--", "-p", "start_bearing_deg:=-2.0"]),
    "planner": ("Favour's planner, px4_bridge_node --skybeta-mode", {}, ["--brain", "planner"]),
}


def run_lap(name, lap_timeout_s):
    _, env, args = CASES[name]
    e = dict(os.environ, **env)
    t0 = time.time()
    try:
        p = subprocess.run(["bash", os.path.join(HERE, "run_master.sh"), *args], env=e, capture_output=True,
                           text=True, timeout=lap_timeout_s)
        out = p.stdout + p.stderr
    except subprocess.TimeoutExpired as exc:
        subprocess.run(["bash", os.path.join(HERE, "stop_master.sh")], capture_output=True)
        return {"error": f"lap timed out after {lap_timeout_s} s", "wall_s": round(time.time() - t0)}
    m = re.findall(r"run saved to (\S+)", out)
    if not m:
        return {"error": "no summary: " + " | ".join(out.strip().splitlines()[-3:]), "wall_s": round(time.time() - t0)}
    with open(os.path.join(m[-1], "summary.json")) as f:
        s = json.load(f)
    td = s["touchdowns"][0] if s["touchdowns"] else None
    loc = (s.get("perception") or {}).get("localize_err_xy_cm") or {}
    cp = s["checkpoints_s"]
    collision = s["result"].startswith("COLLISION")
    return {
        "run": os.path.basename(m[-1]),
        "result": s["result"],
        "pass": bool(td and td["from_pad_cm"] <= 20.0 and not collision and "LANDED" in s["result"]),
        "pad_cm": td["from_pad_cm"] if td else None,
        "touchdown_s": td["t"] if td else None,
        "min_clearance_cm": s["min_clearance_cm"],
        "min_clearance_at": s["min_clearance_at"],
        "through": "window" if cp.get("window") is not None else "gate_b" if cp.get("gate_b") is not None else "-",
        "max_tilt_deg": s["max_tilt_deg"],
        "blind_s": [p["blind_s"] for p in (s.get("tube_blackout") or {}).get("passes", [])],
        "localize_median_cm": loc.get("median"),
        "wall_s": round(time.time() - t0),
    }


def spread(values, fmt="{:.1f}"):
    v = [x for x in values if x is not None]
    if not v:
        return "-"
    if len(v) == 1:
        return fmt.format(v[0])
    return f"{fmt.format(statistics.median(v))} ({fmt.format(min(v))}–{fmt.format(max(v))})"


def table(results):
    lines = ["| case | pass | pad cm | min clearance cm | tightest at | touchdown s | via | results |",
             "|---|---|---|---|---|---|---|---|"]
    for name, laps in results.items():
        ok = [l for l in laps if "error" not in l]
        at = sorted({l["min_clearance_at"] for l in ok if l["min_clearance_at"]})
        via = sorted({l["through"] for l in ok})
        res = sorted({l.get("result", l.get("error", "?"))[:40] for l in laps})
        lines.append(f"| {name} | {sum(l.get('pass', False) for l in laps)}/{len(laps)} "
                     f"| {spread([l['pad_cm'] for l in ok])} | {spread([l['min_clearance_cm'] for l in ok])} "
                     f"| {', '.join(at) or '-'} | {spread([l['touchdown_s'] for l in ok])} | {', '.join(via)} "
                     f"| {'; '.join(res)} |")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cases", nargs="*", help="case names (default: all)")
    ap.add_argument("--laps", type=int, default=3)
    ap.add_argument("--lap-timeout", type=int, default=900, help="wall seconds before a lap is abandoned")
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()
    if a.list:
        for name, (why, env, args) in CASES.items():
            print(f"{name:16s} {why}")
        return
    names = a.cases or list(CASES)
    unknown = [n for n in names if n not in CASES]
    if unknown:
        sys.exit(f"unknown case(s) {unknown}; --list shows them")
    out = os.path.join(RUNS, "regress-" + time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(out, exist_ok=True)
    # safe.directory: from WSL the repo on /mnt/c belongs to the Windows user, and git won't read it otherwise
    head = subprocess.run(["git", "-c", f"safe.directory={os.path.dirname(HERE)}", "-C", HERE, "log",
                           "--oneline", "-1"], capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(["git", "-c", f"safe.directory={os.path.dirname(HERE)}", "-C", HERE, "status",
                            "--porcelain", "--untracked-files=no"], capture_output=True, text=True).stdout.strip()
    head += " + uncommitted changes" if dirty else ""
    results = {n: [] for n in names}
    boot_refusals = {}
    for lap in range(1, a.laps + 1):              # lap-major: a cut-short run still covers every case
        for name in names:
            print(f"[regress] lap {lap}/{a.laps} {name} ...", flush=True)
            r = run_lap(name, a.lap_timeout)
            # PX4's height can boot off by more than the pre-arm limit; the pilot then (rightly) never arms.
            # That is the sim's boot, not the case: count it and fly the lap again on a fresh boot.
            for _ in range(2):
                if "height never settled" not in r.get("result", ""):
                    break
                boot_refusals[name] = boot_refusals.get(name, 0) + 1
                print("[regress]   boot refusal (PX4 height off at power-up): retrying on a fresh sim", flush=True)
                r = run_lap(name, a.lap_timeout)
            results[name].append(r)
            print(f"[regress]   {r.get('result', r.get('error'))}  pad {r.get('pad_cm')} cm  "
                  f"min {r.get('min_clearance_cm')} cm at {r.get('min_clearance_at')}", flush=True)
            with open(os.path.join(out, "results.json"), "w") as f:
                json.dump({"code": head, "laps": a.laps, "cases": {n: CASES[n][0] for n in names},
                           "boot_refusals": boot_refusals, "results": results}, f, indent=2)
    md = f"Code: `{head}`, {a.laps} laps per case. Median (min–max).\n\n" + table(results) + "\n"
    if boot_refusals:
        md += (f"\nBoot refusals (PX4's height off at power-up, so the pre-arm check held; lap re-flown): "
               f"{sum(boot_refusals.values())}, in {', '.join(f'{k} x{v}' for k, v in boot_refusals.items())}\n")
    with open(os.path.join(out, "table.md"), "w") as f:
        f.write(md)
    print(md)
    print(f"[regress] saved to {out}")


if __name__ == "__main__":
    main()
