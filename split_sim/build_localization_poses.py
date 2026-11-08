#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = ["pyulog==1.3.0"]
# ///
"""Build passive localization website data from recorded source poses; no truth fitting.
Requires pyulog,numpy. Run directories are input; no ROS/network/control imports.
"""

import argparse, bisect, json, math, re, tarfile, time, hashlib, signal, sys, os, fcntl
from pathlib import Path

_HASH_CACHE = {}
_TAR_CACHE = {}
_JSONL_CACHE = {}


def digest(path, audit):
    stat = path.stat()
    if not audit:
        return {
            "size_bytes": stat.st_size,
            "mtime_ns": stat.st_mtime_ns,
            "hash_status": "deferred_until_sealed_audit",
        }
    key = (str(path), stat.st_size, stat.st_mtime_ns)
    if key not in _HASH_CACHE:
        _HASH_CACHE[key] = hashlib.sha256(path.read_bytes()).hexdigest()
    return {"sha256": _HASH_CACHE[key]}


def jsonl_rows(path):
    """Incremental source-tail cache; atomic mirror replacement retains prefix."""
    stat = path.stat()
    key = str(path)
    cache = _JSONL_CACHE.get(key)
    with path.open("rb") as head:
        prefix = head.read(128)
    if cache is None or stat.st_size < cache["offset"] or prefix != cache["prefix"]:
        cache = {"offset": 0, "rows": [], "prefix": prefix}
        _JSONL_CACHE[key] = cache
    with path.open("rb") as stream:
        stream.seek(cache["offset"])
        while True:
            offset = stream.tell()
            line = stream.readline()
            if not line:
                break
            if not line.endswith(b"\n"):
                stream.seek(offset)
                break
            try:
                cache["rows"].append(json.loads(line))
            except (ValueError, UnicodeError):
                pass
            cache["offset"] = stream.tell()
    return cache["rows"]


def clean(x):
    if isinstance(x, float) and not math.isfinite(x):
        return None
    if isinstance(x, dict):
        return {k: clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [clean(v) for v in x]
    return x


def yaw(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def build(p, hz=10):
    p = Path(p).resolve()
    f = p / "flight"
    sealed = (f / "grade.json").exists()
    log = (f / "mission.log").read_text()
    route = json.loads((f / "planned-route.json").read_text())
    settings = (
        json.loads((p / "settings.json").read_text())
        if (p / "settings.json").exists()
        else {}
    )
    bearing = settings.get("start_bearing_deg")
    origin = route.get("launch_origin_enu")
    height = None
    runtime = p / "flown-runtime.tgz"
    provenance = {}
    if runtime.exists():
        fingerprint = (str(runtime), runtime.stat().st_size, runtime.stat().st_mtime_ns)
        if fingerprint not in _TAR_CACHE:
            result = (None, None)
            with tarfile.open(runtime) as tar:
                names = tar.getnames()
                name = next((n for n in names if n.endswith("/mission.py")), None)
                if name:
                    code = tar.extractfile(name).read().decode()
                    m = re.search(r"--start-height['\"].*?default=([\d.]+)", code)
                    if m:
                        result = (
                            float(m.group(1)),
                            f"{runtime.name}:{name} --start-height default",
                        )
            _TAR_CACHE[fingerprint] = result
        height, height_source = _TAR_CACHE[fingerprint]
        if height_source:
            provenance["launch_height"] = height_source
    # Verify the archived invocation uses that default; never infer launch height from truth.
    call = (
        (p / "run_split_course.sh").read_text()
        if (p / "run_split_course.sh").exists()
        else ""
    )
    if height is not None and not (
        call and re.search(r"python3 .*course_runtime/mission.py", call)
    ):
        height = None
    if height is not None:
        provenance["launch_height"] += (
            "; archived run_split_course.sh invokes mission.py without height override"
        )
    harg = re.search(r"--start-height\s+([\d.]+)", call)
    if harg:
        height = float(harg.group(1))
        provenance["launch_height"] = (
            "archived run_split_course.sh explicit --start-height"
        )
    bm = re.search(r"qualified split control bearing ([\d.-]+)deg", log)
    dm = re.search(
        r"froze measured landed PX4 minus VO translation NED \(([^)]+)\)", log
    )
    control = float(bm.group(1)) if bm else None
    datum = [float(v.strip()) for v in dm.group(1).split(",")] if dm else None
    leg = (
        (p / "mission-leg").read_text().strip()
        if (p / "mission-leg").exists()
        else settings.get("mission_leg", "outbound")
    )
    return_matrix = None
    if leg.startswith("return"):
        manifest = p / "return-manifest.json"
        if manifest.exists():
            segment = json.loads(manifest.read_text()).get("segment_registration", {})
            matrix = segment.get("world_from_odom")
            if (
                segment.get("world_from_odom_frozen_before_takeoff") is True
                and matrix
                and len(matrix) == 4
                and all(len(row) == 4 for row in matrix)
                and all(math.isfinite(v) for row in matrix for v in row)
            ):
                return_matrix = matrix
        registered = return_matrix is not None
    else:
        registered = origin is not None and bearing is not None and height is not None

    def world(xyz, psi):
        if not registered:
            return None, None
        if return_matrix is not None:
            out = [
                sum(return_matrix[i][j] * xyz[j] for j in range(3))
                + return_matrix[i][3]
                for i in range(3)
            ]
            return out, wrap(math.atan2(return_matrix[1][0], return_matrix[0][0]) + psi)
        b = math.radians(bearing)
        x, y, z = xyz
        # Launch FLU forward/left -> declared course ENU, never derived from truth.
        return [
            origin[0] + math.sin(b) * x - math.cos(b) * y,
            origin[1] + math.cos(b) * x + math.sin(b) * y,
            height + z,
        ], wrap(math.pi / 2 - b + psi)

    def record(t, xyz, psi, nativeframe, conf, prov, worldpose=True):
        out, wy = world(xyz, psi) if worldpose else (None, None)
        return clean(
            {
                "stamp_us": round(t * 1e6),
                "xyz": out if out is not None else xyz,
                "yaw": wy if out is not None else psi,
                "frame": "course_enu" if out is not None else nativeframe,
                "native_xyz": xyz,
                "native_yaw": psi,
                "native_frame": nativeframe,
                "units": {"position": "m", "yaw": "rad", "time": "simulation_us"},
                "registration_status": "fixture_declared"
                if out is not None
                else "unregistered",
                "status": "observed",
                "confidence": conf,
                "provenance": prov,
            }
        )

    streams = {k: [] for k in ["raw_nvidia", "admitted_vo", "px4_fused", "truth"]}
    telepaths = []
    telemetry_name = (
        "telemetry.jsonl"
        if sealed and (f / "telemetry.jsonl").exists()
        else "live-telemetry.jsonl"
    )
    for name in [telemetry_name, "localization-observer.jsonl"]:
        z = f / name
        if not z.exists():
            continue
        telepaths.append(z)
        for r in jsonl_rows(z):
            k = r.get("kind")
            if k in ["vision_raw", "vision_admitted"]:
                sid = "raw_nvidia" if k == "vision_raw" else "admitted_vo"
                q = r["orientation_xyzw"]
                t = r.get("stamp_ns", r["timestamp"] * 1000) * 1e-9
                rec = record(
                    t,
                    r["position"],
                    yaw(q),
                    r.get("frame_id", "odom (recorded topic convention)"),
                    {
                        "covariance_6x6": r.get("pose_covariance_6x6"),
                        "variance_xyz": r.get("variance_xyz"),
                        "covariance_note": "zero or absent covariance is unknown confidence, never certainty",
                    },
                    {
                        "path": str(z),
                        "kind": k,
                        "header_stamp_ns": r.get("stamp_ns"),
                        "child_frame": r.get("child_frame_id"),
                        "source": "actual ROS message",
                    },
                )
                rec["confidence"]["uncertainty_frame"] = r.get(
                    "frame_id", "source native ROS frame"
                )
                rec["receipt_wall_unix_s"] = r.get("receipt_wall_unix_s")
                streams[sid].append(rec)
            elif k == "local_position":
                streams["px4_fused"].append(px4rec(r, str(z), record, control, datum))
    # Archived ULog supersedes any live position capture; exact actual fused pose fields.
    ulog_source = None
    ul = list((p / "px4").rglob("*.ulg")) if (p / "px4").exists() else []
    if ul and sealed:
        from pyulog import ULog

        source = max(ul, key=lambda z: z.stat().st_mtime)
        ulog_source = source
        u = ULog(str(source))
        a = u.get_dataset("vehicle_local_position").data
        last = -math.inf
        streams["px4_fused"] = []
        for i, ts in enumerate(a["timestamp_sample"]):
            t = float(ts) * 1e-6
            if t - last < 1 / hz:
                continue
            last = t
            r = {k: v[i].item() for k, v in a.items()}
            streams["px4_fused"].append(px4rec(r, str(source), record, control, datum))
    trpath = f / "track.json"
    track = json.loads(trpath.read_text()) if trpath.exists() else {"samples": []}
    last = -math.inf
    for r in track["samples"]:
        if r["t"] - last < 1 / hz:
            continue
        last = r["t"]
        streams["truth"].append(
            {
                "stamp_us": round(r["t"] * 1e6),
                "xyz": [r["x"], r["y"], r["z"]],
                "yaw": r["yaw"],
                "frame": "course_enu",
                "native_frame": "Gazebo world ENU",
                "units": {"position": "m", "yaw": "rad", "time": "simulation_us"},
                "registration_status": "independent_reference",
                "status": "reference_only",
                "confidence": {"armed": r.get("armed"), "landed": r.get("landed")},
                "provenance": {
                    "path": str(trpath),
                    "source": "independent physics recorder; never navigation input or estimate alignment",
                },
            }
        )
    for k in streams:
        streams[k] = sorted(
            {r["stamp_us"]: r for r in streams[k]}.values(), key=lambda r: r["stamp_us"]
        )
    alltimes = [r["stamp_us"] * 1e-6 for v in streams.values() for r in v]
    lo = track["samples"][0]["t"] if track.get("samples") else min(alltimes, default=0)
    hi = track["samples"][-1]["t"] if track.get("samples") else max(alltimes, default=0)
    idx = {k: [r["stamp_us"] for r in v] for k, v in streams.items()}
    samples = []
    for n in range(int((hi - lo) * hz) + 1):
        t = lo + n / hz
        pick = {}
        for k, arr in streams.items():
            i = bisect.bisect_right(idx[k], round(t * 1e6)) - 1
            if i < 0:
                continue
            r = dict(arr[i])
            r["age_s"] = max(0, t - r["stamp_us"] * 1e-6)
            r["status"] = "stale" if r["age_s"] > 0.5 else r["status"]
            pick[k] = r
        samples.append({"t": t, "sources": pick})
    end = track.get("samples", [])[-1] if track.get("samples") else {}
    sealed = (f / "grade.json").exists()
    result = {
        "schema_version": 1,
        "run_id": p.name,
        "generated_wall_unix_s": time.time(),
        "audit_telemetry_source": telemetry_name,
        "mode": "replay" if sealed else "live_snapshot",
        "sealed": sealed,
        "live_receipt_age_wall_s": None,
        "time": {
            "units": "simulation_seconds",
            "start": lo,
            "end": hi,
            "sample_hz": hz,
            "wall_freshness_note": "Mirror modification/receipt is not sensor receipt. Source ages are simulation sample ages; browser must separately assess refresh wall age.",
        },
        "registration": {
            "status": "fixture_declared" if registered else "partial_or_unregistered",
            "truth_used_for_alignment": False,
            "course_launch_origin_xy": origin[:2] if origin else None,
            "launch_height_m": height,
            "configured_launch_bearing_deg": bearing,
            "frozen_PX4_control_bearing_deg": control,
            "frozen_PX4_minus_VO_datum": datum,
            "source": str(f / "mission.log"),
            "height_provenance": provenance,
            "mission_leg": leg,
            "return_world_from_odom": return_matrix,
            "return_registration_note": "Return origin withheld until frozen return-manifest.json is available"
            if leg.startswith("return")
            else None,
            "warning": "Declared mission fixture coordinates, not surveyed hardware registration; no truth-based fit. Unknown origin remains unregistered.",
        },
        "sources": [
            {
                "id": k,
                "label": {
                    "raw_nvidia": "NVIDIA raw stereo VO",
                    "admitted_vo": "Bridge-admitted VO",
                    "px4_fused": "PX4 fused local position",
                    "truth": "Independent ground truth reference",
                }[k],
                "frame": "course_enu"
                if registered or k == "truth"
                else "source-native unregistered",
                "units": "m, rad",
                "origin": "declared mission fixture" if registered else "unregistered",
                "status": "available" if streams[k] else "unavailable",
                "count": len(streams[k]),
            }
            for k in streams
        ],
        "samples": samples,
        "source_files": [
            {"path": str(z), **digest(z, sealed)}
            for z in [
                f / "mission.log",
                f / "planned-route.json",
                *telepaths,
                trpath,
                runtime,
                p / "settings.json",
                p / "run_split_course.sh",
                p / "return-manifest.json",
                *([ulog_source] if ulog_source else []),
            ]
            if z.exists()
        ],
        "last_truth_sample": end,
    }
    title = (
        (p / "run-name").read_text().strip()
        if (p / "run-name").exists()
        else "Recorded localization estimates"
    )
    base = None
    change = "Derived actual source poses; display transformation only. Original flight configuration and grading remain unchanged."
    comparison_file = p / "run-comparison.json"
    if comparison_file.exists():
        comparison = json.loads(comparison_file.read_text())
        title = comparison.get("name", title)
        base = comparison.get("baseline_id", base)
        result["flight_comparison"] = {
            "source": "run-comparison.json",
            **{
                key: comparison.get(key)
                for key in (
                    "run_id",
                    "name",
                    "baseline_id",
                    "changes",
                    "notes",
                    "confounds",
                )
            },
        }
    result.update(
        {
            "title": title,
            "artifact_kind": "derived_localization_pose_comparison",
            "source_flight_ids": [p.name],
            "baseline_run_id": base,
            "what_changed": {
                "flight_configuration": change,
                "estimator_recomputed": False,
                "frame_transform": "Recorded launch FLU transformed to declared mission course ENU. PX4 floating FRD uses only frozen preflight bearing and translation from mission log. Native poses preserved.",
                "truth_role": "Independent display/reference only, never registration or navigation.",
                "known_limitations": [
                    "Synthetic simulated environment, not hardware qualification.",
                    "Source sample ages and wall mirror freshness differ.",
                    "Raw/admitted covariance may be zero/absent; unknown confidence.",
                    "Launch height from frozen mission default unless explicit archived override; mission fixture is not a surveyed hardware map.",
                    "Current PX4 unavailable when recorder has not captured local_position and ULog not archived.",
                ],
            },
        }
    )
    receipts = [
        r.get("receipt_wall_unix_s")
        for v in streams.values()
        for r in v
        if r.get("receipt_wall_unix_s") is not None
    ]
    result["last_actual_sensor_receipt_wall_unix_s"] = (
        max(receipts) if receipts else None
    )
    result["live_receipt_age_wall_s"] = (
        time.time() - max(receipts) if receipts else None
    )
    target = p / ("localization-poses.json" if sealed else "localization-live.json")
    tmp = target.with_name(
        target.name + "." + str(__import__("os").getpid()) + ".partial"
    )
    tmp.write_text(
        json.dumps(clean(result), separators=(",", ":"), allow_nan=False) + "\n"
    )
    tmp.replace(target)
    if sealed and (p / "localization-live.json").exists():
        alias = p / "localization-live.json"
        staged = alias.with_name(
            alias.name + "." + str(__import__("os").getpid()) + ".partial"
        )
        staged.write_text(target.read_text())
        staged.replace(alias)
    print(target)
    return result


def px4rec(r, path, record, control, datum):
    xyz = [r["x"], r["y"], r["z"]]
    heading = r["heading"]
    registered = control is not None and datum is not None
    if registered:
        x, y, z = [v - o for v, o in zip(xyz, datum)]
        b = math.radians(control)
        launch = [
            math.cos(b) * x + math.sin(b) * y,
            math.sin(b) * x - math.cos(b) * y,
            -z,
        ]
        psi = wrap(b - heading)
    else:
        launch = xyz
        psi = heading
    confidence = {
        k: r[k]
        for k in [
            "eph",
            "epv",
            "evh",
            "evv",
            "xy_valid",
            "z_valid",
            "v_xy_valid",
            "v_z_valid",
            "heading_good_for_control",
            "dead_reckoning",
            "xy_reset_counter",
            "z_reset_counter",
            "heading_reset_counter",
        ]
        if k in r
    }
    rec = record(
        r.get("timestamp_sample", r["timestamp"]) * 1e-6,
        launch,
        psi,
        "launch_flu" if registered else "PX4 floating FRD",
        confidence,
        {
            "path": path,
            "source": "actual vehicle_local_position",
            "delivery_stamp_us": r["timestamp"],
            "registration": "mission preflight frozen bearing/datum; no truth fit",
        },
        worldpose=registered,
    )
    rec["source_native_xyz"] = xyz
    rec["source_native_heading"] = heading
    rec["receipt_wall_unix_s"] = r.get("receipt_wall_unix_s")
    if not bool(r.get("xy_valid", False)) or not bool(r.get("z_valid", False)):
        rec["status"] = "invalid"
    rec["confidence"]["uncertainty_frame"] = "PX4 source native floating FRD"
    if not registered:
        rec["registration_status"] = "unregistered"
    return rec


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--hz", type=float, default=5)
    ap.add_argument("--watch", action="store_true")
    ap.add_argument("--interval", type=float, default=10)
    ap.add_argument("--max-updates", type=int, default=0)
    a = ap.parse_args()
    if a.watch and a.interval < 5:
        ap.error("live interval must be >=5 seconds")
    stopped = False

    def stop(*_):
        global stopped
        stopped = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    lockfile = None
    if a.watch:
        if len(a.runs) != 1:
            ap.error("one live run per worker")
        run_path = Path(a.runs[0]).resolve()
        lockfile = (run_path / ".localization-worker.lock").open("w")
        try:
            fcntl.flock(lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print("A localization worker already owns this run; no duplicate started.")
            sys.exit(0)
        worker_state = run_path / "localization-worker-state.json"
        worker_state.write_text(
            json.dumps(
                {
                    "pid": os.getpid(),
                    "status": "watching",
                    "run_id": run_path.name,
                    "started_wall_unix_s": time.time(),
                    "interval_s": a.interval,
                    "passive": True,
                }
            )
            + "\n"
        )
    count = 0
    terminal = False
    while True:
        results = []
        for run in a.runs:
            try:
                results.append(build(run, a.hz))
            except (OSError, ValueError, KeyError) as exc:
                if not a.watch:
                    raise
                print(
                    f"Localization snapshot pending: {type(exc).__name__}: {exc}",
                    file=sys.stderr,
                    flush=True,
                )
        count += 1
        terminal = bool(results) and all(result.get("sealed") for result in results)
        if (
            terminal
            or not a.watch
            or stopped
            or (a.max_updates and count >= a.max_updates)
        ):
            break
        deadline = time.monotonic() + a.interval
        while not stopped and time.monotonic() < deadline:
            time.sleep(min(0.25, max(0, deadline - time.monotonic())))
        if stopped:
            break

    if a.watch:
        worker_state.write_text(
            json.dumps(
                {
                    "pid": os.getpid(),
                    "status": "sealed" if terminal else "stopped",
                    "run_id": run_path.name,
                    "ended_wall_unix_s": time.time(),
                    "updates": count,
                    "passive": True,
                }
            )
            + "\n"
        )
