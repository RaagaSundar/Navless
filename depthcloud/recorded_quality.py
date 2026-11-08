#!/usr/bin/env python3
"""Audit existing localizer NPZ frame dumps. Does not run sensors, ICP or a flight.

Accuracy is unknown unless an independently evidenced reference JSON is supplied:
{"frame_id":"course", "pose_frame":"camera_optical"|"base_link", "evidence":"...",
 "samples":[{"stamp":123.0, "T":[[...],[...],[...],[...]]}]}
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def summary(values):
    return ({"count":len(values), "median":float(np.median(values)), "max":float(max(values))}
            if values else None)


def pose(value):
    value = np.asarray(value, float)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise ValueError("pose must be a finite 4x4 transform")
    if not np.allclose(value[3], [0, 0, 0, 1], atol=1e-6):
        raise ValueError("invalid homogeneous transform")
    R = value[:3,:3]
    if not np.allclose(R.T@R, np.eye(3), atol=1e-3) or not np.isclose(np.linalg.det(R),1,atol=1e-3):
        raise ValueError("pose rotation is not a proper orthonormal matrix")
    return value


def audit(directory, *, name, source_run_id, reference=None, match_tolerance_s=.06):
    files = sorted(Path(directory).glob("*.npz"))
    if not files:
        raise ValueError("no recorded NPZ frames found")
    ref = None
    if reference:
        ref = json.loads(Path(reference).read_text(encoding="utf-8"))
        if (ref.get("frame_id") != "course" or ref.get("pose_frame") not in ("camera_optical", "base_link")
                or not ref.get("evidence") or not ref.get("samples")):
            raise ValueError("reference needs course frame, pose_frame, evidence and nonempty samples")
        for sample in ref["samples"]:
            if not math.isfinite(float(sample["stamp"])):
                raise ValueError("nonfinite reference stamp")
            sample["T"] = pose(sample["T"])
    statuses, errors, xy_errors, angle_errors = {}, [], [], []
    fits, matches, source_ages, stamps, rows = [], [], [], [], []
    fully_observed = 0
    for path in files:
        row = {"file":path.name, "sha256":hashlib.sha256(path.read_bytes()).hexdigest()}
        try:
            with np.load(path, allow_pickle=False) as data:
                meta = json.loads(str(data["meta"].item()))
                estimate = pose(data["T"])
                stamp = float(meta["stamp"])
                if not math.isfinite(stamp) or stamp <= 0:
                    raise ValueError("invalid capture timestamp")
                status, info = str(meta.get("status", "unknown")), meta.get("info") or {}
                statuses[status] = statuses.get(status, 0)+1
                stamps.append(stamp)
                observed = all(float((info.get("proj") or {}).get(k, 0)) > .9
                               for k in ("x", "y", "heading"))
                fully_observed += int(status == "tracking" and observed)
                row.update(stamp=stamp, status=status, planar_observed=observed,
                           odom_source=meta.get("odom_source", "unknown"), source_epoch=meta.get("source_epoch"))
                for key, target in (("fitness", fits), ("matches", matches)):
                    v = info.get(key)
                    if isinstance(v, (int,float)) and math.isfinite(v): target.append(v)
                age = meta.get("source_age_s")
                if isinstance(age,(int,float)) and math.isfinite(age): source_ages.append(age)
                if ref:
                    sample = min(ref["samples"],key=lambda s:abs(float(s["stamp"])-stamp))
                    if abs(float(sample["stamp"])-stamp) <= match_tolerance_s:
                        if ref["pose_frame"] == "base_link":
                            estimate = estimate@np.linalg.inv(pose(data["T_base_cam"]))
                        delta = estimate[:3,3]-sample["T"][:3,3]
                        xy_errors.append(float(np.linalg.norm(delta[:2])))
                        errors.append(float(np.linalg.norm(delta)))
                        trace = np.trace(estimate[:3,:3]@sample["T"][:3,:3].T)
                        angle_errors.append(math.degrees(math.acos(float(np.clip((trace-1)/2,-1,1)))))
                        row["reference_stamp"] = float(sample["stamp"])
                    else: row["reference_match"] = "unknown: no close source timestamp"
        except (OSError, ValueError, KeyError, TypeError, np.linalg.LinAlgError) as exc:
            row["invalid"] = str(exc)
        rows.append(row)
    return {"schema_version":1, "name":name, "source_run_id":source_run_id,
            "analysis_kind":"recorded_frame_quality_audit", "new_flight_attempt":False,
            "flight_qualification":"not established by recorded-frame audit",
            "frames":len(files), "valid_frames":sum("invalid" not in r for r in rows),
            "invalid_frames":sum("invalid" in r for r in rows),
            "statuses":statuses, "tracking_with_observed_xy_heading":fully_observed,
            "fitness":summary(fits), "matches":summary(matches), "capture_age_s":summary(source_ages),
            "nonmonotonic_capture_steps":sum(b <= a for a,b in zip(stamps,stamps[1:])),
            "source_capture_gap_s":summary([b-a for a,b in zip(stamps,stamps[1:]) if b>a]),
            "reference":({"file":str(reference), "evidence":ref["evidence"], "pose_frame":ref["pose_frame"],
                          "sha256":hashlib.sha256(Path(reference).read_bytes()).hexdigest()} if ref else None),
            "accuracy_status":"reference compared" if errors else "unknown: no matched independent reference",
            "position_error_xy_m":summary(xy_errors), "position_error_3d_m":summary(errors),
            "orientation_error_deg":summary(angle_errors), "frame_evidence":rows}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("directory",type=Path)
    ap.add_argument("--name",required=True)
    ap.add_argument("--source-run-id",required=True)
    ap.add_argument("--reference",type=Path)
    ap.add_argument("--output",type=Path,required=True)
    args = ap.parse_args()
    report = audit(args.directory, name=args.name, source_run_id=args.source_run_id, reference=args.reference)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    print(f"Audited {report['frames']} recorded frames; accuracy: {report['accuracy_status']}")


if __name__ == "__main__":
    main()
