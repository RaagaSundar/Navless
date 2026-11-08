"""Synthetic file-format fixtures test the auditor, never claim a successful flight."""
import json
import numpy as np
import pytest

from depthcloud.recorded_quality import audit


def frame(path, stamp=1):
    np.savez(path, T=np.eye(4), T_base_cam=np.eye(4),
             meta=json.dumps(dict(stamp=stamp,status="weak",info=dict(fitness=.8,matches=187))))


def test_no_reference_means_no_invented_accuracy_or_flight_pass(tmp_path):
    frame(tmp_path/"frame.npz")
    out = audit(tmp_path,name="Format fixture",source_run_id="unit-test-fixture")
    assert out["position_error_xy_m"] is None and out["orientation_error_deg"] is None
    assert out["accuracy_status"].startswith("unknown")
    assert out["tracking_with_observed_xy_heading"] == 0 and not out["new_flight_attempt"]
    assert out["capture_age_s"] is None


def test_reference_must_be_explicitly_framed_and_evidenced(tmp_path):
    frame(tmp_path/"frame.npz")
    ref = tmp_path/"ref.json"
    ref.write_text(json.dumps(dict(samples=[dict(stamp=1,T=np.eye(4).tolist())])))
    with pytest.raises(ValueError):audit(tmp_path,name="Fixture",source_run_id="fixture",reference=ref)


def test_only_timestamp_matched_independent_reference_produces_error(tmp_path):
    frame(tmp_path/"frame.npz")
    T = np.eye(4);T[0,3]=.2
    ref = tmp_path/"ref.json"
    ref.write_text(json.dumps(dict(frame_id="course",pose_frame="base_link",evidence="unit test fixture only",
                                  samples=[dict(stamp=1,T=T.tolist())])))
    out = audit(tmp_path,name="Fixture",source_run_id="fixture",reference=ref)
    assert out["position_error_xy_m"]["median"] == pytest.approx(.2)
    assert out["position_error_xy_m"]["count"] == 1
    frame(tmp_path/"frame.npz",stamp=2)
    out = audit(tmp_path,name="Fixture",source_run_id="fixture",reference=ref)
    assert out["position_error_xy_m"] is None


def test_corrupt_record_is_reported_not_silently_dropped(tmp_path):
    (tmp_path/"bad.npz").write_bytes(b"not an NPZ")
    out = audit(tmp_path,name="Fixture",source_run_id="fixture")
    assert out["invalid_frames"] == 1 and out["valid_frames"] == 0
