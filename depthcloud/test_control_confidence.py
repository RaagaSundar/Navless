"""Control-facing confidence policy, without ROS or live sensor publishers."""
import numpy as np
import pytest

from depthcloud.cloud_localize import (CORR_STATUSES, SourceEpoch, correction_eligible,
                                     odometry_source_policy, paired_correction_metadata)


def test_corrections_default_to_tracking_only():
    assert CORR_STATUSES == ("tracking",)


@pytest.mark.parametrize("status", [None, False, True])
def test_required_gate_never_bypasses_to_raw_tf(status):
    row, why = odometry_source_policy([], 1_000_000_000, 1.02, status, 0.01)
    assert row is None and why != "tf"


@pytest.mark.parametrize("tracking,age", [(None, 0), (False, 0), (True, .26), (True, -1)])
def test_even_observer_fallback_requires_fresh_positive_tracking(tracking, age):
    row, why = odometry_source_policy([], 1_000_000_000, 1.02, tracking, age, False)
    assert row is None and why != "tf"


def test_gate_capture_time_not_receipt_time_selects_sample():
    row = (1_000_000_000, np.eye(4))
    assert odometry_source_policy([row], 1_010_000_000, 1.02, True, .01)[0] is row
    assert odometry_source_policy([row], 1_010_000_000, 1.30, True, .01)[0] is None
    assert odometry_source_policy([], 1_000_000_000, 1.02, True, .01, False)[1] == "tf"


def test_source_regression_quarantines_until_explicit_restart():
    clock = SourceEpoch()
    assert clock.observe(100)[0]
    assert not clock.observe(100)[0] and clock.epoch == 0
    assert not clock.observe(99)[0] and clock.epoch == 1
    assert not clock.observe(101)[0] and clock.fault


@pytest.mark.parametrize("status,measured,age,odom,fault", [
    ("weak", dict(x=1, y=1, heading=1), .01, True, False),
    ("odometry", dict(x=1, y=1, heading=1), .01, True, False),
    ("tracking", dict(x=1, y=1, heading=.3), .01, True, False),
    ("tracking", dict(x=1, y=1, heading=1), .26, True, False),
    ("tracking", dict(x=1, y=1, heading=1), -.006, True, False),
    ("tracking", dict(x=1, y=1, heading=1), .01, False, False),
    ("tracking", dict(x=1, y=1, heading=1), .01, True, True),
])
def test_partial_old_future_or_untrusted_matches_cannot_steer(status, measured, age, odom, fault):
    assert not correction_eligible(status, measured, 1_000_000_000, 1+age, odom, fault)[0]


def test_fresh_fully_observed_planar_match_can_qualify():
    assert correction_eligible("tracking", dict(x=1, y=1, heading=1), 1_000_000_000, 1.01, True)[0]


@pytest.mark.parametrize("measured", [None, {}, dict(x=1,y=1,heading=None), dict(x=1,y=1,heading=float('nan'))])
def test_missing_or_nonfinite_observability_is_not_a_measurement(measured):
    assert not correction_eligible("tracking", measured, 1_000_000_000, 1.01, True)[0]


def test_old_correction_keeps_original_stamp_and_cannot_be_refreshed_by_new_cloud():
    last = dict(correction_source_stamp_ns=100, correction_source_stamp_s=1e-7,
                correction_epoch=2, correction_status="tracking", correction_measured=dict(x=1,y=1,heading=1))
    assert paired_correction_metadata(last, 100, 2, True)["correction_valid"]
    old = paired_correction_metadata(last, 101, 2, False)
    assert not old["correction_valid"] and old["correction_source_stamp_ns"] == 100
    assert not paired_correction_metadata(last, 100, 3, True)["correction_valid"]
