"""Bounded numerical covariance canonicalization; never edits a pose or clock.

A zero covariance remains unknown confidence. Materially invalid covariance is
rejected; only errors inside a double-precision roundoff bound can be projected
onto the PSD cone. This is a staged interface candidate, not flight approval.
"""
from copy import deepcopy
import numpy as np

EPS = np.finfo(np.float64).eps
ABSOLUTE_CAP = 1e-12


def canonicalize(values):
    a = np.asarray(values, dtype=np.float64)
    if a.size != 36 or not np.all(np.isfinite(a)):
        return None, {"status": "reject", "reason": "nonfinite or non-6x6 covariance"}
    a = a.reshape(6, 6)
    if np.all(np.diag(a) == 1):
        return None, {"status": "reject", "reason": "original cuVSLAM identity-unavailable sentinel"}
    tolerance = min(ABSOLUTE_CAP, 64 * 6 * EPS * max(1., float(np.linalg.norm(a, ord=np.inf))))
    symmetry_error = float(np.max(np.abs(a-a.T)))
    if symmetry_error > tolerance:
        return None, {"status": "reject", "reason": "material asymmetry", "tolerance": tolerance}
    symmetric = (a+a.T)/2
    eigenvalues, eigenvectors = np.linalg.eigh(symmetric)
    minimum = float(eigenvalues[0])
    info = {"tolerance": tolerance, "minimum_eigenvalue": minimum,
            "symmetry_error": symmetry_error, "unknown_confidence": False}
    if minimum < -tolerance:
        return None, {**info, "status": "reject", "reason": "material negative eigenvalue"}
    if np.max(np.abs(symmetric)) <= tolerance:
        output = np.zeros((6, 6))
        info.update(status="unknown_zero", unknown_confidence=True)
    elif minimum < 0 or symmetry_error:
        output = (eigenvectors * np.maximum(eigenvalues, 0)) @ eigenvectors.T
        output = (output+output.T)/2
        # Projection can itself leave a negative signed rounding residue at zero.
        output[np.diag_indices(6)] = np.maximum(np.diag(output), 0)
        info["status"] = "roundoff_projected"
    else:
        output = a.copy()
        info["status"] = "unchanged"
    change = float(np.max(np.abs(output-a)))
    if change > tolerance:
        return None, {**info, "status": "reject", "reason": "correction exceeds bound", "max_change": change}
    info["max_change"] = change
    return output.ravel().tolist(), info


def canonicalize_message(msg):
    """Copy covariance only; retain exact source pose/twist/frames/timestamps."""
    pose, pose_info = canonicalize(msg.pose.covariance)
    twist, twist_info = canonicalize(msg.twist.covariance)
    report = {"pose": pose_info, "twist": twist_info}
    if pose is None or twist is None:
        return None, report
    candidate = deepcopy(msg)
    candidate.pose.covariance = pose
    candidate.twist.covariance = twist
    return candidate, report
