"""Offline planar SLAM + associated floor-line covariance intersection.

No ROS, PX4 publisher, sensor driver, map lookup or recursive prediction.
Each input prior must be NEW RAW validated continuous SLAM (never our output).
State order: x metres, y metres, yaw radians, in a registered ROS odom frame.
"""
from dataclasses import dataclass
import math

import numpy as np

from reference import LineReference


@dataclass(frozen=True)
class SlamSample:
    stamp_s: float
    status_stamp_s: float
    state: tuple
    covariance: tuple  # full 3x3 x/y/yaw covariance, including cross terms
    frame: str
    reset_id: str
    tracking: bool
    covariance_available: bool


@dataclass(frozen=True)
class MapSegment:
    segment_id: str
    start_xy: tuple
    end_xy: tuple
    frame: str
    reset_id: str  # registration epoch; invalidated by SLAM origin/yaw resets


@dataclass(frozen=True)
class LineSample:
    reference: LineReference  # from selected rectified pixels
    covariance: tuple  # 2x2: signed offset (m), relative heading (rad)
    segment_id: str
    association_validated: bool
    sensors_validated: bool  # capture-time image/height/attitude/TF/calibration


@dataclass(frozen=True)
class FusionResult:
    mode: str  # fused, slam_only, unavailable
    reason: str
    state: tuple | None
    covariance: tuple | None
    stamp_s: float


def _covariance(value, size):
    try:
        a = np.array(value, dtype=float)
        if (a.shape != (size, size) or not np.isfinite(a).all()
                or not np.allclose(a, a.T, atol=1e-12, rtol=0)):
            return None
        np.linalg.cholesky(a)
        return a
    except (TypeError, ValueError, np.linalg.LinAlgError):
        return None


def _wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _tuple_matrix(a):
    return tuple(tuple(float(v) for v in row) for row in a)


class FusionSession:
    """One frame/registration epoch; instantiate again after a reviewed reset.

    Conservative fixed CI weight avoids claiming independence of shared-camera
    errors. Input covariances must bound SLAM error and total line error,
    including segmentation, calibration, height, attitude and map registration.
    Fit RMS alone is NOT a measurement covariance. Defaults are unqualified.
    """
    def __init__(self, frame, reset_id, slam_weight=0.75):
        if not frame or not reset_id or not 0 < slam_weight < 1:
            raise ValueError('frame, reset epoch and CI weight in (0,1) required')
        self.frame, self.reset_id, self.weight = frame, reset_id, slam_weight
        self.last_slam_s = self.last_line_s = -math.inf

    def fuse(self, slam, line, segment, now_s):
        """Process one synchronized pair; reject data instead of clipping fixes.

        Healthy SLAM with unusable line -> raw SLAM; unhealthy SLAM -> no pose.
        No recovery of lost SLAM from a single floor line is attempted.
        A line image and a SLAM prior can each be consumed at most once.
        """
        def unavailable(reason):
            return FusionResult('unavailable', reason, None, None, slam.stamp_s)

        times = (now_s, slam.stamp_s, slam.status_stamp_s)
        if (not all(math.isfinite(v) for v in times)
                or not 0 <= now_s - slam.stamp_s <= 0.15
                or not 0 <= now_s - slam.status_stamp_s <= 0.25):
            return unavailable('stale_or_future_slam')
        if (not slam.tracking or not slam.covariance_available
                or slam.frame != self.frame or slam.reset_id != self.reset_id):
            return unavailable('slam_health_frame_or_reset')
        p = _covariance(slam.covariance, 3)
        x = np.asarray(slam.state, dtype=float)
        if p is None or x.shape != (3,) or not np.isfinite(x).all():
            return unavailable('invalid_slam_state_or_covariance')
        if slam.stamp_s <= self.last_slam_s:
            return unavailable('repeated_or_out_of_order_slam')
        self.last_slam_s = slam.stamp_s

        def fallback(reason):
            return FusionResult('slam_only', reason, tuple(float(v) for v in x),
                                _tuple_matrix(p), slam.stamp_s)

        if line is None:
            return fallback('line_missing')
        if (segment is None or not line.association_validated or not line.sensors_validated
                or line.segment_id != segment.segment_id or not segment.segment_id
                or segment.frame != self.frame or segment.reset_id != self.reset_id):
            return fallback('line_association_frame_or_reset')
        ref = line.reference
        if not isinstance(ref, LineReference):
            return fallback('line_missing')
        if (not math.isfinite(ref.stamp_s) or not 0 <= now_s - ref.stamp_s <= 0.15
                or abs(slam.stamp_s - ref.stamp_s) > 0.03):
            return fallback('line_age_or_time_skew')
        if ref.stamp_s <= self.last_line_s:
            return fallback('repeated_or_out_of_order_line')
        self.last_line_s = ref.stamp_s
        r = _covariance(line.covariance, 2)
        if (r is None or not all(math.isfinite(v) for v in
                                (ref.offset_m, ref.heading_rad, ref.rms_m, ref.span_m))
                or ref.samples < 8 or ref.span_m < 0.30 or not 0 <= ref.rms_m <= 0.02
                or abs(ref.heading_rad) > math.pi / 3):
            return fallback('invalid_line_geometry_or_covariance')
        a, b = np.asarray(segment.start_xy, dtype=float), np.asarray(segment.end_xy, dtype=float)
        if a.shape != (2,) or b.shape != (2,) or not np.isfinite([a, b]).all():
            return fallback('invalid_segment')
        length = np.linalg.norm(b-a)
        if length < 0.30:
            return fallback('invalid_segment')
        tangent = (b-a) / length
        normal = np.array([-tangent[1], tangent[0]])
        # No extrapolation around a corner or beyond the associated segment.
        along = float(tangent @ (x[:2]-a))
        if not 0 <= along <= length:
            return fallback('outside_segment')

        map_heading = math.atan2(tangent[1], tangent[0])
        angle = _wrap(map_heading - x[2] - ref.heading_rad)
        # Tape is undirected. Select the branch nearest the current SLAM yaw.
        sign = 1.0 if math.cos(angle) >= 0 else -1.0
        yaw_error = (angle + math.pi/2) % math.pi - math.pi/2
        # n.p = n.a - signed distance from body to the map line.
        offset_error = float(normal @ (a-x[:2]) - sign * ref.offset_m)
        innovation = np.array([offset_error, yaw_error])
        if abs(offset_error) > 0.35 or abs(yaw_error) > math.radians(30):
            return fallback('association_disagreement')
        h = np.array([[normal[0], normal[1], 0], [0, 0, 1]])
        # Reverse offset-heading covariance when the map tangent is reversed.
        signs = np.diag([-sign, -1.0])
        r = signs @ r @ signs.T
        # Unknown cross-correlation: 2*(H P H' + R) upper-bounds residual
        # covariance. This is a conservative diagnostic gate, not a calibrated
        # independent-sensor chi-square claim.
        s = 2 * (h @ p @ h.T + r)
        if float(innovation @ np.linalg.solve(s, innovation)) > 9.21:
            return fallback('innovation_gate')
        w = self.weight
        info = w * np.linalg.inv(p) + (1-w) * h.T @ np.linalg.solve(r, h)
        fused_p = np.linalg.inv(info)
        delta = fused_p @ ((1-w) * h.T @ np.linalg.solve(r, innovation))
        if np.linalg.norm(delta[:2]) > 0.15 or abs(delta[2]) > math.radians(10):
            return fallback('correction_too_large')
        output = x + delta
        output[2] = _wrap(output[2])
        fused_p = (fused_p + fused_p.T) / 2
        if not np.isfinite(output).all() or _covariance(fused_p, 3) is None:
            return fallback('invalid_fusion_result')
        return FusionResult('fused', 'line_and_slam', tuple(float(v) for v in output),
                            _tuple_matrix(fused_p), slam.stamp_s)
