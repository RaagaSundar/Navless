"""Offline floor-line geometry; no ROS, actuator outputs or position fixes.

Inputs must be selected tape-centre pixels from a rectified camera image.
All metric vectors use a gravity-level frame with body heading: x forward,
y left, z up. The optical-to-level rotation includes measured camera mount
AND roll/pitch at image capture. A straight line observes only one XY axis.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class LineReference:
    stamp_s: float
    # Positive offset means the line lies to the left of its forward tangent.
    offset_m: float
    heading_rad: float  # undirected line, resolved toward +x; NOT absolute yaw
    normal_xy: tuple
    rms_m: float
    span_m: float
    samples: int

    @property
    def closest_xy(self):
        """Vector from vehicle origin toward the fitted infinite line."""
        return tuple(self.offset_m * n for n in self.normal_xy)


def floor_points(pixels, intrinsics, optical_to_level, camera_xyz,
                 height_m, max_range_m=3.0):
    """Intersect rectified pixel rays with a locally flat floor z=-height.

    intrinsics=(fx, fy, cx, cy), from matching rectified CameraInfo, with
    optical coordinates x right/y down/z forward. camera_xyz is the optical
    origin relative to base_link in the level frame, in metres. Height is
    base_link-to-floor distance, not raw slant range or PX4 local z.
    Invalid/upward/near-horizon/too-distant rays are discarded.
    """
    fx, fy, cx, cy = intrinsics
    values = [fx, fy, cx, cy, *camera_xyz, height_m, max_range_m]
    if not all(math.isfinite(v) for v in values) or min(fx, fy, height_m, max_range_m) <= 0:
        raise ValueError('invalid calibration, height or range')
    r = optical_to_level
    if len(r) != 3 or any(len(row) != 3 for row in r):
        raise ValueError('rotation must be 3 by 3')
    if not all(math.isfinite(v) for row in r for v in row):
        raise ValueError('nonfinite rotation')
    for i in range(3):
        for j in range(3):
            if abs(sum(r[i][k] * r[j][k] for k in range(3)) - (i == j)) > 1e-5:
                raise ValueError('rotation must be orthonormal')
    det = sum(r[0][i] * (r[1][(i+1) % 3] * r[2][(i+2) % 3]
                        - r[1][(i+2) % 3] * r[2][(i+1) % 3]) for i in range(3))
    if abs(det - 1.0) > 1e-5:
        raise ValueError('rotation must be right handed')
    if camera_xyz[2] <= -height_m:
        raise ValueError('camera must be above floor')
    points = []
    for u, v in pixels:
        if not (math.isfinite(u) and math.isfinite(v)):
            continue
        ray = ((u - cx) / fx, (v - cy) / fy, 1.0)
        d = tuple(sum(r[i][j] * ray[j] for j in range(3)) for i in range(3))
        if d[2] / math.sqrt(sum(x*x for x in d)) >= -0.1:
            continue
        t = (-height_m - camera_xyz[2]) / d[2]
        p = tuple(camera_xyz[i] + t * d[i] for i in range(3))
        if math.hypot(p[0], p[1]) <= max_range_m:
            points.append(p[:2])
    return points


def fit_reference(points, stamp_s, now_s, *, max_age_s=0.15,
                  min_samples=8, min_span_m=0.30, max_rms_m=0.02,
                  max_residual_m=0.05, max_heading_rad=math.pi / 3):
    """Fit ONE locally straight, already selected line, or return None.

    No best-branch guess: corners, mixed candidates and bad fits should fail.
    Thresholds are unqualified prototype settings. Timestamp must be image
    capture time in the same clock as now_s. No cached result is returned.
    """
    settings = (max_age_s, min_span_m, max_rms_m, max_residual_m, max_heading_rad)
    if (not all(math.isfinite(v) and v > 0 for v in settings)
            or min_samples < 3 or not 0 < max_heading_rad < math.pi / 2):
        raise ValueError('invalid fit settings')
    if (not all(math.isfinite(t) for t in (stamp_s, now_s))
            or not 0 <= now_s - stamp_s <= max_age_s):
        return None
    if len(points) < min_samples or any(
            len(p) != 2 or not all(math.isfinite(v) for v in p) for p in points):
        return None
    mx, my = (sum(p[i] for p in points) / len(points) for i in range(2))
    xx = sum((x-mx)**2 for x, y in points)
    yy = sum((y-my)**2 for x, y in points)
    xy = sum((x-mx)*(y-my) for x, y in points)
    heading = 0.5 * math.atan2(2 * xy, xx - yy)
    if abs(heading) > max_heading_rad:
        return None
    tangent = (math.cos(heading), math.sin(heading))
    normal = (-tangent[1], tangent[0])
    offset = normal[0] * mx + normal[1] * my
    residuals = [normal[0]*x + normal[1]*y - offset for x, y in points]
    rms = math.sqrt(sum(e*e for e in residuals) / len(points))
    along = [tangent[0]*x + tangent[1]*y for x, y in points]
    span = max(along) - min(along)
    if span < min_span_m or rms > max_rms_m or max(map(abs, residuals)) > max_residual_m:
        return None
    return LineReference(stamp_s, offset, heading, normal, rms, span, len(points))
