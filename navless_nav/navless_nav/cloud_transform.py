"""Capture-time PointCloud2 projection with an explicitly calibrated SE(3) transform.

No ROS imports or pose fallback: the caller must obtain course <- cloud.header.frame_id
at cloud.header.stamp. A receipt-time pose is not interchangeable with that query.
"""
import math
import operator

import numpy as np


class CloudTransformError(ValueError):
    """The cloud/transform cannot be admitted to the course map."""


def _integer(value, name):
    try:
        if isinstance(value, (bool, np.bool_)):
            raise TypeError
        return operator.index(value)
    except TypeError as exc:
        raise CloudTransformError(f'{name} must be an integer') from exc


def _finite(value, name):
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise CloudTransformError(f'{name} must be finite') from exc
    if not math.isfinite(value):
        raise CloudTransformError(f'{name} must be finite')
    return value


def stamp_s(stamp):
    """Validate a nonzero ROS acquisition stamp; never substitute receipt time."""
    try:
        sec = _integer(stamp.sec, 'stamp.sec')
        nanosec = _integer(stamp.nanosec, 'stamp.nanosec')
    except AttributeError as exc:
        raise CloudTransformError('missing source stamp') from exc
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise CloudTransformError('invalid source stamp')
    value = sec + nanosec * 1e-9
    if value <= 0:
        raise CloudTransformError('source stamp must be nonzero')
    return value


def _vector(value, name):
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise CloudTransformError(f'invalid {name}') from exc
    if result.shape != (3,) or not np.isfinite(result).all():
        raise CloudTransformError(f'invalid {name}')
    return result


def validate_transform(matrix):
    """Return a finite proper rigid transform, rejecting scale/reflection/shear."""
    try:
        matrix = np.asarray(matrix, dtype=float)
    except (TypeError, ValueError) as exc:
        raise CloudTransformError('invalid rigid transform') from exc
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise CloudTransformError('invalid rigid transform')
    r = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-7, rtol=0)
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-6, rtol=0)
            or not math.isclose(float(np.linalg.det(r)), 1.0, abs_tol=1e-6)):
        raise CloudTransformError('transform must be proper SE(3)')
    return matrix


def rigid_transform(translation, quaternion_xyzw):
    """Build a rigid transform from a finite translation and near-unit quaternion."""
    position = _vector(translation, 'translation')
    try:
        q = np.asarray(quaternion_xyzw, dtype=float)
    except (TypeError, ValueError) as exc:
        raise CloudTransformError('invalid xyzw quaternion') from exc
    if q.shape != (4,) or not np.isfinite(q).all():
        raise CloudTransformError('invalid xyzw quaternion')
    norm = float(np.linalg.norm(q))
    if abs(norm - 1.0) > 1e-3:
        raise CloudTransformError('quaternion must be near unit length')
    x, y, z, w = q / norm
    matrix = np.eye(4)
    matrix[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    matrix[:3, 3] = position
    return validate_transform(matrix)


def _frame(value, name):
    if not isinstance(value, str) or not value or any(c.isspace() for c in value):
        raise CloudTransformError(f'invalid {name}')
    return value


def _xyz(msg):
    """Decode organized XYZ, preserving endian, row padding and field offsets."""
    try:
        width = _integer(msg.width, 'width')
        height = _integer(msg.height, 'height')
        point_step = _integer(msg.point_step, 'point_step')
        row_step = _integer(msg.row_step, 'row_step')
        if width < 0 or height <= 0 or point_step <= 0 or row_step < width * point_step:
            raise CloudTransformError('invalid PointCloud2 dimensions/strides')
        if not isinstance(msg.is_bigendian, (bool, np.bool_)):
            raise CloudTransformError('invalid endianness flag')
        endian = '>' if msg.is_bigendian else '<'
        fields = {}
        for field in msg.fields:
            if field.name not in ('x', 'y', 'z'):
                continue
            if field.name in fields:
                raise CloudTransformError('duplicate XYZ field')
            kind = _integer(field.datatype, 'datatype')
            if kind not in (7, 8) or _integer(field.count, 'field count') != 1:
                raise CloudTransformError('XYZ fields must be scalar FLOAT32/FLOAT64')
            offset = _integer(field.offset, 'field offset')
            size = 4 if kind == 7 else 8
            if offset < 0 or offset + size > point_step:
                raise CloudTransformError('XYZ field exceeds point stride')
            fields[field.name] = (endian + ('f4' if kind == 7 else 'f8'), offset, size)
        if set(fields) != {'x', 'y', 'z'}:
            raise CloudTransformError('missing XYZ fields')
        spans = sorted((offset, offset + size) for _, offset, size in fields.values())
        if any(a[1] > b[0] for a, b in zip(spans, spans[1:])):
            raise CloudTransformError('overlapping XYZ fields')
        data = memoryview(msg.data)
        if not data.c_contiguous or data.nbytes != row_step * height:
            raise CloudTransformError('cloud data length does not match row strides')
        if width == 0:
            return np.empty((0, 3), dtype=float)
        dtype = np.dtype({'names': ['x', 'y', 'z'],
                          'formats': [fields[n][0] for n in ('x', 'y', 'z')],
                          'offsets': [fields[n][1] for n in ('x', 'y', 'z')],
                          'itemsize': point_step})
        organized = np.ndarray((height, width), dtype=dtype, buffer=data,
                               strides=(row_step, point_step))
        return np.stack([organized[n] for n in ('x', 'y', 'z')], axis=-1).reshape(-1, 3).astype(float)
    except (AttributeError, TypeError, BufferError) as exc:
        raise CloudTransformError('malformed PointCloud2 layout') from exc


def transform_pointcloud(msg, T_course_camera, *, transform_stamp_s,
                         transform_source_frame, now_s, transform_target_frame='course',
                         expected_target_frame='course', max_capture_age_s=0.5,
                         max_transform_skew_s=0.02, future_tolerance_s=0.005,
                         min_depth_m=0.3, max_depth_m=2.8, crop_min=None, crop_max=None):
    """Return Nx3 world points using a calibrated transform at the capture time.

    Transform metadata describes the successful historical TF query, not the
    camera calibration's static timestamp. The caller must not stamp a latest-
    pose transform with the cloud time to bypass this contract. Optical z is
    depth; crop bounds, when supplied, are in the target course frame.
    """
    try:
        captured = stamp_s(msg.header.stamp)
        source = _frame(msg.header.frame_id, 'cloud frame')
    except AttributeError as exc:
        raise CloudTransformError('missing cloud header') from exc
    if source != _frame(transform_source_frame, 'transform source frame'):
        raise CloudTransformError('transform source does not match cloud frame')
    if _frame(transform_target_frame, 'transform target frame') != _frame(expected_target_frame, 'expected target frame'):
        raise CloudTransformError('transform target is not the requested course frame')
    now = _finite(now_s, 'now_s')
    transformed_at = _finite(transform_stamp_s, 'transform stamp')
    age = _finite(max_capture_age_s, 'capture age budget')
    skew = _finite(max_transform_skew_s, 'transform skew budget')
    future = _finite(future_tolerance_s, 'future tolerance')
    if now <= 0 or transformed_at <= 0 or min(age, skew, future) < 0:
        raise CloudTransformError('invalid clock or time budget')
    if not -future <= now - captured <= age:
        raise CloudTransformError('cloud capture is stale or future')
    if not -future <= now - transformed_at <= age or abs(transformed_at - captured) > skew:
        raise CloudTransformError('transform is stale, future, or not capture-aligned')
    matrix = validate_transform(T_course_camera)
    lower = _finite(min_depth_m, 'minimum depth')
    upper = _finite(max_depth_m, 'maximum depth')
    if lower < 0 or upper <= lower:
        raise CloudTransformError('invalid depth interval')
    lo = _vector(crop_min, 'crop minimum') if crop_min is not None else None
    hi = _vector(crop_max, 'crop maximum') if crop_max is not None else None
    if lo is not None and hi is not None and np.any(lo > hi):
        raise CloudTransformError('inverted crop bounds')
    points = _xyz(msg)
    good = np.isfinite(points).all(axis=1) & (points[:, 2] >= lower) & (points[:, 2] <= upper)
    world = points[good] @ matrix[:3, :3].T + matrix[:3, 3]
    keep = np.isfinite(world).all(axis=1)
    if lo is not None:
        keep &= (world >= lo).all(axis=1)
    if hi is not None:
        keep &= (world <= hi).all(axis=1)
    return world[keep]
