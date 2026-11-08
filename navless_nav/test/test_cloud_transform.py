"""Synthetic math/layout checks only; no ROS, aircraft or recorded-flight replay."""
import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navless_nav.cloud_transform import (CloudTransformError, rigid_transform,  # noqa: E402
                                         stamp_s, transform_pointcloud, validate_transform)


FRAME = 'oak_left_camera_optical_frame'


def cloud(points=((0.0, 0.0, 1.0),), *, big=False, double=False, rows=1, padding=0):
    width = len(points) // rows
    size, datatype, code = (8, 8, 'd') if double else (4, 7, 'f')
    point_step = size * 4  # an unused field/padding in every point
    row_step = width * point_step + padding
    data = bytearray(row_step * rows)
    for i, point in enumerate(points):
        row, col = divmod(i, width)
        struct.pack_into(('>' if big else '<') + code * 3, data,
                         row * row_step + col * point_step, *point)
    return NS(header=NS(stamp=NS(sec=10, nanosec=0), frame_id=FRAME),
              width=width, height=rows, point_step=point_step, row_step=row_step,
              is_bigendian=big, data=bytes(data),
              fields=[NS(name=n, offset=i * size, datatype=datatype, count=1)
                      for i, n in enumerate(('x', 'y', 'z'))])


def project(msg, matrix=None, **kwargs):
    options = dict(transform_stamp_s=10.0, transform_source_frame=FRAME, now_s=10.1)
    options.update(kwargs)
    return transform_pointcloud(msg, np.eye(4) if matrix is None else matrix, **options)


def axis_rotation(axis, angle):
    q = np.zeros(4)
    q[axis], q[3] = math.sin(angle / 2), math.cos(angle / 2)
    return rigid_transform([0, 0, 0], q)


def test_full_se3_includes_camera_lever_arm_roll_pitch_and_yaw():
    course_body = axis_rotation(2, 0.7) @ axis_rotation(1, -0.3) @ axis_rotation(0, 0.25)
    course_body[:3, 3] = [1.4, -0.8, 0.9]
    body_camera = axis_rotation(1, -math.pi / 2)
    body_camera[:3, 3] = [0.12, 0.035, -0.05]
    transform = course_body @ body_camera
    optical = np.array([0.2, -0.1, 1.3, 1.0])
    actual = project(cloud([optical[:3]]), transform)
    np.testing.assert_allclose(actual[0], (transform @ optical)[:3], atol=1e-7)
    without_lever = course_body @ np.r_[body_camera[:3, :3] @ optical[:3], 1]
    assert np.linalg.norm(actual[0] - without_lever[:3]) > 0.1


def test_capture_time_transform_not_receipt_time_pose():
    historical = rigid_transform([0.1, 0, 0], [0, 0, 0, 1])
    np.testing.assert_allclose(project(cloud(), historical)[0], [0.1, 0, 1])
    with pytest.raises(CloudTransformError, match='capture-aligned'):
        project(cloud(), historical, transform_stamp_s=10.1)


@pytest.mark.parametrize('big', [False, True])
@pytest.mark.parametrize('double', [False, True])
def test_organized_layout_preserves_endian_and_row_padding(big, double):
    points = [(0.2, 0.3, 0.5), (-0.4, 0.2, 1.5), (1, 0, 2), (0, -0.5, 1)]
    msg = cloud(points, big=big, double=double, rows=2, padding=13)
    msg.fields.reverse()  # named offsets, never positional field assumptions
    np.testing.assert_allclose(project(msg), points, atol=1e-7)


def test_filters_invalid_and_out_of_depth_points_then_crops_in_world_coordinates():
    msg = cloud([(0, 0, 1), (math.nan, 0, 1), (0, 0, 0.1), (0, 0, 4), (1, 0, 1)])
    matrix = rigid_transform([2, 0, 0], [0, 0, 0, 1])
    got = project(msg, matrix, crop_min=[1, -1, 0.5], crop_max=[2.5, 1, 1.5])
    np.testing.assert_allclose(got, [[2, 0, 1]])
    assert project(msg, matrix, crop_max=[1, 1, 2]).shape == (0, 3)


@pytest.mark.parametrize('changes', [
    {'transform_source_frame': 'base_link'}, {'transform_target_frame': 'odom'},
    {'transform_stamp_s': None}, {'transform_stamp_s': 9.0}, {'transform_stamp_s': 10.2},
    {'now_s': 10.6}, {'now_s': 9.99}, {'now_s': math.nan},
    {'max_capture_age_s': -1}, {'max_transform_skew_s': -1},
    {'min_depth_m': 2, 'max_depth_m': 1}, {'crop_min': [1, 1, 1], 'crop_max': [0, 0, 0]},
])
def test_rejects_unqualified_frame_time_or_filter_config(changes):
    with pytest.raises(CloudTransformError):
        project(cloud(), **changes)


@pytest.mark.parametrize('matrix', [None, np.zeros((4, 4)), np.eye(3),
                                    np.diag([-1, 1, 1, 1]), np.diag([2, 1, 1, 1]),
                                    np.full((4, 4), math.nan)])
def test_missing_or_nonrigid_transform_has_no_fallback(matrix):
    with pytest.raises(CloudTransformError):
        transform_pointcloud(cloud(), matrix, transform_stamp_s=10,
                             transform_source_frame=FRAME, now_s=10.1)


@pytest.mark.parametrize('mutate', [
    lambda m: setattr(m.header, 'frame_id', ''),
    lambda m: setattr(m.header.stamp, 'nanosec', 1_000_000_000),
    lambda m: setattr(m.header.stamp, 'sec', -1),
    lambda m: setattr(m, 'width', -1),
    lambda m: setattr(m, 'row_step', 1),
    lambda m: setattr(m, 'data', m.data[:-1]),
    lambda m: setattr(m, 'data', list(m.data)),
    lambda m: setattr(m.fields[0], 'offset', m.point_step),
    lambda m: setattr(m.fields[0], 'offset', m.fields[1].offset),
    lambda m: setattr(m.fields[0], 'datatype', 4),
    lambda m: setattr(m.fields[0], 'count', 2),
    lambda m: m.fields.pop(),
    lambda m: m.fields.append(m.fields[0]),
])
def test_rejects_malformed_headers_and_cloud_layouts(mutate):
    msg = cloud()
    mutate(msg)
    with pytest.raises(CloudTransformError):
        project(msg)


def test_stamp_and_quaternion_validation():
    assert stamp_s(NS(sec=4, nanosec=250_000_000)) == 4.25
    for bad in [NS(sec=0, nanosec=0), NS(sec=1.5, nanosec=0), None]:
        with pytest.raises(CloudTransformError):
            stamp_s(bad)
    for bad in [[0, 0, 0, 0], [0, 0, 0, 2], [0, math.nan, 0, 1]]:
        with pytest.raises(CloudTransformError):
            rigid_transform([0, 0, 0], bad)
    matrix = rigid_transform([1, 2, 3], [0, 0, 0, 1.00001])
    np.testing.assert_allclose(validate_transform(matrix)[:3, 3], [1, 2, 3])
