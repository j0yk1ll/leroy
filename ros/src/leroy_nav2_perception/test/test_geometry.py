"""Deterministic, ROS-free DepthART geometry and timing tests."""

import math

import numpy as np
import pytest

from leroy_nav2_perception.geometry import (
    Projection, StampGate, decode_depth_32fc1, depth_to_optical_points,
    select_obstacles, transform_xyz,
)


P = [10.0, 0.0, 1.5, 0.0, 0.0, 10.0, 1.5, 0.0, 0.0, 0.0, 1.0, 0.0]


def test_rectified_p_backprojection_optical_axes_and_range():
    # Camera optical coordinates: x right, y down, z forward.
    projection = Projection.from_rectified_p(P)
    depth = np.ones((4, 4), dtype=np.float32)
    depth[0, 0] = 0
    depth[0, 1] = float('nan')
    depth[0, 2] = float('inf')
    depth[0, 3] = -1
    depth[1, 0] = 0.01
    depth[1, 1] = 10
    pts = depth_to_optical_points(
        depth, projection, stride=1, min_depth_m=0.2, max_depth_m=3.0)
    assert pts.shape == (10, 3)
    # Pixel (v=2,u=2), z=1: (x,y,z)=(0.05,0.05,1).
    assert np.any(np.all(np.isclose(pts, [0.05, 0.05, 1.0]), axis=1))
    assert np.all(np.isfinite(pts))


@pytest.mark.parametrize('changed', [
    [0.0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0],
    [1, 0, 0, 2, 0, 1, 0, 0, 0, 0, 1, 0],
    [1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0],
    [1, 0, 0],
    [float('nan')] * 12,
])
def test_invalid_projection_rejected(changed):
    with pytest.raises(ValueError):
        Projection.from_rectified_p(changed)


def test_row_padding_and_big_endian_depth():
    values = np.array([[1.0, 2.0], [3.0, 4.0]])
    padded = b''.join(
        values[i].astype('>f4').tobytes() + b'ABCD'
        for i in range(2))
    array = decode_depth_32fc1(
        encoding='32FC1', width=2, height=2, step=12,
        data=padded, is_bigendian=True)
    np.testing.assert_allclose(array, values)


@pytest.mark.parametrize('kwargs', [
    {'encoding': '16UC1'},
    {'step': 7},
    {'data': b'bad'},
    {'width': 0},
])
def test_invalid_image_buffer_rejected(kwargs):
    base = dict(encoding='32FC1', width=2, height=2,
                step=8, data=np.ones((2, 2), dtype='<f4').tobytes(),
                is_bigendian=False)
    base.update(kwargs)
    with pytest.raises(ValueError):
        decode_depth_32fc1(**base)


def test_optical_to_base_quaternion_and_ground_filter():
    projection = Projection.from_rectified_p(P)
    optical = depth_to_optical_points(
        np.ones((4, 4), dtype=np.float32), projection,
        stride=1, min_depth_m=0.2, max_depth_m=3.0)
    # Exact synthetic fixed-camera transform used by fixture.
    points = transform_xyz(optical, [0, 0, 0.2], [.5, -.5, .5, -.5])
    np.testing.assert_allclose(points[:, 0], 1.0, atol=1e-6)
    # optical y maps to negative base z around 0.2 m.
    np.testing.assert_allclose(
        points[:, 2], 0.2 - optical[:, 1], atol=1e-6)
    filt = select_obstacles(
        points, ground_z_m=0.0, min_clearance_m=0.10,
        max_height_m=1.2, max_xy_range_m=3.0)
    assert 0 < len(filt) < len(points)
    assert np.all(filt[:, 2] >= 0.10)
    assert np.all(filt[:, 2] <= 1.2)


def test_empty_invalid_depth_never_became_free_observation():
    depth = np.full((4, 4), math.nan, dtype=np.float32)
    result = depth_to_optical_points(
        depth, Projection.from_rectified_p(P), stride=1,
        min_depth_m=0.1, max_depth_m=3.0)
    assert result.shape == (0, 3)


def test_timestamp_gates_age_future_reorder():
    gate = StampGate(max_age_s=2.0, max_future_s=0.1)
    assert gate.check(0, 5_000_000_000) == 'missing_timestamp'
    assert gate.check(1_000_000_000, 5_000_000_000) == 'stale_depth'
    assert gate.check(6_000_000_000, 5_000_000_000) == 'future_timestamp'
    assert gate.check(5_000_000_000, 5_000_000_000) is None
    assert gate.check(5_000_000_000, 6_000_000_000) == 'out_of_order'
    assert gate.check(4_000_000_000, 6_000_000_000) == 'out_of_order'
    assert gate.check(5_300_000_000, 5_400_000_000) is None


def test_invalid_quaternions_rejected():
    for q in [[0, 0, 0, 0], [0, 0, 0, float('nan')]]:
        with pytest.raises(ValueError):
            transform_xyz(np.ones((1, 3)), (0, 0, 0), q)


def test_invalid_height_window_rejected():
    with pytest.raises(ValueError):
        select_obstacles(np.ones((1, 3)), ground_z_m=0,
                         min_clearance_m=0.4, max_height_m=0.2,
                         max_xy_range_m=2)
