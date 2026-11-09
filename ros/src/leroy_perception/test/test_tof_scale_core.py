"""ROS-independent geometry tests for the optional ToF relative-depth scaler."""
import numpy as np
import pytest
from leroy_perception.tof_scale_core import project_tof_hit, scale_relative_depth


K = (500.0, 500.0, 320.0, 240.0)
Q = (0.0, 0.0, 0.0, 1.0)


def test_center_beam_projects_to_camera_center():
    u, v, z = project_tof_hit(1.0, K, (0, 0, 0), Q)
    assert (u, v, z) == pytest.approx((320., 240., 1.))


def test_translated_tof_origin_requires_disparity_projection():
    u, v, z = project_tof_hit(1.0, K, (0.10, 0.0, 0.0), Q)
    assert (u, v, z) == pytest.approx((370., 240., 1.))


def test_scale_preserves_nan_and_scales_uniform_relative_depth():
    img = np.full((480, 640), 2.0, dtype=np.float32)
    img[0,0] = np.nan
    result, info = scale_relative_depth(img, 1.0, K, (0, 0, 0), Q, 160)
    assert result[240, 320] == pytest.approx(1.0)
    assert np.isnan(result[0,0])
    assert info["scale"] == pytest.approx(0.5)


def test_heterogeneous_cone_roi_is_rejected():
    img = np.full((480, 640), 2.0, dtype=np.float32)
    img[80:240, 180:460] = 5.0
    with pytest.raises(ValueError, match="heterogeneous"):
        scale_relative_depth(img, 1.0, K, (0, 0, 0), Q, 160)


def test_out_of_range_tof_rejected():
    with pytest.raises(ValueError, match="range"):
        project_tof_hit(1.6, K, (0, 0, 0), Q)


def test_uncalibrated_rotation_rejected():
    with pytest.raises(ValueError, match="quaternion"):
        project_tof_hit(0.6, K, (0, 0, 0), (0, 0, 0, 0))


def test_tiny_roi_cannot_substitute_for_tof_cone():
    img = np.full((480, 640), 2.0, dtype=np.float32)
    with pytest.raises(ValueError, match="cone"):
        scale_relative_depth(img, 1.0, K, (0, 0, 0), Q, 20)
