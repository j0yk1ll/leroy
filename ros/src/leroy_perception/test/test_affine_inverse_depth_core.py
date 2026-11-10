"""ROS-free tests: fit diagnostics are never navigation authorization."""
import numpy as np
import pytest

from leroy_perception.affine_inverse_depth_core import (
    DepthAnchor, fit_diagnostic_inverse_depth,
)


def dataset(*, outlier=False):
    # For these constructed camera-Z targets, prediction is an affine
    # function of inverse Z. Groups are distinct physical targets.
    distances = (0.32, 0.42, 0.55, 0.67, 0.83, 1.05, 1.17, 1.28, 0.95)
    anchors = [
        DepthAnchor(
            prediction=(1.0 / z - 0.6) / 1.8,
            camera_z_m=z + (0.45 if outlier and i == 4 else 0.0),
            evidence_id=f"measurement-{i}",
            target_id=f"target-{i // 3}",
        )
        for i, z in enumerate(distances)
    ]
    return anchors


def test_valid_multi_target_geometry_recovers_known_transform():
    fit = fit_diagnostic_inverse_depth(dataset())
    assert fit.slope == pytest.approx(1.8, abs=1e-9)
    assert fit.intercept == pytest.approx(0.6, abs=1e-9)
    assert fit.anchor_count == 9
    assert fit.inlier_count == 9
    assert fit.independent_targets == 3
    assert fit.median_error_m < 1e-8
    assert fit.navigation_qualified is False


def test_single_large_outlier_is_rejected():
    fit = fit_diagnostic_inverse_depth(dataset(outlier=True))
    assert fit.inlier_count == 8
    assert fit.slope == pytest.approx(1.8, abs=1e-9)
    assert fit.navigation_qualified is False


def test_extrapolated_and_nonfinite_predictions_stay_invalid():
    fit = fit_diagnostic_inverse_depth(dataset())
    raw = np.array([0.5, np.nan, -100, 100], dtype=np.float32)
    result = fit.depth_image(raw)
    assert result[0] == pytest.approx(1.0 / 1.5)
    assert np.isnan(result[1:]).all()


def test_duplicate_measurement_identity_rejected():
    points = dataset()
    points[1] = DepthAnchor(points[1].prediction, points[1].camera_z_m,
                            evidence_id=points[0].evidence_id,
                            target_id=points[1].target_id)
    with pytest.raises(ValueError, match="duplicate"):
        fit_diagnostic_inverse_depth(points)


def test_one_surface_repeated_does_not_create_independent_targets():
    points = [
        DepthAnchor(p.prediction, p.camera_z_m, p.evidence_id, "same-target")
        for p in dataset()
    ]
    with pytest.raises(ValueError, match="three"):
        fit_diagnostic_inverse_depth(points)


def test_constant_range_rejected():
    points = [
        DepthAnchor(float(i), 0.75, f"m{i}", f"t{i // 3}")
        for i in range(9)
    ]
    with pytest.raises(ValueError, match="span"):
        fit_diagnostic_inverse_depth(points)


def test_unreliable_outlier_majority_fails_closed():
    points = dataset()
    points = [
        DepthAnchor(p.prediction, p.camera_z_m + (0.55 if i >= 4 else 0.0),
                    p.evidence_id, p.target_id)
        for i, p in enumerate(points)
    ]
    with pytest.raises(ValueError):
        fit_diagnostic_inverse_depth(points, min_inlier_fraction=0.85)


def test_too_few_anchors_rejected():
    with pytest.raises(ValueError, match="6..128"):
        fit_diagnostic_inverse_depth(dataset()[:5])
