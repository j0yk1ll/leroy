"""Diagnostic-only affine inverse-depth calibration for ZipDepth-like outputs.

The model 1/Z = a*prediction + b is a TESTABLE HYPOTHESIS, not proof that
ZipDepth (or any other model) is physically calibrated. Inputs must already
be independently associated with measured camera-Z target depths using
calibrated camera/ToF geometry and trustworthy acquisition timestamps.

No ROS, motor APIs, or automatic navigation authorization are present.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class DepthAnchor:
    prediction: float
    camera_z_m: float
    evidence_id: str
    target_id: str


@dataclass(frozen=True)
class DiagnosticFit:
    slope: float
    intercept: float
    prediction_min: float
    prediction_max: float
    anchor_count: int
    inlier_count: int
    independent_targets: int
    median_error_m: float
    p90_error_m: float
    # A successful numerical fit is NEVER a physical-safety qualification.
    navigation_qualified: bool = False

    def depth_image(self, prediction):
        """Convert only within the observed prediction support; outside => NaN.

        Finite positive predicted inverse depths are required. The resulting
        array represents a diagnostic hypothesis in meters, not a certified map.
        """
        arr = np.asarray(prediction, dtype=np.float64)
        inv = self.slope * arr + self.intercept
        valid = (
            np.isfinite(arr) & (arr >= self.prediction_min)
            & (arr <= self.prediction_max) & np.isfinite(inv)
            & (inv > 1.0 / 5.0) & (inv < 1.0 / 0.10)
        )
        result = np.full(arr.shape, np.nan, dtype=np.float32)
        np.divide(1.0, inv, out=result, where=valid, casting="unsafe")
        return result


def fit_diagnostic_inverse_depth(
    anchors, *, max_inlier_error_m=0.08, max_median_error_m=0.05,
    max_p90_error_m=0.10, min_inlier_fraction=0.75,
):
    """Robustly fit inverse-depth scale and shift to independent metric anchors.

    Deterministic two-point consensus search followed by least-squares
    refinement. A fit passing these *internal* residual gates is not validated
    on independent held-out scenes and must not be used for motion control.

    Requires >=6 independently identified measurements, >=3 targets, and
    >=0.20 m measured depth span. Repeated readings of one target alone are
    insufficient. A caller must independently ensure target correspondence,
    calibration, timing, and that targets genuinely provide distinct geometry.
    """
    points = list(anchors)
    if len(points) < 6 or len(points) > 128:
        raise ValueError("requires 6..128 independently identified anchors")
    if len({p.evidence_id for p in points}) != len(points):
        raise ValueError("duplicate anchor evidence_id")
    targets = {p.target_id for p in points}
    if len(targets) < 3 or any(not x for x in targets):
        raise ValueError("requires at least three independently identified targets")
    if any(not p.evidence_id for p in points):
        raise ValueError("anchor evidence_id required")

    raw = np.array([p.prediction for p in points], dtype=np.float64)
    z = np.array([p.camera_z_m for p in points], dtype=np.float64)
    if not np.isfinite(raw).all() or not np.isfinite(z).all():
        raise ValueError("nonfinite anchor")
    if np.any(z < 0.10) or np.any(z > 5.0):
        raise ValueError("camera-Z targets must be in 0.10..5.0 m")
    if float(np.ptp(z)) < 0.20:
        raise ValueError("insufficient independent metric depth span")
    raw_span = float(np.ptp(raw))
    if raw_span <= 1e-6 * max(1.0, float(np.max(np.abs(raw)))):
        raise ValueError("relative predictor has insufficient variation")
    if not all(math.isfinite(float(v)) for v in (
            max_inlier_error_m, max_median_error_m,
            max_p90_error_m, min_inlier_fraction)):
        raise ValueError("invalid fit thresholds")
    if not (0 < max_median_error_m <= max_p90_error_m
            and 0 < max_inlier_error_m <= max_p90_error_m
            and 0.5 < min_inlier_fraction <= 1.0):
        raise ValueError("invalid fit thresholds")

    inverse = 1.0 / z
    best = None
    # Each pair proposes a model. Score by physically meaningful depth error.
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            dx = raw[j] - raw[i]
            if abs(dx) < raw_span * 1e-4:
                continue
            a = (inverse[j] - inverse[i]) / dx
            b = inverse[i] - a * raw[i]
            predicted_inverse = a * raw + b
            good = predicted_inverse > 0
            predicted_z = np.full(z.shape, np.nan)
            np.divide(1.0, predicted_inverse, out=predicted_z, where=good)
            errors = np.abs(predicted_z - z)
            inliers = np.isfinite(errors) & (errors <= max_inlier_error_m)
            n = int(inliers.sum())
            if n < 4:
                continue
            score = (n, -float(np.median(errors[inliers])))
            if best is None or score > best[0]:
                best = (score, inliers)
    if best is None:
        raise ValueError("no physically consistent inverse-depth hypothesis")
    inliers = best[1]
    if float(inliers.mean()) < min_inlier_fraction:
        raise ValueError("insufficient multi-anchor consensus")
    if len({points[i].target_id for i in np.flatnonzero(inliers)}) < 3:
        raise ValueError("insufficient independent target consensus")
    if float(np.ptp(z[inliers])) < 0.20:
        raise ValueError("insufficient inlier depth span")

    # Refit only the consensus anchors; recompute physical residuals.
    a, b = np.polyfit(raw[inliers], inverse[inliers], 1)
    inv_fit = a * raw + b
    if np.any(inv_fit[inliers] <= 0):
        raise ValueError("nonpositive fitted inverse depth")
    residual = np.abs(1.0 / inv_fit[inliers] - z[inliers])
    median = float(np.median(residual))
    p90 = float(np.quantile(residual, 0.90))
    if median > max_median_error_m or p90 > max_p90_error_m:
        raise ValueError("fit residuals exceed diagnostic thresholds")

    # Explicitly reject new outliers introduced by refinement.
    if np.any(residual > max_p90_error_m):
        raise ValueError("refined inlier fit contains excessive residual")
    return DiagnosticFit(
        slope=float(a), intercept=float(b),
        prediction_min=float(raw[inliers].min()),
        prediction_max=float(raw[inliers].max()),
        anchor_count=len(points), inlier_count=int(inliers.sum()),
        independent_targets=len({points[i].target_id
                                 for i in np.flatnonzero(inliers)}),
        median_error_m=median, p90_error_m=p90,
    )
