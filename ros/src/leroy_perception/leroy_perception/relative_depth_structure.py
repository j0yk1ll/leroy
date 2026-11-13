"""Diagnostic-only relative-depth structure analysis.

Output classes:
    0 = UNKNOWN
    1 = POSSIBLE_DEPTH_BOUNDARY

Neither class establishes traversable free space or authorizes motion.
No assumption is made about the prediction's near/far ordering.
"""

import math

import numpy as np


UNKNOWN = np.uint8(0)
POSSIBLE_DEPTH_BOUNDARY = np.uint8(1)


def detect_relative_boundaries(
    prediction,
    *,
    contrast_threshold=0.15,
    min_valid_fraction=0.50,
):
    """Detect local relative-depth discontinuities.

    Contrast is normalized using the frame's robust prediction range.
    This is a visual diagnostic, not a physically calibrated detector.

    Returns a uint8 classification image matching the input dimensions.
    """
    if (
        not math.isfinite(contrast_threshold)
        or not 0 < contrast_threshold < 1
        or not math.isfinite(min_valid_fraction)
        or not 0 < min_valid_fraction <= 1
    ):
        raise ValueError("Invalid diagnostic thresholds")

    depth = np.asarray(prediction, dtype=np.float64)

    if depth.ndim != 2 or min(depth.shape) < 3:
        raise ValueError("Expected a 2D depth image of at least 3x3")

    result = np.full(depth.shape, UNKNOWN, dtype=np.uint8)
    valid = np.isfinite(depth)

    if np.count_nonzero(valid) / depth.size < min_valid_fraction:
        return result

    values = depth[valid]
    low, high = np.percentile(values, [5, 95])
    span = high - low

    if not np.isfinite(span) or span <= 1e-12:
        return result

    # Compare horizontal and vertical neighbors without assuming whether
    # larger predictions represent nearer or farther surfaces.
    horizontal = (
        valid[:, 1:] & valid[:, :-1]
        & (np.abs(depth[:, 1:] - depth[:, :-1])
           >= contrast_threshold * span)
    )
    vertical = (
        valid[1:, :] & valid[:-1, :]
        & (np.abs(depth[1:, :] - depth[:-1, :])
           >= contrast_threshold * span)
    )

    result[:, 1:][horizontal] = POSSIBLE_DEPTH_BOUNDARY
    result[:, :-1][horizontal] = POSSIBLE_DEPTH_BOUNDARY
    result[1:, :][vertical] = POSSIBLE_DEPTH_BOUNDARY
    result[:-1, :][vertical] = POSSIBLE_DEPTH_BOUNDARY

    return result
