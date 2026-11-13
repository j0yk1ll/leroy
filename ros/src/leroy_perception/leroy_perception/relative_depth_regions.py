"""Diagnostic-only regional relative-depth contrast.

A high score means that adjacent image regions have different
relative-depth predictions. It does NOT identify near/far ordering,
physical obstacles, free space, or safe motion.
"""

import numpy as np


def regional_contrast(prediction, *, rows=6, columns=8,
                      min_valid_fraction=0.5):
    """Return regional contrast scores and validity.

    Scores are normalized by the image's robust 5th–95th percentile
    prediction range. Absolute differences make the results invariant
    to near/far sign reversal and positive/negative affine scaling.

    Invalid or unsupported regions receive NaN scores.
    """
    image = np.asarray(prediction, dtype=np.float64)

    if image.ndim != 2 or min(image.shape) < 2:
        raise ValueError("Expected a 2D relative-depth image")
    if not (isinstance(rows, int) and isinstance(columns, int)
            and 2 <= rows <= image.shape[0]
            and 2 <= columns <= image.shape[1]):
        raise ValueError("Invalid regional grid dimensions")
    if not np.isfinite(min_valid_fraction) or not (
            0 < min_valid_fraction <= 1):
        raise ValueError("Invalid minimum valid fraction")

    finite = np.isfinite(image)
    scores = np.full((rows, columns), np.nan, dtype=np.float64)
    medians = np.full((rows, columns), np.nan, dtype=np.float64)
    valid = np.zeros((rows, columns), dtype=bool)

    if not finite.any():
        return scores, valid

    low, high = np.percentile(image[finite], (5, 95))
    span = high - low
    if not np.isfinite(span) or span <= 1e-12:
        return scores, valid

    y_edges = np.linspace(0, image.shape[0], rows + 1, dtype=int)
    x_edges = np.linspace(0, image.shape[1], columns + 1, dtype=int)

    for y in range(rows):
        for x in range(columns):
            region = image[
                y_edges[y]:y_edges[y + 1],
                x_edges[x]:x_edges[x + 1],
            ]
            values = region[np.isfinite(region)]
            if values.size / region.size >= min_valid_fraction:
                medians[y, x] = np.median(values)
                valid[y, x] = True

    for y in range(rows):
        for x in range(columns):
            if not valid[y, x]:
                continue
            differences = [
                abs(medians[y, x] - medians[ny, nx]) / span
                for ny, nx in (
                    (y - 1, x), (y + 1, x),
                    (y, x - 1), (y, x + 1),
                )
                if 0 <= ny < rows and 0 <= nx < columns
                and valid[ny, nx]
            ]
            if differences:
                scores[y, x] = max(differences)

    return scores, valid
