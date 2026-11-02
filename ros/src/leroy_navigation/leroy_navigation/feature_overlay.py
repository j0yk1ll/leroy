"""Read-only camera keypoint overlay for RViz.

The features are OpenCV Shi-Tomasi corners, NOT XRSLAM internal tracks.
"""
import cv2
import numpy as np


def draw_corners(bgr, *, max_corners=180, quality=0.015, spacing=8.0):
    if not isinstance(bgr, np.ndarray) or bgr.dtype != np.uint8 or (
            bgr.ndim != 3 or bgr.shape[2] != 3):
        raise ValueError("expected uint8 BGR image")
    if bgr.shape[0] < 8 or bgr.shape[1] < 8:
        raise ValueError("invalid frame size")
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    corners = cv2.goodFeaturesToTrack(
        gray, maxCorners=int(max_corners),
        qualityLevel=float(quality), minDistance=float(spacing))
    out = bgr.copy()
    if corners is not None:
        for x, y in corners.reshape(-1, 2):
            cv2.circle(out, (int(round(x)), int(round(y))), 3,
                       (0, 215, 255), 1, cv2.LINE_AA)
    return out
