"""Pure helpers for M4 map-center/trajectory acceptance, no ROS dependencies."""

import math


def spans(samples, minimum):
    if not samples or not math.isfinite(minimum) or minimum <= 0:
        return False
    if not all(math.isfinite(v) for v in samples):
        return False
    return max(samples) - min(samples) > minimum


def tracks_robot(origin_x, origin_y, width, height, resolution,
                 robot_x, robot_y, tolerance=0.35):
    values = (origin_x, origin_y, resolution, robot_x, robot_y, tolerance)
    if not all(math.isfinite(v) for v in values):
        return False
    if width <= 0 or height <= 0 or resolution <= 0 or tolerance <= 0:
        return False
    cx = origin_x + width * resolution * 0.5
    cy = origin_y + height * resolution * 0.5
    return abs(cx - robot_x) < tolerance and abs(cy - robot_y) < tolerance
