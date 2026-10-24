#!/usr/bin/env python3
"""Independent stdlib Brown-Conrady V3 synthetic rectification oracle.

No ROS imports, no cv2, no device calls and no reuse of image_geometry maps.
This tests a precisely specified monocular K/R/P and five plumb_bob
coefficients. It does not validate general physical calibration.
"""
import math

WIDTH, HEIGHT = 80, 60
FX, FY, CX, CY = 60.0, 62.0, 39.5, 29.5
D = (0.40, 0.10, 0.010, -0.012, 0.0)  # k1, k2, p1, p2, k3
K = (FX, 0., CX, 0., FY, CY, 0., 0., 1.)
R = (1., 0., 0., 0., 1., 0., 0., 0., 1.)
P = (FX, 0., CX, 0., 0., FY, CY, 0., 0., 0., 1., 0.)
MAX_ERROR = 4.0  # OpenCV INTER_LINEAR uses 1/32px interpolation table
MAX_MEAN_ERROR = 0.8
MIN_WARPED_PIXELS = 800
MIN_FULL_BORDER_PIXELS = 400


def source_pixel(x, y, phase):
    if not (0 <= x < WIDTH and 0 <= y < HEIGHT and 0 <= phase < 13):
        raise ValueError("BLOCKED_V3_GEOMETRY_SOURCE_COORD")
    return 16 + (2 * x + y if phase % 2 == 0 else x + 2 * y) + phase


def source_image(phase):
    return bytes(source_pixel(x, y, phase)
                 for y in range(HEIGHT) for x in range(WIDTH))


def distorted_coordinate(x, y):
    """Rectified output pixel -> continuous distorted source pixel.

    R=I and K=P[0:3,0:3]. Implements Brown-Conrady forward distortion
    instead of using cv::initUndistortRectifyMap or cv::remap.
    """
    xn = (x - CX) / FX
    yn = (y - CY) / FY
    r2 = xn * xn + yn * yn
    k1, k2, p1, p2, k3 = D
    radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    xd = xn * radial + 2.0 * p1 * xn * yn + p2 * (r2 + 2.0 * xn * xn)
    yd = yn * radial + p1 * (r2 + 2.0 * yn * yn) + 2.0 * p2 * xn * yn
    return FX * xd + CX, FY * yd + CY


def reference_pixel(x, y, phase):
    """Independent continuous bilinear interpolation; zero outside image."""
    u, v = distorted_coordinate(x, y)
    ix, iy = math.floor(u), math.floor(v)
    wx, wy = u - ix, v - iy
    result = 0.0
    for dy in (0, 1):
        for dx in (0, 1):
            xx, yy = ix + dx, iy + dy
            if 0 <= xx < WIDTH and 0 <= yy < HEIGHT:
                result += (source_pixel(xx, yy, phase) *
                           (wx if dx else 1.0 - wx) *
                           (wy if dy else 1.0 - wy))
    return result


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def _check_calibration(info, stamp):
    if (stamp_ns(info.header) != stamp or
            info.header.frame_id != "camera_optical_frame" or
            info.width != WIDTH or info.height != HEIGHT or
            info.distortion_model != "plumb_bob" or
            bool(info.roi.do_rectify)):
        raise ValueError("BLOCKED_V3_GEOMETRY_CAMERA_INFO")
    for name, expected in (("d", D), ("k", K), ("r", R), ("p", P)):
        got = getattr(info, name)
        if (len(got) != len(expected) or
                any(not math.isfinite(float(a)) or
                    abs(float(a) - float(b)) > 1e-10
                    for a, b in zip(got, expected))):
            raise ValueError("BLOCKED_V3_GEOMETRY_CALIBRATION_" + name.upper())


def audit_image(rect, info, phase):
    stamp = stamp_ns(rect.header)
    _check_calibration(info, stamp)
    if (rect.header.frame_id != "camera_optical_frame" or
            rect.width != WIDTH or rect.height != HEIGHT or
            rect.encoding != "mono8" or rect.is_bigendian != 0 or
            rect.step != WIDTH or len(rect.data) != WIDTH * HEIGHT):
        raise ValueError("BLOCKED_V3_GEOMETRY_IMAGE_LAYOUT")
    if not 0 <= phase < 13:
        raise ValueError("BLOCKED_V3_GEOMETRY_PHASE")

    abs_total = 0.0
    max_error = 0.0
    warped = 0
    far_border = 0
    samples = WIDTH * HEIGHT
    for y in range(HEIGHT):
        for x in range(WIDTH):
            actual = int(rect.data[y * rect.step + x])
            ideal = reference_pixel(x, y, phase)
            error = abs(actual - ideal)
            if error > MAX_ERROR:
                raise ValueError(
                    "BLOCKED_V3_GEOMETRY_PIXEL_MISMATCH:" +
                    str((x, y, round(error, 3), actual, round(ideal, 3))))
            max_error = max(max_error, error)
            abs_total += error
            if abs(actual - source_pixel(x, y, phase)) >= 4:
                warped += 1
            u, v = distorted_coordinate(x, y)
            if u < -1.0 or u > WIDTH or v < -1.0 or v > HEIGHT:
                far_border += 1
                if actual != 0:
                    raise ValueError("BLOCKED_V3_GEOMETRY_BORDER")
    mean_error = abs_total / samples
    if mean_error > MAX_MEAN_ERROR:
        raise ValueError("BLOCKED_V3_GEOMETRY_MEAN_ERROR")
    if warped < MIN_WARPED_PIXELS:
        raise ValueError("BLOCKED_V3_GEOMETRY_NOT_RECTIFIED")
    if far_border < MIN_FULL_BORDER_PIXELS:
        raise ValueError("BLOCKED_V3_GEOMETRY_BORDER_NOT_EXERCISED")
    return {"pixels": samples, "max_error": round(max_error, 3),
            "mean_error": round(mean_error, 3),
            "warped_pixels": warped, "zero_border_pixels": far_border}
