"""ROS-free optical depth geometry, time guards and rigid transforms.

DepthART output is estimated metric depth, not safety-grade ranging. Unknown
pixels never count as observed free space. No guessed camera extrinsics.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Projection:
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_rectified_p(cls, p):
        if len(p) != 12 or not all(math.isfinite(float(x)) for x in p):
            raise ValueError('CameraInfo.P must have 12 finite values')
        fx, fy, cx, cy = map(float, (p[0], p[5], p[2], p[6]))
        if fx <= 0.0 or fy <= 0.0:
            raise ValueError('CameraInfo.P must have positive fx and fy')
        # Stereo right-camera translation is not supported by this adapter.
        # Reject rather than silently treating it as a monocular pinhole.
        if abs(float(p[3])) > 1e-8 or abs(float(p[7])) > 1e-8:
            raise ValueError('CameraInfo.P projection translation not supported')
        return cls(fx, fy, cx, cy)


def decode_depth_32fc1(*, encoding, width, height, step, data, is_bigendian):
    """Copy a potentially row-padded, endian-specific ROS image as float32."""
    if encoding != '32FC1' or width < 1 or height < 1:
        raise ValueError('Depth image must be nonempty 32FC1')
    if step < width * 4 or step % 4:
        raise ValueError('Invalid 32FC1 row stride')
    if len(data) != step * height:
        raise ValueError('Depth buffer size does not match header and row stride')
    dtype = np.dtype('>f4' if is_bigendian else '<f4')
    return np.ndarray(
        shape=(height, width), dtype=dtype, buffer=bytes(data),
        strides=(step, 4)
    ).astype(np.float32, copy=True)


def depth_to_optical_points(depth, projection, *, stride, min_depth_m, max_depth_m):
    """Return Nx3 optical coordinates (+x right, +y down, +z forward)."""
    if not isinstance(depth, np.ndarray) or depth.ndim != 2:
        raise ValueError('Depth must be a two-dimensional ndarray')
    if not isinstance(stride, int) or stride < 1:
        raise ValueError('stride must be >= 1')
    if (not math.isfinite(min_depth_m) or not math.isfinite(max_depth_m)
            or min_depth_m <= 0 or max_depth_m <= min_depth_m):
        raise ValueError('Depth range must be positive, finite and ordered')
    h, w = depth.shape
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    z = depth[::stride, ::stride]
    good = np.isfinite(z) & (z >= min_depth_m) & (z <= max_depth_m)
    if not np.any(good):
        return np.empty((0, 3), dtype=np.float32)
    zz = z[good].astype(np.float64)
    uu = u[good]
    vv = v[good]
    x = (uu - projection.cx) * zz / projection.fx
    y = (vv - projection.cy) * zz / projection.fy
    return np.stack((x, y, zz), axis=-1).astype(np.float32)


def transform_xyz(points, translation, quaternion_xyzw):
    """Apply verified TF: base_xyz = R(q) * optical_xyz + translation."""
    xyz = np.asarray(points, dtype=np.float64)
    t = np.asarray(translation, dtype=np.float64)
    q = np.asarray(quaternion_xyzw, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or t.shape != (3,) or q.shape != (4,):
        raise ValueError('Invalid point/TF dimensions')
    if not np.all(np.isfinite(xyz)) or not np.all(np.isfinite(t)) or not np.all(np.isfinite(q)):
        raise ValueError('Point/TF components must be finite')
    norm = np.linalg.norm(q)
    if norm < 1e-6 or abs(norm - 1.0) > 0.01:
        raise ValueError('Invalid TF quaternion norm')
    q = q / norm
    vec = q[:3]
    # Rodrigues rotation using quaternion with scalar last.
    cross = np.cross(np.broadcast_to(vec, xyz.shape), xyz)
    rotated = xyz + 2.0 * (q[3] * cross + np.cross(np.broadcast_to(vec, xyz.shape), cross))
    return (rotated + t).astype(np.float32)


def select_obstacles(points_in_base, *, ground_z_m, min_clearance_m,
                     max_height_m, max_xy_range_m):
    """Height-window selection relative to an explicitly validated floor datum.

    Merely bounding z is not proof of obstacle classification. In particular,
    overhangs, ramps, missing depth and camera blind spots remain unresolved.
    """
    xyz = np.asarray(points_in_base, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError('Expected Nx3 base-frame points')
    numbers = (ground_z_m, min_clearance_m, max_height_m, max_xy_range_m)
    if not all(math.isfinite(float(n)) for n in numbers):
        raise ValueError('Ground/obstacle parameters must be finite')
    if min_clearance_m < 0 or max_height_m <= min_clearance_m or max_xy_range_m <= 0:
        raise ValueError('Invalid ground clearance, height or range')
    height = xyz[:, 2] - ground_z_m
    radial = np.hypot(xyz[:, 0], xyz[:, 1])
    valid = (np.all(np.isfinite(xyz), axis=1)
             & (height >= min_clearance_m)
             & (height <= max_height_m)
             & (radial <= max_xy_range_m))
    return xyz[valid]


class StampGate:
    """Reject missing, future, aged, or out-of-order acquisition timestamps."""

    def __init__(self, max_age_s=2.0, max_future_s=0.1):
        if (not math.isfinite(max_age_s) or max_age_s <= 0
                or not math.isfinite(max_future_s) or max_future_s < 0):
            raise ValueError('Invalid timestamp age limits')
        self.max_age_ns = round(max_age_s * 1e9)
        self.max_future_ns = round(max_future_s * 1e9)
        self.last_stamp_ns = None

    def check(self, stamp_ns, now_ns):
        if stamp_ns <= 0 or now_ns <= 0:
            return 'missing_timestamp'
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            return 'out_of_order'
        age = now_ns - stamp_ns
        if age < -self.max_future_ns:
            return 'future_timestamp'
        if age > self.max_age_ns:
            return 'stale_depth'
        self.last_stamp_ns = stamp_ns
        return None
