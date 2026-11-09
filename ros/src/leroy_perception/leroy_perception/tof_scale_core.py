"""Strict, ROS-free ToF / relative-depth metric anchoring calculations.

No camera scale is generated without measured ToF->camera 6-DoF alignment,
time alignment, and confirmation that depth values are proportional to range.
The VL53L1X sees a wide cone: the center-ray projection is only a reference.
A high-variation ROI is rejected as geometrically ambiguous.
"""
import math
import numpy as np


def project_tof_hit(range_m, k, origin_xyz, rotation_xyzw):
    """Project the measured point along the ToF optical-axis center ray.

    Extrinsics transform ToF coordinates into the rectified camera frame;
    +Z in ToF space is the nominal measurement axis.
    """
    if not math.isfinite(range_m) or not 0.04 <= range_m <= 1.30:
        raise ValueError("VL53L1X short-mode range outside valid interval")
    tx, ty, tz = map(float, origin_xyz)
    qx, qy, qz, qw = map(float, rotation_xyzw)
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if not (0.999 <= norm <= 1.001):
        raise ValueError("uncalibrated ToF quaternion")
    # Third column of the ToF->camera rotation matrix (R * (0,0,1)).
    dx = 2.0 * (qx*qz + qw*qy)
    dy = 2.0 * (qy*qz - qw*qx)
    dz = 1.0 - 2.0 * (qx*qx + qy*qy)
    x, y, z = tx + range_m*dx, ty + range_m*dy, tz + range_m*dz
    fx, fy, cx, cy = map(float, k)
    if not all(map(math.isfinite, (x,y,z,fx,fy,cx,cy))) or z <= 0.05 or fx <= 0 or fy <= 0:
        raise ValueError("invalid camera projection or ToF beam behind image")
    return fx*x/z + cx, fy*y/z + cy, z


def scale_relative_depth(image, range_m, k, origin_xyz, rotation_xyzw,
                         radius_px, max_relative_spread=0.20):
    """Return (metric_depth, diagnostics) or raise ValueError.

    Only valid for relative DEPTH proportional to distance, not disparity.
    Uses the 20th percentile as a conservative target proxy, but ToF cone
    geometry is still ambiguous: reject nonuniform ROIs rather than assert
    exact pixel correspondence.
    """
    arr = np.asarray(image, dtype=np.float32)
    if arr.ndim != 2 or not 4 <= int(radius_px) <= 100:
        raise ValueError("invalid relative depth or ROI radius")
    u, v, z = project_tof_hit(range_m, k, origin_xyz, rotation_xyzw)
    h, w = arr.shape
    if u < radius_px or v < radius_px or u >= w-radius_px or v >= h-radius_px:
        raise ValueError("projected ToF center outside usable camera image")
    x0, x1 = int(round(u))-radius_px, int(round(u))+radius_px+1
    y0, y1 = int(round(v))-radius_px, int(round(v))+radius_px+1
    roi = arr[y0:y1, x0:x1]
    valid = roi[np.isfinite(roi) & (roi > 0)]
    if valid.size < roi.size * 0.90:
        raise ValueError("insufficient valid relative depth in ToF ROI")
    p10, p20, p90 = np.quantile(valid, [0.10,0.20,0.90])
    if p20 <= 0 or (p90-p10)/p20 > max_relative_spread:
        raise ValueError("heterogeneous ToF cone ROI, refuse scale")
    scale = z / float(p20)
    if not math.isfinite(scale) or not 0.01 <= scale <= 100.0:
        raise ValueError("implausible relative-depth scale")
    result = arr * np.float32(scale)
    result[~np.isfinite(result) | (result <= 0.05) | (result >= 50.0)] = np.nan
    return result, {"center_u": float(u), "center_v": float(v),
                    "range_m": float(range_m), "camera_z_m": float(z),
                    "scale": float(scale), "roi_spread": float((p90-p10)/p20)}
