"""ROS-free read-only orientation-domain camera/IMU yaw lag diagnostics.

No robot commands; does not change the live temporal calibration estimator.
The board is assumed stationary. A pose matrix R_C_B maps board coordinates
into the current camera frame. Its relative rotation therefore has the
opposite sign to the camera's physical angular displacement.

Over Leroy's small supervised yaw excursions, projecting its rotation vector
into the *existing* IMU z axis gives a noisy visual angular position.
Comparing that against integrated IMU angular velocity avoids taking a noisy
adjacent-frame visual derivative, without smearing timestamps over 0.25s.
This is a diagnostic, not a validated calibration estimator.
"""
import math

import cv2
import numpy as np


def visual_yaw_angles(pose_rows, rot_imu_cam):
    """Return (timestamp_ns, visual yaw radians), using first board pose as 0."""
    rows = sorted(pose_rows, key=lambda item: item[0])
    if not rows:
        return []
    reference = np.asarray(rows[0][1], dtype=float)
    r_imu_cam = np.asarray(rot_imu_cam, dtype=float)
    if reference.shape != (3, 3) or r_imu_cam.shape != (3, 3):
        raise ValueError('Expected 3x3 camera orientation and camera-to-IMU rotation')
    result = []
    for ts, rotation in rows:
        current = np.asarray(rotation, dtype=float)
        if current.shape != (3, 3) or not np.isfinite(current).all():
            continue
        relative = current @ reference.T
        rotvec, _ = cv2.Rodrigues(relative)
        yaw = -float((r_imu_cam @ rotvec.reshape(3))[2])
        if math.isfinite(yaw):
            result.append((int(ts), yaw))
    return result


def orientation_lag_scan(pose_rows, imu_rows, rot_imu_cam,
                         max_lag_ms=250, lag_step_ms=2):
    """Fit offset and constant gyro bias at each candidate lag.

    Visual angle ~= integrated gyro angle at (camera timestamp - lag)
                    + constant angle + linear drift.

    Fits nuisance intercept and slope separately per candidate and per half.
    Uses identical frames for all tested lags, avoiding edge-selection bias.
    Reports curvature and split stability but NEVER declares a lag valid.
    """
    observations = visual_yaw_angles(pose_rows, rot_imu_cam)
    imu = sorted((int(t), float(z)) for t, z in imu_rows
                 if math.isfinite(float(z)))
    if len(observations) < 100 or len(imu) < 500:
        return {'reason': 'Insufficient ChArUco observations or IMU samples',
                'visual_pose_count': len(observations), 'imu_count': len(imu)}
    if not 10 <= max_lag_ms <= 500 or lag_step_ms <= 0:
        raise ValueError('Invalid lag search range')

    origin_ns = min(observations[0][0], imu[0][0])
    camera_t = np.asarray([(t - origin_ns) * 1e-9 for t, _ in observations])
    observed = np.asarray([angle for _, angle in observations])
    imu_t = np.asarray([(t - origin_ns) * 1e-9 for t, _ in imu])
    imu_z = np.asarray([z for _, z in imu])

    if (np.any(np.diff(camera_t) <= 0) or np.any(np.diff(imu_t) <= 0)):
        return {'reason': 'Nonmonotonic or repeated sensor timestamps'}
    if not (np.isfinite(camera_t).all() and np.isfinite(imu_t).all()):
        return {'reason': 'Nonfinite timestamps'}
    if np.std(imu_z) < 0.035:
        return {'reason': 'Insufficient real IMU yaw variation',
                'imu_yaw_std_rad_s': float(np.std(imu_z))}

    imu_integral = np.r_[
        0.0, np.cumsum((imu_z[1:] + imu_z[:-1]) * np.diff(imu_t) * 0.5)
    ]
    max_lag_s = max_lag_ms * 1e-3
    common = ((camera_t - max_lag_s >= imu_t[0]) &
              (camera_t + max_lag_s <= imu_t[-1]))
    camera_t = camera_t[common]
    observed = observed[common]
    if len(camera_t) < 100:
        return {'reason': 'Too few camera observations with common IMU coverage',
                'visual_pose_count': len(camera_t)}

    lags = np.arange(-max_lag_ms, max_lag_ms + 1,
                     lag_step_ms, dtype=int)
    mid = (camera_t[0] + camera_t[-1]) * 0.5

    def fit(selection):
        t = camera_t[selection]
        y = observed[selection]
        if len(t) < 45:
            return None
        x = np.column_stack((np.ones(len(t)), t - np.mean(t)))
        losses = []
        slopes = []
        for lag in lags:
            reference = np.interp(t - lag * 1e-3, imu_t, imu_integral)
            # Unknown reference yaw and constant gyro bias are nuisance
            # variables, not changes to the existing camera-to-IMU TF.
            coefficients, _, _, _ = np.linalg.lstsq(x, y - reference, rcond=None)
            residual = y - reference - x @ coefficients
            losses.append(float(np.mean(residual**2)))
            slopes.append(float(coefficients[1]))
        losses = np.asarray(losses)
        if not np.isfinite(losses).all():
            return None
        k = int(np.argmin(losses))
        allowed = np.abs(lags - lags[k]) >= 40
        competitor = float(np.min(losses[allowed])) if allowed.any() else float('nan')
        minimum = float(losses[k])
        # The local connected 1% loss contour describes the width near
        # the best lag. A broad contour implies weak time localization.
        lo = hi = k
        while lo > 0 and losses[lo - 1] <= minimum * 1.01:
            lo -= 1
        while hi + 1 < len(losses) and losses[hi + 1] <= minimum * 1.01:
            hi += 1
        return {
            'lag_ms': int(lags[k]),
            'angular_residual_rmse_deg': math.degrees(math.sqrt(minimum)),
            'loss_gap_40ms_pct': (100.0 * (competitor - minimum)
                                  / max(minimum, 1e-12)),
            'one_pct_loss_interval_ms': [int(lags[lo]), int(lags[hi])],
            'bias_drift_rad_s': float(slopes[k]),
            'pose_count': len(t),
        }

    total = fit(np.ones(len(camera_t), dtype=bool))
    first = fit(camera_t < mid)
    second = fit(camera_t >= mid)
    return {
        'method': 'visual angle vs integrated IMU yaw (diagnostic only)',
        'visual_pose_count': len(camera_t),
        'imu_count': len(imu),
        'imu_yaw_std_rad_s': float(np.std(imu_z)),
        'visual_angle_span_deg': math.degrees(float(np.ptp(observed))),
        'all': total,
        'first_half': first,
        'second_half': second,
        'valid': False,
        'warning': ('Do not apply the resulting lag to XRSLAM. '
                    'This offline diagnostic is not independently validated '
                    'for final timing calibration.'),
    }
