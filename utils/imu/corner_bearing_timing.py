"""Read-only ChArUco fiducial-bearing versus IMU-gyro timing diagnostic.

The visual observable is the observed pixel location of one physically fixed
ChArUco corner. Existing intrinsics and existing camera-to-IMU rotation
convert each measured image point into a bearing in the IMU XY plane. NO PnP
orientation, camera-IMU spatial recalibration, or motor command is involved.

A fitted scale accommodates the fact that the moving camera optical center
may not coincide with Leroy's yaw center. This is diagnostic ONLY; it must
never be used to certify or apply a temporal offset without independent tests.
"""
import math

import numpy as np


def bearing_lag_scan(track, imu_rows, max_lag_ms=250, step_ms=2):
    """Compare raw ChArUco corner bearing with time-shifted integrated gyro.

    track: sequence of (camera_header_timestamp_ns, bearing_rad) for ONE corner.
    imu_rows: sequence of (imu_header_timestamp_ns, gyro_z_rad_s).
    For each lag, fit a bearing intercept, slow linear drift, and gyro angle
    scale. All candidate lags use the exact same camera observations.
    """
    track = sorted((int(t), float(a)) for t, a in track if math.isfinite(float(a)))
    imu = sorted((int(t), float(w)) for t, w in imu_rows if math.isfinite(float(w)))
    if len(track) < 100 or len(imu) < 500:
        return {'reason': 'Insufficient corner tracking or IMU data',
                'corner_observations': len(track), 'imu_samples': len(imu)}
    if not 10 <= max_lag_ms <= 500 or step_ms < 1:
        raise ValueError('Invalid lag search range')
    origin = min(track[0][0], imu[0][0])
    t = np.array([(ts - origin) * 1e-9 for ts, _ in track])
    # Short yaw-only excursions avoid phase-wrap ambiguity; unwrap for safety.
    bearing = np.unwrap(np.array([ang for _, ang in track]))
    ti = np.array([(ts - origin) * 1e-9 for ts, _ in imu])
    gyro = np.array([w for _, w in imu])
    if np.any(np.diff(t) <= 0) or np.any(np.diff(ti) <= 0):
        return {'reason': 'Duplicate/nonmonotonic timestamps'}
    if np.std(gyro) < 0.035:
        return {'reason': 'Insufficient IMU yaw excitation'}
    integ = np.r_[0., np.cumsum((gyro[1:] + gyro[:-1]) * np.diff(ti) * 0.5)]
    bound = max_lag_ms * 1e-3
    common = (t - bound >= ti[0]) & (t + bound <= ti[-1])
    t, bearing = t[common], bearing[common]
    if len(t) < 100:
        return {'reason': 'Insufficient corner frames with common time coverage',
                'corner_observations': len(t)}
    lags = np.arange(-max_lag_ms, max_lag_ms + 1, step_ms)
    split = (t[0] + t[-1]) / 2

    def fit(mask):
        tt, yy = t[mask], bearing[mask]
        if len(tt) < 45:
            return None
        constant = np.ones(len(tt))
        centered_time = tt - tt.mean()
        losses, scales = [], []
        for lag in lags:
            integrated = np.interp(tt - lag / 1000, ti, integ)
            design = np.column_stack((constant, centered_time, integrated))
            coeff, _, _, _ = np.linalg.lstsq(design, yy, rcond=None)
            resid = yy - design @ coeff
            losses.append(float(np.mean(resid * resid)))
            scales.append(float(coeff[2]))
        losses = np.asarray(losses)
        if not np.isfinite(losses).all():
            return None
        best = int(np.argmin(losses))
        lowest = float(losses[best])
        far = np.abs(lags - lags[best]) >= 40
        competitor = float(np.min(losses[far])) if far.any() else float('nan')
        left = right = best
        while left > 0 and losses[left - 1] <= lowest * 1.01:
            left -= 1
        while right + 1 < len(lags) and losses[right + 1] <= lowest * 1.01:
            right += 1
        return {
            'lag_ms': int(lags[best]),
            'bearing_rmse_deg': math.degrees(math.sqrt(lowest)),
            'gain': scales[best],
            'loss_gap_40ms_pct': (competitor - lowest) * 100 / max(lowest, 1e-12),
            'one_pct_loss_interval_ms': [int(lags[left]), int(lags[right])],
            'observations': len(tt),
        }

    return {
        'all': fit(np.ones(len(t), dtype=bool)),
        'first_half': fit(t < split),
        'second_half': fit(t >= split),
        'corner_observations': len(t),
        'bearing_span_deg': math.degrees(float(np.ptp(bearing))),
        'valid': False,
        'warning': 'Independent visual bearing diagnostic only. Do NOT apply lag to XRSLAM.',
    }


def pixels_to_imu_bearings(pixel_xy, camera_matrix, distortion, rot_imu_cam):
    """Project existing raw camera pixels to observed azimuth in IMU frame.

    Uses measured CameraInfo K/D and existing TF rotation, WITHOUT PnP.
    Positive bearing is towards IMU +Y (left); when Leroy yaws left,
    a fixed target generally moves towards decreasing bearing.
    """
    import cv2
    pixel_xy = np.asarray(pixel_xy, dtype=np.float64).reshape(-1, 1, 2)
    k = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    d = np.asarray(distortion, dtype=np.float64).reshape(-1)
    r = np.asarray(rot_imu_cam, dtype=np.float64).reshape(3, 3)
    undistorted = cv2.undistortPoints(pixel_xy, k, d).reshape(-1, 2)
    cam_rays = np.column_stack((undistorted, np.ones(len(undistorted))))
    imu_rays = (r @ cam_rays.T).T
    angles = np.arctan2(imu_rays[:, 1], imu_rays[:, 0])
    return angles, imu_rays[:, 0], undistorted
