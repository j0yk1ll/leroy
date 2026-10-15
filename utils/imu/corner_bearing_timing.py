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


def bearing_interval_lag_scan(track, imu_rows, baseline_s=0.12,
                              max_lag_ms=250, step_ms=2):
    """Independent *nonoverlapping* raw-corner bearing-rate vs gyro test.

    In contrast to bearing_lag_scan(), this compares only angular changes
    over bounded disjoint intervals. Each interval's reference uses gyro
    trapezoidal integration over that very same shifted interval. Thus
    long-lived integrated-gyro drift, its initial orientation, and most slow
    camera-target translations cannot themselves create an apparent lag.
    Intervals are intentionally nonoverlapping to reduce pseudoreplication.
    Returns diagnostic-only estimates, not a calibrated offset.
    """
    if not 0.08 <= baseline_s <= 0.40:
        raise ValueError('baseline_s must be between 0.08 and 0.40 seconds')
    if not 10 <= max_lag_ms <= 500 or step_ms < 1:
        raise ValueError('Invalid lag search range')
    track = sorted((int(t), float(a)) for t, a in track
                   if math.isfinite(float(a)))
    imu = sorted((int(t), float(w)) for t, w in imu_rows
                 if math.isfinite(float(w)))
    if len(track) < 100 or len(imu) < 500:
        return {'reason': 'Insufficient corner/IMU samples',
                'bearing_observations': len(track)}
    origin = min(track[0][0], imu[0][0])
    t = np.array([(ts - origin) * 1e-9 for ts, _ in track])
    angle = np.unwrap(np.array([a for _, a in track]))
    ti = np.array([(ts - origin) * 1e-9 for ts, _ in imu])
    z = np.array([w for _, w in imu])
    if np.any(np.diff(t) <= 0) or np.any(np.diff(ti) <= 0):
        return {'reason': 'Nonmonotonic or repeated timestamps'}
    if np.std(z) < 0.035:
        return {'reason': 'Insufficient IMU yaw excitation'}
    integ = np.r_[0., np.cumsum((z[1:] + z[:-1]) * np.diff(ti) * 0.5)]

    # A nonoverlapping secant is constructed only from the same physical
    # ChArUco corner ID (track is supplied per-ID by the bag analyzer).
    pairs = []
    i = 0
    while i + 1 < len(t):
        j = i + 1
        while j < len(t) and t[j] - t[i] < baseline_s:
            j += 1
        if j == len(t):
            break
        dt = t[j] - t[i]
        if dt <= baseline_s + 0.12:
            pairs.append((t[i], t[j], (angle[j] - angle[i]) / dt))
            i = j
        else:
            i += 1
    if len(pairs) < 40:
        return {'reason': 'Insufficient disjoint visual angle intervals',
                'visual_intervals': len(pairs)}
    a, b, vz = (np.array(x) for x in zip(*pairs))
    bound = max_lag_ms * 1e-3
    common = ((a - bound >= ti[0]) & (b + bound <= ti[-1]))
    a, b, vz = a[common], b[common], vz[common]
    if len(a) < 40:
        return {'reason': 'Insufficient common-coverage visual intervals',
                'visual_intervals': len(a)}
    lags = np.arange(-max_lag_ms, max_lag_ms + 1, step_ms, dtype=int)
    mid = (a[0] + b[-1]) * 0.5

    def fit(mask):
        aa, bb, vv = a[mask], b[mask], vz[mask]
        if len(aa) < 18 or np.std(vv) < 0.025:
            return None
        losses = []
        gains = []
        corrs = []
        for lag in lags:
            offset_s = lag / 1000.
            ref = (np.interp(bb-offset_s, ti, integ) -
                   np.interp(aa-offset_s, ti, integ)) / (bb-aa)
            if np.std(ref) < 0.025:
                losses.append(float('nan'))
                gains.append(float('nan'))
                corrs.append(float('nan'))
                continue
            matrix = np.column_stack((np.ones(len(ref)), ref))
            coeff, _, _, _ = np.linalg.lstsq(matrix, vv, rcond=None)
            residue = vv - matrix @ coeff
            losses.append(float(np.mean(residue**2)))
            gains.append(float(coeff[1]))
            corrs.append(float(np.corrcoef(vv, ref)[0, 1]))
        losses = np.asarray(losses)
        if not np.isfinite(losses).any():
            return None
        k = int(np.nanargmin(losses))
        lowest = float(losses[k])
        allowed = (np.abs(lags - lags[k]) >= 40) & np.isfinite(losses)
        competing = float(np.min(losses[allowed])) if allowed.any() else float('nan')
        left = right = k
        while left > 0 and np.isfinite(losses[left-1]) and losses[left-1] <= lowest * 1.01:
            left -= 1
        while right+1 < len(lags) and np.isfinite(losses[right+1]) and losses[right+1] <= lowest * 1.01:
            right += 1
        return {
            'lag_ms': int(lags[k]),
            'correlation': float(corrs[k]),
            'gain': float(gains[k]),
            'bearing_rate_rmse_rad_s': float(math.sqrt(lowest)),
            'loss_gap_40ms_pct': 100 * (competing-lowest)/max(lowest, 1e-12),
            'one_pct_loss_interval_ms': [int(lags[left]), int(lags[right])],
            'intervals': len(aa),
        }

    return {
        'method': 'nonoverlapping raw-corner bearing intervals vs integrated gyro',
        'baseline_s': baseline_s,
        'visual_intervals': len(a),
        'all': fit(np.ones(len(a), dtype=bool)),
        'first_half': fit((a+b)/2 < mid),
        'second_half': fit((a+b)/2 >= mid),
        'valid': False,
        'warning': 'Offline cross-check ONLY, not an accepted XRSLAM lag.',
    }
