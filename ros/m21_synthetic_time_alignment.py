#!/usr/bin/env python3
"""Offline scalar motion-signal lag experiment; no ROS, camera, serial or motors.

Offset convention: IMU timestamp minus camera timestamp for the same feature.
This estimates effective feature alignment only, NOT exposure/acquisition phase.
"""
import argparse
from bisect import bisect_left
import json
import math
import random
import statistics


def validate(samples, label, minimum):
    if not isinstance(samples, list) or len(samples) < minimum:
        raise ValueError(label + ': insufficient samples')
    result = []
    old_t = -math.inf
    for pair in samples:
        if not isinstance(pair, (list, tuple)) or len(pair) != 2:
            raise ValueError(label + ': each sample requires [stamp_s, scalar_motion]')
        t, value = pair
        if not isinstance(t, (int, float)) or not isinstance(value, (int, float)):
            raise ValueError(label + ': nonnumeric sample')
        if not math.isfinite(t) or not math.isfinite(value) or t <= old_t:
            raise ValueError(label + ': timestamps must be finite and strictly increasing')
        result.append((float(t), float(value)))
        old_t = t
    if result[-1][0] - result[0][0] < 7:
        raise ValueError(label + ': at least seven seconds of data required')
    if statistics.pstdev(p[1] for p in result) < 0.05:
        raise ValueError(label + ': insufficient motion excitation')
    return result


def percentile(values, fraction):
    ordered = sorted(values)
    x = fraction * (len(ordered) - 1)
    a = int(x)
    return ordered[a] + (ordered[min(a + 1, len(ordered) - 1)] - ordered[a]) * (x - a)


def clip_series(samples, lo=0.05, hi=0.95):
    values = [v for _, v in samples]
    lower, upper = percentile(values, lo), percentile(values, hi)
    return [(t, max(lower, min(upper, v))) for t, v in samples]


def correlation(xs, ys):
    if len(xs) < 20:
        return None
    mean_x, mean_y = statistics.fmean(xs), statistics.fmean(ys)
    vx = sum((v - mean_x) ** 2 for v in xs)
    vy = sum((v - mean_y) ** 2 for v in ys)
    if vx < 1e-8 or vy < 1e-8:
        return None
    return sum((a - mean_x) * (b - mean_y) for a, b in zip(xs, ys)) / math.sqrt(vx * vy)


def score(camera, imu_t, imu_v, lag, max_imu_gap_s=0.075):
    xs, ys = [], []
    for t_cam, v_cam in camera:
        target = t_cam + lag
        index = bisect_left(imu_t, target)
        if index == 0 or index >= len(imu_t):
            continue
        a, b = imu_t[index - 1], imu_t[index]
        if b - a > max_imu_gap_s:
            continue  # avoid fabricating values across packet outages
        alpha = (target - a) / (b - a)
        motion = imu_v[index - 1] * (1.0 - alpha) + imu_v[index] * alpha
        xs.append(v_cam)
        ys.append(motion)
    return (correlation(xs, ys), len(xs))


def estimate(camera_samples, imu_samples, limit_s=0.20):
    if not 0.03 <= limit_s <= 0.5:
        raise ValueError('lag search limit must be 30 to 500 ms')
    camera = clip_series(validate(camera_samples, 'camera_motion', 80))
    imu = clip_series(validate(imu_samples, 'imu_motion', 180))
    imu_t, imu_v = zip(*imu)
    required = max(70, math.ceil(0.72 * len(camera)))
    scored = []
    # Coarse grid, then refine near the best; never optimize against a true offset.
    steps = int(round(2 * limit_s / 0.004))
    for k in range(steps + 1):
        lag = -limit_s + k * (2 * limit_s / steps)
        corr, used = score(camera, imu_t, imu_v, lag)
        if corr is not None and used >= required:
            scored.append((corr, lag, used))
    if not scored:
        raise ValueError('no sufficiently overlapping, valid signals')
    best = max(scored)
    refined = []
    for step in range(-20, 21):
        lag = best[1] + step * 0.0005
        if abs(lag) > limit_s:
            continue
        corr, used = score(camera, imu_t, imu_v, lag)
        if corr is not None and used >= required:
            refined.append((corr, lag, used))
    if not refined:
        raise ValueError('lag-refinement failed')
    best = max(refined)
    alternatives = [p[0] for p in scored if abs(p[1] - best[1]) >= 0.060]
    alternative = max(alternatives, default=-1)
    windows = [camera[:len(camera)//2], camera[len(camera)//2:]]
    stability = []
    for window in windows:
        vals = []
        for k in range(-30, 31):
            lag = best[1] + k * 0.001
            if abs(lag) > limit_s:
                continue
            corr, used = score(window, imu_t, imu_v, lag)
            if corr is not None and used >= max(30, math.ceil(0.65 * len(window))):
                vals.append((corr, lag))
        if vals:
            stability.append(max(vals))
    reasons=[]
    if best[0] < 0.85: reasons.append('weak_feature_correlation')
    if best[0] - alternative < 0.015: reasons.append('ambiguous_lag_peak')
    if abs(best[1]) > limit_s - 0.015: reasons.append('lag_at_search_boundary')
    if len(stability) != 2 or any(p[0] < 0.75 for p in stability):
        reasons.append('insufficient_split_window_correlation')
    if len(stability) == 2 and abs(stability[0][1] - stability[1][1]) > 0.025:
        reasons.append('unstable_split_window_lags')
    return {
        'method': 'offline_scalar_feature_interpolated_correlation',
        'sign_convention': 'imu_timestamp_minus_camera_timestamp_for_same_feature',
        'effective_feature_lag_s': round(best[1], 5) if not reasons else None,
        'candidate_lag_s': round(best[1], 5),
        'correlation': round(best[0], 4),
        'second_peak_correlation_outside_60ms': round(alternative, 4),
        'camera_samples': len(camera), 'imu_samples': len(imu),
        'matched_camera_samples': best[2],
        'split_candidate_lags_s': [round(p[1], 5) for p in stability],
        'rejection_reasons': reasons,
        'physical_camera_imu_offset_validated': False,
        'camera_imu_extrinsics_validated': False,
        'verdict': ('OFFLINE_FEATURE_ALIGNMENT_CANDIDATE' if not reasons
                    else 'OFFLINE_FEATURE_ALIGNMENT_REJECTED'),
    }


def signal(t):
    return (1.35 + 0.43 * math.sin(2 * math.pi * (0.31 * t + 0.019 * t * t))
            + 0.30 * math.sin(2 * math.pi * (0.76 * t - 0.011 * t * t) + 0.3)
            + 0.17 * math.cos(2 * math.pi * 0.15 * t))


def synthetic(offset_s=0.047, seed=7, drop_window=(4.2, 5.6),
              outlier_fraction=0.0, stationary=False):
    rng = random.Random(seed)
    camera, imu = [], []
    for k in range(18 * 17):
        t = 0.15 + k / 17.36 + rng.uniform(-0.001, 0.001)
        if drop_window and drop_window[0] <= t <= drop_window[1]:
            continue
        v = 1.1 if stationary else signal(t) + rng.gauss(0, 0.012)
        if rng.random() < outlier_fraction:
            v += rng.choice([-1, 1]) * 1.7
        camera.append([round(t, 7), round(v, 7)])
    for k in range(18 * 50 + 20):
        t = k / 50 + rng.uniform(-0.0006, 0.0006)
        v = 1.1 if stationary else signal(t - offset_s) + rng.gauss(0, 0.014)
        imu.append([round(t, 7), round(v, 7)])
    return camera, imu


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--demo', action='store_true', help='generate deterministic synthetic traces')
    ap.add_argument('--offset-ms', type=float, default=47.0,
                    help='known injected SYNTHETIC offset for demo only')
    ap.add_argument('--input', help='offline JSON: {"camera_motion": [[t,scalar]], "imu_motion": [[t,scalar]]}')
    args = ap.parse_args(argv)
    if args.demo == bool(args.input):
        ap.error('choose exactly one of --demo or --input')
    try:
        if args.demo:
            camera, imu = synthetic(offset_s=args.offset_ms / 1000.0)
        else:
            with open(args.input, encoding='utf-8') as stream:
                data = json.load(stream)
            camera, imu = data['camera_motion'], data['imu_motion']
        report = estimate(camera, imu)
        report['evidence_type'] = 'SYNTHETIC_INJECTION' if args.demo else 'OFFLINE_FEATURE_DATA_UNCALIBRATED'
        if args.demo:
            report['synthetic_injected_offset_s'] = args.offset_ms / 1000.0
            report['injection_recovered_within_10ms'] = bool(
                report['effective_feature_lag_s'] is not None and
                abs(report['effective_feature_lag_s'] - args.offset_ms / 1000.0) <= 0.010)
        candidate_ok = report['verdict'] == 'OFFLINE_FEATURE_ALIGNMENT_CANDIDATE'
        if args.demo:
            accepted = candidate_ok and report['injection_recovered_within_10ms']
            report['verdict'] = ('SYNTHETIC_KNOWN_LAG_RECOVERED' if accepted
                                 else 'SYNTHETIC_KNOWN_LAG_NOT_RECOVERED')
        else:
            accepted = candidate_ok
            report['verdict'] = ('OFFLINE_FEATURE_ALIGNMENT_CANDIDATE_UNCALIBRATED'
                                 if accepted else 'OFFLINE_FEATURE_ALIGNMENT_REJECTED')
        print('M21_ALIGNMENT_REPORT_JSON=' + json.dumps(report, sort_keys=True))
        return 0 if accepted else 2
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print('M21_ALIGNMENT_ERROR=' + type(exc).__name__ + ': ' + str(exc))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
