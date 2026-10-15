#!/usr/bin/env python3
"""Diagnose camera/IMU temporal offset from raw ChArUco pixels, NO PnP.

Read-only; does not publish /cmd_vel or alter sensor calibration or any TF.
Uses the existing camera intrinsics, existing camera-to-IMU TF, and bag.
"""
import argparse
from collections import defaultdict
from pathlib import Path
import sys

import cv2
import numpy as np

from analyze_charuco_bag import load_bag, live_extrinsic_rotation, stamp_ns
from corner_bearing_timing import bearing_lag_scan, pixels_to_imu_bearings

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'charuco'))
from calibrate import load_board, detect_charuco, charuco_object_points


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag', required=True)
    ap.add_argument('--target', required=True)
    ap.add_argument('--max-tracks', type=int, default=8)
    args = ap.parse_args()

    images, imu, info, camera_frame, imu_frame = load_bag(args.bag)
    rotation = live_extrinsic_rotation(camera_frame, imu_frame)
    cfg, dictionary, board = load_board(Path(args.target))
    xyz = np.asarray(charuco_object_points(cfg), dtype=float)
    k = np.asarray(info.k, dtype=float).reshape(3, 3)
    d = np.asarray(info.d, dtype=float)
    tracks = defaultdict(list)
    stats = {'attempted_frames': 0, 'detector_frames': 0,
             'homography_rejections': 0, 'usable_frames': 0}
    last_detection_ns = None

    print('READ-ONLY RAW-CORNER BEARING DIAGNOSTIC — NO PnP / NO MOTORS', flush=True)
    print(f'Existing TF: {camera_frame} -> {imu_frame}', flush=True)
    print(f'Bag: {args.bag}', flush=True)
    for msg in images:
        ts = stamp_ns(msg.header.stamp)
        if last_detection_ns is not None and ts-last_detection_ns < 55_000_000:
            continue
        last_detection_ns = ts
        stats['attempted_frames'] += 1
        gray = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8),
                             cv2.IMREAD_GRAYSCALE)
        if gray is None:
            continue
        corners, ids, _, _ = detect_charuco(gray, board, dictionary)
        if ids is None or len(ids) < 8:
            continue
        ids = np.asarray(ids, dtype=int).reshape(-1)
        if len(np.unique(ids)) != len(ids) or ids.min() < 0 or ids.max() >= len(xyz):
            continue
        stats['detector_frames'] += 1
        pix = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
        bearings, forward, normalized = pixels_to_imu_bearings(
            pix, k, d, rotation)
        # A robust planar homography is used only to reject mismatched raw
        # corner IDs. Unlike solvePnP, it supplies NO visual orientation.
        h, mask = cv2.findHomography(
            xyz[ids, :2], normalized, cv2.RANSAC, 0.0035)
        if h is None or mask is None or np.count_nonzero(mask) < 8:
            stats['homography_rejections'] += 1
            continue
        stats['usable_frames'] += 1
        for fid, angle, front, good in zip(
                ids, bearings, forward, mask.reshape(-1)):
            if good and front > 0.05 and np.isfinite(angle):
                tracks[int(fid)].append((ts, float(angle)))

    print('Image processing:', stats, flush=True)
    order = sorted(tracks,
                   key=lambda fid: (-len(tracks[fid]),
                                    float(np.linalg.norm(xyz[fid, :2] -
                                                         xyz[:, :2].mean(axis=0))),
                                    fid))
    raw_imu = [(t, z) for t, z, _ in imu]
    print('ID | frames | lag ms | first/second | gain | RMSE deg | '
          'loss-gap-40ms % | 1% interval ms', flush=True)
    for fid in order[:args.max_tracks]:
        result = bearing_lag_scan(tracks[fid], raw_imu)
        all_fit = result.get('all')
        halves = [result.get('first_half'), result.get('second_half')]
        if all_fit is None:
            print(f'{fid:2d} | {len(tracks[fid]):>6d} | '
                  f"{result.get('reason')}", flush=True)
            continue
        halves_text = ('/'.join(f"{x['lag_ms']:+d}" if x else '?'
                                for x in halves))
        print(f"{fid:2d} | {len(tracks[fid]):>6d} | "
              f"{all_fit['lag_ms']:+5d} | {halves_text:>12s} | "
              f"{all_fit['gain']:+6.3f} | "
              f"{all_fit['bearing_rmse_deg']:.3f} | "
              f"{all_fit['loss_gap_40ms_pct']:6.2f} | "
              f"{all_fit['one_pct_loss_interval_ms']}", flush=True)
    print('Independent raw-pixel diagnostic only. '
          'DO NOT apply any lag to XRSLAM.', flush=True)


if __name__ == '__main__':
    main()
