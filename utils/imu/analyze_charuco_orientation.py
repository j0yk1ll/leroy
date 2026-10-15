#!/usr/bin/env python3
"""Read-only camera-orientation versus integrated IMU yaw timing diagnostic.

Loads the already-recorded ChArUco bag. Never publishes motor commands,
edits camera intrinsics, writes bags, or changes camera-to-IMU extrinsics.
"""
import argparse
import json
import sys
from pathlib import Path

from analyze_charuco_bag import load_bag, live_extrinsic_rotation, extract_poses
from orientation_timing import orientation_lag_scan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, help='Existing ROS 2 bag directory')
    parser.add_argument('--target', required=True,
                        help='Existing printed ChArUco board_config.json')
    args = parser.parse_args()

    images, imu, info, camera_frame, imu_frame = load_bag(args.bag)
    rotation = live_extrinsic_rotation(camera_frame, imu_frame)
    poses, stats = extract_poses(images, info, Path(args.target), 8, 2.5)
    imu_rows = [(t, z) for t, z, _ in imu]
    print('READ-ONLY ORIENTATION/IMU DIAGNOSTIC — NO MOTOR COMMANDS')
    print(f'Existing TF: {camera_frame} -> {imu_frame}')
    print(f'Bag: {args.bag}')
    print('ChArUco replay:', json.dumps(stats))
    result = orientation_lag_scan(poses, imu_rows, rotation)
    print(json.dumps(result, indent=2))
    print('These are diagnostic lag candidates, not validated calibration.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
