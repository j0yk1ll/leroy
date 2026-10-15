#!/usr/bin/env python3
"""Read-only ChArUco bag diagnostics for camera–IMU yaw timing.

Compare the current adjacent-pose visual derivative against longer visual
rotation intervals. This script DOES NOT publish /cmd_vel, alter calibration,
or update the robot. It queries the existing live camera-to-IMU TF.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import rosbag2_py
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Imu
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'charuco'))
from pose import CharucoPoseEstimator
from calibrate_time_offset import (
    implausible_pose_jump, quaternion_matrix, search_offset,
)


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def load_bag(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    images, imu, info = [], [], None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/camera/camera_info':
            info = deserialize_message(data, CameraInfo)
        elif topic == '/camera/image_raw/compressed':
            images.append(deserialize_message(data, CompressedImage))
        elif topic == '/imu/data_raw':
            msg = deserialize_message(data, Imu)
            imu.append((stamp_ns(msg.header.stamp),
                        float(msg.angular_velocity.z), msg.header.frame_id))
    if info is None or not images or not imu:
        raise RuntimeError('Bag requires CameraInfo, compressed images, and IMU data')
    frames = {item.header.frame_id for item in images}
    imu_frames = {item[2] for item in imu}
    if len(frames) != 1 or len(imu_frames) != 1:
        raise RuntimeError('Inconsistent camera or IMU frame IDs in bag')
    return images, imu, info, frames.pop(), imu_frames.pop()


def live_extrinsic_rotation(camera_frame, imu_frame):
    # Read from robot TF. Do NOT recalibrate or replace this spatial transform.
    rclpy.init()
    node = Node('leroy_charuco_bag_diagnostic')
    tf_buffer = Buffer()
    listener = TransformListener(tf_buffer, node, spin_thread=False)
    try:
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                t = tf_buffer.lookup_transform(imu_frame, camera_frame, Time())
            except Exception:
                continue
            q = t.transform.rotation
            return quaternion_matrix((q.x, q.y, q.z, q.w))
        raise RuntimeError(
            f'Cannot query existing TF {camera_frame} -> {imu_frame}. '
            'Start the regular ROS robot state publisher first.')
    finally:
        del listener
        node.destroy_node()
        rclpy.shutdown()


def extract_poses(images, info, board, min_corners, max_rmse):
    estimator = CharucoPoseEstimator(board, min_corners, max_rmse)
    poses = []
    stats = {'attempted_frames': 0, 'valid_poses': 0,
             'failed_detections': 0, 'rejected_pose_jumps': 0}
    prior = None
    last_accepted_ns = None
    last_detection_ns = None

    for msg in images:
        ts = stamp_ns(msg.header.stamp)
        # Mimic the live detector's ~15Hz subsampling. Bag replay cannot
        # exactly reproduce wall-clock callback scheduling.
        if last_detection_ns is not None and (ts - last_detection_ns) < 55_000_000:
            continue
        last_detection_ns = ts
        stats['attempted_frames'] += 1
        gray = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8),
                            cv2.IMREAD_GRAYSCALE)
        if gray is None:
            stats['failed_detections'] += 1
            continue
        p = estimator.pose(gray, info)
        if p is None:
            stats['failed_detections'] += 1
            continue
        # Do not silently bridge long gaps after tracking is lost.
        if last_accepted_ns is not None and (ts - last_accepted_ns) > 300_000_000:
            prior = None
        if prior is not None:
            jump = implausible_pose_jump(
                p, prior, (ts - last_accepted_ns) / 1e9)
            if jump is not None:
                stats['rejected_pose_jumps'] += 1
                continue
        poses.append((ts, p['rot'].copy()))
        prior = p
        last_accepted_ns = ts
        stats['valid_poses'] += 1
    return poses, stats


def fmt(value):
    if value is None:
        return 'n/a'
    return f'{value:.3f}'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, help='Existing ROS 2 bag directory')
    parser.add_argument('--target', required=True,
                        help='Existing printed ChArUco board_config.json')
    parser.add_argument('--min-corners', type=int, default=8)
    parser.add_argument('--max-rmse', type=float, default=2.5)
    args = parser.parse_args()
    images, raw_imu, info, camera_frame, imu_frame = load_bag(args.bag)
    rotation = live_extrinsic_rotation(camera_frame, imu_frame)
    pose_rows, stats = extract_poses(
        images, info, Path(args.target), args.min_corners, args.max_rmse)
    imu_rows = [(stamp, omega) for stamp, omega, _ in raw_imu
                if np.isfinite(omega)]
    print('READ-ONLY BAG DIAGNOSTIC — no motor publisher')
    print(f'Camera frame: {camera_frame}; IMU frame: {imu_frame}')
    print('Existing TF camera -> IMU rotation:')
    print(rotation)
    print(f'Image frames: {len(images)}; IMU samples: {len(imu_rows)}')
    print('ChArUco pose reconstruction:', json.dumps(stats, sort_keys=True))
    print('Alignment comparison (diagnostic, NOT an accepted calibration):')
    print('mode | lag_ms | corr | peak_sep | visual_std | imu_std | first/second_ms | valid')
    for baseline in (None, 0.12, 0.25, 0.40):
        label = 'adjacent' if baseline is None else f'{baseline:.2f}s'
        result = search_offset(pose_rows, imu_rows, rotation,
                               visual_baseline_s=baseline)
        halves = '/'.join(str(result.get(k, {}).get('lag_ms', '-'))
                          if result.get(k) else '-'
                          for k in ('first_half', 'second_half'))
        print(f'{label:>8} | {str(result.get("lag_ms", "-")):>6} | '
              f'{fmt(result.get("correlation")):>5} | '
              f'{fmt(result.get("peak_separation")):>8} | '
              f'{fmt(result.get("visual_yaw_std_rad_s")):>10} | '
              f'{fmt(result.get("imu_yaw_std_rad_s")):>7} | '
              f'{halves:>15} | {result.get("valid", False)}')
        if 'reason' in result:
            print(f'           reason: {result["reason"]}')
    print('Longer intervals reduce PnP derivative noise but broaden temporal')
    print('resolution and create correlated overlapping visual samples.')
    print('Do NOT apply any lag from this diagnostic to XRSLAM.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
