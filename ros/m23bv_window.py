#!/usr/bin/env python3
"""Pure, hardware-free M23BV selection contract for a closed ROS bag.

Selects by ORIGINAL sensor header stamps. Internal camera gaps remain
fatal. The original raw bag is never edited and physical time offset is
not inferred from stamp-domain overlap.
"""
from collections import Counter
from dataclasses import dataclass

TOPICS = {
    "/camera/image_rect": "image",
    "/camera/camera_info": "info",
    "/imu/data_raw": "imu",
}
GAP_IMAGE_NS = 250_000_000
GAP_IMU_NS = 300_000_000
MIN_COUNTS = {"image": 400, "info": 400, "imu": 5600}


@dataclass(frozen=True)
class Event:
    topic: str
    bag_ns: int
    header_ns: int
    frame_id: str
    width: int = 0
    height: int = 0


def select(events, *, min_counts=None):
    """Return (per-record keep flags, evidence). Raises on ANY bad input.

    `events` follows rosbag receipt order. The final interval is bounded by
    the earliest and latest IMU stamps; every retained Image must have
    matching CameraInfo and there must be zero >250ms internal image gaps.
    """
    if min_counts is None:
        min_counts = MIN_COUNTS
    if not events:
        raise ValueError("BLOCKED_EMPTY_BAG")
    if set(min_counts) != set(TOPICS.values()):
        raise ValueError("BLOCKED_INVALID_THRESHOLDS")
    first_imu = None
    last_imu = None
    bag_prev = 0
    stamp_prev = {}
    for e in events:
        kind = TOPICS.get(e.topic)
        if kind is None:
            raise ValueError("BLOCKED_UNEXPECTED_TOPIC")
        if type(e.bag_ns) is not int or e.bag_ns <= 0 or e.bag_ns < bag_prev:
            raise ValueError("BLOCKED_INVALID_BAG_TIME")
        bag_prev = e.bag_ns
        if type(e.header_ns) is not int or e.header_ns <= 0:
            raise ValueError("BLOCKED_INVALID_HEADER_STAMP")
        if (kind == "imu" and e.frame_id != "imu_link") or (
                kind != "imu" and e.frame_id != "camera_optical_frame"):
            raise ValueError("BLOCKED_UNEXPECTED_FRAME_ID")
        before = stamp_prev.get(kind)
        if before is not None and e.header_ns <= before:
            raise ValueError("BLOCKED_NONMONOTONIC_" + kind.upper())
        if kind == "imu":
            if before is not None and e.header_ns - before > GAP_IMU_NS:
                raise ValueError("BLOCKED_IMU_STAMP_GAP")
            if first_imu is None:
                first_imu = e.header_ns
            last_imu = e.header_ns
        stamp_prev[kind] = e.header_ns
    if first_imu is None:
        raise ValueError("BLOCKED_NO_IMU")
    # Strict intersection with observed IMU stamp window, inclusive.
    keep = []
    images = []
    info = {}
    counts = Counter()
    for e in events:
        kind = TOPICS[e.topic]
        retained = first_imu <= e.header_ns <= last_imu
        keep.append(retained)
        if not retained:
            continue
        counts[kind] += 1
        if kind in ("image", "info"):
            if not 0 < e.width <= 4096 or not 0 < e.height <= 4096:
                raise ValueError("BLOCKED_INVALID_CAMERA_GEOMETRY")
            key = (e.header_ns, e.frame_id)
            dims = (e.width, e.height)
            if kind == "info":
                if key in info and info[key] != dims:
                    raise ValueError("BLOCKED_CHANGED_CAMERA_INFO")
                info[key] = dims
            else:
                images.append((e.header_ns, key, dims))
    if any(counts[kind] < min_counts[kind] for kind in min_counts):
        raise ValueError("BLOCKED_INSUFFICIENT_RETAINED_STREAMS")
    for stamp, key, dims in images:
        if info.get(key) != dims:
            raise ValueError("BLOCKED_UNMATCHED_CAMERA_INFO")
    header_gaps = [b[0] - a[0] for a, b in zip(images, images[1:])]
    max_gap_ns = max(header_gaps, default=0)
    if any(gap > GAP_IMAGE_NS or gap <= 0 for gap in header_gaps):
        raise ValueError("BLOCKED_IMAGE_STAMP_GAP")
    if not (first_imu <= images[0][0] <= images[-1][0] <= last_imu):
        raise ValueError("BLOCKED_NO_IMU_STAMP_COVERAGE")
    report = {
        "milestone": "M23BV", "verdict": "HEADER_WINDOW_PRECHECK_PASS_ONLY",
        "first_imu_header_ns": first_imu, "last_imu_header_ns": last_imu,
        "first_retained_image_header_ns": images[0][0],
        "last_retained_image_header_ns": images[-1][0],
        "retained_counts": dict(counts),
        "excluded_counts": dict(Counter(TOPICS[e.topic] for e, take in zip(events, keep) if not take)),
        "max_image_gap_ms": max_gap_ns / 1e6,
        "image_gaps_over_250ms": 0,
        "matching_image_info": len(images),
        "physical_camera_imu_offset_calibrated": False,
        "metric_pose_authorized": False, "physical_motion_authorized": False,
    }
    return keep, report
