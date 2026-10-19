#!/usr/bin/env python3
"""M23J: read-only camera-gap and IMU boundary chronology of a CLOSED bag.

Inspects only an explicitly supplied existing sqlite3 ROS 2 bag.
No ROS node, publishers, camera, serial, device maps, motors or TF.
Header-vs-bag receipt comparisons are descriptive, not a physical
exposure-to-MCU acquisition time-offset calibration.
"""
import argparse
from bisect import bisect_left, bisect_right
import json
from pathlib import Path
import sys

from m23a_xrslam_rosbag_preflight import (
    IMAGE_MAX_GAP_NS, IMU_MAX_GAP_NS, stamp_ns)
from m23d_recorded_bag_readiness import _ensure_regular_bag_files
from m23e_offline_evidence_snapshot import bag_fingerprints

TOPIC_TYPES = {
    "/camera/image_rect": ("image", "sensor_msgs/msg/Image"),
    "/camera/camera_info": ("info", "sensor_msgs/msg/CameraInfo"),
    "/imu/data_raw": ("imu", "sensor_msgs/msg/Imu"),
}
KINDS = ("image", "info", "imu")
MAX_MESSAGES_PER_STREAM = 100_000
MAX_TOP_GAPS = 8


def _ms(nanoseconds):
    return round(nanoseconds / 1_000_000, 3)


def _quantiles_ns(items):
    if not items:
        return None
    data = sorted(items)
    n = len(data)
    return {
        "p50_ms": _ms(data[(n - 1) * 50 // 100]),
        "p90_ms": _ms(data[(n - 1) * 90 // 100]),
        "p95_ms": _ms(data[(n - 1) * 95 // 100]),
        "p99_ms": _ms(data[(n - 1) * 99 // 100]),
        "max_ms": _ms(data[-1]),
    }


def _buckets(ns):
    result = {
        "up_to_50ms": 0,
        "50_to_100ms": 0,
        "100_to_250ms": 0,
        "250_to_500ms": 0,
        "500_to_1000ms": 0,
        "over_1000ms": 0,
    }
    for dt in ns:
        if dt <= 50_000_000:
            result["up_to_50ms"] += 1
        elif dt <= 100_000_000:
            result["50_to_100ms"] += 1
        elif dt <= IMAGE_MAX_GAP_NS:
            result["100_to_250ms"] += 1
        elif dt <= 500_000_000:
            result["250_to_500ms"] += 1
        elif dt <= 1_000_000_000:
            result["500_to_1000ms"] += 1
        else:
            result["over_1000ms"] += 1
    return result


def _internal_count(sorted_timestamps, before, after):
    """How many other stream headers lie strictly INSIDE one image gap."""
    return max(0, bisect_left(sorted_timestamps, after) -
               bisect_right(sorted_timestamps, before))


class Timeline:
    def __init__(self):
        self.headers = {kind: [] for kind in KINDS}
        self.receipts = {kind: [] for kind in KINDS}
        self.faults = []

    def _fault(self, name):
        if name not in self.faults and len(self.faults) < 32:
            self.faults.append(name)

    def ingest(self, kind, header_ns, bag_receipt_ns):
        if kind not in KINDS:
            raise ValueError("unexpected topic kind")
        if (type(header_ns) is not int or header_ns <= 0
                or type(bag_receipt_ns) is not int or bag_receipt_ns <= 0):
            raise ValueError("invalid timestamp")
        stream = self.headers[kind]
        if len(stream) >= MAX_MESSAGES_PER_STREAM:
            raise ValueError("bounded timestamp limit exceeded")
        if stream and header_ns <= stream[-1]:
            self._fault("NONMONOTONIC_" + kind.upper() + "_HEADER")
        self.headers[kind].append(header_ns)
        self.receipts[kind].append(bag_receipt_ns)

    def report(self):
        counts = {kind: len(self.headers[kind]) for kind in KINDS}
        if any(counts[k] == 0 for k in KINDS):
            self._fault("MISSING_REQUIRED_SENSOR_STREAM")
        image = self.headers["image"]
        imu = self.headers["imu"]
        gap_records = []
        image_gaps = []
        image_receipt_gaps = []
        info_sorted = sorted(self.headers["info"])
        imu_sorted = sorted(imu)
        for i in range(1, len(image)):
            dt = image[i] - image[i - 1]
            rx = self.receipts["image"][i] - self.receipts["image"][i - 1]
            if rx <= 0:
                self._fault("NONMONOTONIC_IMAGE_BAG_RECEIPT")
            else:
                image_receipt_gaps.append(rx)
            if dt <= 0:
                continue
            image_gaps.append(dt)
            if dt > IMAGE_MAX_GAP_NS:
                gap_records.append({
                    "after_image_index": i - 1,
                    "start_relative_to_first_image_s": round(
                        (image[i - 1] - image[0]) / 1e9, 4),
                    "end_relative_to_first_image_s": round(
                        (image[i] - image[0]) / 1e9, 4),
                    "image_header_gap_ms": _ms(dt),
                    "image_bag_receipt_gap_ms": _ms(rx),
                    "camera_info_stamps_inside_gap": _internal_count(
                        info_sorted, image[i - 1], image[i]),
                    "imu_stamps_inside_gap": _internal_count(
                        imu_sorted, image[i - 1], image[i]),
                })
        longest = sorted(gap_records, key=lambda r:
                         (-r["image_header_gap_ms"], r["after_image_index"]))
        first_imu_minus_first_image_ms = None
        last_image_minus_last_imu_ms = None
        if image and imu:
            first_imu_minus_first_image_ms = _ms(imu[0] - image[0])
            last_image_minus_last_imu_ms = _ms(image[-1] - imu[-1])
        imu_coverage = (bool(image and imu)
                        and imu[0] <= image[0] and imu[-1] >= image[-1])
        imu_gaps = [
            max(0, imu[i] - imu[i - 1])
            for i in range(1, len(imu))
            if imu[i] > imu[i - 1]
        ]
        if gap_records:
            self._fault("BLOCKED_IMAGE_STAMP_GAP")
        if any(g > IMU_MAX_GAP_NS for g in imu_gaps):
            self._fault("BLOCKED_IMU_STAMP_GAP")
        if not imu_coverage:
            self._fault("BLOCKED_NO_IMU_STAMP_COVERAGE")
        timing_blocked = bool(self.faults)
        return {
            "milestone": "M23J",
            "verdict": ("BLOCKED_IMAGE_TIMING_OR_IMU_COVERAGE"
                        if timing_blocked else
                        "TIMELINE_DIAGNOSTIC_ONLY"),
            "counts": counts,
            "first_header_ns": {k: self.headers[k][0]
                                if self.headers[k] else None for k in KINDS},
            "last_header_ns": {k: self.headers[k][-1]
                               if self.headers[k] else None for k in KINDS},
            "image_header_intervals_ms": _quantiles_ns(image_gaps),
            "image_bag_receipt_intervals_ms": _quantiles_ns(
                image_receipt_gaps),
            "imu_header_intervals_ms": _quantiles_ns(imu_gaps),
            "image_header_gap_buckets": _buckets(image_gaps),
            "image_bag_receipt_gap_buckets": _buckets(image_receipt_gaps),
            "image_header_gaps_over_250ms": len(gap_records),
            "largest_image_header_gaps": longest[:MAX_TOP_GAPS],
            "first_imu_minus_first_image_ms":
                first_imu_minus_first_image_ms,
            "last_image_minus_last_imu_ms":
                last_image_minus_last_imu_ms,
            "imu_covers_entire_image_header_window": imu_coverage,
            "faults": list(self.faults),
            "physical_exposure_to_imu_phase_calibrated": False,
            "extrinsics_validated": False,
            "xrslam_tracking_validated": False,
            "device_access": False,
            "physical_motion_authorized": False,
        }


def inspect_closed_bag(folder):
    """ROS 2 CDR decode only; original metadata and DB never written."""
    _ensure_regular_bag_files(folder)
    before = bag_fingerprints(folder)
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image, CameraInfo, Imu

    types = {"image": Image, "info": CameraInfo, "imu": Imu}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(
        uri=str(folder), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"))
    topics = {x.name: x.type for x in reader.get_all_topics_and_types()}
    if set(topics) != set(TOPIC_TYPES):
        raise ValueError("unexpected or missing recorded sensor topics")
    for name, (_, expected) in TOPIC_TYPES.items():
        if topics[name] != expected:
            raise ValueError("wrong recorded message type for " + name)

    t = Timeline()
    while reader.has_next():
        name, payload, receipt_ns = reader.read_next()
        if name not in TOPIC_TYPES:
            raise ValueError("unexpected recorded topic")
        kind = TOPIC_TYPES[name][0]
        msg = deserialize_message(payload, types[kind])
        t.ingest(kind, stamp_ns(msg.header), receipt_ns)
    del reader
    after = bag_fingerprints(folder)
    report = t.report()
    report["bag_files_stable_during_audit"] = before == after
    report["sha256_manifest"] = before
    if before != after:
        report["faults"].append("BAG_CHANGED_DURING_AUDIT")
        report["verdict"] = "BLOCKED_IMAGE_TIMING_OR_IMU_COVERAGE"
    return report


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bag", type=Path, required=True)
    args = p.parse_args(argv)
    try:
        result = inspect_closed_bag(args.bag)
    except (OSError, ValueError, TypeError, RuntimeError,
            AttributeError, KeyError, ImportError) as exc:
        print("M23J_ERROR=" + type(exc).__name__ + ": " +
              str(exc), file=sys.stderr, flush=True)
        return 2
    print("M23J_TIMELINE_JSON=" + json.dumps(result, sort_keys=True),
          flush=True)
    print("M23J_VERDICT=" + result["verdict"], flush=True)
    print("M23J_DEVICE_ACCESS=NONE", flush=True)
    return 2 if result["verdict"].startswith("BLOCKED_") else 0


if __name__ == "__main__":
    sys.exit(main())
