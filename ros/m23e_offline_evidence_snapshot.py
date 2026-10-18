#!/usr/bin/env python3
"""M23E immutable-ish evidence snapshot of an EXISTING read-only ROS 2 bag.

No live hardware, ROS node, publisher, pose, motor, sensor capture or
calibration estimation. SHA-256 fingerprints detect later byte changes,
NOT source authenticity. M23D's input profile remains authoritative for
format-only readiness, and is never converted into real-VIO approval.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import sys

from m23a_xrslam_rosbag_preflight import stamp_ns
from m23d_recorded_bag_readiness import (
    TOPICS, _ensure_regular_bag_files, inspect_recorded_candidate,
)

CHUNK_BYTES = 1024 * 1024
KNOWN_TOPIC_KINDS = ("image", "info", "imu")


def _file_digest(path):
    """O_NOFOLLOW + fstat: never hash symlinks/FIFOs/devices."""
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_NONBLOCK", 0)
    fd = os.open(path, flags)
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode):
            raise ValueError("evidence source must be a regular file")
        digest = hashlib.sha256()
        while True:
            data = os.read(fd, CHUNK_BYTES)
            if not data:
                break
            digest.update(data)
        after = os.fstat(fd)
        if (info.st_size != after.st_size
                or info.st_mtime_ns != after.st_mtime_ns):
            raise ValueError("file changed while hashing")
        return {"bytes": info.st_size, "sha256": digest.hexdigest()}
    finally:
        os.close(fd)


def bag_fingerprints(bag):
    """Fingerprint only bag metadata and explicitly named .db3 files."""
    _ensure_regular_bag_files(bag)
    import yaml
    manifest = yaml.safe_load((bag / "metadata.yaml").read_text("utf-8"))
    refs = manifest["rosbag2_bagfile_information"]["relative_file_paths"]
    if len(refs) != len(set(refs)):
        raise ValueError("duplicate rosbag file reference")
    names = ["metadata.yaml"] + sorted(refs)
    result = {}
    for name in names:
        if (Path(name).name != name or name in (".", "..")
                or name.startswith("/") or "/" in name or "\\" in name):
            raise ValueError("unsafe file reference")
        result[name] = _file_digest(bag / name)
    return result


class StreamEvidence:
    """Streaming summary of header vs bag receipt timestamps and metadata."""
    def __init__(self):
        self.counts = {kind: 0 for kind in KNOWN_TOPIC_KINDS}
        self.delay_min = {kind: None for kind in KNOWN_TOPIC_KINDS}
        self.delay_max = {kind: None for kind in KNOWN_TOPIC_KINDS}
        self.min_interval_ns = {kind: None for kind in KNOWN_TOPIC_KINDS}
        self.max_interval_ns = {kind: None for kind in KNOWN_TOPIC_KINDS}
        self.last = {}
        self.camera_projection = None
        self.camera_projection_changes = 0
        self.image_formats = set()
        self.image_geometry = set()
        self.issues = []

    def issue(self, code):
        if code not in self.issues and len(self.issues) < 32:
            self.issues.append(code)

    def consume(self, kind, receipt_ns, msg):
        if kind not in KNOWN_TOPIC_KINDS:
            self.issue("UNKNOWN_TOPIC")
            return
        self.counts[kind] += 1
        try:
            timestamp = stamp_ns(msg.header)
            if type(receipt_ns) is not int or receipt_ns <= 0:
                raise ValueError("invalid receipt timestamp")
        except (AttributeError, ValueError, TypeError, OverflowError):
            self.issue("INVALID_" + kind.upper() + "_TIMESTAMP")
            return
        # ROS timestamps may represent different clock domains. These
        # deltas are descriptive ONLY, NOT camera-IMU physical phase.
        delay = receipt_ns - timestamp
        low = self.delay_min[kind]
        high = self.delay_max[kind]
        self.delay_min[kind] = delay if low is None else min(low, delay)
        self.delay_max[kind] = delay if high is None else max(high, delay)
        if kind in self.last:
            interval = timestamp - self.last[kind]
            if interval <= 0:
                self.issue("BACKWARD_" + kind.upper() + "_HEADER")
            else:
                low = self.min_interval_ns[kind]
                high = self.max_interval_ns[kind]
                self.min_interval_ns[kind] = interval if low is None else min(low, interval)
                self.max_interval_ns[kind] = interval if high is None else max(high, interval)
        self.last[kind] = timestamp
        if kind == "image":
            try:
                self.image_formats.add(str(msg.encoding))
                self.image_geometry.add((int(msg.width), int(msg.height),
                                         int(msg.step)))
            except (AttributeError, ValueError, TypeError):
                self.issue("INVALID_IMAGE_GEOMETRY")
        if kind == "info":
            try:
                # P describes the rectified projection. Changes matter
                # for review but do NOT prove calibration accuracy.
                projection = tuple(float(x) for x in msg.p)
                if len(projection) != 12 or not all(map(math.isfinite, projection)):
                    raise ValueError("invalid projection")
                if self.camera_projection is None:
                    self.camera_projection = projection
                elif projection != self.camera_projection:
                    self.camera_projection_changes += 1
            except (TypeError, AttributeError, ValueError, OverflowError):
                self.issue("INVALID_CAMERA_PROJECTION")

    def report(self):
        as_ms = lambda n: None if n is None else round(n / 1e6, 6)
        return {
            "counts": dict(self.counts),
            "receipt_minus_header_ms": {
                kind: {"min": as_ms(self.delay_min[kind]),
                       "max": as_ms(self.delay_max[kind])}
                for kind in KNOWN_TOPIC_KINDS
            },
            "inter_header_interval_ms": {
                kind: {"min": as_ms(self.min_interval_ns[kind]),
                       "max": as_ms(self.max_interval_ns[kind])}
                for kind in KNOWN_TOPIC_KINDS
            },
            "image_encodings": sorted(self.image_formats),
            "image_geometry_variants": [
                {"width": w, "height": h, "step": st}
                for w, h, st in sorted(self.image_geometry)
            ],
            "camera_info_projection_changes": self.camera_projection_changes,
            "faults": list(self.issues),
        }


def inspect_existing_bag(bag, *, origin_label="unverified"):
    """Never starts ROS. M23D profile + two file hashes bracket CDR scan."""
    if origin_label not in ("unverified", "user_declared_recorded", "synthetic"):
        raise ValueError("unsupported origin label")
    before = bag_fingerprints(bag)
    profile = inspect_recorded_candidate(bag, declared_origin=origin_label)
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image, CameraInfo, Imu
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    topic_types = {x.name: x.type for x in reader.get_all_topics_and_types()}
    klass = {"image": Image, "info": CameraInfo, "imu": Imu}
    stream = StreamEvidence()
    while reader.has_next():
        topic, cdr, receipt_ns = reader.read_next()
        if topic not in TOPICS:
            continue
        kind, ros_type = TOPICS[topic]
        if topic_types.get(topic) != ros_type:
            stream.issue("WRONG_" + kind.upper() + "_TYPE")
            continue
        try:
            message = deserialize_message(cdr, klass[kind])
        except (TypeError, RuntimeError, ValueError):
            stream.issue("INVALID_" + kind.upper() + "_CDR")
            continue
        stream.consume(kind, receipt_ns, message)
    del reader
    after = bag_fingerprints(bag)
    if before != after:
        stream.issue("BAG_CHANGED_DURING_AUDIT")
    details = stream.report()
    if details["counts"] != profile["counts"]:
        stream.issue("M23D_COUNT_MISMATCH")
        details = stream.report()
    valid = (
        profile["verdict"] == "REPLAY_INPUT_RECORDING_PROFILE_ONLY"
        and not details["faults"]
    )
    return {
        "milestone": "M23E",
        "verdict": ("OFFLINE_EVIDENCE_SNAPSHOT_ONLY" if valid
                    else "BLOCKED_OFFLINE_EVIDENCE"),
        "declared_origin": origin_label,
        "bag_files": before,
        "bag_files_stable_during_audit": before == after,
        "m23d_profile": profile,
        "stream_evidence": details,
        "origin_authenticated": False,
        "recording_provenance_verified": False,
        "camera_imu_physical_offset_calibrated": False,
        "camera_imu_extrinsics_validated": False,
        "imu_noise_calibrated": False,
        "xrslam_tracking_validated": False,
        "metric_pose_authorized": False,
        "physical_motion_authorized": False,
        "device_access": False,
        "limitations": [
            "SHA-256 identifies these bytes; it does not authenticate their source",
            "Header-minus-bag-receipt deltas do not measure camera/IMU phase",
            "CameraInfo stability does not validate intrinsic/extrinsic accuracy",
            "No visual tracking or robot safety is tested",
        ],
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--origin-label", choices=(
        "unverified", "user_declared_recorded", "synthetic"),
        default="unverified")
    args = parser.parse_args(argv)
    try:
        result = inspect_existing_bag(args.bag, origin_label=args.origin_label)
    except (OSError, ValueError, RuntimeError, ImportError,
            TypeError, KeyError, AttributeError) as exc:
        print("M23E_ERROR=" + type(exc).__name__ + ": " + str(exc),
              file=sys.stderr)
        return 2
    print("M23E_EVIDENCE_JSON=" + json.dumps(result, sort_keys=True))
    print("M23E_VERDICT=" + result["verdict"])
    print("M23E_REAL_RECORDING_VERIFIED=FALSE")
    print("M23E_CALIBRATION_VALIDATED=FALSE")
    print("M23E_XRSLAM_TRACKING_VALIDATED=FALSE")
    print("M23E_DEVICE_ACCESS=NONE")
    return 0 if result["verdict"] == "OFFLINE_EVIDENCE_SNAPSHOT_ONLY" else 2


if __name__ == "__main__":
    raise SystemExit(main())
