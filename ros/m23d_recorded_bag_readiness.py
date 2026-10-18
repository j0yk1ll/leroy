#!/usr/bin/env python3
"""M23D: read-only, non-actuating ROS 2 camera/IMU bag readiness audit.

Uses the already accepted M23A format preflight, followed by a separate
recording-wide timing/variation profile. All signatures are sampled DATA
fingerprints, never proof of real image motion, capture provenance, clock
phase, or calibration. No rclpy.init, node, TF, serial, device, or actuator.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

from m23a_xrslam_rosbag_preflight import inspect_bag, stamp_ns

TOPICS = {
    "/camera/image_rect": ("image", "sensor_msgs/msg/Image"),
    "/camera/camera_info": ("info", "sensor_msgs/msg/CameraInfo"),
    "/imu/data_raw": ("imu", "sensor_msgs/msg/Imu"),
}
KINDS = ("image", "info", "imu")
MAX_SAMPLE_BYTES = 1024
MAX_DIAGNOSTIC_GAP_MS = 250.0


def _sample_signature(msg):
    """Bounded sample, NOT pixel-content validation or source identity."""
    data = msg.data
    n = len(data)
    if n <= 0:
        raise ValueError("image has no data")
    stride = max(1, math.ceil(n / MAX_SAMPLE_BYTES))
    try:
        sample = bytes(data[::stride][:MAX_SAMPLE_BYTES])
    except (TypeError, ValueError, OverflowError):
        raise ValueError("image has invalid byte payload") from None
    return hashlib.blake2b(sample, digest_size=8).hexdigest()


class RecordingProfile:
    def __init__(self):
        self.count = {kind: 0 for kind in KINDS}
        self.first = {}
        self.last = {}
        self.max_gap_ns = {kind: 0 for kind in KINDS}
        self.gaps_over_250ms = {kind: 0 for kind in KINDS}
        self.image_signature_set = set()
        self._global_last_bag_ns = None
        self.failures = []

    def fault(self, code):
        if code not in self.failures and len(self.failures) < 32:
            self.failures.append(code)

    def consume(self, kind, bag_ns, msg):
        if kind not in KINDS:
            self.fault("UNKNOWN_STREAM")
            return
        if type(bag_ns) is not int or bag_ns <= 0:
            self.fault("INVALID_BAG_TIME")
        elif (self._global_last_bag_ns is not None
              and bag_ns < self._global_last_bag_ns):
            self.fault("BACKWARD_BAG_TIME")
        if type(bag_ns) is int and bag_ns > 0:
            self._global_last_bag_ns = bag_ns
        self.count[kind] += 1
        try:
            t = stamp_ns(msg.header)
        except (AttributeError, TypeError, ValueError, OverflowError):
            self.fault("INVALID_" + kind.upper() + "_STAMP")
            return
        if kind in self.last:
            dt = t - self.last[kind]
            if dt <= 0:
                self.fault("NONMONOTONIC_" + kind.upper() + "_STAMP")
            else:
                self.max_gap_ns[kind] = max(dt, self.max_gap_ns[kind])
                if dt > 250_000_000:
                    self.gaps_over_250ms[kind] += 1
        self.first.setdefault(kind, t)
        self.last[kind] = t
        if kind == "image":
            try:
                self.image_signature_set.add(_sample_signature(msg))
            except (AttributeError, TypeError, ValueError):
                self.fault("INVALID_IMAGE_SAMPLED_PAYLOAD")

    def finish(self, base, *, declared_origin):
        if not isinstance(base, dict):
            raise TypeError("M23A preflight report required")
        if base.get("verdict") != "REPLAY_INPUT_CONTRACT_PASS":
            self.fault("M23A_FORMAT_PREFLIGHT_FAILED")
        if base.get("mode") != "monocular_imu":
            self.fault("VIO_MODE_REQUIRED")
        if self.count != base.get("counts"):
            self.fault("PROFILE_COUNTS_DISAGREE_WITH_M23A")
        if any(self.count[k] == 0 for k in KINDS):
            self.fault("MISSING_STREAM")
        # Header-based overlap is NOT optical exposure/IMU phase.
        overlap_ns = None
        if all(k in self.first and k in self.last for k in KINDS):
            overlap_ns = max(
                0, min(self.last[k] for k in KINDS)
                - max(self.first[k] for k in KINDS))
            if overlap_ns < 1_000_000_000:
                self.fault("SHORT_HEADER_TIME_OVERLAP")
        variation = (len(self.image_signature_set) > 1
                     if self.count["image"] > 0 else False)
        return {
            "milestone": "M23D",
            "verdict": ("REPLAY_INPUT_RECORDING_PROFILE_ONLY"
                        if not self.failures else "BLOCKED_RECORDING_PROFILE"),
            "declared_origin": declared_origin,
            "origin_authenticated": False,
            "synthetic_data_excluded_by_proof": False,
            "physical_camera_imu_offset_validated": False,
            "spatial_extrinsics_validated": False,
            "imu_noise_calibration_validated": False,
            "xrslam_tracking_accuracy_validated": False,
            "pose_convention_validated": False,
            "production_motion_enabled": False,
            "physical_motion_authorized": False,
            "device_access": False,
            "ready_for_metric_vio": False,
            "counts": dict(self.count),
            "header_overlap_s": (round(overlap_ns / 1e9, 6)
                                 if overlap_ns is not None else None),
            "max_header_gap_ms": {
                k: round(self.max_gap_ns[k] / 1e6, 3) for k in KINDS
            },
            "gaps_over_250ms": dict(self.gaps_over_250ms),
            "sampled_image_fingerprints": len(self.image_signature_set),
            "sampled_image_variation_observed": variation,
            "camera_header_gap_warning": (
                self.max_gap_ns["image"] > int(MAX_DIAGNOSTIC_GAP_MS * 1e6)),
            "faults": list(self.failures),
            "limitations": [
                "ROS message header times do not prove physical acquisition phase",
                "Sampling image payload does not establish motion, depth or texture quality",
                "Declared origin and ROS publisher names are unauthenticated",
                "Input-profile result is not XRSLAM or physical safety acceptance",
            ],
        }


def _ensure_regular_bag_files(bag):
    """Reject symlink/device paths before passing metadata to rosbag2_py."""
    if not bag.is_dir() or bag.is_symlink():
        raise ValueError("bag must be an existing non-symlink directory")
    metadata = bag / "metadata.yaml"
    if metadata.is_symlink() or not metadata.is_file():
        raise ValueError("bag metadata.yaml must be a regular file")
    # sqlite3 is the only format exercised by the current regression.
    # Reject storage paths that could escape the explicitly mounted bag.
    import yaml
    doc = yaml.safe_load(metadata.read_text(encoding="utf-8"))
    root = doc.get("rosbag2_bagfile_information", {}) if isinstance(doc, dict) else {}
    names = root.get("relative_file_paths", [])
    if not isinstance(names, list) or not names:
        raise ValueError("missing rosbag2 relative_file_paths")
    for rel in names:
        if not isinstance(rel, str) or not rel:
            raise ValueError("invalid rosbag file reference")
        p = Path(rel)
        if p.is_absolute() or ".." in p.parts or len(p.parts) != 1:
            raise ValueError("unsafe rosbag file reference")
        f = bag / p
        if f.is_symlink() or not f.is_file() or f.suffix != ".db3":
            raise ValueError("sqlite3 bag file must be a regular .db3 file")


def inspect_recorded_candidate(bag, *, declared_origin):
    """Read an EXISTING sqlite3 ROS bag, with zero hardware or ROS nodes."""
    if declared_origin not in ("unverified", "user_declared_recorded", "synthetic"):
        raise ValueError("origin label not recognized")
    _ensure_regular_bag_files(bag)
    base = inspect_bag(
        bag, storage_id="sqlite3", mode="monocular_imu",
        provenance=declared_origin, camera_frame="camera_optical_frame",
        imu_frame="imu_link")
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image, CameraInfo, Imu
    klass = {"image": Image, "info": CameraInfo, "imu": Imu}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    profile = RecordingProfile()
    while reader.has_next():
        topic, payload, receipt = reader.read_next()
        if topic not in TOPICS:
            continue
        kind, expected = TOPICS[topic]
        if types.get(topic) != expected:
            profile.fault("WRONG_" + kind.upper() + "_TYPE")
            continue
        try:
            msg = deserialize_message(payload, klass[kind])
        except (TypeError, ValueError, RuntimeError):
            profile.fault("INVALID_" + kind.upper() + "_CDR")
            continue
        profile.consume(kind, receipt, msg)
    return profile.finish(base, declared_origin=declared_origin)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True,
                        help="Existing rosbag2 sqlite3 directory; never a device")
    parser.add_argument("--declared-origin", choices=(
        "unverified", "user_declared_recorded", "synthetic"),
        default="unverified")
    args = parser.parse_args(argv)
    try:
        report = inspect_recorded_candidate(
            args.bag, declared_origin=args.declared_origin)
    except (OSError, ValueError, ImportError, RuntimeError,
            AttributeError, TypeError, KeyError) as exc:
        print("M23D_ERROR=" + type(exc).__name__ + ": " + str(exc),
              file=sys.stderr)
        return 2
    print("M23D_RECORDING_PROFILE_JSON=" + json.dumps(report, sort_keys=True))
    print("M23D_VERDICT=" + report["verdict"])
    print("M23D_DEVICE_ACCESS=NONE")
    return 0 if report["verdict"] == "REPLAY_INPUT_RECORDING_PROFILE_ONLY" else 2


if __name__ == "__main__":
    raise SystemExit(main())
