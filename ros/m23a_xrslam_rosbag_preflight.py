#!/usr/bin/env python3
"""M23A: read-only, no-device ROS 2 bag INPUT preflight for XRSLAM.

This checks message shapes, temporal ordering and CameraInfo association.
It does NOT run XRSLAM, establish camera/IMU extrinsics, determine physical
exposure-to-MCU time offset, authenticate recording origin, or authorize
motor motion. No serial/USB/ROS graph/node APIs are used.
"""
import argparse
import json
import math
from pathlib import Path
import sys

TOPICS = {
    "/camera/image_rect": ("image", "sensor_msgs/msg/Image"),
    "/camera/camera_info": ("info", "sensor_msgs/msg/CameraInfo"),
    "/imu/data_raw": ("imu", "sensor_msgs/msg/Imu"),
}
CHANNELS = {"mono8": 1, "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}
IMAGE_MAX_GAP_NS = 250_000_000
IMU_MAX_GAP_NS = 300_000_000
MAX_WIDTH = 4096
MAX_HEIGHT = 4096
MAX_PAYLOAD = 64 * 1024 * 1024
MAX_PENDING = 2048


def stamp_ns(header):
    stamp = header.stamp
    if (type(stamp.sec) is not int or type(stamp.nanosec) is not int
            or stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000):
        raise ValueError("invalid header timestamp")
    result = stamp.sec * 1_000_000_000 + stamp.nanosec
    if result <= 0:
        raise ValueError("zero header timestamp")
    return result


class ReplayInputAudit:
    def __init__(self, *, mode, camera_frame="camera_optical_frame",
                 imu_frame="imu_link", provenance="unverified"):
        if mode not in ("monocular", "monocular_imu"):
            raise ValueError("XRSLAM mode must be monocular or monocular_imu")
        if (type(camera_frame) is not str or not camera_frame
                or type(imu_frame) is not str or not imu_frame):
            raise ValueError("explicit sensor frame IDs required")
        self.mode = mode
        self.camera_frame = camera_frame
        self.imu_frame = imu_frame
        self.provenance = provenance
        self.counts = {"image": 0, "info": 0, "imu": 0}
        self.first = {}
        self.last = {}
        self.largest_gap = {"image": 0, "info": 0, "imu": 0}
        self.last_bag_ns = None
        self.info_by_stamp = {}
        self.pending_image = {}
        self.matched = 0
        self.faults = []

    def fault(self, reason):
        if reason not in self.faults and len(self.faults) < 32:
            self.faults.append(reason)

    def ingest(self, kind, bag_ns, msg):
        if kind not in self.counts:
            self.fault("BLOCKED_UNEXPECTED_TOPIC")
            return
        if type(bag_ns) is not int or bag_ns <= 0:
            self.fault("BLOCKED_INVALID_BAG_TIME")
        elif self.last_bag_ns is not None and bag_ns < self.last_bag_ns:
            self.fault("BLOCKED_REORDERED_BAG_TIME")
        if type(bag_ns) is int and bag_ns > 0:
            self.last_bag_ns = bag_ns
        self.counts[kind] += 1
        try:
            ts = stamp_ns(msg.header)
            if kind == "imu":
                self._validate_imu(msg)
            else:
                self._validate_camera_frame(kind, msg)
            prior = self.last.get(kind)
            if prior is not None:
                if ts <= prior:
                    self.fault("BLOCKED_REPEATED_OR_BACKWARD_" + kind.upper())
                else:
                    self.largest_gap[kind] = max(self.largest_gap[kind],
                                                 ts - prior)
            self.last[kind] = ts
            self.first.setdefault(kind, ts)
            if kind == "info":
                self._info(msg, ts)
            if kind == "image":
                self._image(msg, ts)
        except (AttributeError, TypeError, ValueError, IndexError, OverflowError):
            self.fault("BLOCKED_MALFORMED_" + kind.upper())

    def _validate_camera_frame(self, kind, msg):
        if msg.header.frame_id != self.camera_frame:
            self.fault("BLOCKED_CAMERA_FRAME")
        if (type(msg.width) is not int or type(msg.height) is not int
                or not 0 < msg.width <= MAX_WIDTH
                or not 0 < msg.height <= MAX_HEIGHT):
            raise ValueError("invalid camera dimensions")
        if kind == "info":
            p = msg.p
            if (len(p) != 12
                    or any(not math.isfinite(float(v)) for v in p)
                    or p[0] <= 0 or p[5] <= 0
                    or abs(p[3]) > 1e-8 or abs(p[7]) > 1e-8
                    or bool(msg.roi.do_rectify)):
                self.fault("BLOCKED_UNRECTIFIED_OR_INVALID_CAMERA_INFO")
        else:
            channels = CHANNELS.get(msg.encoding)
            if (channels is None or type(msg.step) is not int
                    or msg.step < msg.width * channels
                    or msg.step * msg.height > MAX_PAYLOAD
                    or len(msg.data) < msg.step * msg.height):
                self.fault("BLOCKED_INVALID_IMAGE_LAYOUT")

    def _validate_imu(self, msg):
        if msg.header.frame_id != self.imu_frame:
            self.fault("BLOCKED_IMU_FRAME")
        values = (
            msg.angular_velocity.x, msg.angular_velocity.y,
            msg.angular_velocity.z, msg.linear_acceleration.x,
            msg.linear_acceleration.y, msg.linear_acceleration.z,
        )
        if any(not math.isfinite(float(x)) for x in values):
            self.fault("BLOCKED_INVALID_IMU_VECTOR")

    def _info(self, msg, stamp):
        key = (stamp, msg.header.frame_id)
        geometry = (msg.width, msg.height)
        if key in self.info_by_stamp and self.info_by_stamp[key] != geometry:
            self.fault("BLOCKED_CHANGED_CAMERA_INFO")
        self.info_by_stamp[key] = geometry
        outstanding = self.pending_image.pop(key, [])
        for expected in outstanding:
            if expected != geometry:
                self.fault("BLOCKED_IMAGE_INFO_DIMENSION_MISMATCH")
            else:
                self.matched += 1
        if len(self.info_by_stamp) > MAX_PENDING:
            self.fault("BLOCKED_UNBOUNDED_CAMERA_INFO")

    def _image(self, msg, stamp):
        key = (stamp, msg.header.frame_id)
        geometry = (msg.width, msg.height)
        if key in self.info_by_stamp:
            if geometry != self.info_by_stamp[key]:
                self.fault("BLOCKED_IMAGE_INFO_DIMENSION_MISMATCH")
            else:
                self.matched += 1
        else:
            self.pending_image.setdefault(key, []).append(geometry)
            if sum(map(len, self.pending_image.values())) > MAX_PENDING:
                self.fault("BLOCKED_UNBOUNDED_UNMATCHED_IMAGES")

    def finish(self):
        if self.counts["image"] < 3 or self.counts["info"] < 1:
            self.fault("BLOCKED_INSUFFICIENT_CAMERA_MESSAGES")
        if self.matched != self.counts["image"]:
            self.fault("BLOCKED_UNMATCHED_CAMERA_INFO")
        if self.largest_gap["image"] > IMAGE_MAX_GAP_NS:
            self.fault("BLOCKED_IMAGE_STAMP_GAP")
        if self.mode == "monocular_imu":
            if self.counts["imu"] < 4:
                self.fault("BLOCKED_INSUFFICIENT_IMU_MESSAGES")
            if self.largest_gap["imu"] > IMU_MAX_GAP_NS:
                self.fault("BLOCKED_IMU_STAMP_GAP")
            if all(k in self.first for k in ("image", "imu")) and (
                    self.first["imu"] > self.first["image"]
                    or self.last["imu"] < self.last["image"]):
                self.fault("BLOCKED_NO_IMU_STAMP_COVERAGE")
        verdict = ("REPLAY_INPUT_CONTRACT_PASS" if not self.faults
                   else "BLOCKED_REPLAY_INPUT")
        return {
            "milestone": "M23A",
            "verdict": verdict,
            "mode": self.mode,
            "provenance": self.provenance,
            "provenance_authenticated": False,
            "device_access": False,
            "physical_motion_authorized": False,
            "xrslam_executed": False,
            "xrslam_tracking_validated": False,
            "extrinsics_validated": False,
            "camera_imu_physical_offset_validated": False,
            "camera_frame": self.camera_frame,
            "imu_frame": self.imu_frame,
            "counts": dict(self.counts),
            "matched_image_info_pairs": self.matched,
            "max_header_gap_ms": {
                kind: round(gap / 1_000_000, 3)
                for kind, gap in self.largest_gap.items()
            },
            "faults": list(self.faults),
        }


def inspect_bag(path, *, storage_id, mode, provenance,
                camera_frame, imu_frame):
    """Read a pre-existing ROS bag via rosbag2_py; never writes to it."""
    if not path.is_dir() or not (path / "metadata.yaml").is_file():
        raise ValueError("ROS 2 bag directory with metadata.yaml required")
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image, CameraInfo, Imu

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    audit = ReplayInputAudit(mode=mode, provenance=provenance,
                             camera_frame=camera_frame, imu_frame=imu_frame)
    required = ("/camera/image_rect", "/camera/camera_info")
    if mode == "monocular_imu":
        required += ("/imu/data_raw",)
    for name in required:
        if topics.get(name) != TOPICS[name][1]:
            audit.fault("BLOCKED_MISSING_OR_WRONG_TYPE_" + TOPICS[name][0].upper())
    types = {"image": Image, "info": CameraInfo, "imu": Imu}
    while reader.has_next():
        name, payload, receipt_ns = reader.read_next()
        if name not in TOPICS:
            continue
        kind, expected_type = TOPICS[name]
        if topics.get(name) != expected_type:
            audit.fault("BLOCKED_WRONG_TOPIC_TYPE")
            continue
        try:
            message = deserialize_message(payload, types[kind])
        except (RuntimeError, TypeError, ValueError):
            audit.fault("BLOCKED_CDR_DECODE_" + kind.upper())
            continue
        audit.ingest(kind, receipt_ns, message)
    return audit.finish()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True,
                        help="Existing ROS 2 bag directory; read-only")
    parser.add_argument("--storage-id", choices=("sqlite3", "mcap"),
                        default="sqlite3")
    parser.add_argument("--mode", choices=("monocular", "monocular_imu"),
                        default="monocular_imu")
    parser.add_argument("--provenance", choices=(
        "unverified", "user_declared_recorded", "synthetic"),
        default="unverified")
    parser.add_argument("--camera-frame", default="camera_optical_frame")
    parser.add_argument("--imu-frame", default="imu_link")
    args = parser.parse_args(argv)
    try:
        report = inspect_bag(
            args.bag, storage_id=args.storage_id, mode=args.mode,
            provenance=args.provenance, camera_frame=args.camera_frame,
            imu_frame=args.imu_frame)
    except (ImportError, RuntimeError, ValueError, OSError) as exc:
        print("M23A_BAG_PREFLIGHT_ERROR=" + type(exc).__name__ + ": " + str(exc),
              file=sys.stderr)
        return 2
    print("M23A_BAG_PREFLIGHT_REPORT_JSON=" +
          json.dumps(report, sort_keys=True))
    print("M23A_BAG_PREFLIGHT_VERDICT=" + report["verdict"])
    print("M23A_DEVICE_ACCESS=NONE")
    return 0 if report["verdict"] == "REPLAY_INPUT_CONTRACT_PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
