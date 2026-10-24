#!/usr/bin/env python3
"""Derive a new, strictly covered sqlite3 ROS2 bag without modifying source.

ROS Jazzy-only execution; NEVER opens cameras, serial, or ROS graph.
Two independent passes: read/decode/audit, then copy ORIGINAL CDR bytes
and original bag receipt timestamps into a separate rosbag2 writer.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

from m23bv_window import Event, TOPICS, select

MESSAGE_TYPES = {
    "/camera/image_rect": "sensor_msgs/msg/Image",
    "/camera/camera_info": "sensor_msgs/msg/CameraInfo",
    "/imu/data_raw": "sensor_msgs/msg/Imu",
}


def fingerprints(folder):
    import yaml
    if not folder.is_dir() or folder.is_symlink():
        raise ValueError("BLOCKED_SOURCE_DIRECTORY")
    metadata = folder / "metadata.yaml"
    if not metadata.is_file() or metadata.is_symlink():
        raise ValueError("BLOCKED_SOURCE_METADATA")
    details = yaml.safe_load(metadata.read_text())
    bag = details.get("rosbag2_bagfile_information", {})
    if bag.get("storage_identifier") != "sqlite3":
        raise ValueError("BLOCKED_NOT_SQLITE3")
    names = bag.get("relative_file_paths")
    if not isinstance(names, list) or not 1 <= len(names) <= 64:
        raise ValueError("BLOCKED_STORAGE_MANIFEST")
    result = {}
    for name in ["metadata.yaml", *names]:
        if (not isinstance(name, str) or Path(name).name != name or
                name in (".", "..") or "\\" in name or
                (name != "metadata.yaml" and not name.endswith(".db3"))):
            raise ValueError("BLOCKED_UNSAFE_STORAGE_REFERENCE")
        path = folder / name
        if not path.is_file() or path.is_symlink():
            raise ValueError("BLOCKED_SOURCE_FILE")
        h = hashlib.sha256()
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
                h.update(block)
        result[name] = h.hexdigest()
    return result


def new_reader(folder, baglib):
    reader = baglib.SequentialReader()
    reader.open(baglib.StorageOptions(uri=str(folder), storage_id="sqlite3"),
                baglib.ConverterOptions("cdr", "cdr"))
    metadata = reader.get_all_topics_and_types()
    seen = {t.name: t.type for t in metadata}
    if seen != MESSAGE_TYPES or len(metadata) != len(TOPICS):
        raise ValueError("BLOCKED_UNEXPECTED_TOPICS_OR_TYPES")
    return reader, metadata


def scan(reader, decode):
    events = []
    while reader.has_next():
        topic, payload, receipt = reader.read_next()
        msg = decode(topic, payload)
        stamp = msg.header.stamp
        if type(stamp.sec) is not int or type(stamp.nanosec) is not int or not 0 <= stamp.nanosec < 1_000_000_000:
            raise ValueError("BLOCKED_MALFORMED_HEADER_STAMP")
        events.append(Event(topic, int(receipt), stamp.sec * 1_000_000_000 + stamp.nanosec,
                            msg.header.frame_id, getattr(msg, "width", 0),
                            getattr(msg, "height", 0)))
    return events


def derive(source, dest, *, baglib, decode, min_counts=None):
    source, dest = Path(source), Path(dest)
    if dest.exists() or dest.is_symlink():
        raise ValueError("BLOCKED_OUTPUT_EXISTS")
    if dest.resolve(strict=False) == source.resolve(strict=True) or source.resolve(strict=True) in dest.resolve(strict=False).parents:
        raise ValueError("BLOCKED_OUTPUT_INSIDE_SOURCE")
    if dest.parent.resolve(strict=True) == source.resolve(strict=True):
        raise ValueError("BLOCKED_OUTPUT_UNSAFE_PARENT")
    before = fingerprints(source)
    reader, topics = new_reader(source, baglib)
    events = scan(reader, decode)
    if hasattr(reader, "close"):
        reader.close()
    keep, report = select(events, min_counts=min_counts)
    # Only after COMPLETE preflight passes may an output directory be created.
    reader2, topics2 = new_reader(source, baglib)
    if [(x.name, x.type) for x in topics] != [(x.name, x.type) for x in topics2]:
        raise ValueError("BLOCKED_CHANGED_TOPIC_METADATA")
    writer = baglib.SequentialWriter()
    copied = 0
    try:
        writer.open(baglib.StorageOptions(uri=str(dest), storage_id="sqlite3"),
                    baglib.ConverterOptions("cdr", "cdr"))
        for topic in topics2:
            writer.create_topic(topic)
        for i, event in enumerate(events):
            if not reader2.has_next():
                raise RuntimeError("BLOCKED_CHANGED_SOURCE_MESSAGE_COUNT")
            topic, payload, receipt_ns = reader2.read_next()
            if topic != event.topic or receipt_ns != event.bag_ns:
                raise RuntimeError("BLOCKED_CHANGED_SOURCE_MESSAGE_ORDER")
            if keep[i]:
                writer.write(topic, payload, receipt_ns)
                copied += 1
        if reader2.has_next():
            raise RuntimeError("BLOCKED_CHANGED_SOURCE_MESSAGE_COUNT")
    finally:
        # Fail closed on no close() (do not assume destructor finalizes).
        writer.close()
        if hasattr(reader2, "close"):
            reader2.close()
    after = fingerprints(source)
    if before != after:
        raise RuntimeError("BLOCKED_SOURCE_BYTES_CHANGED")
    fingerprints(dest)  # metadata and DB must be explicitly finalized
    if copied != sum(report["retained_counts"].values()):
        raise RuntimeError("BLOCKED_RETAINED_COUNT_MISMATCH")
    report.update({"verdict": "DERIVED_BAG_CLOSED_PENDING_M23A_M23I_M23F_M23J",
                   "source": str(source), "derived": str(dest),
                   "source_sha256_verified_unchanged": before,
                   "source_messages_copied_byte_for_byte": copied,
                   "imu_full_retained_image_header_window_covered": True,
                   "cdr_content_independently_validated": False})
    return report


def ros_decode():
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image, CameraInfo, Imu
    classes = {"/camera/image_rect": Image,
               "/camera/camera_info": CameraInfo, "/imu/data_raw": Imu}
    def decode(topic, payload):
        try:
            return deserialize_message(payload, classes[topic])
        except (KeyError, RuntimeError, ValueError, TypeError) as exc:
            raise ValueError("BLOCKED_CDR_DESERIALIZATION") from exc
    return decode


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        import rosbag2_py
        report = derive(args.source, args.output,
                        baglib=rosbag2_py, decode=ros_decode())
    except (OSError, RuntimeError, ValueError, ImportError, KeyError, TypeError) as exc:
        print("M23BV_DERIVATION=BLOCKED:" + type(exc).__name__ + ":" + str(exc),
              file=sys.stderr, flush=True)
        return 2
    print("M23BV_DERIVATION_JSON=" + json.dumps(report, sort_keys=True), flush=True)
    print("M23BV_DERIVATION=DERIVED_PENDING_STRICT_AUDITS", flush=True)
    print("M23BV_PHYSICAL_HARDWARE_ACCESS=NONE", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
