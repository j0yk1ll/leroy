#!/usr/bin/env python3
"""Stage 4C tiny ROS 2 Jazzy CDR/SQLite round-trip. SYNTHETIC, NO hardware.

This module writes only into an automatically removed temporary directory.
It MUST run in a disposable --network=none, no-device container with /tmp
on tmpfs. It never uses the actual serial or camera and publishes no ROS
topics; it tests rosbag2_py serialized message preservation only.
"""
import gc
import json
from pathlib import Path
from tempfile import TemporaryDirectory

from live_stage4c_capture_contract import TOPICS, source_metadata
from live_stage4c_source_meta import SourceMetadataTap


def run_offline():
    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import CameraInfo, Image, Imu
    from std_msgs.msg import String

    types = {
        "/camera/image_rect": Image,
        "/camera/camera_info": CameraInfo,
        "/imu/data_raw": Imu,
        "/stage4c/imu_source_meta": String,
    }
    if set(types) != set(TOPICS):
        raise RuntimeError("topic_contract_drift")
    base_ns = 1_790_000_000_000_000_000
    second, nano = divmod(base_ns, 1_000_000_000)
    image = Image()
    image.header.stamp.sec = second
    image.header.stamp.nanosec = nano
    image.header.frame_id = "camera_optical_frame"
    image.height = 2
    image.width = 2
    image.encoding = "rgb8"
    image.step = 6
    image.data = bytes(range(12))
    camera_info = CameraInfo()
    camera_info.header = image.header
    camera_info.width = image.width
    camera_info.height = image.height
    camera_info.k = [500., 0., 1., 0., 500., 1., 0., 0., 1.]
    imu = Imu()
    imu.header.stamp.sec = second
    imu.header.stamp.nanosec = nano
    imu.header.frame_id = "imu_link"
    imu.angular_velocity.z = 0.13
    imu.linear_acceleration.z = 9.8
    metadata_sent = []
    tap = SourceMetadataTap(
        metadata_sent.append, monotonic_ns=lambda: 2_000_000_000)
    tap.on_imu_published(
        {"seq": 42, "t_us": 5_000_000, "cal": 1}, base_ns, 1_999_999_999)
    source_metadata(metadata_sent[0])
    metadata = String()
    metadata.data = metadata_sent[0]
    examples = [
        ("/camera/image_rect", image),
        ("/camera/camera_info", camera_info),
        ("/imu/data_raw", imu),
        ("/stage4c/imu_source_meta", metadata),
    ]
    expected = [(topic, serialize_message(msg), base_ns)
                for topic, msg in examples]

    with TemporaryDirectory(prefix="stage4c-synthetic-rosbag-") as tmp:
        bag = Path(tmp) / "capture"
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr"))
        for topic, typ in TOPICS.items():
            writer.create_topic(rosbag2_py.TopicMetadata(
                id=0, name=topic, type=typ[0],
                serialization_format="cdr"))
        for topic, raw, stamp in expected:
            writer.write(topic, raw, stamp)
        del writer
        gc.collect()
        if not (bag / "metadata.yaml").is_file():
            raise RuntimeError("missing_rosbag_metadata_yaml")
        if not list(bag.glob("*.db3")):
            raise RuntimeError("missing_sqlite_storage")
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr"))
        actual = []
        while reader.has_next():
            actual.append(reader.read_next())
        if len(actual) != len(expected):
            raise RuntimeError("missing_or_extra_rosbag_samples")
        if set((topic for topic, _, _ in actual)) != set(TOPICS):
            raise RuntimeError("rosbag_topic_set_mismatch")
        # Timestamp ties are legal (especially IMU+MCU metadata); SQLite
        # does not promise the relative order of equal-time topic records.
        originals = {topic: (raw, stamp) for topic, raw, stamp in expected}
        for topic, raw, stamp in actual:
            original_raw, original_stamp = originals[topic]
            if stamp != original_stamp or bytes(raw) != bytes(original_raw):
                raise RuntimeError("rosbag_cdr_timestamp_or_payload_mismatch")
            message = deserialize_message(raw, types[topic])
            if topic == "/stage4c/imu_source_meta":
                value = source_metadata(message.data)
                if (value["seq"] != 42 or value["t_us"] != 5_000_000 or
                        value["imu_stamp_ns"] != base_ns):
                    raise RuntimeError("lost_original_mcu_metadata")
            elif topic == "/camera/image_rect":
                if bytes(message.data) != bytes(range(12)):
                    raise RuntimeError("lost_image_pixels")
        del reader
        gc.collect()
    print("STAGE4C_SYNTHETIC_ROSBAG_ROUNDTRIP=PASS_NO_DEVICE", flush=True)
    print("STAGE4C_PHYSICAL_CALIBRATION_VALIDATED=FALSE", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(run_offline())
