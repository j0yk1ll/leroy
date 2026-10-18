#!/usr/bin/env python3
"""M23D ROS Jazzy synthetic CDR integration, no hardware or ROS nodes.

Writes a disposable generated camera/IMU rosbag solely as a test INPUT.
M23D itself must read the data, report a profile, and NEVER classify
synthetic data as a real, calibrated or safe physical recording.
"""
import gc
from pathlib import Path
import tempfile

import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu

from m23b_synthetic_monocular_bag import (
    FRAME_COUNT, START_NS, synthetic_records,
)
from m23d_recorded_bag_readiness import inspect_recorded_candidate


def generate_bag(path):
    if path.exists():
        raise ValueError("target synthetic bag path already exists")
    imu_events = []
    for i in range(104):
        at = START_NS - 40_000_000 + i * 20_000_000
        msg = Imu()
        msg.header.stamp.sec = at // 1_000_000_000
        msg.header.stamp.nanosec = at % 1_000_000_000
        msg.header.frame_id = "imu_link"
        msg.angular_velocity.z = 0.01
        msg.linear_acceleration.z = 9.81
        imu_events.append((at, "/imu/data_raw", msg))
    events = sorted(list(synthetic_records()) + imu_events,
                    key=lambda record: record[0])
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(
        uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"))
    for topic, ros_type in (
        ("/camera/camera_info", "sensor_msgs/msg/CameraInfo"),
        ("/camera/image_rect", "sensor_msgs/msg/Image"),
        ("/imu/data_raw", "sensor_msgs/msg/Imu"),
    ):
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name=topic, type=ros_type, serialization_format="cdr"))
    for at, topic, message in events:
        writer.write(topic, serialize_message(message), at)
    if hasattr(writer, "close"):
        writer.close()
    del writer
    gc.collect()
    return path


def main():
    with tempfile.TemporaryDirectory(prefix="m23d-synthetic-rosbag-") as td:
        bag = generate_bag(Path(td) / "generated")
        report = inspect_recorded_candidate(
            bag, declared_origin="synthetic")
        if (report["verdict"] != "REPLAY_INPUT_RECORDING_PROFILE_ONLY"
                or report["counts"] != {
                    "image": FRAME_COUNT, "info": FRAME_COUNT, "imu": 104}
                or report["origin_authenticated"] is not False
                or report["ready_for_metric_vio"] is not False
                or report["device_access"] is not False):
            raise AssertionError("M23D synthetic profile contract failure " +
                                 str(report))
        print("M23D_SYNTHETIC_ROSBAG_PROFILE=PASS", flush=True)

        # A tampered rosbag metadata path MUST be rejected before the
        # rosbag2 reader can follow any unsafe relative filename.
        import yaml
        manifest = bag / "metadata.yaml"
        data = yaml.safe_load(manifest.read_text(encoding="utf-8"))
        data["rosbag2_bagfile_information"]["relative_file_paths"] = [
            "../outside-input.db3"]
        manifest.write_text(yaml.safe_dump(data), encoding="utf-8")
        try:
            inspect_recorded_candidate(bag, declared_origin="synthetic")
        except ValueError as exc:
            if "unsafe rosbag file reference" not in str(exc):
                raise
        else:
            raise AssertionError("bag-relative traversal was not rejected")
        print("M23D_BAG_PATH_TRAVERSAL_REJECTED=PASS", flush=True)

        print("M23D_ROSBAG_READER_SELFTEST=PASS", flush=True)
        print("M23D_VERDICT=SYNTHETIC_RECORDED_INPUT_PROFILE_ONLY",
              flush=True)
        print("M23D_REAL_RECORDED_BAG_VERIFIED=FALSE", flush=True)
        print("M23D_CALIBRATION_VALIDATED=FALSE", flush=True)
        print("M23D_XRSLAM_TRACKING_VALIDATED=FALSE", flush=True)
        print("M23D_DEVICE_ACCESS=NONE", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
