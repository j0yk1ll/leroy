#!/usr/bin/env python3
"""M23A ROS Jazzy CDR rosbag round-trip using generated test messages ONLY.

Generates disposable synthetic Image/CameraInfo/Imu input records and
passes them to the exact read-only preflight. Does not run XRSLAM,
connect ROS nodes or devices, publish topics, or validate calibration.
"""
import gc
from pathlib import Path
import tempfile

import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, CameraInfo, Imu

from m23a_xrslam_rosbag_preflight import inspect_bag

FIRST_NS = 1_800_000_000_000_000_000
TYPE_MAP = (
    ("/camera/image_rect", "sensor_msgs/msg/Image"),
    ("/camera/camera_info", "sensor_msgs/msg/CameraInfo"),
    ("/imu/data_raw", "sensor_msgs/msg/Imu"),
)


def time_header(msg, ns, frame):
    msg.header.stamp.sec = ns // 1_000_000_000
    msg.header.stamp.nanosec = ns % 1_000_000_000
    msg.header.frame_id = frame
    return msg


def fixture_messages(*, wrong_frame=False, include_imu=True):
    events = []
    for i in range(3):
        at = FIRST_NS + 100_000_000 * i
        frame = "other_camera_frame" if (wrong_frame and i == 1) else "camera_optical_frame"
        info = time_header(CameraInfo(), at, "camera_optical_frame")
        info.width = 4
        info.height = 3
        info.p = [200., 0., 2., 0., 0., 200., 1.5, 0.,
                  0., 0., 1., 0.]
        image = time_header(Image(), at, frame)
        image.width = 4
        image.height = 3
        image.encoding = "mono8"
        image.step = 4
        image.data = bytes(12)
        events.append((at, "/camera/camera_info", info))
        events.append((at + 1, "/camera/image_rect", image))
    if include_imu:
        for i in range(7):
            at = FIRST_NS - 50_000_000 + 50_000_000 * i
            imu = time_header(Imu(), at, "imu_link")
            imu.angular_velocity.z = 0.01
            imu.linear_acceleration.z = 9.81
            events.append((at, "/imu/data_raw", imu))
    return sorted(events, key=lambda item: item[0])


def write_bag(folder, *, wrong_frame=False, include_imu=True):
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(folder), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    for topic, ros_type in TYPE_MAP:
        if topic == "/imu/data_raw" and not include_imu:
            continue
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name=topic, type=ros_type, serialization_format="cdr"))
    for bag_time, topic, message in fixture_messages(
            wrong_frame=wrong_frame, include_imu=include_imu):
        writer.write(topic, serialize_message(message), bag_time)
    if hasattr(writer, "close"):
        writer.close()
    del writer
    gc.collect()
    if not (folder / "metadata.yaml").is_file():
        raise AssertionError("rosbag2 did not write metadata.yaml")


def run_case(folder, *, wrong_frame=False, include_imu=True):
    write_bag(folder, wrong_frame=wrong_frame, include_imu=include_imu)
    return inspect_bag(folder, storage_id="sqlite3",
                       mode="monocular_imu", provenance="synthetic",
                       camera_frame="camera_optical_frame",
                       imu_frame="imu_link")


def main():
    with tempfile.TemporaryDirectory(prefix="m23a-synthetic-bag-") as td:
        root = Path(td)
        healthy = run_case(root / "healthy")
        if (healthy["verdict"] != "REPLAY_INPUT_CONTRACT_PASS"
                or healthy["matched_image_info_pairs"] != 3
                or healthy["counts"] != {"image": 3, "info": 3, "imu": 7}
                or healthy["xrslam_executed"]
                or healthy["provenance_authenticated"]):
            raise AssertionError("synthetic rosbag2 roundtrip unexpectedly invalid")
        print("M23A_SYNTHETIC_ROSBAG_ROUNDTRIP=PASS", flush=True)

        wrong = run_case(root / "wrong-frame", wrong_frame=True)
        if (wrong["verdict"] != "BLOCKED_REPLAY_INPUT"
                or "BLOCKED_CAMERA_FRAME" not in wrong["faults"]):
            raise AssertionError("wrong camera frame not rejected")
        print("M23A_ROSBAG_WRONG_FRAME_INHIBIT=PASS", flush=True)

        missing = run_case(root / "missing-imu", include_imu=False)
        if (missing["verdict"] != "BLOCKED_REPLAY_INPUT"
                or "BLOCKED_MISSING_OR_WRONG_TYPE_IMU" not in missing["faults"]):
            raise AssertionError("missing IMU bag topic not rejected")
        print("M23A_ROSBAG_MISSING_IMU_INHIBIT=PASS", flush=True)

        print("M23A_ROSBAG_READER_SELFTEST=PASS", flush=True)
        print("M23A_VERDICT=SYNTHETIC_BAG_READER_CONTRACT_ONLY", flush=True)
        print("M23A_REAL_RECORDED_BAG_VALIDATED=FALSE", flush=True)
        print("M23A_XRSLAM_TRACKING_VALIDATED=FALSE", flush=True)
        print("M23A_DEVICE_ACCESS=NONE", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
