#!/usr/bin/env python3
"""M21F simulated ROS camera adapter contract: NO physical sensor access.

This selftest intentionally publishes synthetic camera *metadata* only from
two disposable ROS nodes with the expected names. Such graph names prove
neither genuine device provenance nor camera/IMU physical synchronization.
It never opens a serial device or creates a motor publisher. The M18
ImuStreamSource publishes synthetic messages from in-memory telemetry bytes.
"""
import json
import os
from pathlib import Path
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu

from m18_imu_stream_source import ImuStreamSource

from m21_record_contract import load
from m21d_metadata_capture import MetadataCapture
from m21d_passive_ros_adapter import (
    EXPECTED_ENV, PassiveCameraMetadataNode, record_from_m18_source,
)


def wait_until(observer, predicate, name, seconds=6.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(observer, timeout_sec=0.04)
        if predicate():
            return
    raise RuntimeError("M21F simulated ROS graph/callback timeout: " + name)


def stamp(message, stamp_ns):
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(
        stamp_ns, 1_000_000_000)
    message.header.frame_id = "camera_optical_frame"
    return message


def assert_no_physical_devices():
    for path in ("/dev/video2", "/dev/ttyACM0"):
        if os.path.exists(path):
            raise RuntimeError("M21F refuses mapped physical sensor device: " + path)


def fake_packet(index, *, calibration=1):
    """Precisely the M18 MCU telemetry format; synthetic input bytes only."""
    return (
        f"telemetry imu seq={index} t_us={index * 20000} cal={calibration} "
        f"af_mm_s2=0 al_mm_s2=0 au_mm_s2=9810 "
        f"gr_mrad_s=200 gp_mrad_s=-100 gy_mrad_s=50 "
        f"mag_valid=0 mx_raw=0 my_raw=0 mz_raw=0\r\n"
    ).encode("ascii")


def run():
    assert_no_physical_devices()
    for key, val in EXPECTED_ENV.items():
        if os.environ.get(key) != val:
            raise RuntimeError("M21F refuses non-isolated ROS environment: " + key)
    capture = MetadataCapture("SYNTHETIC", max_events=40)
    observer = cam_pub_node = info_pub_node = imu_source = imu_observer = None
    received_imus = []
    rclpy.init(args=[])
    try:
        observer = PassiveCameraMetadataNode(capture)
        imu_source = ImuStreamSource(
            on_imu_published=lambda fields, mapped, received:
                record_from_m18_source(capture, fields, mapped, received))
        imu_observer = Node("m21f_imu_receipt_observer",
                            enable_rosout=False, start_parameter_services=False)
        _imu_subscription = imu_observer.create_subscription(
            Imu, "/imu/data_raw", received_imus.append, qos_profile_sensor_data)
        if _imu_subscription.topic_name != "/imu/data_raw":
            raise RuntimeError("M21F IMU subscription remapped")
        # No camera publishers: callback must fail closed on graph provenance.
        try:
            observer._verify_graph("/camera/image_rect", "rectify_node",
                                   "/camera", "sensor_msgs/msg/Image")
        except RuntimeError:
            pass
        else:
            raise RuntimeError("M21F accepted image without any publisher")

        cam_pub_node = Node("rectify_node", namespace="/camera",
                            enable_rosout=False, start_parameter_services=False)
        info_pub_node = Node("usb_cam", namespace="/camera",
                             enable_rosout=False, start_parameter_services=False)
        cam_pub = cam_pub_node.create_publisher(
            Image, "/camera/image_rect", qos_profile_sensor_data)
        info_pub = info_pub_node.create_publisher(
            CameraInfo, "/camera/camera_info", qos_profile_sensor_data)

        wait_until(
            observer,
            lambda: (len(observer.get_publishers_info_by_topic("/camera/image_rect")) == 1
                     and len(observer.get_publishers_info_by_topic("/camera/camera_info")) == 1),
            "publisher discovery")

        # Verify exactly the graph guard the real adapter will enforce.
        observer._verify_graph(
            "/camera/image_rect", "rectify_node", "/camera",
            "sensor_msgs/msg/Image")
        observer._verify_graph(
            "/camera/camera_info", "usb_cam", "/camera",
            "sensor_msgs/msg/CameraInfo")

        for index in range(11):
            now_ns = time.time_ns()
            image = stamp(Image(), now_ns)
            image.width, image.height = 640, 480
            # Deliberately no pixel bytes in simulated event.
            if len(image.data) != 0:
                raise AssertionError("M21F fake frame includes pixel data")
            cam_pub.publish(image)
            wait_until(observer, lambda: observer.frame_index == index + 1,
                       "rectified image metadata")

            info = stamp(CameraInfo(), max(time.time_ns(), now_ns + 1))
            info.width, info.height = 640, 480
            info_pub.publish(info)
            wait_until(
                observer,
                lambda: len([r for r in capture.rows if r["kind"] == "camera_info"]) == index + 1,
                "camera info metadata")

            # Source.consume parses EXACT MCU telemetry bytes, maps t_us
            # using its McuClockSync and publishes the real ROS Imu message.
            # The opt-in hook then stores the genuine parsed seq/t_us/stamp.
            time.sleep(0.022)
            imu_source.consume(fake_packet(index))
            wait_until(imu_observer, lambda: len(received_imus) == index + 1,
                       "M18 synthetic IMU publication")

        # Cal=0 packets and malformed bytes must NOT trigger source tap.
        before = len(capture.rows)
        imu_source.consume(fake_packet(11, calibration=0))
        imu_source.consume(b"telemetry imu seq=12 NOT_A_PACKET\r\n")
        if len(capture.rows) != before:
            raise AssertionError("M21F captured withheld/malformed IMU packet")
        counts = imu_source.export()["counts"]
        if not (counts["published_imu_messages"] == 11
                and counts["calibration_withheld"] == 1
                and counts["malformed_imu_packets"] == 1
                and len(received_imus) == 11):
            raise RuntimeError("M21F M18 publication/withhold counters mismatch")
        source_rows = [row for row in capture.rows if row["kind"] == "imu"]
        if [row["mcu_seq"] for row in source_rows] != list(range(11)):
            raise AssertionError("M21F MCU sequence was not source-preserved")
        if [row["mcu_t_us"] for row in source_rows] != [
                i * 20000 for i in range(11)]:
            raise AssertionError("M21F MCU t_us was not source-preserved")
        for row, msg in zip(source_rows, received_imus):
            actual_ros_stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            if row["header_ros_ns"] != actual_ros_stamp:
                raise AssertionError("M21F source tap timestamp differs from ROS Imu")

        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "synthetic_m21f.jsonl"
            report = capture.finish(out)
            if report["recorder_event_count"] != 33:
                raise RuntimeError("M21F wrong number of simulated metadata events")
            if len(load(out)) != 34:
                raise RuntimeError("M21F malformed JSONL recorder output")
            if not (report["verdict"] == "M21C_OFFLINE_METADATA_FORMAT_VALID"
                    and report["physical_camera_imu_offset_validated"] is False
                    and report["recorder_device_access"] is False):
                raise RuntimeError("M21F offline metadata contract rejected")
            print("M21F_SOURCE_REPORT_JSON=" + json.dumps(report, sort_keys=True),
                  flush=True)
        print("M21F_SOURCE_METADATA_SELFTEST=PASS", flush=True)
        print("M21F_TEST_VERDICT=SIMULATED_IMU_SOURCE_HOOK_ONLY", flush=True)
        print("M21F_DEVICE_ACCESS=NONE", flush=True)
        return 0
    finally:
        for node in (observer, cam_pub_node, info_pub_node, imu_observer, imu_source):
            if node is not None:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(run())
    except Exception as exc:
        print("M21F_ROS_ADAPTER_SELFTEST=FAIL", flush=True)
        print("M21F_ERROR=" + type(exc).__name__ + ": " + str(exc), flush=True)
        raise
