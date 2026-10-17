#!/usr/bin/env python3
"""M21E simulated ROS camera adapter contract: NO physical sensor access.

This selftest intentionally publishes synthetic camera *metadata* only from
two disposable ROS nodes with the expected names. Such graph names prove
neither genuine device provenance nor camera/IMU physical synchronization.
It never creates a serial device or a motor/IMU publisher.
"""
import json
import os
from pathlib import Path
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

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
    raise RuntimeError("M21E simulated ROS graph/callback timeout: " + name)


def stamp(message, stamp_ns):
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(
        stamp_ns, 1_000_000_000)
    message.header.frame_id = "camera_optical_frame"
    return message


def assert_no_physical_devices():
    for path in ("/dev/video2", "/dev/ttyACM0"):
        if os.path.exists(path):
            raise RuntimeError("M21E refuses mapped physical sensor device: " + path)


def run():
    assert_no_physical_devices()
    for key, val in EXPECTED_ENV.items():
        if os.environ.get(key) != val:
            raise RuntimeError("M21E refuses non-isolated ROS environment: " + key)
    capture = MetadataCapture("SYNTHETIC", max_events=40)
    observer = cam_pub_node = info_pub_node = None
    rclpy.init(args=[])
    try:
        observer = PassiveCameraMetadataNode(capture)
        # No camera publishers: callback must fail closed on graph provenance.
        try:
            observer._verify_graph("/camera/image_rect", "rectify_node",
                                   "/camera", "sensor_msgs/msg/Image")
        except RuntimeError:
            pass
        else:
            raise RuntimeError("M21E accepted image without any publisher")

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
                raise AssertionError("M21E fake frame includes pixel data")
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

            raw = {
                "cal": 1, "seq": index, "t_us": index * 100_000,
                "gr_mrad_s": 200, "gp_mrad_s": -100, "gy_mrad_s": 50,
            }
            record_from_m18_source(
                capture, raw, max(time.time_ns(), info.header.stamp.sec * 10**9
                                  + info.header.stamp.nanosec + 1),
                time.monotonic_ns())

        # No source metadata from a packet that M18 would withhold.
        before = len(capture.rows)
        try:
            record_from_m18_source(
                capture, {"cal": 0}, time.time_ns(), time.monotonic_ns())
        except ValueError:
            pass
        else:
            raise AssertionError("M21E accepted unpublished MCU packet")
        if len(capture.rows) != before:
            raise AssertionError("M21E recorded rejected MCU packet")

        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "synthetic_m21e.jsonl"
            report = capture.finish(out)
            if report["recorder_event_count"] != 33:
                raise RuntimeError("M21E wrong number of simulated metadata events")
            if len(load(out)) != 34:
                raise RuntimeError("M21E malformed JSONL recorder output")
            if not (report["verdict"] == "M21C_OFFLINE_METADATA_FORMAT_VALID"
                    and report["physical_camera_imu_offset_validated"] is False
                    and report["recorder_device_access"] is False):
                raise RuntimeError("M21E offline metadata contract rejected")
            print("M21E_SIMULATED_REPORT_JSON=" + json.dumps(report, sort_keys=True),
                  flush=True)
        print("M21E_ROS_ADAPTER_SELFTEST=PASS", flush=True)
        print("M21E_TEST_VERDICT=SIMULATED_ROS_METADATA_CONTRACT_ONLY", flush=True)
        print("M21E_DEVICE_ACCESS=NONE", flush=True)
        return 0
    finally:
        for node in (observer, cam_pub_node, info_pub_node):
            if node is not None:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(run())
    except Exception as exc:
        print("M21E_ROS_ADAPTER_SELFTEST=FAIL", flush=True)
        print("M21E_ERROR=" + type(exc).__name__ + ": " + str(exc), flush=True)
        raise
