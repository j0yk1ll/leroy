#!/usr/bin/env python3
"""M21L: simulated ROS executor + source worker quiescence, no hardware.

Run only inside the no-device M21L image. Publishers and in-memory M18
telemetry are SYNTHETIC. Real graph names do not authenticate real devices.
"""
import json
import os
from pathlib import Path
import tempfile
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from m18_imu_stream_source import ImuStreamSource
from m21d_metadata_capture import MetadataCapture
from m21d_passive_ros_adapter import (
    EXPECTED_ENV, PassiveCameraMetadataNode, record_from_m18_source,
)
from m21f_imu_source_hook_selftest import fake_packet
from m21k_offline_quiescence import finish_after_quiescence
from m21_record_contract import load, summarize


def environment_guard():
    for key, val in EXPECTED_ENV.items():
        if os.environ.get(key) != val:
            raise RuntimeError("M21L isolated ROS environment mismatch: " + key)
    for path in ("/dev/video2", "/dev/ttyACM0"):
        if os.path.exists(path):
            raise RuntimeError("M21L unexpected physical sensor device: " + path)


def stamped(message, ns):
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(
        ns, 1_000_000_000)
    message.header.frame_id = "camera_optical_frame"
    return message


def wait_for(predicate, description, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise RuntimeError("M21L timeout: " + description)


def run():
    environment_guard()
    rclpy.init(args=[])
    capture = MetadataCapture("SYNTHETIC", max_events=200)
    end = threading.Event()
    errors = []
    order = []
    nodes = []
    threads = []
    executor = None
    try:
        camera_observer = PassiveCameraMetadataNode(capture)
        nodes.append(camera_observer)
        imu = ImuStreamSource(
            on_imu_published=lambda fields, stamp, receipt:
                record_from_m18_source(capture, fields, stamp, receipt))
        nodes.append(imu)
        fake_image_node = Node("rectify_node", namespace="/camera",
                               enable_rosout=False, start_parameter_services=False)
        fake_info_node = Node("usb_cam", namespace="/camera",
                              enable_rosout=False, start_parameter_services=False)
        nodes.extend((fake_image_node, fake_info_node))
        image_pub = fake_image_node.create_publisher(
            Image, "/camera/image_rect", qos_profile_sensor_data)
        info_pub = fake_info_node.create_publisher(
            CameraInfo, "/camera/camera_info", qos_profile_sensor_data)

        executor = SingleThreadedExecutor()
        executor.add_node(camera_observer)

        def executor_worker():
            try:
                while not end.is_set() and rclpy.ok():
                    executor.spin_once(timeout_sec=0.02)
            except Exception as exc:
                errors.append("executor: " + type(exc).__name__ + ": " + str(exc))
                end.set()
            finally:
                order.append("executor_exit")

        spin_thread = threading.Thread(
            target=executor_worker, name="m21l_ros_executor", daemon=True)
        threads.append(spin_thread)
        spin_thread.start()

        wait_for(
            lambda: (
                len(camera_observer.get_publishers_info_by_topic(
                    "/camera/image_rect")) == 1
                and len(camera_observer.get_publishers_info_by_topic(
                    "/camera/camera_info")) == 1),
            "two synthetic camera publishers")
        camera_observer._verify_graph(
            "/camera/image_rect", "rectify_node", "/camera",
            "sensor_msgs/msg/Image")
        camera_observer._verify_graph(
            "/camera/camera_info", "usb_cam", "/camera",
            "sensor_msgs/msg/CameraInfo")

        def camera_worker():
            index = 0
            try:
                while not end.is_set() and index < 40:
                    now = time.time_ns()
                    image = stamped(Image(), now)
                    image.width, image.height = 640, 480
                    assert len(image.data) == 0
                    info = stamped(CameraInfo(), max(time.time_ns(), now + 1))
                    info.width, info.height = 640, 480
                    image_pub.publish(image)
                    info_pub.publish(info)
                    index += 1
                    end.wait(0.025)
            except Exception as exc:
                errors.append("camera_worker: " + repr(exc))
                end.set()
            finally:
                order.append("camera_exit")

        def imu_worker():
            index = 0
            try:
                while not end.is_set() and index < 40:
                    imu.consume(fake_packet(index))
                    index += 1
                    end.wait(0.025)
            except Exception as exc:
                errors.append("imu_worker: " + repr(exc))
                end.set()
            finally:
                order.append("imu_exit")

        cam_thread = threading.Thread(
            target=camera_worker, name="m21l_fake_camera_producer", daemon=True)
        imu_thread = threading.Thread(
            target=imu_worker, name="m21l_fake_imu_source", daemon=True)
        threads.extend((cam_thread, imu_thread))
        cam_thread.start()
        imu_thread.start()

        def sufficient_records():
            with capture._lock:
                return all(sum(row["kind"] == kind for row in capture.rows[1:]) >= 4
                           for kind in ("camera_image", "camera_info", "imu"))
        wait_for(lambda: bool(errors) or sufficient_records(),
                 "four events of each simulated sensor")
        if errors:
            raise RuntimeError("M21L producer failed: " + repr(errors))

        with tempfile.TemporaryDirectory(prefix="m21l-ros-sim-") as tmp:
            destination = Path(tmp) / "synthetic.jsonl"
            def fake_stop():
                order.append("fake_STOP")
                return True
            def request_shutdown():
                order.append("request_quiescence")
                end.set()
            lifecycle = finish_after_quiescence(
                capture, destination, fake_stop, request_shutdown, threads,
                join_timeout_s=2.0)
            if errors:
                raise RuntimeError("M21L producer fault: " + repr(errors))
            if not lifecycle["producers_quiescent"]:
                raise RuntimeError("M21L executor or source producer still alive")
            if lifecycle["verdict"] != "SYNTHETIC_STOP_QUIESCE_SAVE_CONTRACT":
                raise RuntimeError("M21L lifecycle refused: " + repr(lifecycle))
            if (order[:2] != ["fake_STOP", "request_quiescence"]
                    or "executor_exit" not in order
                    or "camera_exit" not in order
                    or "imu_exit" not in order):
                raise RuntimeError("M21L incorrect simulated lifecycle order")
            records = load(destination)
            report = summarize(records)
            if (len(records) != lifecycle["saved_event_count"] + 1
                    or report["evidence_type"] != "SYNTHETIC"
                    or report["physical_camera_imu_offset_validated"] is not False):
                raise RuntimeError("M21L JSONL evidence invariant failed")
            if any(report["stream_diagnostics"][kind]["messages"] < 4
                   for kind in ("camera_image", "camera_info", "imu")):
                raise RuntimeError("M21L insufficient simulated sensor events")
            print("M21L_LIFECYCLE_REPORT_JSON=" + json.dumps(lifecycle, sort_keys=True),
                  flush=True)
            print("M21L_SCHEMA_REPORT_JSON=" + json.dumps(report, sort_keys=True),
                  flush=True)

        print("M21L_ROS_QUIESCENCE_SELFTEST=PASS", flush=True)
        print("M21L_VERDICT=SIMULATED_EXECUTOR_AND_SOURCE_JOIN_ONLY", flush=True)
        print("M21L_DEVICE_ACCESS=NONE", flush=True)
        return 0
    finally:
        end.set()
        for thread in threads:
            if thread.ident is not None:
                thread.join(timeout=1.0)
        # ROS shutdown occurs only after executor spin thread termination.
        if executor is not None and not any(t.is_alive() for t in threads):
            executor.shutdown(timeout_sec=1.0)
            for node in nodes:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        elif any(t.is_alive() for t in threads):
            print("M21L_CLEANUP=EXECUTOR_OR_PRODUCER_STILL_ACTIVE",
                  flush=True)


if __name__ == "__main__":
    try:
        raise SystemExit(run())
    except Exception as exc:
        print("M21L_ROS_QUIESCENCE_SELFTEST=FAIL", flush=True)
        print("M21L_ERROR=" + type(exc).__name__ + ": " + str(exc), flush=True)
        raise
