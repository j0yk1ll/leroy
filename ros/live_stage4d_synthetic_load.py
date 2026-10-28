#!/usr/bin/env python3
"""Stage 4D: 5s paced 50Hz IMU + 30Hz 640x480 camera, synthetic DDS+SQLite.

This is NOT the real M18 source or the disabled live recorder. The synthetic
ROS nodes have NO device access, STOP/START packets, motor commands, or
physical sensor provenance. Run only in rootless, no-network, private IPC/PID,
no-device Podman with tmpfs. Fail closed on gaps, disk error and bad graph.
"""
import gc
import hashlib
import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import time

from live_stage4c_capture_contract import (
    CaptureContract, CaptureError, METADATA_TOPIC, TOPICS, source_metadata,
)
from live_stage4d_capture_ingress import CaptureIngress
from live_stage4c_source_meta_ros import SourceMetadataRosAdapter
from live_stage4c_synthetic_dds_bag import check_isolation, FORBIDDEN_TOPICS

IMU_COUNT = 250
CAMERA_COUNT = 150
PERIOD_S = 0.02
MAX_DATABASE_BYTES = 96_000_000


def run_offline():
    check_isolation()
    import rclpy
    import rosbag2_py
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import serialize_message, deserialize_message
    from sensor_msgs.msg import Image, CameraInfo, Imu
    from std_msgs.msg import String

    types = {
        "/camera/image_rect": Image,
        "/camera/camera_info": CameraInfo,
        "/imu/data_raw": Imu,
        METADATA_TOPIC: String,
    }
    if set(types) != set(TOPICS):
        raise CaptureError("stage4d_topic_contract_drift")

    # No live capture/MCU node; create only disposable, fake ROS sources.
    rclpy.init(args=[])
    source = camera = receiver = None
    writer = reader = None
    try:
        source = Node("m18_imu_source", enable_rosout=False,
                      start_parameter_services=False)
        camera = Node("stage4d_synthetic_camera", enable_rosout=False,
                      start_parameter_services=False)
        receiver = Node("stage4d_synthetic_recorder", enable_rosout=False,
                        start_parameter_services=False)
        p_imu = source.create_publisher(Imu, "/imu/data_raw", qos_profile_sensor_data)
        meta_adapter = SourceMetadataRosAdapter(source)
        p_image = camera.create_publisher(
            Image, "/camera/image_rect", qos_profile_sensor_data)
        p_info = camera.create_publisher(
            CameraInfo, "/camera/camera_info", qos_profile_sensor_data)
        publishers = {
            "/imu/data_raw": p_imu,
            METADATA_TOPIC: meta_adapter._publisher,
            "/camera/image_rect": p_image,
            "/camera/camera_info": p_info,
        }
        errors = []
        counts = dict.fromkeys(TOPICS, 0)
        replay_hashes = {}
        last_publish_ns = {}
        delivery_ms = []

        with TemporaryDirectory(prefix="stage4d-synthetic-load-") as tmp:
            bag = Path(tmp) / "bag"
            writer = rosbag2_py.SequentialWriter()
            writer.open(
                rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions(
                    input_serialization_format="cdr",
                    output_serialization_format="cdr"))
            for topic, (type_name, _) in TOPICS.items():
                writer.create_topic(rosbag2_py.TopicMetadata(
                    id=0, name=topic, type=type_name,
                    serialization_format="cdr"))

            contract = CaptureContract(duration_s=30)
            ingress = CaptureIngress(
                contract=contract, serialize=lambda msg: bytes(
                    serialize_message(msg)), write=writer.write)
            subscriptions = []
            for topic, cls in types.items():
                def receive(msg, name=topic):
                    if errors:
                        return
                    try:
                        received_ns = time.monotonic_ns()
                        stamp = (
                            source_metadata(msg.data)["imu_stamp_ns"]
                            if name == METADATA_TOPIC else
                            int(msg.header.stamp.sec) * 1_000_000_000
                            + int(msg.header.stamp.nanosec))
                        key = (name, stamp)
                        if key in replay_hashes:
                            raise CaptureError("duplicate_stage4d_receipt")
                        ingress.accept(name, msg, receipt_ns=received_ns)
                        # Hash the exact payload stored via the same ingress
                        # serializer and writer path used by the dormant
                        # Stage 4C candidate. Do not retain image payloads.
                        replay_hashes[key] = hashlib.sha256(
                            serialize_message(msg)).digest()
                        counts[name] += 1
                        sent_ns = last_publish_ns.get(key)
                        if sent_ns is not None:
                            delay = (received_ns - sent_ns) / 1_000_000
                            if delay < 0:
                                raise CaptureError("negative_stage4d_delay")
                            delivery_ms.append(delay)
                    except Exception as exc:
                        errors.append(type(exc).__name__ + ":" + str(exc))
                subscriptions.append(receiver.create_subscription(
                    cls, topic, receive, qos_profile_sensor_data))

            wait_until = time.monotonic() + 12.0
            while time.monotonic() < wait_until:
                if all(p.get_subscription_count() == 1
                       for p in publishers.values()):
                    break
                rclpy.spin_once(receiver, timeout_sec=0.05)
            else:
                raise CaptureError("stage4d_dds_discovery_timeout")
            topics_seen = {x for x, _ in receiver.get_topic_names_and_types()}
            if topics_seen & FORBIDDEN_TOPICS:
                raise CaptureError("stage4d_forbidden_graph_topic")
            for topic, (type_name, _) in TOPICS.items():
                pubs = receiver.get_publishers_info_by_topic(topic)
                wanted = ("m18_imu_source" if topic in (
                    "/imu/data_raw", METADATA_TOPIC) else
                    "stage4d_synthetic_camera")
                if (len(pubs) != 1 or pubs[0].node_name != wanted or
                        pubs[0].node_namespace != "/" or
                        pubs[0].topic_type != type_name):
                    raise CaptureError("stage4d_source_provenance_mismatch")

            origin = 1_790_000_000_000_000_000
            pixels = bytes([90]) * (640 * 480)
            clock_start = time.monotonic()
            images_sent = 0
            for i in range(IMU_COUNT):
                target = clock_start + i * PERIOD_S
                while time.monotonic() < target and not errors:
                    rclpy.spin_once(
                        receiver, timeout_sec=min(
                            0.002, max(0.0, target - time.monotonic())))
                if errors:
                    raise CaptureError(errors[0])
                stamp = origin + i * 20_000_000
                sec, nano = divmod(stamp, 1_000_000_000)
                imu = Imu()
                imu.header.stamp.sec = sec
                imu.header.stamp.nanosec = nano
                imu.header.frame_id = "imu_link"
                imu.orientation_covariance[0] = -1.0
                imu.angular_velocity.z = 0.11
                imu.linear_acceleration.z = 9.8
                last_publish_ns[("/imu/data_raw", stamp)] = time.monotonic_ns()
                p_imu.publish(imu)
                meta_adapter.on_imu_published(
                    {"seq": i, "t_us": i * 20_000, "cal": 1},
                    stamp, time.monotonic_ns())
                last_publish_ns[(METADATA_TOPIC, stamp)] = time.monotonic_ns()

                # 3 of every 5 IMU ticks => exactly 150 frames / 5 seconds,
                # 30 Hz average. The 640x480 payload exercises disk load.
                if i % 5 in (0, 2, 4):
                    images_sent += 1
                    image_stamp = origin + (
                        (images_sent - 1) * 1_000_000_000 // 30)
                    cs, cn = divmod(image_stamp, 1_000_000_000)
                    img = Image()
                    img.header.stamp.sec = cs
                    img.header.stamp.nanosec = cn
                    img.header.frame_id = "camera_optical_frame"
                    img.width = 640
                    img.height = 480
                    img.encoding = "mono8"
                    img.step = 640
                    img.data = pixels
                    info = CameraInfo()
                    info.header.stamp.sec = cs
                    info.header.stamp.nanosec = cn
                    info.header.frame_id = "camera_optical_frame"
                    info.width = 640
                    info.height = 480
                    info.k = [
                        500., 0., 320., 0., 500., 240., 0., 0., 1.]
                    last_publish_ns[("/camera/image_rect", image_stamp)] = time.monotonic_ns()
                    p_image.publish(img)
                    last_publish_ns[("/camera/camera_info", image_stamp)] = time.monotonic_ns()
                    p_info.publish(info)
                for _ in range(8):
                    rclpy.spin_once(receiver, timeout_sec=0.0)
                if errors:
                    raise CaptureError(errors[0])
                if time.monotonic() - clock_start > 15.0:
                    raise CaptureError("stage4d_producer_overrun")

            expected = {
                "/imu/data_raw": IMU_COUNT,
                METADATA_TOPIC: IMU_COUNT,
                "/camera/image_rect": CAMERA_COUNT,
                "/camera/camera_info": CAMERA_COUNT,
            }
            until = time.monotonic() + 5.0
            while (time.monotonic() < until and not errors and counts != expected):
                rclpy.spin_once(receiver, timeout_sec=0.02)
            if errors:
                raise CaptureError("stage4d_subscriber_or_disk_fault:" + errors[0])
            if counts != expected or images_sent != CAMERA_COUNT:
                raise CaptureError("stage4d_incomplete_synthetic_receipts:" +
                                   json.dumps(counts, sort_keys=True))
            report = contract.report()
            if (report["matched_imu_metadata_stamps"] != IMU_COUNT or
                    report["readiness"] != "BLOCKED_INCOMPLETE_CAPTURE" or
                    report["physical_source_authenticated"] is not False or
                    report["physical_camera_imu_offset_validated"] is not False):
                raise CaptureError("stage4d_synthetic_contract_invalid")
            del writer
            writer = None
            gc.collect()
            files = list(bag.glob("*.db3"))
            if len(files) != 1 or files[0].stat().st_size > MAX_DATABASE_BYTES:
                raise CaptureError("stage4d_database_size_limit")
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions(
                    input_serialization_format="cdr",
                    output_serialization_format="cdr"))
            actual = {}
            while reader.has_next():
                topic, raw, stamp = reader.read_next()
                key = (topic, stamp)
                if key in actual or key not in replay_hashes:
                    raise CaptureError("stage4d_bad_bag_topic_stamp")
                actual[key] = hashlib.sha256(bytes(raw)).digest()
                if topic == METADATA_TOPIC:
                    record = source_metadata(
                        deserialize_message(raw, String).data)
                    if record["imu_stamp_ns"] != stamp:
                        raise CaptureError("stage4d_metadata_stamp_changed")
            if actual != replay_hashes:
                raise CaptureError("stage4d_bag_payload_or_count_mismatch")
            del reader
            reader = None
            gc.collect()
        delays = sorted(delivery_ms)
        p95 = delays[int(0.95 * (len(delays)-1))] if delays else None
        print("STAGE4D_SYNTHETIC_DDS_LOAD=PASS_NO_DEVICE", flush=True)
        print("STAGE4D_DDS_COUNTS_JSON=" + json.dumps(counts, sort_keys=True),
              flush=True)
        print("STAGE4D_DDS_CALLBACK_DELAY_MS_P95=" + str(round(p95, 3)),
              flush=True)
        print("STAGE4D_LIVE_CAPTURE_AUTHORIZED=FALSE", flush=True)
        print("STAGE4D_PHYSICAL_CALIBRATION_VALIDATED=FALSE", flush=True)
        return 0
    finally:
        if reader is not None:
            del reader
        if writer is not None:
            del writer
        for node in (receiver, camera, source):
            if node is not None:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(run_offline())
