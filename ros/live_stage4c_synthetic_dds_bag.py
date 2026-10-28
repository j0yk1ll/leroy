#!/usr/bin/env python3
"""Real rclpy/DDS + SQLite integration of wholly SYNTHETIC Stage 4C data.

No STM32, camera, serial, M18 runtime, motor publisher, live recorder or
robot bridge is imported or launched. A fake M18-NAMED ROS Node is used only
inside a disconnected, no-device test container. The live capture hard gate
remains unchanged and no physical calibration is claimed.
"""
import gc
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import time

from live_stage4c_capture_contract import (
    CaptureContract, CaptureError, METADATA_TOPIC, TOPICS, source_metadata,
)
from live_stage4c_source_meta_ros import SourceMetadataRosAdapter

SYNTHETIC_COUNT = 12
DEVICE_PATHS = ("/dev/ttyACM0", "/dev/video0", "/dev/video2", "/dev/gpiochip0")
FORBIDDEN_TOPICS = {
    "/cmd_vel", "/cmd_vel_nav", "/cmd_vel_smoothed", "/motor/command",
    "/motor/enable", "/joint_commands", "/odom", "/tf",
}


def check_isolation(environ=None, exists=None):
    """Pure preflight; no ROS runtime or hardware access."""
    env = os.environ if environ is None else environ
    test = (lambda path: path.exists()) if exists is None else exists
    for key, expected in (
        ("LEROY_STAGE4C_SYNTHETIC_ONLY", "1"),
        ("ROS_DOMAIN_ID", "171"),
        ("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
        ("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
    ):
        if env.get(key) != expected:
            raise CaptureError("synthetic_preflight_environment_" + key)
    if any(test(Path(path)) for path in DEVICE_PATHS):
        raise CaptureError("synthetic_preflight_device_present")
    return True


def run_offline():
    check_isolation()
    import rclpy
    import rosbag2_py
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import CameraInfo, Image, Imu
    from std_msgs.msg import String

    types = {
        "/camera/image_rect": Image,
        "/camera/camera_info": CameraInfo,
        "/imu/data_raw": Imu,
        METADATA_TOPIC: String,
    }
    if set(types) != set(TOPICS):
        raise CaptureError("synthetic_topic_drift")

    rclpy.init(args=[])
    source = camera = observer = None
    writer = reader = None
    try:
        # FAKE source with the expected identity, not ImuStreamSource.
        source = Node("m18_imu_source", enable_rosout=False,
                      start_parameter_services=False)
        camera = Node("stage4c_synthetic_camera", enable_rosout=False,
                      start_parameter_services=False)
        observer = Node("stage4c_synthetic_receipt", enable_rosout=False,
                        start_parameter_services=False)
        imu_pub = source.create_publisher(
            Imu, "/imu/data_raw", qos_profile_sensor_data)
        adapter = SourceMetadataRosAdapter(source)
        image_pub = camera.create_publisher(
            Image, "/camera/image_rect", qos_profile_sensor_data)
        info_pub = camera.create_publisher(
            CameraInfo, "/camera/camera_info", qos_profile_sensor_data)
        publishers = {
            "/camera/image_rect": image_pub,
            "/camera/camera_info": info_pub,
            "/imu/data_raw": imu_pub,
            METADATA_TOPIC: adapter._publisher,
        }
        policy = CaptureContract(duration_s=45)
        written = {}
        faults = []
        with TemporaryDirectory(prefix="stage4c-synthetic-dds-") as tmp:
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

            subscriptions = []
            for topic, cls in types.items():
                def receive(msg, name=topic):
                    try:
                        metadata = (
                            source_metadata(msg.data)
                            if name == METADATA_TOPIC else None)
                        stamp = (
                            metadata["imu_stamp_ns"] if metadata is not None
                            else (int(msg.header.stamp.sec) * 1_000_000_000
                                  + int(msg.header.stamp.nanosec)))
                        frame = (None if metadata is not None
                                 else msg.header.frame_id)
                        if name == "/camera/image_rect" and (
                            not msg.data or msg.width != 2 or msg.height != 2
                        ):
                            raise CaptureError("invalid_synthetic_image")
                        raw = bytes(serialize_message(msg))
                        key = (name, stamp)
                        if key in written:
                            raise CaptureError("duplicate_synthetic_receipt")
                        policy.accept(
                            name, stamp, frame, raw, time.monotonic_ns(),
                            writer.write, metadata=metadata)
                        written[key] = raw
                    except Exception as exc:
                        faults.append(type(exc).__name__ + ":" + str(exc))
                subscriptions.append(observer.create_subscription(
                    cls, topic, receive, qos_profile_sensor_data))

            deadline = time.monotonic() + 12.0
            while time.monotonic() < deadline:
                if all(pub.get_subscription_count() == 1
                       for pub in publishers.values()):
                    break
                rclpy.spin_once(observer, timeout_sec=0.05)
            else:
                raise CaptureError("synthetic_dds_discovery_timeout")
            visible = {name for name, _ in observer.get_topic_names_and_types()}
            if visible & FORBIDDEN_TOPICS:
                raise CaptureError("forbidden_synthetic_topic")
            for topic, (topic_type, _) in TOPICS.items():
                pubs = observer.get_publishers_info_by_topic(topic)
                owner = ("m18_imu_source" if topic in (
                    "/imu/data_raw", METADATA_TOPIC)
                    else "stage4c_synthetic_camera")
                if (len(pubs) != 1 or pubs[0].node_name != owner
                        or pubs[0].node_namespace != "/"
                        or pubs[0].topic_type != topic_type):
                    raise CaptureError("wrong_synthetic_publisher_provenance")

            origin = 1_790_000_000_000_000_000
            for n in range(SYNTHETIC_COUNT):
                stamp = origin + n * 20_000_000
                sec, nano = divmod(stamp, 1_000_000_000)
                img = Image()
                img.header.stamp.sec = sec
                img.header.stamp.nanosec = nano
                img.header.frame_id = "camera_optical_frame"
                img.width = 2
                img.height = 2
                img.encoding = "mono8"
                img.step = 2
                img.data = bytes((n, n+1, n+2, n+3))
                info = CameraInfo()
                info.header.stamp.sec = sec
                info.header.stamp.nanosec = nano
                info.header.frame_id = "camera_optical_frame"
                info.width = 2
                info.height = 2
                info.k = [500., 0., 1., 0., 500., 1., 0., 0., 1.]
                imu = Imu()
                imu.header.stamp.sec = sec
                imu.header.stamp.nanosec = nano
                imu.header.frame_id = "imu_link"
                imu.orientation_covariance[0] = -1.0
                imu.angular_velocity.z = 0.12
                imu.linear_acceleration.z = 9.8

                image_pub.publish(img)
                info_pub.publish(info)
                imu_pub.publish(imu)
                # Inject original-shaped *synthetic* seq/t_us only AFTER
                # successfully publishing this synthetic IMU message.
                adapter.on_imu_published(
                    {"seq": n, "t_us": 1_000_000 + 20_000*n, "cal": 1},
                    stamp, time.monotonic_ns())

                until = time.monotonic() + 3.0
                while (time.monotonic() < until and not faults and
                       any(policy.counts[t] <= n for t in TOPICS)):
                    rclpy.spin_once(observer, timeout_sec=0.02)
                if faults:
                    raise CaptureError("synthetic_dds_receipt_fault:" + faults[0])
                if any(policy.counts[t] != n+1 for t in TOPICS):
                    raise CaptureError("synthetic_dds_message_missing")

            outcome = policy.report()
            if (adapter.tap.count != SYNTHETIC_COUNT
                    or outcome["matched_imu_metadata_stamps"] != SYNTHETIC_COUNT
                    or outcome["unmatched_imu_stamps"] != 0
                    or outcome["unmatched_metadata_stamps"] != 0
                    or outcome["readiness"] != "BLOCKED_INCOMPLETE_CAPTURE"
                    or outcome["physical_source_authenticated"] is not False
                    or outcome["physical_camera_imu_offset_validated"] is not False):
                raise CaptureError("synthetic_integration_report_invalid")

            del writer
            writer = None
            gc.collect()
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions(
                    input_serialization_format="cdr",
                    output_serialization_format="cdr"))
            replayed = {}
            while reader.has_next():
                topic, raw, stamp = reader.read_next()
                key = (topic, stamp)
                if key in replayed or key not in written:
                    raise CaptureError("synthetic_bag_extra_or_duplicate_record")
                replayed[key] = bytes(raw)
                if topic == METADATA_TOPIC:
                    record = source_metadata(
                        deserialize_message(raw, String).data)
                    if record["imu_stamp_ns"] != stamp:
                        raise CaptureError("synthetic_bag_metadata_stamp_invalid")
            if replayed != written:
                raise CaptureError("synthetic_bag_cdr_payload_mismatch")
            del reader
            reader = None
            gc.collect()
        print("STAGE4C_SYNTHETIC_DDS_BAG_INTEGRATION=PASS_NO_DEVICE", flush=True)
        print("STAGE4C_SYNTHETIC_IMU_META_MATCHED=" + str(SYNTHETIC_COUNT), flush=True)
        print("STAGE4C_LIVE_CAPTURE_AUTHORIZED=FALSE", flush=True)
        print("STAGE4C_PHYSICAL_CALIBRATION_VALIDATED=FALSE", flush=True)
        return 0
    finally:
        if reader is not None:
            del reader
        if writer is not None:
            del writer
        for node in (observer, camera, source):
            if node is not None:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(run_offline())
