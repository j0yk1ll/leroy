#!/usr/bin/env python3
"""M23H Jazzy recorder smoke: actual ros2 bag CLI, fake ROS messages ONLY.

A disposable /tmp SQLite bag is written from a deliberately fabricated
camera/CameraInfo/IMU publisher. No MCU, video device, bridge, robot,
odom, TF, motion or physical measurement participates.
"""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from m23g_live_recording_preflight import recorder_argv, TOPICS
from m23h_bounded_capture import closed_sqlite_counts, _signal_and_wait


def main():
    if Path("/dev/ttyACM0").exists() or Path("/dev/video2").exists():
        raise RuntimeError("M23H synthetic smoke refuses physical device mounts")
    if (os.environ.get("ROS_DOMAIN_ID") != "170"
            or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST"):
        raise RuntimeError("M23H synthetic smoke requires isolated ROS domain")
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy)
    from sensor_msgs.msg import Image, CameraInfo, Imu

    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    with tempfile.TemporaryDirectory(prefix="m23h-fake-cli-recorder-") as td:
        bag = Path(td) / "synthetic_generated"
        log = Path(td) / "recorder.log"
        recorder = None
        node = None
        rclpy.init(args=[])
        try:
            node = Node("m23h_fake_sources_only", enable_rosout=False,
                        start_parameter_services=False)
            image_pub = node.create_publisher(Image, TOPICS[0], qos)
            info_pub = node.create_publisher(CameraInfo, TOPICS[1], qos)
            imu_pub = node.create_publisher(Imu, TOPICS[2], qos)
            with log.open("wb") as output:
                recorder = subprocess.Popen(
                    recorder_argv(bag), stdout=output,
                    stderr=subprocess.STDOUT, start_new_session=True)
                deadline = time.monotonic() + 12.0
                subscribed = False
                while time.monotonic() < deadline:
                    if recorder.poll() is not None:
                        raise RuntimeError("rosbag CLI exited before discovery")
                    rclpy.spin_once(node, timeout_sec=0.05)
                    if all(pub.get_subscription_count() >= 1 for pub in
                           (image_pub, info_pub, imu_pub)):
                        subscribed = True
                        break
                    time.sleep(0.05)
                if not subscribed:
                    raise RuntimeError("rosbag CLI did not subscribe to all 3 topics")
                print("M23H_FAKE_RECORDER_SUBSCRIPTIONS=PASS", flush=True)

                # 128 IMU ticks at ~50Hz, 16 image/info pairs at ~6.25Hz.
                for i in range(128):
                    at = time.time_ns()
                    stamp_sec, stamp_ns = divmod(at, 1_000_000_000)
                    imu = Imu()
                    imu.header.stamp.sec = stamp_sec
                    imu.header.stamp.nanosec = stamp_ns
                    imu.header.frame_id = "imu_link"
                    imu.angular_velocity.z = 0.01
                    imu.linear_acceleration.z = 9.81
                    imu_pub.publish(imu)
                    if i % 8 == 0:
                        pic = Image()
                        pic.header.stamp.sec = stamp_sec
                        pic.header.stamp.nanosec = stamp_ns
                        pic.header.frame_id = "camera_optical_frame"
                        pic.width, pic.height = 4, 4
                        pic.encoding, pic.step, pic.is_bigendian = "rgb8", 12, 0
                        pic.data = bytes([i % 256, 127, 255] * 16)
                        info = CameraInfo()
                        info.header = pic.header
                        info.width, info.height = 4, 4
                        info.k = [100., 0., 2., 0., 100., 2.,
                                  0., 0., 1.]
                        info.p = [100., 0., 2., 0., 0., 100.,
                                  2., 0., 0., 0., 1., 0.]
                        image_pub.publish(pic)
                        info_pub.publish(info)
                    rclpy.spin_once(node, timeout_sec=0.001)
                    time.sleep(0.02)

                time.sleep(0.4)  # let bag writer drain fake DDS samples
                rc, finalized = _signal_and_wait(
                    recorder, signal.SIGINT, 35, process_group=True)
                recorder = None if finalized else recorder
                if not finalized or rc != 0:
                    raise RuntimeError("synthetic recorder did not finalize rc=" +
                                       str(rc))
        finally:
            if recorder is not None and recorder.poll() is None:
                _signal_and_wait(recorder, signal.SIGINT, 10,
                                 process_group=True)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        try:
            counts = closed_sqlite_counts(bag)
        except Exception:
            # Keep a bounded diagnostic excerpt (synthetic-only log).
            print("M23H_FAKE_RECORDER_LOG_TAIL=" +
                  log.read_text(errors="replace")[-4000:], flush=True)
            raise
        if (counts["/camera/image_rect"] < 12
                or counts["/camera/camera_info"] < 12
                or counts["/imu/data_raw"] < 100):
            raise AssertionError("synthetic CDR counts too low: " + str(counts))
        print("M23H_SYNTHETIC_ROS2_RECORDER_SQLITE=PASS", flush=True)
        print("M23H_SYNTHETIC_RECORDER_COUNTS=" + str(counts), flush=True)
        print("M23H_REAL_SENSOR_CAPTURE_VERIFIED=FALSE", flush=True)
        print("M23H_CALIBRATION_VALIDATED=FALSE", flush=True)
        print("M23H_DEVICE_ACCESS=NONE", flush=True)


if __name__ == "__main__":
    main()
