#!/usr/bin/env python3
"""No-device M23BV DDS / rectifier / rosbag2 integration test.

Runs only in a pinned isolated container. All image and IMU data are synthetic.
Real production thresholds and all physical-use gates remain unchanged.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import get_rmw_implementation_identifier
from sensor_msgs.msg import CameraInfo, Image, Imu

from m23bv_qos_guard import snapshot, validate_camera_graph
from m23bv_derive_closed_bag import derive, fingerprints, new_reader, scan, ros_decode
from m23bv_window import select

IMAGE_RECT = '/camera/image_rect'
IMAGE_RAW = '/camera/image_raw'
INFO = '/camera/camera_info'
IMU = '/imu/data_raw'


def stop_process(proc, label, timeout=20):
    if proc is None:
        return None
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
    try:
        rc = proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        # Ephemeral synthetic test only; not the physical M20 sensor process.
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
        print('M23BV_DDS_' + label + '=BLOCKED_SHUTDOWN_TIMEOUT', flush=True)
        return None
    print('M23BV_DDS_' + label + '_EXIT=' + str(rc), flush=True)
    return rc


def publish_stamp(msg, timestamp_ns, frame):
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(timestamp_ns, 1_000_000_000)
    msg.header.frame_id = frame


def main():
    print('M23BV_DDS_CONTAINER_SCRIPT_STARTED=PASS', flush=True)
    if any(Path(p).exists() for p in ('/dev/video2', '/dev/ttyACM0')):
        raise RuntimeError('BLOCKED_PHYSICAL_SENSOR_DEVICE_PRESENT')
    if os.environ.get('ROS_DOMAIN_ID') != '178' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise RuntimeError('BLOCKED_ISOLATION_ENV')
    rect_path = Path(os.popen('ros2 pkg prefix image_proc').read().strip()) / 'lib/image_proc/rectify_node'
    if not rect_path.is_file():
        raise RuntimeError('BLOCKED_RECTIFIER_EXECUTABLE_MISSING')
    rclpy.init(args=[])
    node = rclpy.create_node('usb_cam', namespace='/camera',
                             start_parameter_services=False, enable_rosout=False)
    reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                          reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE)
    sensor_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                            reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE)
    pub_image = node.create_publisher(Image, IMAGE_RAW, reliable)
    pub_info = node.create_publisher(CameraInfo, INFO, reliable)
    pub_imu = node.create_publisher(Imu, IMU, sensor_qos)
    rectified = []
    node.create_subscription(Image, IMAGE_RECT,
                             lambda m: rectified.append(m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec),
                             reliable)
    recorder = rectifier = None
    recorder_rc = rectifier_rc = None
    with tempfile.TemporaryDirectory(prefix='m23bv-synthetic-', dir='/tmp') as scratch:
        scratch = Path(scratch)
        bag_dir = scratch / 'bag'
        rect_log = (scratch / 'rectifier.log').open('w+')
        bag_log = (scratch / 'recorder.log').open('w+')
        try:
            # Use same YAML QoS policy and a separately running real rosbag2 recorder.
            recorder = subprocess.Popen([
                'ros2', 'bag', 'record', '--storage', 'sqlite3',
                '--output', str(bag_dir),
                '--qos-profile-overrides-path', '/opt/m23bv/m23bv_sensor_qos.yaml',
                '--topics', IMAGE_RECT, INFO, IMU,
            ], stdout=bag_log, stderr=subprocess.STDOUT)
            # Exact override structure matches the additive M23BV launch file.
            params_path = scratch / 'rectifier.yaml'
            params_path.write_text('''/camera/rectify_node:
  ros__parameters:
    qos_overrides:
      /camera/image_raw:
        subscription:
          reliability: reliable
          history: keep_last
          depth: 100
      /camera/image_rect:
        publisher:
          reliability: reliable
          history: keep_last
          depth: 100
''')
            env = dict(os.environ)
            env['LD_LIBRARY_PATH'] = '/opt/m23aj-opencv/lib:' + env.get('LD_LIBRARY_PATH', '')
            rectifier = subprocess.Popen([
                str(rect_path), '--ros-args', '-r', '__node:=rectify_node',
                '-r', '__ns:=/camera', '-r', 'image:=image_raw',
                '--params-file', str(params_path),
            ], env=env, stdout=rect_log, stderr=subprocess.STDOUT)
            wait_until = time.monotonic() + 16.0
            next_cam = next_imu = time.monotonic()
            graph_pass = False
            last_graph_error = 'BLOCKED_GRAPH_NOT_READY'
            last_graph = None
            sample_end = None
            images_sent = imu_sent = 0
            while True:
                now = time.monotonic()
                if recorder.poll() is not None or rectifier.poll() is not None:
                    raise RuntimeError('BLOCKED_EARLY_PROCESS_EXIT:' + str((recorder.poll(), rectifier.poll())))
                if now >= next_cam:
                    stamp = time.time_ns()
                    img = Image()
                    publish_stamp(img, stamp, 'camera_optical_frame')
                    img.height, img.width = 16, 16
                    img.encoding, img.is_bigendian, img.step = 'mono8', 0, 16
                    img.data = bytes([images_sent % 256] * 256)
                    ci = CameraInfo()
                    publish_stamp(ci, stamp, 'camera_optical_frame')
                    ci.height, ci.width = 16, 16
                    ci.distortion_model = 'plumb_bob'
                    ci.d = [0.03, -0.01, 0., 0., 0.]
                    ci.k = [100., 0., 8., 0., 100., 8., 0., 0., 1.]
                    ci.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
                    ci.p = [100., 0., 8., 0., 0., 100., 8., 0., 0., 0., 1., 0.]
                    pub_info.publish(ci)
                    pub_image.publish(img)
                    images_sent += 1
                    next_cam = now + (1.0 / 30.0)
                if graph_pass and now >= next_imu:
                    imu = Imu()
                    publish_stamp(imu, time.time_ns(), 'imu_link')
                    pub_imu.publish(imu)
                    imu_sent += 1
                    next_imu = now + 0.005
                rclpy.spin_once(node, timeout_sec=0.001)
                if not graph_pass and images_sent % 4 == 0:
                    last_graph = snapshot(node, get_rmw_implementation_identifier())
                    try:
                        validate_camera_graph(last_graph)
                        graph_pass = True
                        sample_end = time.monotonic() + 9.0
                        next_imu = time.monotonic()
                        print('M23BV_DDS_GRAPH_QOS=PASS', flush=True)
                    except ValueError as err:
                        last_graph_error = str(err)
                if not graph_pass and now >= wait_until:
                    print('M23BV_DDS_GRAPH_JSON=' + json.dumps(last_graph, sort_keys=True), flush=True)
                    raise RuntimeError('M23BV_DDS_GRAPH_QOS=' + last_graph_error)
                if sample_end is not None and now >= sample_end:
                    break
            print('M23BV_DDS_PUBLISHED_IMAGE_RAW=' + str(images_sent), flush=True)
            print('M23BV_DDS_PUBLISHED_IMU=' + str(imu_sent), flush=True)
            print('M23BV_DDS_SUBSCRIBED_RECT=' + str(len(rectified)), flush=True)
            if len(rectified) < 80:
                raise RuntimeError('BLOCKED_RECTIFIED_IMAGE_DELIVERY')
        finally:
            # The synthetic rectifier exits before the bag recorder finalizes.
            rectifier_rc = stop_process(rectifier, 'RECTIFIER')
            recorder_rc = stop_process(recorder, 'RECORDER')
            try:
                node.destroy_node()
                rclpy.shutdown()
            finally:
                for name, log in [('RECTIFIER', rect_log), ('RECORDER', bag_log)]:
                    log.seek(0)
                    print('M23BV_DDS_' + name + '_LOG_BEGIN', flush=True)
                    print(log.read()[-5000:], flush=True)
                    print('M23BV_DDS_' + name + '_LOG_END', flush=True)
                    log.close()
        if rectifier_rc != 0 or recorder_rc != 0:
            raise RuntimeError('BLOCKED_UNCLEAN_RECTIFIER_OR_RECORDER')
        original_fingerprints = fingerprints(bag_dir)
        reader, topics = new_reader(bag_dir, __import__('rosbag2_py'))
        events = scan(reader, ros_decode())
        if callable(getattr(reader, 'close', None)):
            reader.close()
        del reader
        keep, report = select(events, min_counts={'image': 80, 'info': 80, 'imu': 700})
        if original_fingerprints != fingerprints(bag_dir):
            raise RuntimeError('BLOCKED_BAG_MUTATED_BY_READONLY_AUDIT')
        print('M23BV_DDS_SYNTHETIC_WINDOW=' + json.dumps(report, sort_keys=True), flush=True)
        print('M23BV_DDS_BAG_CDR_AND_WINDOW=PASS', flush=True)

        # End-to-end test of the actual M23BV derived-bag writer on a real
        # closed, synthetic Jazzy sqlite3 source. Test-only thresholds are
        # explicitly supplied; the physical production defaults stay intact.
        # Independent readers compare ORIGINAL serialized CDR payloads and
        # rosbag receipt timestamps against every selected source record.
        import rosbag2_py
        selected = []
        raw_reader, _ = new_reader(bag_dir, rosbag2_py)
        try:
            for take in keep:
                if not raw_reader.has_next():
                    raise RuntimeError('BLOCKED_SOURCE_READER_SHORT')
                row = raw_reader.read_next()
                if take:
                    selected.append(row)
            if raw_reader.has_next():
                raise RuntimeError('BLOCKED_SOURCE_READER_EXTRA')
        finally:
            if callable(getattr(raw_reader, 'close', None)):
                raw_reader.close()
            del raw_reader
        source_hashes_before = fingerprints(bag_dir)
        derived_dir = scratch / 'derived'
        derivation = derive(
            bag_dir, derived_dir, baglib=rosbag2_py, decode=ros_decode(),
            min_counts={'image': 80, 'info': 80, 'imu': 700})
        if source_hashes_before != fingerprints(bag_dir):
            raise RuntimeError('BLOCKED_SOURCE_CHANGED_AFTER_DERIVATION')
        if not fingerprints(derived_dir):
            raise RuntimeError('BLOCKED_DERIVED_BAG_NOT_FINALIZED')
        output_reader, _ = new_reader(derived_dir, rosbag2_py)
        try:
            for expected in selected:
                if not output_reader.has_next():
                    raise RuntimeError('BLOCKED_DERIVED_RECORDS_SHORT')
                actual = output_reader.read_next()
                if tuple(actual) != tuple(expected):
                    raise RuntimeError('BLOCKED_DERIVED_CDR_RECEIPT_OR_ORDER_MISMATCH')
            if output_reader.has_next():
                raise RuntimeError('BLOCKED_DERIVED_RECORDS_EXTRA')
        finally:
            if callable(getattr(output_reader, 'close', None)):
                output_reader.close()
            del output_reader
        if derivation['source_messages_copied_byte_for_byte'] != len(selected):
            raise RuntimeError('BLOCKED_DERIVED_COPY_COUNT')
        print('M23BV_DERIVED_SYNTHETIC_MESSAGES=' + str(len(selected)), flush=True)
        print('M23BV_DERIVED_SOURCE_SHA256_UNCHANGED=PASS', flush=True)
        print('M23BV_DERIVED_CDR_AND_RECEIPT_EXACT_MATCH=PASS', flush=True)
        print('M23BV_DERIVED_SQLITE_FINALIZATION=PASS', flush=True)
        print('M23BV_SYNTHETIC_CLOSED_BAG_DERIVATION=PASS', flush=True)
        print('M23BV_SYNTHETIC_DDS_CAMERA_TRANSPORT=PASS', flush=True)
        print('M23BV_ACTUAL_HARDWARE_OR_PRODUCTION_CAPTURE=NOT_TESTED', flush=True)
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print('M23BV_SYNTHETIC_DDS_CAMERA_TRANSPORT=BLOCKED:' +
              type(exc).__name__ + ':' + str(exc), flush=True)
        sys.exit(2)
