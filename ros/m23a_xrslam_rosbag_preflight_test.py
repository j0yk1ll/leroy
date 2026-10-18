"""M23A pure-Python offline XRSLAM rosbag input-contract regressions.

No rclpy, rosbag2_py, ROS nodes, camera, STM32 or serial I/O needed.
These tests inspect synthetic in-memory message-shaped objects only.
"""
import copy
import math
from pathlib import Path
from types import SimpleNamespace as S
import unittest

from m23a_xrslam_rosbag_preflight import (
    IMAGE_MAX_GAP_NS, ReplayInputAudit,
)

START = 1_800_000_000_000_000_000


def header(stamp, frame):
    return S(stamp=S(sec=stamp // 1_000_000_000,
                     nanosec=stamp % 1_000_000_000), frame_id=frame)


def info(stamp, *, width=4, height=3, frame="camera_optical_frame",
         fx=200.0, baseline=0.0, rectify=False):
    p = [200.0, 0.0, 2.0, baseline,
         0.0, 200.0, 1.5, 0.0,
         0.0, 0.0, 1.0, 0.0]
    p[0] = fx
    return S(header=header(stamp, frame), width=width, height=height,
             p=p, roi=S(do_rectify=rectify))


def image(stamp, *, width=4, height=3, frame="camera_optical_frame",
          encoding="mono8", step=4, length=12):
    return S(header=header(stamp, frame), width=width, height=height,
             encoding=encoding, step=step, data=bytes(length))


def imu(stamp, *, frame="imu_link", angular=0.0):
    return S(header=header(stamp, frame),
             angular_velocity=S(x=angular, y=0.0, z=0.01),
             linear_acceleration=S(x=0.0, y=0.0, z=9.81))


def records(*, mode="monocular_imu"):
    base = START + 1_000_000_000
    events = []
    for i in range(3):
        stamp = base + i * 100_000_000
        events.append(("info", stamp, info(stamp)))
        events.append(("image", stamp + 1, image(stamp)))
    if mode == "monocular_imu":
        for i in range(7):
            stamp = base - 50_000_000 + 50_000_000 * i
            events.append(("imu", stamp, imu(stamp)))
    return sorted(events, key=lambda item: item[1])


def evaluate(events, *, mode="monocular_imu", provenance="synthetic"):
    audit = ReplayInputAudit(mode=mode, provenance=provenance)
    for kind, bag_stamp, msg in events:
        audit.ingest(kind, bag_stamp, msg)
    return audit.finish()


class M23AReplayInputTests(unittest.TestCase):
    def assert_blocked(self, report, fault):
        self.assertEqual(report["verdict"], "BLOCKED_REPLAY_INPUT")
        self.assertIn(fault, report["faults"])
        self.assertIs(report["physical_motion_authorized"], False)
        self.assertIs(report["device_access"], False)

    def test_valid_synthetic_vio_contract_is_not_calibration(self):
        result = evaluate(records())
        self.assertEqual(result["verdict"], "REPLAY_INPUT_CONTRACT_PASS")
        self.assertEqual(result["counts"], {"image": 3, "info": 3, "imu": 7})
        self.assertEqual(result["matched_image_info_pairs"], 3)
        for key in ("provenance_authenticated", "xrslam_executed",
                    "xrslam_tracking_validated", "extrinsics_validated",
                    "camera_imu_physical_offset_validated",
                    "physical_motion_authorized", "device_access"):
            self.assertIs(result[key], False)

    def test_mono_mode_requires_no_imu(self):
        result = evaluate(records(mode="monocular"), mode="monocular")
        self.assertEqual(result["verdict"], "REPLAY_INPUT_CONTRACT_PASS")
        self.assertEqual(result["counts"]["imu"], 0)

    def test_image_before_camera_info_is_matched(self):
        seq = records()
        # The IMU event has the same bag timestamp as CameraInfo,
        # so Image and CameraInfo are never adjacent in this fixture.
        # Select their matching acquisition stamps instead of looking
        # for adjacent records (which silently skipped the test).
        original_info_index = next(
            i for i, event in enumerate(seq) if event[0] == "info")
        info_event = seq.pop(original_info_index)
        image_index = next(
            i for i, event in enumerate(seq)
            if event[0] == "image"
            and event[2].header.stamp == info_event[2].header.stamp)
        after_image_ns = seq[image_index][1] + 1
        seq.insert(image_index + 1, ("info", after_image_ns, info_event[2]))
        # Prove the intended late CameraInfo order really happened.
        self.assertEqual(seq[image_index][0], "image")
        self.assertEqual(seq[image_index + 1][0], "info")
        self.assertLess(seq[image_index][1], seq[image_index + 1][1])
        self.assertEqual(evaluate(seq)["verdict"], "REPLAY_INPUT_CONTRACT_PASS")

    def test_missing_imu_blocks_vio(self):
        events = [p for p in records() if p[0] != "imu"]
        self.assert_blocked(evaluate(events), "BLOCKED_INSUFFICIENT_IMU_MESSAGES")

    def test_no_info_blocks_all_images(self):
        events = [p for p in records() if p[0] != "info"]
        self.assert_blocked(evaluate(events), "BLOCKED_UNMATCHED_CAMERA_INFO")

    def test_wrong_image_frame_blocks(self):
        events = records()
        k = next(i for i, x in enumerate(events) if x[0] == "image")
        stamp = events[k][2].header.stamp
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        events[k] = (events[k][0], events[k][1], image(ns, frame="wrong"))
        self.assert_blocked(evaluate(events), "BLOCKED_CAMERA_FRAME")

    def test_camera_info_baseline_blocks(self):
        events = records()
        k = next(i for i, x in enumerate(events) if x[0] == "info")
        events[k][2].p[3] = 1.0
        self.assert_blocked(evaluate(events),
                            "BLOCKED_UNRECTIFIED_OR_INVALID_CAMERA_INFO")

    def test_invalid_focal_length_blocks(self):
        events = records()
        k = next(i for i, x in enumerate(events) if x[0] == "info")
        events[k][2].p[0] = math.nan
        self.assert_blocked(evaluate(events),
                            "BLOCKED_UNRECTIFIED_OR_INVALID_CAMERA_INFO")

    def test_invalid_rgb_image_length_blocks(self):
        events = records()
        k = next(i for i, x in enumerate(events) if x[0] == "image")
        events[k][2].encoding = "rgb8"
        self.assert_blocked(evaluate(events), "BLOCKED_INVALID_IMAGE_LAYOUT")

    def test_unsupported_image_encoding_blocks(self):
        events = records()
        k = next(i for i, x in enumerate(events) if x[0] == "image")
        events[k][2].encoding = "16UC1"
        self.assert_blocked(evaluate(events), "BLOCKED_INVALID_IMAGE_LAYOUT")

    def test_wrong_camera_info_geometry_blocks(self):
        events = records()
        k = next(i for i, x in enumerate(events) if x[0] == "info")
        events[k][2].width = 5
        self.assert_blocked(evaluate(events),
                            "BLOCKED_IMAGE_INFO_DIMENSION_MISMATCH")

    def test_backward_header_timestamp_blocks(self):
        events = records()
        images = [i for i, ev in enumerate(events) if ev[0] == "image"]
        events[images[1]][2].header.stamp = copy.deepcopy(
            events[images[0]][2].header.stamp)
        self.assert_blocked(evaluate(events), "BLOCKED_REPEATED_OR_BACKWARD_IMAGE")

    def test_bad_imu_nan_blocks(self):
        events = records()
        k = next(i for i, x in enumerate(events) if x[0] == "imu")
        events[k][2].angular_velocity.x = math.nan
        self.assert_blocked(evaluate(events), "BLOCKED_INVALID_IMU_VECTOR")

    def test_wrong_imu_frame_blocks(self):
        events = records()
        k = next(i for i, x in enumerate(events) if x[0] == "imu")
        events[k][2].header.frame_id = "not_imu_link"
        self.assert_blocked(evaluate(events), "BLOCKED_IMU_FRAME")

    def test_imu_gap_above_adapter_limit_blocks(self):
        events = records()
        last_imu = max((i for i, x in enumerate(events) if x[0] == "imu"))
        events[last_imu][2].header.stamp.sec += 2
        self.assert_blocked(evaluate(events), "BLOCKED_IMU_STAMP_GAP")

    def test_camera_gap_above_250ms_blocks(self):
        events = records(mode="monocular")
        last_image = max((i for i, x in enumerate(events) if x[0] == "image"))
        events[last_image][2].header.stamp.nanosec += IMAGE_MAX_GAP_NS
        # The 250ms adjustment stays under 1s and makes the header gap 350ms.
        self.assert_blocked(evaluate(events, mode="monocular"),
                            "BLOCKED_IMAGE_STAMP_GAP")

    def test_missing_imu_time_coverage_blocks(self):
        events = records()
        for i, (kind, when, msg) in enumerate(events):
            if kind == "imu":
                msg.header.stamp.sec -= 1
        self.assert_blocked(evaluate(events), "BLOCKED_NO_IMU_STAMP_COVERAGE")

    def test_unverified_origin_is_never_authenticated(self):
        result = evaluate(records(), provenance="user_declared_recorded")
        self.assertEqual(result["verdict"], "REPLAY_INPUT_CONTRACT_PASS")
        self.assertIs(result["provenance_authenticated"], False)

    def test_invalid_bag_timestamp_blocks(self):
        events = records(mode="monocular")
        kind, _at, msg = events[0]
        events[0] = (kind, 0, msg)
        self.assert_blocked(evaluate(events, mode="monocular"),
                            "BLOCKED_INVALID_BAG_TIME")

    def test_no_input_blocks(self):
        self.assert_blocked(evaluate([], mode="monocular"),
                            "BLOCKED_INSUFFICIENT_CAMERA_MESSAGES")

    def test_source_has_no_hardware_or_ros_node_import(self):
        source = (Path(__file__).resolve().parent /
                  "m23a_xrslam_rosbag_preflight.py").read_text()
        self.assertNotIn("import serial", source)
        self.assertNotIn("rclpy.init(", source)
        self.assertNotIn("serial.Serial(", source)
        self.assertNotIn("create_publisher(", source)
        self.assertNotIn("set_wheels ", source)


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            M23AReplayInputTests))
    if result.wasSuccessful():
        print("M23A_XRSLAM_REPLAY_INPUT_CONTRACT=PASS", flush=True)
        print("M23A_VERDICT=OFFLINE_BAG_PREFLIGHT_ONLY", flush=True)
        print("M23A_RECORDED_SENSOR_PROVENANCE_VERIFIED=FALSE", flush=True)
        print("M23A_XRSLAM_TRACKING_VALIDATED=FALSE", flush=True)
        print("M23A_MOTION_ENABLED=FALSE", flush=True)
        print("M23A_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    print("M23A_XRSLAM_REPLAY_INPUT_CONTRACT=FAIL", flush=True)
    raise SystemExit(2)
