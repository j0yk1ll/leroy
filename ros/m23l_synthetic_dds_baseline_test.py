"""M23L pure Python synthetic DDS probe regression fixtures. No ROS nodes."""
from pathlib import Path
from unittest import mock
import unittest

from m23l_synthetic_dds_baseline import (
    EXPECTED_PAYLOAD_BYTES, PUBLISHED_FRAMES,
    MIN_SUBSCRIBER_FRACTION, parse_publisher_log, interpret,
    REQUIRED_ENV, FORBIDDEN_DEVICE_PATHS,
)


def publisher(span_ns=30_000_000_000):
    return {"image": PUBLISHED_FRAMES, "info": PUBLISHED_FRAMES,
            "span_ns": span_ns, "max_callback_ns": 12_000_000,
            "image_payload_bytes": EXPECTED_PAYLOAD_BYTES}


def observer(raw=900, info=900, gap=0):
    return {
        "counts": {"raw": raw, "rect": 0, "info": info},
        "streams": {"raw": {"header_intervals": {
            "gaps_over_250ms": gap, "max_gap_ms": 33.333}}},
    }


def lines(count="900,900", span="30000000000",
          callback="12000000", payload="921600"):
    return "\n".join((
        "M23L_PUBLISHER_COUNTS=" + count,
        "M23L_PUBLISHER_START_SPAN_NS=" + span,
        "M23L_PUBLISHER_MAX_CALLBACK_NS=" + callback,
        "M23L_PUBLISHER_PAYLOAD_BYTES=" + payload,
    )) + "\n"


class M23LSyntheticDDSContracts(unittest.TestCase):
    def test_full_rgb_payload_exact(self):
        self.assertEqual(EXPECTED_PAYLOAD_BYTES, 640 * 480 * 3)
        self.assertEqual(PUBLISHED_FRAMES, 900)

    def test_nominal_publisher_log_parses(self):
        self.assertEqual(parse_publisher_log(lines()), publisher())

    def test_missing_publisher_marker_blocks(self):
        with self.assertRaisesRegex(ValueError, "incomplete"):
            parse_publisher_log(lines().replace(
                "M23L_PUBLISHER_PAYLOAD_BYTES=921600\n", ""))

    def test_duplicate_publisher_count_blocks(self):
        with self.assertRaisesRegex(ValueError, "duplicate"):
            parse_publisher_log(lines() + "M23L_PUBLISHER_COUNTS=900,900\n")

    def test_fabricated_frame_count_blocks(self):
        with self.assertRaisesRegex(ValueError, "exact"):
            parse_publisher_log(lines(count="899,900"))

    def test_wrong_pixel_payload_size_blocks(self):
        with self.assertRaisesRegex(ValueError, "duration/payload"):
            parse_publisher_log(lines(payload="1024"))

    def test_noninteger_publisher_count_blocks(self):
        with self.assertRaisesRegex(ValueError, "invalid"):
            parse_publisher_log(lines(count="X,900"))

    def test_reported_publisher_fatal_blocks(self):
        with self.assertRaisesRegex(ValueError, "fatal"):
            parse_publisher_log(lines() + "M23L_PUBLISHER_FATAL=bad\n")

    def test_healthy_synthetic_transport_not_camera_verification(self):
        result = interpret(publisher(), observer(raw=895, info=899))
        self.assertEqual(result["verdict"],
                         "SYNTHETIC_FULL_IMAGE_DDS_PROFILE_ONLY")
        self.assertIs(result["no_physical_device_access"], True)
        self.assertIs(result["real_camera_rate_verified"], False)
        self.assertIs(result["camera_imu_calibrated"], False)
        self.assertIs(result["xrslam_tracking_validated"], False)
        self.assertIs(result["physical_motion_authorized"], False)

    def test_image_loss_with_full_info_classified_as_delivery(self):
        result = interpret(publisher(), observer(raw=180, info=900, gap=42))
        self.assertEqual(result["verdict"],
                         "BLOCKED_FULL_IMAGE_DDS_SUBSCRIBER_DELIVERY")
        self.assertEqual(result["raw_delivery_fraction"], 0.2)
        self.assertEqual(result["subscriber_raw_header_gaps_over_250ms"], 42)

    def test_slow_publisher_is_not_called_dds_loss(self):
        result = interpret(publisher(span_ns=60_000_000_000),
                           observer(raw=180, info=900))
        self.assertEqual(result["verdict"],
                         "BLOCKED_SYNTHETIC_PUBLISHER_SCHEDULING")

    def test_camera_info_delivery_loss_has_separate_fault(self):
        result = interpret(publisher(), observer(raw=900, info=500))
        self.assertEqual(result["verdict"],
                         "BLOCKED_CAMERA_INFO_SUBSCRIBER_DELIVERY")

    def test_delivery_fraction_threshold_exact(self):
        n = int(PUBLISHED_FRAMES * MIN_SUBSCRIBER_FRACTION)
        self.assertEqual(interpret(publisher(), observer(raw=n))["verdict"],
                         "SYNTHETIC_FULL_IMAGE_DDS_PROFILE_ONLY")
        self.assertEqual(interpret(publisher(), observer(raw=n - 1))["verdict"],
                         "BLOCKED_FULL_IMAGE_DDS_SUBSCRIBER_DELIVERY")

    def test_forbidden_device_paths_explicit(self):
        self.assertIn("/dev/ttyACM0", FORBIDDEN_DEVICE_PATHS)
        self.assertIn("/dev/video2", FORBIDDEN_DEVICE_PATHS)
        self.assertEqual(REQUIRED_ENV["ROS_AUTOMATIC_DISCOVERY_RANGE"],
                         "LOCALHOST")

    def test_cpp_publisher_never_accesses_real_device(self):
        root = Path(__file__).resolve().parent
        host = root / "m23k_cpp/src/m23l_synthetic_image_publisher.cpp"
        image = Path(
            "/opt/m23k_cpp/src/m23l_synthetic_image_publisher.cpp")
        source = (image if image.exists() else host).read_text()
        for bad in ("serial.Serial(", "/dev/video", "/dev/ttyACM",
                    "create_subscription<", "set_wheels ",
                    "camera.set_capture", "cv::VideoCapture"):
            self.assertNotIn(bad, source)
        self.assertIn('"/camera/image_raw"', source)
        self.assertIn('"/camera/camera_info"', source)
        self.assertIn("kFrames = 900", source)
        self.assertIn("image_.data.resize(kWidth * kHeight * 3", source)
        # Jazzy rclcpp::Time has an implicit msg conversion, not to_msg().
        self.assertNotIn("now().to_msg()", source)
        self.assertIn("const auto stamp = now();", source)
        self.assertIn("image_.header.stamp = stamp;", source)
        self.assertIn("info_.header.stamp = stamp;", source)

    def test_python_orchestrator_source_has_no_live_sensor_paths(self):
        root = Path(__file__).resolve().parent
        source = (root / "m23l_synthetic_dds_baseline.py").read_text()
        self.assertNotIn("rclpy.init(", source)
        self.assertNotIn("import serial", source)
        self.assertNotIn("ros2 launch", source)
        self.assertNotIn("create_publisher(", source)


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            M23LSyntheticDDSContracts))
    if result.wasSuccessful():
        print("M23L_OFFLINE_SYNTHETIC_DDS_CONTRACT=PASS", flush=True)
        print("M23L_NO_DEVICE_SOURCE_CONTRACT=PASS", flush=True)
        print("M23L_CAMERA_IMU_CALIBRATED=FALSE", flush=True)
        print("M23L_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    raise SystemExit(2)
