"""M23M nine no-device DDS QoS profile regression tests."""
from pathlib import Path
import unittest

from m23l_synthetic_dds_baseline import (
    M23M_QOS_PROFILES, read_counter_qos_profile, interpret,
    EXPECTED_PAYLOAD_BYTES, PUBLISHED_FRAMES,
)
from m23l_synthetic_dds_baseline_test import publisher, observer


class M23MReceiverQoSContracts(unittest.TestCase):
    def test_exact_receiver_profiles(self):
        self.assertEqual(M23M_QOS_PROFILES,
                         ("best_effort_5", "best_effort_100", "reliable_100"))

    def test_each_reported_profile_parses(self):
        for mode in M23M_QOS_PROFILES:
            with self.subTest(mode=mode):
                self.assertEqual(read_counter_qos_profile(
                    "M23M_COUNTER_QOS_PROFILE=" + mode + "\n"), mode)

    def test_missing_duplicate_and_unknown_profile_blocked(self):
        for sample in ("", "M23M_COUNTER_QOS_PROFILE=bad\n",
                       "M23M_COUNTER_QOS_PROFILE=reliable_100\n"
                       "M23M_COUNTER_QOS_PROFILE=reliable_100\n"):
            with self.assertRaises(ValueError):
                read_counter_qos_profile(sample)

    def test_good_dds_is_not_real_camera_approval(self):
        result = interpret(publisher(), observer(raw=900, info=900))
        self.assertEqual(result["verdict"],
                         "SYNTHETIC_FULL_IMAGE_DDS_PROFILE_ONLY")
        self.assertFalse(result["real_camera_rate_verified"])
        self.assertFalse(result["physical_motion_authorized"])

    def test_299_of_900_images_remains_blocked(self):
        result = interpret(publisher(), observer(raw=299, info=887, gap=26))
        self.assertEqual(result["verdict"],
                         "BLOCKED_FULL_IMAGE_DDS_SUBSCRIBER_DELIVERY")
        self.assertEqual(result["raw_delivery_fraction"], round(299/900, 4))
        self.assertEqual(EXPECTED_PAYLOAD_BYTES, 921600)
        self.assertEqual(PUBLISHED_FRAMES, 900)

    def test_cpp_counter_preserves_default_and_rejects_unknown(self):
        root = Path(__file__).resolve().parent
        image = Path("/opt/m23k_cpp/src/m23k_camera_header_counter.cpp")
        local = root / "m23k_cpp/src/m23k_camera_header_counter.cpp"
        s = (image if image.exists() else local).read_text()
        for token in ('std::getenv("M23M_SUB_QOS")',
                      'profile == "best_effort_5"',
                      'profile == "best_effort_100"',
                      'profile == "reliable_100"',
                      'M23M_COUNTER_QOS_PROFILE='):
            self.assertIn(token, s)
        for token in ("create_publisher(", "/dev/ttyACM", "/dev/video"):
            self.assertNotIn(token, s)

    def test_parent_checks_reported_qos_and_drains_two_seconds(self):
        root = Path(__file__).resolve().parent
        image = Path("/opt/m23l/m23l_synthetic_dds_baseline.py")
        local = root / "m23l_synthetic_dds_baseline.py"
        s = (image if image.exists() else local).read_text()
        for token in ('requested_qos not in M23M_QOS_PROFILES',
                      'env["M23M_SUB_QOS"] = requested_qos',
                      'if actual_qos != requested_qos:',
                      'time.sleep(2.0)', 'M23M_QOS_JSON=',
                      'M23M_QOS_VERDICT='):
            self.assertIn(token, s)
        self.assertNotIn("import serial", s)

    def test_publisher_keeps_900_rgb_frames_at_30hz(self):
        root = Path(__file__).resolve().parent
        image = Path("/opt/m23k_cpp/src/m23l_synthetic_image_publisher.cpp")
        local = root / "m23k_cpp/src/m23l_synthetic_image_publisher.cpp"
        s = (image if image.exists() else local).read_text()
        for token in ("kFrames = 900", "kPeriodNs = 33333333LL",
                      "image_.data.resize(kWidth * kHeight * 3",
                      "rclcpp::KeepLast(100)"):
            self.assertIn(token, s)

    def test_disconnected_image_only_uses_local_base(self):
        root = Path(__file__).resolve().parent
        image = Path("/opt/m23m/Dockerfile.m23m-qos")
        local = root / "Dockerfile.m23m-qos"
        s = (image if image.exists() else local).read_text()
        self.assertIn("FROM localhost/leroy-sensors:m23l-synthetic-image-dds", s)
        self.assertIn("M23M_CPP_INSTALL_ROOT=/opt/m23m-cpp-install", s)
        for forbidden in ("--device", "curl ", "apt-get ", "wget "):
            self.assertNotIn(forbidden, s)


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            M23MReceiverQoSContracts))
    if result.wasSuccessful():
        print("M23M_OFFLINE_RECEIVER_QOS_MATRIX=PASS", flush=True)
        print("M23M_CAMERA_IMU_CALIBRATED=FALSE", flush=True)
        print("M23M_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    raise SystemExit(2)
