"""M23N: camera-only QoS observer contracts. No live sensors opened."""
import importlib
from pathlib import Path
import unittest

from m23k_camera_path_probe import (
    camera_qos_request, verified_receiver_qos_log,
    M23N_CAMERA_QOS_PROFILES, M23N_COUNTER_PATH, COUNTER_PATH,
    PROBE_VARIANTS, CameraStats,
)


class M23NCameraQoSContracts(unittest.TestCase):
    def test_profile_allowlist_has_only_two_conditions(self):
        self.assertEqual(M23N_CAMERA_QOS_PROFILES,
                         ("best_effort_100", "reliable_100"))

    def test_existing_m23k_modes_keep_their_old_counter(self):
        self.assertNotEqual(M23N_COUNTER_PATH, COUNTER_PATH)
        self.assertTrue(COUNTER_PATH.endswith("m23k_camera_header_counter"))
        self.assertTrue(M23N_COUNTER_PATH.startswith("/opt/m23m-cpp-install/"))
        self.assertIsNone(camera_qos_request({}, "full_mjpeg"))

    def test_qos_enablement_requires_selected_raw_mjpeg_only(self):
        for mode in M23N_CAMERA_QOS_PROFILES:
            with self.subTest(mode=mode):
                env = {"M23N_CAMERA_QOS_ONLY": "1", "M23N_SUB_QOS": mode}
                self.assertEqual(camera_qos_request(env, "raw_only_mjpeg"),
                                 mode)
                for other in ("full_mjpeg", "raw_only_yuyv"):
                    with self.assertRaises(ValueError):
                        camera_qos_request(env, other)

    def test_unknown_or_missing_live_receiver_profile_refused(self):
        for mode in ("best_effort_5", "", "none", "BEST_EFFORT_100"):
            with self.subTest(mode=mode):
                env = {"M23N_CAMERA_QOS_ONLY": "1", "M23N_SUB_QOS": mode}
                with self.assertRaises(ValueError):
                    camera_qos_request(env, "raw_only_mjpeg")
        with self.assertRaises(ValueError):
            camera_qos_request({"M23N_CAMERA_QOS_ONLY": "1"},
                               "raw_only_mjpeg")

    def test_compiled_qos_marker_must_match_requested(self):
        for profile in M23N_CAMERA_QOS_PROFILES:
            data = "M23M_COUNTER_QOS_PROFILE=" + profile + "\n"
            self.assertEqual(verified_receiver_qos_log(data, profile),
                             profile)

    def test_compiled_qos_marker_missing_duplicate_mismatch_blocks(self):
        for data in ("",
                     "M23M_COUNTER_QOS_PROFILE=best_effort_5\n",
                     "M23M_COUNTER_QOS_PROFILE=reliable_100\n",
                     "M23M_COUNTER_QOS_PROFILE=best_effort_100\n"
                     "M23M_COUNTER_QOS_PROFILE=best_effort_100\n"):
            with self.subTest(data=data):
                with self.assertRaises(ValueError):
                    verified_receiver_qos_log(data, "best_effort_100")

    def test_camera_only_raw_stream_remains_required(self):
        r = CameraStats()
        self.assertIn("raw_only_mjpeg", PROBE_VARIANTS)
        self.assertFalse(PROBE_VARIANTS["raw_only_mjpeg"][1])
        self.assertTrue(PROBE_VARIANTS["full_mjpeg"][1])
        self.assertIn("MISSING_CAMERA_STREAM",
                      r.report(required_streams=("raw", "info"))["faults"])

    def test_python_probe_checks_mode_and_two_second_drain(self):
        source = Path(importlib.import_module(
            "m23k_camera_path_probe").__file__).read_text()
        for required in (
            "M23N_CAMERA_QOS_ONLY", "M23N_SUB_QOS",
            'counter_env["M23M_SUB_QOS"] = requested_qos',
            'time.sleep(2.0)',
            'verified_receiver_qos_log(counter_text, requested_qos)',
            'M23N_CAMERA_QOS_JSON=',
            'M23N_CAMERA_QOS_VERDICT=',
            'stats_report["camera_publisher_rate_verified"] = False',
        ):
            self.assertIn(required, source)
        for disallowed in ("import serial", "serial.Serial(",
                           "set_wheels ", "create_publisher("):
            self.assertNotIn(disallowed, source)

    def test_operator_host_no_mcu_and_exact_verified_camera(self):
        root = Path(__file__).resolve().parent
        image = Path("/opt/m23n/m23n_operator_camera_qos_probe.sh")
        host = root / "m23n_operator_camera_qos_probe.sh"
        s = (image if image.exists() else host).read_text()
        self.assertIn('AUTHORIZE_M23N', s)
        self.assertIn('usb-Wed_Camera_Wed_Camera_20240420112206-video-index0', s)
        self.assertIn('best_effort_100|reliable_100', s)
        self.assertIn('--device "$CAMERA_DEV":/dev/video2', s)
        self.assertIn('--network none', s)
        self.assertIn('fuser "$CAMERA_DEV"', s)
        self.assertNotIn('--device "$MCU_DEV"', s)
        self.assertNotIn('/dev/ttyACM0', s)
        self.assertNotIn('m23h_operator_host_capture.sh', s)

    def test_offline_image_reuses_compiled_qos_counter_no_rebuild(self):
        root = Path(__file__).resolve().parent
        image = Path("/opt/m23n/Dockerfile.m23n-camera-qos")
        host = root / "Dockerfile.m23n-camera-qos"
        s = (image if image.exists() else host).read_text()
        self.assertIn("FROM localhost/leroy-sensors:m23m-synthetic-qos", s)
        self.assertIn("M23N_JAZZY_CAMERA_QOS_IMPORT=PASS", s)
        self.assertIn('/opt/m23m-cpp-install/lib/leroy_m23k_camera_counter/',
                      s)
        for forbidden in ("curl ", "wget ", "apt-get ", "--device "):
            self.assertNotIn(forbidden, s)


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            M23NCameraQoSContracts))
    if result.wasSuccessful():
        print("M23N_CAMERA_QOS_SOURCE_CONTRACT=PASS", flush=True)
        print("M23N_CAMERA_ONLY_LIVE_RUN=NOT_PERFORMED", flush=True)
        print("M23N_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    raise SystemExit(2)
