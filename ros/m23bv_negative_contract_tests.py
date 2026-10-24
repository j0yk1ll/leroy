#!/usr/bin/env python3
"""M23BV no-device negative cases and additive-source parity regressions.

Source-only/mocked tests: no rclpy init, no ROS graph, no video/serial, Podman,
MCU, motors, navigation, or external recording access.
"""
import ast
import contextlib
import hashlib
import io
import os
from pathlib import Path
from types import SimpleNamespace as N
import unittest
from unittest import mock

import m23bv_qos_guard as guard
import m23bv_window as window
from m23bv_m23i_independent_preflight import ReplayInputAudit, MAX_PENDING
from m23bv_offline_tests import FakeProcess, graph, load_capture_with_stubs

HERE = Path(__file__).resolve().parent
START = 1_800_000_000_000_000_000
STEP = 33_333_333


def header(stamp, frame="camera_optical_frame"):
    return N(stamp=N(sec=stamp // 1_000_000_000,
                     nanosec=stamp % 1_000_000_000),
             frame_id=frame)


def info(stamp, *, baseline=0.0):
    return N(header=header(stamp), width=4, height=3,
             p=[200., 0., 2., baseline, 0., 200., 1.5, 0.,
                0., 0., 1., 0.],
             roi=N(do_rectify=False))


def image(stamp):
    return N(header=header(stamp), width=4, height=3,
             encoding="mono8", step=4, data=bytes(12))


def scan_long(*, count=3906, pick=None, missing_info=None):
    auditor = ReplayInputAudit(mode="monocular", provenance="synthetic")
    selected = 0
    for i in range(count):
        ts = START + i * STEP
        if i != missing_info:
            auditor.ingest("info", ts, info(ts))
        if pick is None or i in pick:
            auditor.ingest("image", ts + 1, image(ts))
            selected += 1
    return auditor, auditor.finish(), selected


def git_blob_sha(path):
    payload = path.read_bytes()
    return hashlib.sha1(
        b"blob " + str(len(payload)).encode() + b"\0" + payload).hexdigest()


def camera_ast(path):
    script = ast.parse(path.read_text())
    func = next(n for n in script.body
                if isinstance(n, ast.FunctionDef) and n.name == "_camera_only")
    call = next(n.value for n in func.body
                if isinstance(n, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id == "camera"
                        for t in n.targets))
    return {kw.arg: kw.value for kw in call.keywords}


class M23BVLongBagNegativeTests(unittest.TestCase):
    def test_3906_info_all_images_cache_bounded(self):
        auditor, report, count = scan_long()
        self.assertEqual(count, 3906)
        self.assertEqual(report["verdict"], "REPLAY_INPUT_CONTRACT_PASS")
        self.assertEqual(report["matched_image_info_pairs"], 3906)
        self.assertEqual(report["camera_info_cache_evictions"], 3906 - MAX_PENDING)
        self.assertLessEqual(len(auditor.info_by_stamp), MAX_PENDING)
        self.assertFalse(report["physical_motion_authorized"])

    def test_3906_info_sparse_images_still_valid(self):
        chosen = set(range(0, 3906, 5))
        auditor, report, selected = scan_long(pick=chosen)
        self.assertEqual(report["counts"]["info"], 3906)
        self.assertEqual(report["matched_image_info_pairs"], selected)
        self.assertEqual(report["verdict"], "REPLAY_INPUT_CONTRACT_PASS")
        self.assertLessEqual(len(auditor.info_by_stamp), MAX_PENDING)

    def test_evictions_do_not_hide_missing_camera_info(self):
        _auditor, report, _count = scan_long(missing_info=2300)
        self.assertEqual(report["verdict"], "BLOCKED_REPLAY_INPUT")
        self.assertIn("BLOCKED_UNMATCHED_CAMERA_INFO", report["faults"])
        self.assertGreater(report["camera_info_cache_evictions"], 0)

    def test_long_info_stream_cannot_mask_image_gap(self):
        chosen = set(range(3906)) - set(range(1500, 1530))
        _auditor, report, _count = scan_long(pick=chosen)
        self.assertEqual(report["verdict"], "BLOCKED_REPLAY_INPUT")
        self.assertIn("BLOCKED_IMAGE_STAMP_GAP", report["faults"])
        self.assertGreater(report["max_header_gap_ms"]["image"], 250)

    def test_bad_calibration_projection_still_blocks(self):
        audit = ReplayInputAudit(mode="monocular", provenance="synthetic")
        for i in range(5):
            ts = START + i * STEP
            audit.ingest("info", ts, info(ts, baseline=0.125 if i == 2 else 0.0))
            audit.ingest("image", ts + 1, image(ts))
        report = audit.finish()
        self.assertIn("BLOCKED_UNRECTIFIED_OR_INVALID_CAMERA_INFO", report["faults"])

    def test_independent_auditor_does_not_authorize_tracking(self):
        _auditor, report, _count = scan_long(count=15)
        for field in ("device_access", "provenance_authenticated",
                      "xrslam_executed", "xrslam_tracking_validated",
                      "extrinsics_validated", "camera_imu_physical_offset_validated",
                      "physical_motion_authorized"):
            self.assertIs(report[field], False, field)

    def test_production_window_thresholds_remain_unchanged(self):
        self.assertEqual(window.MIN_COUNTS, {
            "image": 400, "info": 400, "imu": 5600})
        self.assertEqual(window.GAP_IMAGE_NS, 250_000_000)
        self.assertEqual(window.GAP_IMU_NS, 300_000_000)


class M23BVSourceAndLifecycleTests(unittest.TestCase):
    def test_m20_and_m23h_original_sources_pinned(self):
        expected = {
            "m20_camera_only.launch.py": "a72000f97b72ec3b773879fb1066405b505f764a",
            "m20_sensor_only_run.sh": "88c300b174f6e53bd536467f2e445d22e5ed1f8c",
            "m20_combined_observer.py": "5cf2f88cb2f79be80821888401b57a6393ee5e3b",
            "m23h_bounded_capture.py": "eb40aa6e636e7ba602554136a7f8272e9b407025",
        }
        for filename, expected_sha in expected.items():
            with self.subTest(filename=filename):
                self.assertEqual(git_blob_sha(HERE / filename), expected_sha)

    def test_actual_camera_node_parameters_match_m20(self):
        old = camera_ast(HERE / "m20_camera_only.launch.py")
        new = camera_ast(HERE / "m23bv_camera_only.launch.py")
        self.assertEqual(set(old), set(new))
        for key, old_value in old.items():
            self.assertEqual(ast.dump(old_value), ast.dump(new[key]), key)

    def test_opencv_path_is_rectifier_private_only(self):
        launch = (HERE / "m23bv_camera_only.launch.py").read_text()
        self.assertIn('executable="rectify_node"', launch)
        self.assertIn('"LD_LIBRARY_PATH": ALT_OPENCV + ":"', launch)
        self.assertIn("additional_env={", launch)
        self.assertNotIn('os.environ["LD_LIBRARY_PATH"] =', launch)
        self.assertNotIn("ComposableNodeContainer(", launch)

    def test_serial_observer_starts_only_after_qos_guard(self):
        runner = (HERE / "m23bv_sensor_only_run.sh").read_text()
        self.assertLess(runner.index("ros2 launch /opt/m23bv/m23bv_camera_only.launch.py"),
                        runner.index("m23bv_qos_guard.py"))
        self.assertLess(runner.index("m23bv_qos_guard.py"),
                        runner.index("python3 -u /opt/m20/m20_combined_observer.py &"))
        self.assertIn('for name in imu camera', runner)
        self.assertIn('kill -TERM "$pid"', runner)
        self.assertNotIn("set_wheels", runner)

    def test_best_effort_rectifier_publisher_blocks(self):
        q = graph()
        q["topics"][guard.IMAGE]["publishers"][0]["qos"]["reliability"] = "BEST_EFFORT"
        with self.assertRaisesRegex(ValueError, "BLOCKED_RELIABLE_ENDPOINT"):
            guard.validate_camera_graph(q)

    def test_missing_camera_info_publisher_blocks(self):
        q = graph()
        q["topics"][guard.INFO]["publishers"] = []
        with self.assertRaisesRegex(ValueError, "BLOCKED_CAMERAINFO_PUBLISHER"):
            guard.validate_camera_graph(q)

    def test_nonvolatile_raw_endpoint_blocks(self):
        q = graph()
        q["topics"][guard.RAW]["publishers"][0]["qos"]["durability"] = "TRANSIENT_LOCAL"
        with self.assertRaisesRegex(ValueError, "BLOCKED_RELIABLE_ENDPOINT"):
            guard.validate_camera_graph(q)

    def _mock_capture_with_failed_stop(self, fail):
        module, events = load_capture_with_stubs()
        argv = []
        def spawn(cmd, **kwargs):
            argv.append(cmd)
            return FakeProcess("recorder" if len(argv) == 1 else "sensor")
        def stop(proc, signum, grace, process_group=False):
            events.append((proc.name, signum, process_group))
            return (None, False) if proc.name == fail else (0, True)
        from tempfile import TemporaryDirectory
        from pathlib import Path
        with TemporaryDirectory() as tmp:
            with mock.patch.dict(os.environ, module.EXPECTED_ENV, clear=True):
                with mock.patch.object(module, "_signal_and_wait", side_effect=stop):
                    with contextlib.redirect_stdout(io.StringIO()) as capture:
                        rc = module.capture(Path(tmp), "mock", spawn=spawn, pause=lambda _: None)
        return rc, events, capture.getvalue()

    def test_launch_tree_synthetic_source_never_executes_physical_launcher(self):
        source = (HERE / "m23bv_synthetic_launch_tree_probe.py").read_text()
        self.assertIn('ast.unparse(call)', source)
        self.assertIn('rectifier = rectifier_expression()', source)
        self.assertIn('["ros2", "launch", str(launch_path)]', source)
        self.assertIn('BLOCKED_PHYSICAL_DEVICE_PRESENT', source)
        self.assertIn('M23BV_SYNTHETIC_LAUNCH_LIFECYCLE=PASS', source)
        self.assertNotIn('ros2 launch /opt/m23bv/m23bv_camera_only.launch.py', source)
        self.assertNotIn('set_wheels(', source)

    def test_launch_tree_wrapper_is_pinned_and_device_free(self):
        runner = (HERE / "m23bv_synthetic_launch_tree_preflight.sh").read_text()
        self.assertIn('EXPECTED_ID=57d886bf0b903e08ff1dc9f766d3daea28f44a20bcf9fd34733e7d3eab3029ec', runner)
        for setting in ('--network none', '--ipc private', '--pid private',
                        '--cap-drop all', '--read-only', 'ROS_DOMAIN_ID=179'):
            with self.subTest(setting=setting):
                self.assertIn(setting, runner)
        self.assertNotIn('--device ', runner)
        self.assertNotIn('--privileged', runner)
        self.assertNotIn('--volume', runner)

    def test_sensor_shutdown_timeout_blocks(self):
        import signal
        rc, events, output = self._mock_capture_with_failed_stop("sensor")
        self.assertEqual(rc, 2)
        self.assertEqual(events, [("sensor", signal.SIGTERM, False),
                                  ("recorder", signal.SIGINT, True)])
        self.assertIn("BLOCKED_CAPTURE_OR_FINALIZATION", output)

    def test_recorder_finalization_timeout_blocks(self):
        import signal
        rc, events, output = self._mock_capture_with_failed_stop("recorder")
        self.assertEqual(rc, 2)
        self.assertEqual(events, [("sensor", signal.SIGTERM, False),
                                  ("recorder", signal.SIGINT, True)])
        self.assertIn("BLOCKED_CAPTURE_OR_FINALIZATION", output)


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromModule(
            __import__(__name__)))
    print("M23BV_NEGATIVE_SOURCE_REGRESSIONS=" +
          ("PASS" if result.wasSuccessful() else "FAIL"), flush=True)
    print("M23BV_NEGATIVE_TEST_DEVICE_ACCESS=NONE", flush=True)
    raise SystemExit(0 if result.wasSuccessful() else 2)
