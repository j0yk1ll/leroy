"""Stage 4C passive capture candidate: pure no-device validation and guards."""
import ast
import json
from pathlib import Path
import unittest

from live_stage4c_capture_contract import (
    CaptureContract, CaptureError, METADATA_TOPIC, TOPICS, source_metadata,
)
from live_stage4c_passive_recorder import (
    LIVE_CAPTURE_COMMISSIONED, _contract_smoke,
)

HERE = Path(__file__).resolve().parent


def meta(stamp=1_000_000_000, seq=40, ticks=1_000_000, source_mono=1000):
    return {
        "schema": "leroy_stage4c_mcu_v1",
        "publisher_node": "/m18_imu_source",
        "seq": seq, "t_us": ticks, "imu_stamp_ns": stamp,
        "publish_monotonic_ns": source_mono, "cal": 1,
    }


class PureCaptureContract(unittest.TestCase):
    def setUp(self):
        self.c = CaptureContract()
        self.writes = []
        self.now = 10_000_000

    def write(self, topic, payload, stamp_ns):
        self.writes.append((topic, payload, stamp_ns))

    def add(self, topic, stamp, frame, *, extra=None, payload=b"cdr"):
        self.now += 10_000
        self.c.accept(topic, stamp, frame, payload, self.now, self.write,
                      metadata=extra)

    def test_four_exact_whitelisted_topics(self):
        self.assertEqual(
            set(TOPICS), {"/camera/image_rect", "/camera/camera_info",
                          "/imu/data_raw", METADATA_TOPIC})
        with self.assertRaisesRegex(CaptureError, "unexpected_topic"):
            self.add("/cmd_vel", 1, None)
        self.assertEqual(self.writes, [])

    def test_missing_metadata_keeps_calibration_blocked(self):
        self.add("/imu/data_raw", 1_000_000_000, "imu_link")
        report = self.c.report()
        self.assertEqual(report["unmatched_imu_stamps"], 1)
        self.assertEqual(report["readiness"], "BLOCKED_INCOMPLETE_CAPTURE")
        for flag in ("physical_camera_imu_offset_validated",
                     "imu_noise_validated", "spatial_extrinsics_validated",
                     "xrslam_vio_authorized", "metric_odometry_authorized"):
            self.assertFalse(report[flag])

    def test_metadata_can_precede_or_follow_imu_receipt(self):
        self.add(METADATA_TOPIC, 1_000_000_000, None, extra=meta())
        self.add("/imu/data_raw", 1_000_000_000, "imu_link")
        report = self.c.report()
        self.assertEqual(report["matched_imu_metadata_stamps"], 1)
        self.assertEqual(report["unmatched_imu_stamps"], 0)
        self.assertEqual(report["unmatched_metadata_stamps"], 0)
        self.assertFalse(report["mcu_metadata_stamp_match_complete"])
        self.assertFalse(report["physical_source_authenticated"])

    def test_complete_bounded_synthetic_capture_does_not_certify_offset(self):
        for k in range(500):
            stamp = 1_000_000_000 + k * 20_000_000
            self.add("/camera/image_rect", stamp, "camera_optical_frame")
            self.add("/camera/camera_info", stamp, "camera_optical_frame")
            self.add("/imu/data_raw", stamp, "imu_link")
            self.add(METADATA_TOPIC, stamp, None, extra=meta(
                stamp, seq=k, ticks=k * 20_000, source_mono=1000 + k))
        report = self.c.report()
        self.assertEqual(report["readiness"],
                         "INPUT_FORMAT_COMPLETE_UNVERIFIED")
        self.assertTrue(report["mcu_metadata_stamp_match_complete"])
        self.assertFalse(report["physical_source_authenticated"])
        self.assertEqual(report["matched_imu_metadata_stamps"], 500)
        self.assertFalse(report["physical_camera_imu_offset_validated"])
        self.assertFalse(report["motor_commands_sent"])

    def test_source_metadata_strict_schema_and_no_duplicate_keys(self):
        self.assertEqual(source_metadata(json.dumps(meta()))["seq"], 40)
        for source in (
            '{"seq":0,"seq":1}',
            {"schema": "leroy_stage4c_mcu_v1"},
            {**meta(), "cal": True},
            {**meta(), "cal": 0},
            {**meta(), "schema": "not_provenance"},
            {**meta(), "publisher_node": "/wrong_node"},
            {**meta(), "seq": -1},
            {**meta(), "t_us": 4_294_967_296},
            {**meta(), "imu_stamp_ns": True},
            {**meta(), "unexpected": 9},
        ):
            with self.subTest(source=source), self.assertRaises(CaptureError):
                source_metadata(source)

    def test_timestamp_and_frame_faults_fail_before_writer(self):
        self.add("/imu/data_raw", 10_000, "imu_link")
        for topic, stamp, frame in (
            ("/imu/data_raw", 10_000, "imu_link"),
            ("/imu/data_raw", 9_999, "imu_link"),
            ("/camera/image_rect", 10_001, "base_link"),
            (METADATA_TOPIC, 10_001, "imu_link"),
            ("/camera/camera_info", 0, "camera_optical_frame"),
        ):
            with self.subTest(topic=topic, stamp=stamp), self.assertRaises(
                CaptureError
            ):
                self.add(topic, stamp, frame,
                         extra=meta(10_001) if topic == METADATA_TOPIC else None)
        self.assertEqual(len(self.writes), 1)

    def test_metadata_stamp_discrepancy_and_sequence_gap(self):
        with self.assertRaisesRegex(CaptureError, "metadata_stamp_mismatch"):
            self.add(METADATA_TOPIC, 1_000_000_000, None,
                     extra=meta(1_000_000_001))
        self.assertFalse(self.writes)
        self.add(METADATA_TOPIC, 1_000_000_000, None, extra=meta(seq=41))
        with self.assertRaisesRegex(CaptureError, "mcu_sequence_gap"):
            self.add(METADATA_TOPIC, 1_020_000_000, None,
                     extra=meta(1_020_000_000, seq=43))
        self.assertEqual(len(self.writes), 1)

    def test_mcu_sequence_wraparound(self):
        self.add(METADATA_TOPIC, 1_000_000_000, None,
                 extra=meta(seq=4_294_967_295))
        self.add(METADATA_TOPIC, 1_020_000_000, None,
                 extra=meta(1_020_000_000, seq=0))
        self.assertEqual(self.c.report()["counts"][METADATA_TOPIC], 2)

    def test_serialized_byte_count_limit_is_bounded(self):
        c = CaptureContract(max_bytes=6, max_records=2)
        c.accept("/imu/data_raw", 1, "imu_link", b"1234", 2, self.write)
        with self.assertRaisesRegex(CaptureError, "serialized_byte_limit"):
            c.accept("/imu/data_raw", 2, "imu_link", b"345", 3, self.write)
        self.assertEqual(c.report()["total_serialized_bytes"], 4)
        c.accept("/imu/data_raw", 2, "imu_link", b"12", 4, self.write)
        with self.assertRaisesRegex(CaptureError, "record_limit"):
            c.accept("/imu/data_raw", 3, "imu_link", b"1", 5, self.write)
        self.assertEqual(len(self.writes), 2)

    def test_writer_failure_never_commits_accounting(self):
        def fail(*_):
            raise OSError("disk full")
        with self.assertRaisesRegex(OSError, "disk full"):
            self.c.accept("/imu/data_raw", 2, "imu_link", b"cdr", 10, fail)
        self.assertEqual(self.c.counts["/imu/data_raw"], 0)
        self.assertEqual(self.c.bytes_written, 0)

    def test_monotonic_receipts_and_duration_limit(self):
        self.c = CaptureContract(duration_s=1)
        self.c.accept("/imu/data_raw", 1, "imu_link", b"a", 2, self.write)
        with self.assertRaisesRegex(CaptureError, "nonmonotonic_receipt"):
            self.c.accept("/imu/data_raw", 2, "imu_link", b"a", 1, self.write)
        with self.assertRaisesRegex(CaptureError, "duration_limit"):
            self.c.accept("/imu/data_raw", 2, "imu_link", b"a",
                          1_000_000_003, self.write)

    def test_invalid_limits_rejected(self):
        for kwargs in (
            {"duration_s": 0}, {"duration_s": 76},
            {"duration_s": float("nan")}, {"max_records": 0},
            {"max_records": 20001}, {"max_bytes": -1},
            {"max_bytes": 2_000_000_001},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(CaptureError):
                CaptureContract(**kwargs)


class StaticNonActuation(unittest.TestCase):
    def test_live_capture_is_explicitly_commissioned_but_remains_guarded(self):
        self.assertIs(LIVE_CAPTURE_COMMISSIONED, True)
        _contract_smoke()
        script = (HERE / "live_stage4c_passive_recorder.py").read_text()
        self.assertIn("if not LIVE_CAPTURE_COMMISSIONED:", script)
        self.assertIn("STAGE4C=BLOCKED_LIVE_CAPTURE_NOT_COMMISSIONED", script)
        self.assertNotIn("serial.Serial(", script)
        self.assertNotIn("device.write(", script)
        self.assertNotIn("create_publisher(", script)
        self.assertNotIn("subprocess.", script)
        self.assertNotIn("os.system(", script)
        self.assertNotIn("--arm", script)
        self.assertNotIn("ros2 launch", script)

    def test_no_return_in_recorder_finally_block(self):
        # Python 3.14 warns because return in finally could swallow a
        # critical capture, cleanup or disk error.
        tree = ast.parse((HERE / "live_stage4c_passive_recorder.py").read_text())
        recorder = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                        and n.name == "_run_live")
        for node in ast.walk(recorder):
            if isinstance(node, ast.Try):
                self.assertFalse(any(
                    isinstance(descendant, ast.Return)
                    for stmt in node.finalbody for descendant in ast.walk(stmt)
                ), "A return inside finally can suppress capture faults")
        self.assertEqual(len([
            n for n in recorder.body if isinstance(n, ast.Return)
        ]), 1)

    def test_dockerfile_defaults_to_no_action(self):
        docker = (HERE / "Dockerfile.live-stage4c-capture").read_text()
        self.assertIn("FROM localhost/leroy-ros:stage4b-imu", docker)
        self.assertIn('CMD ["/bin/false"]', docker)
        self.assertNotIn("ENTRYPOINT", docker)
        self.assertIn("live_stage4c_passive_recorder.py", docker)
        # ROS 2 Jazzy Python modules are installed under ROS-managed
        # paths: import checks MUST source Jazzy and the Leroy overlay.
        self.assertIn("source /opt/ros/jazzy/setup.bash", docker)
        self.assertIn("source /leroy_ws/install/setup.bash", docker)
        self.assertIn('python3 -c "import rosbag2_py; import rclpy"', docker)
        self.assertLess(
            docker.index("source /opt/ros/jazzy/setup.bash"),
            docker.index('python3 -c "import rosbag2_py; import rclpy"'))

    def test_graph_requires_single_source_provenance_and_no_devices(self):
        s = (HERE / "live_stage4c_passive_recorder.py").read_text()
        for token in (
            "get_publishers_info_by_topic(topic)",
            '"m18_imu_source"',
            "FORBIDDEN_NODES", "FORBIDDEN_TOPICS",
            "STAGE4C_CALIBRATION=BLOCKED_NOT_VALIDATED",
            "rosbag2_py.SequentialWriter",
            "metadata.yaml",
            "if not node.graph_ready():",
            '"/dev/ttyACM0"',
            '"/dev/video2"',
            '"/dev/gpiochip0"',
            '"/capture"',
            "LIVE_CAPTURE_COMMISSIONED = True",
        ):
            self.assertIn(token, s)


if __name__ == "__main__":
    unittest.main()
