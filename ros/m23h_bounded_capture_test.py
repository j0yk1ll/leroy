"""M23H deterministic capture-orchestrator tests: fake subprocesses only.

No ROS nodes, camera devices, serial, robot launch or live bag recorder
are invoked. Offline temporary SQLite databases model closed rosbag data.
"""
from pathlib import Path
from types import SimpleNamespace as S
from unittest import mock
import os
import signal
import sqlite3
import tempfile
import unittest

import m23h_bounded_capture as m
from m23g_live_recording_preflight import TOPICS


class FakeProc:
    next_pid = 42000
    def __init__(self, *, code=0, timeout=False):
        FakeProc.next_pid += 1
        self.pid = FakeProc.next_pid
        self.code = code
        self.timeout = timeout
        self.returncode = None
        self.signals = []
    def poll(self):
        return self.returncode
    def send_signal(self, sig):
        self.signals.append(sig)
    def wait(self, timeout=None):
        if self.timeout:
            raise m.subprocess.TimeoutExpired("fake", timeout)
        self.returncode = self.code
        return self.code


def make_fake_bag(root, *, topics=TOPICS):
    path = root / "bag"
    path.mkdir()
    (path / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n"
        "  storage_identifier: sqlite3\n"
        "  relative_file_paths: [bag_0.db3]\n")
    conn = sqlite3.connect(path / "bag_0.db3")
    try:
        conn.execute("CREATE TABLE topics (id INTEGER, name TEXT)")
        conn.execute("CREATE TABLE messages (id INTEGER, topic_id INTEGER)")
        for i, t in enumerate(topics, 1):
            conn.execute("INSERT INTO topics VALUES (?,?)", (i, t))
            conn.execute("INSERT INTO messages VALUES (?,?)", (i, i))
        conn.commit()
    finally:
        conn.close()
    return path


class M23HCaptureTests(unittest.TestCase):
    def test_environment_is_fail_closed(self):
        with self.assertRaises(ValueError):
            m.check_environment({})
        m.check_environment(dict(m.EXPECTED_ENV))

    def test_no_authorization_without_explicit_operator_approval(self):
        env = dict(m.EXPECTED_ENV)
        env.pop("M23H_OPERATOR_CAPTURE_APPROVED")
        with self.assertRaises(ValueError):
            m.check_environment(env)

    def test_expected_ros_domain_matches_existing_m20(self):
        self.assertEqual(m.EXPECTED_ENV["ROS_DOMAIN_ID"], "170")

    def test_exact_sensor_topic_allowlist(self):
        self.assertEqual(set(m.MIN_COUNTS), set(TOPICS))
        self.assertEqual(len(m.MIN_COUNTS), 3)

    def test_no_motor_topics_or_nodes_started(self):
        source = (Path(__file__).resolve().parent /
                  "m23h_bounded_capture.py").read_text()
        self.assertNotIn("robot.launch.py", source)
        self.assertNotIn("LeroyBridgeNode(", source)
        self.assertNotIn("set_wheels ", source)
        self.assertNotIn("create_publisher(", source)
        self.assertNotIn("ros2 topic pub", source)

    def test_counts_threshold_excludes_insufficient_capture(self):
        good = dict(m.MIN_COUNTS)
        self.assertTrue(m.assess_counts(good))
        bad = dict(good)
        bad["/imu/data_raw"] -= 1
        self.assertFalse(m.assess_counts(bad))

    def test_topic_not_in_allowlist_rejected_by_assessment(self):
        extra = dict(m.MIN_COUNTS)
        extra["/cmd_vel"] = 1
        self.assertFalse(m.assess_counts(extra))

    def test_fake_sqlite_counts_read_only(self):
        with tempfile.TemporaryDirectory() as td:
            bag = make_fake_bag(Path(td))
            self.assertEqual(m.closed_sqlite_counts(bag),
                             {t: 1 for t in TOPICS})

    def test_missing_metadata_is_not_a_completed_bag(self):
        with tempfile.TemporaryDirectory() as td:
            bag = make_fake_bag(Path(td))
            (bag / "metadata.yaml").unlink()
            with self.assertRaises(ValueError):
                m.closed_sqlite_counts(bag)

    def test_metadata_symlink_is_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            bag = make_fake_bag(Path(td))
            metadata = bag / "metadata.yaml"
            real = Path(td) / "original.yaml"
            metadata.rename(real)
            metadata.symlink_to(real)
            with self.assertRaises(ValueError):
                m.closed_sqlite_counts(bag)

    def test_unexpected_topic_is_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            bag = make_fake_bag(Path(td), topics=(*TOPICS, "/cmd_vel"))
            with self.assertRaisesRegex(ValueError, "unexpected recorded topic"):
                m.closed_sqlite_counts(bag)

    def test_missing_db_file_is_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            bag = make_fake_bag(Path(td))
            (bag / "bag_0.db3").unlink()
            with self.assertRaises(ValueError):
                m.closed_sqlite_counts(bag)

    def test_path_traversal_metadata_is_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            bag = make_fake_bag(Path(td))
            (bag / "metadata.yaml").write_text(
                "rosbag2_bagfile_information:\n"
                "  storage_identifier: sqlite3\n"
                "  relative_file_paths: [../outside.db3]\n")
            with self.assertRaises(ValueError):
                m.closed_sqlite_counts(bag)

    def test_recorder_termination_uses_process_group_sigint(self):
        p = FakeProc()
        with mock.patch.object(m.os, "killpg") as killpg:
            rc, completed = m._signal_and_wait(
                p, signal.SIGINT, 10, process_group=True)
        killpg.assert_called_once_with(p.pid, signal.SIGINT)
        self.assertEqual(rc, 0)
        self.assertTrue(completed)

    def test_sensor_termination_uses_parent_only_sigterm(self):
        p = FakeProc()
        with mock.patch.object(m.os, "killpg") as killpg:
            rc, completed = m._signal_and_wait(
                p, signal.SIGTERM, 10)
        killpg.assert_not_called()
        self.assertEqual(p.signals, [signal.SIGTERM])
        self.assertTrue(completed)
        self.assertEqual(rc, 0)

    def test_unresponsive_recorder_never_reports_finalized(self):
        p = FakeProc(timeout=True)
        with mock.patch.object(m.os, "killpg"):
            rc, done = m._signal_and_wait(
                p, signal.SIGINT, 1, process_group=True)
        self.assertIsNone(rc)
        self.assertFalse(done)

    def _capture_fake(self, td, *, sensor_code=0, recorder_code=0,
                      counts=None):
        root = Path(td)
        recorder = FakeProc(code=recorder_code)
        sensor = FakeProc(code=sensor_code)
        started = []
        def spawn(argv, **kwargs):
            started.append(argv)
            return recorder if len(started) == 1 else sensor
        fake_env = dict(m.EXPECTED_ENV)
        fake_space = {"disk_capacity_sufficient": True,
                      "output_directory": str(root / "bag")}
        with mock.patch.dict(m.os.environ, fake_env, clear=True), \
             mock.patch.object(m, "check_devices"), \
             mock.patch.object(m, "preflight", return_value=fake_space), \
             mock.patch.object(m, "closed_sqlite_counts",
                               return_value=counts or dict(m.MIN_COUNTS)), \
             mock.patch.object(m.os, "killpg"):
            rc = m.capture(root, "test", spawn=spawn,
                           wait_start=lambda _: None)
        return rc, started

    def test_fake_clean_capture_lifecycle(self):
        with tempfile.TemporaryDirectory() as td:
            rc, started = self._capture_fake(td)
            self.assertEqual(rc, 0)
            self.assertEqual(started[0][:3], ["ros2", "bag", "record"])
            self.assertEqual(started[1],
                             ["/usr/local/bin/leroy-m20-sensor-run"])
            self.assertTrue((Path(td) / "m23h_capture_test.log").is_file())

    def test_sensor_failure_blocks_capture(self):
        with tempfile.TemporaryDirectory() as td:
            rc, _ = self._capture_fake(td, sensor_code=2)
        self.assertEqual(rc, 2)

    def test_recorder_failure_blocks_capture(self):
        with tempfile.TemporaryDirectory() as td:
            rc, _ = self._capture_fake(td, recorder_code=2)
        self.assertEqual(rc, 2)

    def test_existing_log_file_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "m23h_capture_test.log"
            path.write_text("keep-existing-log")
            with self.assertRaises(FileExistsError):
                self._capture_fake(td)
            self.assertEqual(path.read_text(), "keep-existing-log")


if __name__ == "__main__":
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(M23HCaptureTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    if result.wasSuccessful():
        print("M23H_OFFLINE_CAPTURE_LIFECYCLE_CONTRACT=PASS", flush=True)
        print("M23H_REAL_SENSOR_CAPTURE_VERIFIED=FALSE", flush=True)
        print("M23H_CAMERA_IMU_CALIBRATION_VALIDATED=FALSE", flush=True)
        print("M23H_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    raise SystemExit(2)
