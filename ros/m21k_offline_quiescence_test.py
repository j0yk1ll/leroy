"""M21K tests: pure Python, synthetic sensor metadata and fake STOP only."""
from pathlib import Path
import tempfile
import threading
import unittest
from unittest import mock

from m21d_metadata_capture import MetadataCapture
from m21k_offline_quiescence import QuiescenceError, finish_after_quiescence


def filled():
    capture = MetadataCapture("SYNTHETIC", max_events=100)
    for index in range(3):
        stamp = 1_800_000_000_000_000_000 + index * 100_000_000
        receipt = 5_000_000_000 + index * 100_000_000
        capture.image(frame_index=index, stamp_ns=stamp,
                      receipt_mono_ns=receipt,
                      frame_id="camera_optical_frame", width=640, height=480)
        capture.camera_info(stamp_ns=stamp + 1000,
                            receipt_mono_ns=receipt + 1000,
                            frame_id="camera_optical_frame", width=640, height=480)
        capture.imu_source(mcu_seq=index, mcu_t_us=index * 100_000,
                           stamp_ns=stamp + 2000, receipt_mono_ns=receipt + 2000,
                           gyro_xyz_rad_s=[0.1, 0.2, 0.3])
    return capture


class QuiescenceTests(unittest.TestCase):
    def test_stop_then_join_then_save(self):
        capture = filled()
        release = threading.Event()
        running = threading.Event()
        order = []
        def producer():
            running.set()
            release.wait(timeout=3)
            order.append("producer_exited")
        worker = threading.Thread(target=producer, name="simulated_camera")
        worker.start()
        self.assertTrue(running.wait(1))
        real_finish = capture.finish
        def traced_finish(path):
            order.append("save")
            return real_finish(path)
        try:
            with tempfile.TemporaryDirectory() as folder:
                with mock.patch.object(capture, "finish", side_effect=traced_finish):
                    report = finish_after_quiescence(
                        capture, Path(folder) / "synthetic.jsonl",
                        lambda: (order.append("stop"), True)[1],
                        lambda: (order.append("request_quiesce"), release.set()),
                        [worker], join_timeout_s=1)
                self.assertEqual(
                    order, ["stop", "request_quiesce", "producer_exited", "save"])
                self.assertTrue(report["producers_quiescent"])
                self.assertTrue(report["metadata_saved"])
                self.assertEqual(report["saved_event_count"], 9)
                self.assertEqual(report["verdict"],
                                 "SYNTHETIC_STOP_QUIESCE_SAVE_CONTRACT")
                self.assertFalse(report["physical_stop_acknowledgement_validated"])
        finally:
            release.set()
            worker.join(timeout=1)

    def test_missing_stop_ack_keeps_forensic_file_but_rejects(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "forensic.jsonl"
            report = finish_after_quiescence(
                filled(), path, lambda: False, lambda: None, [])
            self.assertTrue(report["metadata_saved"])
            self.assertTrue(path.exists())
            self.assertFalse(report["stop_acknowledged"])
            self.assertEqual(report["verdict"], "STOP_QUIESCENCE_OR_SAVE_UNVERIFIED")

    def test_stop_exception_still_requests_quiescence(self):
        calls = []
        def bad_stop():
            calls.append("stop")
            raise OSError("synthetic STOP transport failure")
        with tempfile.TemporaryDirectory() as folder:
            result = finish_after_quiescence(
                filled(), Path(folder) / "forensic.jsonl",
                bad_stop, lambda: calls.append("quiesce"), [])
            self.assertEqual(calls, ["stop", "quiesce"])
            self.assertIn("synthetic STOP", result["stop_error"])
            self.assertTrue(result["metadata_saved"])
            self.assertEqual(result["verdict"], "STOP_QUIESCENCE_OR_SAVE_UNVERIFIED")

    def test_join_timeout_blocks_persistence(self):
        capture = filled()
        release = threading.Event()
        started = threading.Event()
        def stalled():
            started.set()
            release.wait(timeout=3)
        worker = threading.Thread(target=stalled, name="stalled_camera", daemon=True)
        worker.start()
        self.assertTrue(started.wait(1))
        try:
            with tempfile.TemporaryDirectory() as folder:
                path = Path(folder) / "must_not_exist.jsonl"
                result = finish_after_quiescence(
                    capture, path, lambda: True, lambda: None,
                    [worker], join_timeout_s=0.01)
                self.assertFalse(result["producers_quiescent"])
                self.assertEqual(result["active_producers"], ["stalled_camera"])
                self.assertFalse(result["metadata_saved"])
                self.assertFalse(path.exists())
                self.assertFalse(capture.closed)
                self.assertEqual(result["verdict"],
                                 "STOP_QUIESCENCE_OR_SAVE_UNVERIFIED")
        finally:
            release.set()
            worker.join(timeout=1)

    def test_quiescence_callback_error_rejects_even_if_saved(self):
        def failure():
            raise ValueError("synthetic executor shutdown failure")
        with tempfile.TemporaryDirectory() as folder:
            result = finish_after_quiescence(
                filled(), Path(folder) / "forensic.jsonl",
                lambda: True, failure, [])
            self.assertTrue(result["metadata_saved"])
            self.assertIn("shutdown failure", result["quiescence_error"])
            self.assertEqual(result["verdict"], "STOP_QUIESCENCE_OR_SAVE_UNVERIFIED")

    def test_not_started_worker_prevents_persistence(self):
        never_started = threading.Thread(target=lambda: None,
                                         name="never_started")
        capture = filled()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "must_not_exist.jsonl"
            result = finish_after_quiescence(
                capture, path, lambda: True, lambda: None, [never_started])
            self.assertEqual(result["unstarted_producers"], ["never_started"])
            self.assertFalse(result["producers_quiescent"])
            self.assertFalse(path.exists())

    def test_existing_destination_does_not_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "existing.jsonl"
            path.write_text("KEEP")
            report = finish_after_quiescence(
                filled(), path, lambda: True, lambda: None, [])
            self.assertEqual(path.read_text(), "KEEP")
            self.assertFalse(report["metadata_saved"])
            self.assertIn("FileExistsError", report["save_error"])
            self.assertEqual(report["verdict"], "STOP_QUIESCENCE_OR_SAVE_UNVERIFIED")

    def test_reject_bad_preflight_without_calling_stop(self):
        calls = []
        stop = lambda: (calls.append("stop"), True)[1]
        capture = filled()
        thread = threading.Thread(target=lambda: None)
        invalid = [
            {"producer_threads": [thread, thread]},
            {"producer_threads": [object()]},
            {"producer_threads": [threading.current_thread()]},
            {"producer_threads": [], "join_timeout_s": -1},
            {"producer_threads": [], "join_timeout_s": True},
            {"producer_threads": [], "join_timeout_s": float("nan")},
        ]
        with tempfile.TemporaryDirectory() as folder:
            for args in invalid:
                with self.subTest(args=args), self.assertRaises(QuiescenceError):
                    finish_after_quiescence(
                        capture, Path(folder) / "untouched.jsonl",
                        stop, lambda: None, **args)
        self.assertEqual(calls, [])

    def test_already_closed_capture_rejected(self):
        capture = filled()
        with tempfile.TemporaryDirectory() as folder:
            capture.finish(Path(folder) / "once.jsonl")
            with self.assertRaises(QuiescenceError):
                finish_after_quiescence(
                    capture, Path(folder) / "twice.jsonl",
                    lambda: True, lambda: None, [])


if __name__ == "__main__":
    unittest.main()
