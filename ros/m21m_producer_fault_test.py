"""M21M: joined producer faults must invalidate an otherwise valid session.

Pure-Python synthetic tests. No ROS, serial, video, containers or motors.
"""
from pathlib import Path
import tempfile
import threading
import unittest

from m21k_offline_quiescence import QuiescenceError, finish_after_quiescence
from m21k_offline_quiescence_test import filled


class ProducerFaultTests(unittest.TestCase):
    def test_completed_worker_exception_rejects_valid_metadata(self):
        capture = filled()
        faults, calls = [], []
        ended = threading.Event()

        def failed_camera():
            faults.append("camera_executor: synthetic publisher mismatch")
            ended.set()

        worker = threading.Thread(target=failed_camera, name="failed_camera")
        worker.start()
        self.assertTrue(ended.wait(timeout=1))
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "forensic.jsonl"
            result = finish_after_quiescence(
                capture, output,
                lambda: (calls.append("stop"), True)[1],
                lambda: calls.append("quiesce"),
                [worker], producer_faults=faults)
            self.assertEqual(calls, ["stop", "quiesce"])
            self.assertTrue(result["stop_acknowledged"])
            self.assertTrue(result["producers_quiescent"])
            self.assertTrue(result["metadata_saved"])
            self.assertTrue(output.exists())
            self.assertEqual(result["producer_fault_count"], 1)
            self.assertIn("publisher mismatch", result["producer_faults"][0])
            self.assertEqual(result["verdict"],
                             "STOP_QUIESCENCE_OR_SAVE_UNVERIFIED")

    def test_multiple_imu_and_camera_errors_reject(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = finish_after_quiescence(
                filled(), Path(tmp) / "forensic.jsonl",
                lambda: True, lambda: None, [],
                producer_faults=[
                    "camera: synthetic bad image source",
                    "imu: synthetic source terminal fault"])
            self.assertEqual(result["producer_fault_count"], 2)
            self.assertEqual(result["verdict"],
                             "STOP_QUIESCENCE_OR_SAVE_UNVERIFIED")
            self.assertTrue(result["metadata_saved"])

    def test_empty_fault_list_preserves_existing_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = finish_after_quiescence(
                filled(), Path(tmp) / "ok.jsonl",
                lambda: True, lambda: None, [], producer_faults=[])
            self.assertEqual(result["producer_fault_count"], 0)
            self.assertEqual(result["producer_faults"], [])
            self.assertEqual(result["verdict"],
                             "SYNTHETIC_STOP_QUIESCE_SAVE_CONTRACT")

    def test_invalid_fault_collection_rejects_before_stop(self):
        capture = filled()
        calls = []
        for faults in ({"err": "fake"}, "fake", 0, False):
            with self.subTest(faults=faults):
                with self.assertRaises(QuiescenceError):
                    finish_after_quiescence(
                        capture, "must-not-exist.jsonl",
                        lambda: (calls.append("STOP"), True)[1],
                        lambda: None, [], producer_faults=faults)
        self.assertEqual(calls, [])

    def test_late_worker_error_observed_after_join(self):
        capture = filled()
        faults = []
        release = threading.Event()
        started = threading.Event()
        order = []
        def worker_func():
            started.set()
            release.wait(timeout=2)
            faults.append("imu: synthetic delayed failure")
            order.append("exit")
        worker = threading.Thread(target=worker_func, name="imu_worker")
        worker.start()
        self.assertTrue(started.wait(timeout=1))
        try:
            with tempfile.TemporaryDirectory() as tmp:
                output = Path(tmp) / "forensic.jsonl"
                result = finish_after_quiescence(
                    capture, output,
                    lambda: (order.append("stop"), True)[1],
                    lambda: (order.append("shutdown"), release.set()),
                    [worker], join_timeout_s=1.0, producer_faults=faults)
                self.assertEqual(order, ["stop", "shutdown", "exit"])
                self.assertTrue(result["producers_quiescent"])
                self.assertEqual(result["producer_fault_count"], 1)
                self.assertEqual(result["verdict"],
                                 "STOP_QUIESCENCE_OR_SAVE_UNVERIFIED")
        finally:
            release.set()
            worker.join(timeout=1)


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(ProducerFaultTests))
    if result.wasSuccessful():
        print("M21M_PRODUCER_FAULT_SELFTEST=PASS", flush=True)
        print("M21M_VERDICT=SYNTHETIC_PRODUCER_FAULT_REJECTION", flush=True)
        print("M21M_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    print("M21M_PRODUCER_FAULT_SELFTEST=FAIL", flush=True)
    raise SystemExit(2)
