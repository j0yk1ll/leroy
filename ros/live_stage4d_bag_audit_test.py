"""No-device exact-write SQLite CDR audit tests.

Rejects the false assumption that serializing the same ROS message twice
must yield identical byte-for-byte CDR buffers (padding may differ).
"""
import gc
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import weakref

from live_stage4c_capture_contract import CaptureContract, CaptureError
from live_stage4d_bag_audit import ExactWriteBagAudit
from live_stage4d_capture_ingress import CaptureIngress

HERE = Path(__file__).resolve().parent
STAMP = 1_790_000_000_000_000_000


def imu():
    sec, nano = divmod(STAMP, 1_000_000_000)
    return NS(
        header=NS(stamp=NS(sec=sec, nanosec=nano), frame_id="imu_link"),
        angular_velocity=NS(x=0., y=0., z=0.1),
        linear_acceleration=NS(x=0., y=0., z=9.8))


class ExactCDRPayloadAudit(unittest.TestCase):
    def setUp(self):
        self.stored = []
        self.audit = ExactWriteBagAudit(
            lambda topic, data, stamp: self.stored.append(
                (topic, bytes(data), stamp)))
        self.topic = "/imu/data_raw"

    def record(self, data=b"CDR_A", stamp=STAMP):
        self.audit.write(self.topic, data, stamp)

    def test_exact_written_cdr_bytes_verify_after_close(self):
        self.record()
        self.audit.close()
        self.audit.verify_readback(*self.stored[0])
        self.assertEqual(self.audit.verify_complete(), 1)

    def test_second_serialization_differs_but_written_bytes_match(self):
        calls = []
        def serialize(_msg):
            payload = b"CDR_PADDING_V" + bytes([len(calls)])
            calls.append(payload)
            return payload
        policy = CaptureContract()
        ingress = CaptureIngress(
            contract=policy, serialize=serialize, write=self.audit.write)
        ingress.accept(self.topic, imu(), receipt_ns=1000)
        self.assertEqual(len(calls), 1, "Write path should serialize once")
        second_serialization = serialize(imu())
        self.assertNotEqual(second_serialization, self.stored[0][1])
        ingress.close()
        self.audit.close()
        self.audit.verify_readback(*self.stored[0])
        self.assertEqual(self.audit.verify_complete(), 1)
        self.assertEqual(policy.counts[self.topic], 1)

    def test_corrupt_readback_fails_with_topic_diagnostic(self):
        self.record()
        self.audit.close()
        with self.assertRaisesRegex(CaptureError, "bag_audit_cdr_payload_mismatch:/imu/data_raw"):
            self.audit.verify_readback(self.topic, b"bad_payload", STAMP)
        with self.assertRaisesRegex(CaptureError, "bag_audit_missing_readback:/imu/data_raw"):
            self.audit.verify_complete()

    def test_missing_readback_fails(self):
        self.record()
        self.audit.close()
        with self.assertRaisesRegex(CaptureError, "bag_audit_missing_readback"):
            self.audit.verify_complete()

    def test_duplicate_replay_fails(self):
        self.record()
        self.audit.close()
        self.audit.verify_readback(*self.stored[0])
        with self.assertRaisesRegex(CaptureError, "bag_audit_duplicate_readback"):
            self.audit.verify_readback(*self.stored[0])

    def test_duplicate_write_and_unexpected_replay_fail(self):
        self.record()
        with self.assertRaisesRegex(CaptureError, "bag_audit_duplicate_write"):
            self.record()
        self.assertEqual(len(self.stored), 1)
        self.audit.close()
        with self.assertRaisesRegex(CaptureError, "bag_audit_unexpected_readback"):
            self.audit.verify_readback(self.topic, b"CDR", STAMP + 1)

    def test_verify_requires_closed_writer(self):
        self.record()
        with self.assertRaisesRegex(CaptureError, "bag_audit_verify_before_writer_close"):
            self.audit.verify_readback(*self.stored[0])
        with self.assertRaisesRegex(CaptureError, "bag_audit_not_closed"):
            self.audit.verify_complete()

    def test_disk_error_does_not_commit_expected_digest(self):
        n = [0]
        def failed(*_):
            n[0] += 1
            raise OSError("disk_full")
        audit = ExactWriteBagAudit(failed)
        with self.assertRaisesRegex(OSError, "disk_full"):
            audit.write(self.topic, b"CDR", STAMP)
        self.assertEqual(len(audit.expected), 0)
        with self.assertRaisesRegex(CaptureError, "bag_audit_closed"):
            audit.write(self.topic, b"CDR", STAMP)
        self.assertEqual(n[0], 1)

    def test_close_releases_bound_writer_reference(self):
        class FakeWriter:
            def write(self, _topic, _payload, _stamp):
                pass
        writer = FakeWriter()
        ref = weakref.ref(writer)
        audit = ExactWriteBagAudit(writer.write)
        del writer
        gc.collect()
        self.assertIsNotNone(ref())
        audit.close()
        gc.collect()
        self.assertIsNone(ref())
        with self.assertRaisesRegex(CaptureError, "bag_audit_closed"):
            audit.write(self.topic, b"CDR", STAMP)

    def test_replay_source_uses_exact_write_audit_and_no_second_serializer(self):
        script = (HERE / "live_stage4d_synthetic_load.py").read_text()
        self.assertIn("audit = ExactWriteBagAudit(writer.write)", script)
        self.assertIn("write=audit.write", script)
        self.assertIn("audit.verify_readback(topic, raw, stamp)", script)
        self.assertIn("audit.verify_complete()", script)
        self.assertIn("audit.close()", script)
        self.assertEqual(script.count("serialize_message(msg)"), 1)
        self.assertNotIn("replay_hashes", script)


if __name__ == "__main__":
    unittest.main()
