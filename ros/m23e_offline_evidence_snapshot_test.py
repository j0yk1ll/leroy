"""M23E pure Python, hardware-free evidence manifest regression tests.

Do not load ROS or visit any physical device. These tests establish only
hashing, timestamp-summary and negative provenance contracts.
"""
import hashlib
from pathlib import Path
from types import SimpleNamespace as S
import tempfile
import unittest

from m23e_offline_evidence_snapshot import (
    CHUNK_BYTES, StreamEvidence, _file_digest,
)

N = 1_800_000_000_000_000_000


def msg(t, *, proj=None, data=b"\x00\x01"):
    m = S(header=S(stamp=S(sec=t // 1_000_000_000,
                           nanosec=t % 1_000_000_000)))
    m.encoding, m.width, m.height, m.step = "mono8", 2, 1, 2
    m.data = data
    m.p = proj if proj is not None else [
        100., 0., 1., 0., 0., 100., 0.5, 0., 0., 0., 1., 0.
    ]
    return m


class M23EEvidenceContractTests(unittest.TestCase):
    def test_sha256_matches_known_bytes(self):
        with tempfile.TemporaryDirectory(prefix="m23e-bytes-") as td:
            p = Path(td) / "record.db3"
            p.write_bytes(b"completely-fabricated-rosbag-bytes")
            result = _file_digest(p)
            self.assertEqual(result["bytes"], p.stat().st_size)
            self.assertEqual(result["sha256"], hashlib.sha256(
                p.read_bytes()).hexdigest())

    def test_digest_changes_after_mutation(self):
        with tempfile.TemporaryDirectory(prefix="m23e-change-") as td:
            p = Path(td) / "record.db3"
            p.write_bytes(b"abc")
            old = _file_digest(p)
            p.write_bytes(b"abd")
            self.assertNotEqual(old["sha256"], _file_digest(p)["sha256"])

    def test_zero_length_file_is_hashable_not_a_real_recording(self):
        with tempfile.TemporaryDirectory(prefix="m23e-empty-") as td:
            p = Path(td) / "empty.db3"
            p.touch()
            self.assertEqual(_file_digest(p)["bytes"], 0)
            self.assertEqual(len(_file_digest(p)["sha256"]), 64)

    def test_symlink_is_refused_by_fd_open(self):
        with tempfile.TemporaryDirectory(prefix="m23e-symlink-") as td:
            p = Path(td) / "record.db3"
            p.write_bytes(b"abc")
            alias = Path(td) / "alias.db3"
            alias.symlink_to(p)
            with self.assertRaises(OSError):
                _file_digest(alias)

    def test_directories_cannot_be_hashed_as_bags(self):
        with tempfile.TemporaryDirectory(prefix="m23e-dir-") as td:
            with self.assertRaises(ValueError):
                _file_digest(Path(td))

    def test_hash_chunk_size_is_bounded(self):
        self.assertEqual(CHUNK_BYTES, 1024 * 1024)

    def test_per_stream_count_and_delta(self):
        e = StreamEvidence()
        e.consume("image", N + 5_000_000, msg(N))
        e.consume("image", N + 125_000_000 + 6_000_000,
                  msg(N + 125_000_000))
        r = e.report()
        self.assertEqual(r["counts"]["image"], 2)
        self.assertEqual(r["receipt_minus_header_ms"]["image"],
                         {"min": 5.0, "max": 6.0})
        self.assertEqual(r["inter_header_interval_ms"]["image"],
                         {"min": 125.0, "max": 125.0})

    def test_header_minus_receipt_is_not_phase_calibration(self):
        e = StreamEvidence()
        e.consume("imu", N - 50_000_000, msg(N))
        r = e.report()
        self.assertEqual(r["receipt_minus_header_ms"]["imu"]["min"],
                         -50.0)
        self.assertIsNone(r["inter_header_interval_ms"]["imu"]["min"])

    def test_three_streams_counted_independently(self):
        e = StreamEvidence()
        for kind in ("image", "info", "imu"):
            e.consume(kind, N, msg(N))
        self.assertEqual(e.report()["counts"],
                         {"image": 1, "info": 1, "imu": 1})

    def test_backwards_timestamp_is_blocked(self):
        e = StreamEvidence()
        e.consume("imu", N, msg(N))
        e.consume("imu", N + 1, msg(N - 10))
        self.assertIn("BACKWARD_IMU_HEADER", e.report()["faults"])

    def test_zero_bag_receipt_is_blocked(self):
        e = StreamEvidence()
        e.consume("image", 0, msg(N))
        self.assertIn("INVALID_IMAGE_TIMESTAMP", e.report()["faults"])

    def test_missing_header_is_blocked(self):
        e = StreamEvidence()
        e.consume("imu", N, S())
        self.assertIn("INVALID_IMU_TIMESTAMP", e.report()["faults"])

    def test_projection_changes_are_diagnostic(self):
        e = StreamEvidence()
        p = msg(N).p
        q = list(p)
        q[0] += 1.0
        e.consume("info", N, msg(N, proj=p))
        e.consume("info", N + 125_000_000, msg(N + 125_000_000, proj=p))
        e.consume("info", N + 250_000_000, msg(N + 250_000_000, proj=q))
        self.assertEqual(e.report()["camera_info_projection_changes"], 1)

    def test_nonfinite_projection_is_blocked(self):
        e = StreamEvidence()
        p = msg(N).p
        p[0] = float("nan")
        e.consume("info", N, msg(N, proj=p))
        self.assertIn("INVALID_CAMERA_PROJECTION", e.report()["faults"])

    def test_image_geometry_variants_are_descriptive(self):
        e = StreamEvidence()
        a = msg(N)
        b = msg(N + 1)
        b.step = 3
        e.consume("image", N, a)
        e.consume("image", N + 1, b)
        self.assertEqual(
            [entry["step"] for entry in e.report()["image_geometry_variants"]],
            [2, 3])

    def test_unknown_stream_is_refused(self):
        e = StreamEvidence()
        e.consume("wheels", N, msg(N))
        self.assertIn("UNKNOWN_TOPIC", e.report()["faults"])
        self.assertEqual(sum(e.counts.values()), 0)

    def test_source_never_starts_robot_or_serial(self):
        src = (Path(__file__).resolve().parent /
               "m23e_offline_evidence_snapshot.py").read_text()
        for forbidden in (
                "rclpy.init(", "serial.Serial(", "subprocess.Popen(",
                "create_publisher(", "create_subscription(",
                "robot.launch.py", "/dev/ttyACM"):
            self.assertNotIn(forbidden, src)

    def test_false_calibration_and_motion_claims_are_immutable(self):
        src = (Path(__file__).resolve().parent /
               "m23e_offline_evidence_snapshot.py").read_text()
        for field in (
                "origin_authenticated", "recording_provenance_verified",
                "camera_imu_physical_offset_calibrated",
                "camera_imu_extrinsics_validated", "imu_noise_calibrated",
                "xrslam_tracking_validated", "metric_pose_authorized",
                "physical_motion_authorized", "device_access"):
            self.assertIn('"' + field + '": False', src)

    def test_digest_only_accepts_regular_input_fd(self):
        src = (Path(__file__).resolve().parent /
               "m23e_offline_evidence_snapshot.py").read_text()
        self.assertIn("os.O_RDONLY", src)
        self.assertIn("O_NOFOLLOW", src)
        self.assertIn("S_ISREG", src)
        self.assertIn("bag_fingerprints(bag)", src)


if __name__ == "__main__":
    out = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            M23EEvidenceContractTests))
    if out.wasSuccessful():
        print("M23E_OFFLINE_MANIFEST_CONTRACT=PASS", flush=True)
        print("M23E_VERDICT=STATIC_EVIDENCE_HASHING_ONLY", flush=True)
        print("M23E_REAL_RECORDED_BAG_VERIFIED=FALSE", flush=True)
        print("M23E_CALIBRATION_VALIDATED=FALSE", flush=True)
        print("M23E_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    print("M23E_OFFLINE_MANIFEST_CONTRACT=FAIL", flush=True)
    raise SystemExit(2)
