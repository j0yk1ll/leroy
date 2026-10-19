"""M23I forensic contracts for long, genuine-sized ROS camera bags.

Everything is fake/in-memory: these checks neither open bag files nor
start ROS nodes, serial, cameras, XRSLAM, motion, or the production bridge.
"""
from pathlib import Path
from importlib import import_module
from unittest import mock
import unittest

from m23a_xrslam_rosbag_preflight import MAX_PENDING, ReplayInputAudit
from m23a_xrslam_rosbag_preflight_test import START, info, image
from m23f_existing_recording_inventory import _audit_one

BASE = START + 9_000_000_000
INTERVAL = 33_333_333


def summarize(count=3906, image_indices=None, *, bad_stamp=None,
              provenance="synthetic"):
    if image_indices is None:
        image_indices = set(range(count))
    audit = ReplayInputAudit(mode="monocular", provenance=provenance)
    for i in range(count):
        t = BASE + i * INTERVAL
        audit.ingest("info", t, info(t))
        if i in image_indices:
            stamp = t if i != bad_stamp else t - 1
            audit.ingest("image", t + 1, image(stamp))
    return audit, audit.finish()


class M23ILongBagDiagnosticsTests(unittest.TestCase):
    def test_more_than_2048_valid_camera_info_not_false_blocked(self):
        audit, report = summarize()
        self.assertEqual(report["verdict"], "REPLAY_INPUT_CONTRACT_PASS")
        self.assertEqual(report["counts"]["info"], 3906)
        self.assertEqual(report["matched_image_info_pairs"], 3906)
        self.assertNotIn("BLOCKED_UNBOUNDED_CAMERA_INFO", report["faults"])
        self.assertLessEqual(len(audit.info_by_stamp), MAX_PENDING)
        self.assertEqual(report["camera_info_cache_evictions"], 3906 - MAX_PENDING)

    def test_real_recording_counts_can_be_stamped_and_matched(self):
        chosen = {i * 3905 // 728 for i in range(729)}
        self.assertEqual(len(chosen), 729)
        audit, report = summarize(image_indices=chosen)
        self.assertEqual(report["counts"], {"image": 729, "info": 3906, "imu": 0})
        self.assertEqual(report["matched_image_info_pairs"], 729)
        self.assertEqual(report["verdict"], "REPLAY_INPUT_CONTRACT_PASS")
        self.assertLessEqual(len(audit.info_by_stamp), MAX_PENDING)

    def test_long_camera_stream_with_3s_gap_still_blocks(self):
        # Skip 90 frames in the center -> >3 seconds missing image stamps.
        chosen = set(range(3906)) - set(range(2000, 2090))
        _audit, report = summarize(image_indices=chosen)
        self.assertEqual(report["verdict"], "BLOCKED_REPLAY_INPUT")
        self.assertIn("BLOCKED_IMAGE_STAMP_GAP", report["faults"])
        self.assertGreater(report["max_header_gap_ms"]["image"], 250.0)

    def test_unmatched_camera_image_still_blocks(self):
        chosen = set(range(0, 3906, 5))
        audit = ReplayInputAudit(mode="monocular", provenance="synthetic")
        for i in range(3906):
            t = BASE + i * INTERVAL
            audit.ingest("info", t, info(t))
            if i in chosen:
                fake_stamp = t + 1_000_000
                audit.ingest("image", t + 1, image(fake_stamp))
        result = audit.finish()
        self.assertIn("BLOCKED_UNMATCHED_CAMERA_INFO", result["faults"])
        self.assertLessEqual(len(audit.info_by_stamp), MAX_PENDING)

    def test_late_camera_info_still_matches_pending_image(self):
        audit = ReplayInputAudit(mode="monocular", provenance="synthetic")
        # Preserve late CameraInfo ordering: image before its matching info.
        for i in range(10):
            t = BASE + i * INTERVAL
            audit.ingest("image", t, image(t))
            audit.ingest("info", t + 1, info(t))
        report = audit.finish()
        self.assertEqual(report["matched_image_info_pairs"], 10)
        self.assertEqual(report["verdict"], "REPLAY_INPUT_CONTRACT_PASS")

    def test_capped_cache_does_not_authenticate_real_origin(self):
        _audit, report = summarize(provenance="user_declared_recorded")
        self.assertIs(report["provenance_authenticated"], False)
        self.assertIs(report["xrslam_tracking_validated"], False)
        self.assertIs(report["camera_imu_physical_offset_validated"], False)
        self.assertIs(report["physical_motion_authorized"], False)

    def test_m23f_carries_exact_faults_without_promoting_blocked(self):
        nested = {
            "verdict": "BLOCKED_OFFLINE_EVIDENCE",
            "stream_evidence": {
                "counts": {"image": 729, "info": 3906, "imu": 6001},
                "faults": [],
                "camera_info_projection_changes": 0},
            "m23d_profile": {
                "faults": ["M23A_FORMAT_PREFLIGHT_FAILED"],
                "m23a_preflight": {
                    "verdict": "BLOCKED_REPLAY_INPUT",
                    "faults": ["BLOCKED_IMAGE_STAMP_GAP"],
                    "matched_image_info_pairs": 729,
                    "camera_info_cache_evictions": 1858},
                "header_overlap_s": 119.997594,
                "max_header_gap_ms": {
                    "image": 2951.988, "info": 52.074, "imu": 20.005},
                "gaps_over_250ms": {
                    "image": 4, "info": 0, "imu": 0},
                "sampled_image_variation_observed": True},
            "bag_files": {"metadata.yaml": {"sha256": "fake"}},
            "bag_files_stable_during_audit": True}
        with mock.patch(
                "m23f_existing_recording_inventory._ensure_regular_bag_files"
        ), mock.patch(
                "m23e_offline_evidence_snapshot.inspect_existing_bag",
                return_value=nested):
            report = _audit_one(Path("/not-a-real-bag"), full_cdr=True)
        self.assertEqual(report["candidate_verdict"], "BLOCKED_OFFLINE_EVIDENCE")
        self.assertIn("BLOCKED_IMAGE_STAMP_GAP", report["faults"])
        self.assertEqual(report["m23a_faults"], ["BLOCKED_IMAGE_STAMP_GAP"])
        self.assertEqual(report["m23a_matched_image_info_pairs"], 729)
        self.assertEqual(report["m23a_camera_info_cache_evictions"], 1858)
        self.assertEqual(report["gaps_over_250ms"]["image"], 4)

    def test_m23f_old_profile_still_exposes_generic_block(self):
        nested = {
            "verdict": "BLOCKED_OFFLINE_EVIDENCE",
            "stream_evidence": {
                "counts": {"image": 2, "info": 2, "imu": 2},
                "faults": [], "camera_info_projection_changes": 0},
            "m23d_profile": {
                "faults": ["M23A_FORMAT_PREFLIGHT_FAILED"],
                "header_overlap_s": 1.0,
                "max_header_gap_ms": {
                    "image": 0, "info": 0, "imu": 0},
                "sampled_image_variation_observed": False},
            "bag_files": {},
            "bag_files_stable_during_audit": True}
        with mock.patch(
                "m23f_existing_recording_inventory._ensure_regular_bag_files"
        ), mock.patch(
                "m23e_offline_evidence_snapshot.inspect_existing_bag",
                return_value=nested):
            report = _audit_one(Path("/not-a-real-bag"), full_cdr=True)
        self.assertIn("M23A_FORMAT_PREFLIGHT_FAILED", report["faults"])
        self.assertEqual(report["m23a_faults"], [])
        self.assertIsNone(report["m23a_verdict"])

    def test_m23d_preserves_original_base_report_for_forensics(self):
        # The source lives beside this test on the Arch checkout, but
        # under /opt/m23d in the isolated Jazzy image.
        text = Path(
            import_module("m23d_recorded_bag_readiness").__file__
        ).read_text()
        self.assertIn('result["m23a_preflight"] = base', text)
        self.assertIn('result = profile.finish(base,', text)

    def test_all_modified_sources_remain_non_actuating(self):
        # Resolve each module's actual imported source location, rather
        # than assuming all sources were copied beside the M23I test.
        for name in ("m23a_xrslam_rosbag_preflight",
                     "m23d_recorded_bag_readiness",
                     "m23f_existing_recording_inventory"):
            with self.subTest(name=name):
                source = Path(import_module(name).__file__).read_text()
                for unsafe in ("import serial", "rclpy.init(",
                               "serial.Serial(", "create_publisher(",
                               "set_wheels ", "subprocess.Popen("):
                    self.assertNotIn(unsafe, source)


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            M23ILongBagDiagnosticsTests))
    if result.wasSuccessful():
        print("M23I_LONG_BAG_FORENSICS_CONTRACT=PASS", flush=True)
        print("M23I_CAMERA_GAP_GATE_UNCHANGED=PASS", flush=True)
        print("M23I_REAL_BAG_REPLAY_VALIDATED=FALSE", flush=True)
        print("M23I_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    raise SystemExit(2)
