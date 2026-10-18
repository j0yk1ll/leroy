"""M23D pure-Python, device-free recorded-bag profile regressions.

These use synthetic in-memory message-shaped objects. They do NOT assert
that a bag comes from real hardware, is calibrated, or enables motion.
"""
from pathlib import Path
from types import SimpleNamespace as S
import unittest

from m23d_recorded_bag_readiness import (
    RecordingProfile, _sample_signature,
)

B = 1_800_000_000_000_000_000


def _header(t, frame):
    return S(stamp=S(sec=t // 1_000_000_000,
                     nanosec=t % 1_000_000_000), frame_id=frame)


def _message(kind, t, *, payload=b"\x20" * 256):
    m = S(header=_header(t, "imu_link" if kind == "imu"
                          else "camera_optical_frame"))
    if kind == "image":
        m.data = payload
    return m


def good_profile(*, images_constant=False):
    p = RecordingProfile()
    events = []
    for i in range(16):
        at = B + i * 125_000_000
        events += [("info", at, _message("info", at)),
                   ("image", at + 1, _message(
                       "image", at,
                       payload=(bytes([0 if images_constant else i]) * 256)))]
    for i in range(104):
        at = B - 40_000_000 + i * 20_000_000
        events.append(("imu", at, _message("imu", at)))
    for kind, bag_ns, msg in sorted(events, key=lambda row: row[1]):
        p.consume(kind, bag_ns, msg)
    return p


def preflight(*, verdict="REPLAY_INPUT_CONTRACT_PASS",
              mode="monocular_imu",
              counts=None):
    return {
        "verdict": verdict,
        "mode": mode,
        "counts": counts if counts is not None else {
            "image": 16, "info": 16, "imu": 104},
    }


class M23DRecordedReadinessContract(unittest.TestCase):
    def test_valid_synthetic_profile_is_only_data_inventory(self):
        result = good_profile().finish(
            preflight(), declared_origin="synthetic")
        self.assertEqual(result["verdict"],
                         "REPLAY_INPUT_RECORDING_PROFILE_ONLY")
        self.assertEqual(result["counts"],
                         {"image": 16, "info": 16, "imu": 104})
        self.assertEqual(result["header_overlap_s"], 1.875)
        self.assertTrue(result["sampled_image_variation_observed"])

    def test_source_claim_never_authenticates_real_capture(self):
        result = good_profile().finish(
            preflight(), declared_origin="user_declared_recorded")
        self.assertEqual(result["verdict"],
                         "REPLAY_INPUT_RECORDING_PROFILE_ONLY")
        for key in (
                "origin_authenticated", "synthetic_data_excluded_by_proof",
                "physical_camera_imu_offset_validated",
                "spatial_extrinsics_validated",
                "imu_noise_calibration_validated",
                "xrslam_tracking_accuracy_validated",
                "pose_convention_validated", "production_motion_enabled",
                "physical_motion_authorized", "device_access",
                "ready_for_metric_vio"):
            self.assertIs(result[key], False, key)

    def test_static_images_report_only_warning_not_tracking_pass(self):
        result = good_profile(images_constant=True).finish(
            preflight(), declared_origin="synthetic")
        self.assertEqual(result["sampled_image_fingerprints"], 1)
        self.assertIs(result["sampled_image_variation_observed"], False)
        self.assertIs(result["xrslam_tracking_accuracy_validated"], False)

    def test_sample_signature_is_deterministic(self):
        msg = _message("image", B, payload=bytes(range(256)))
        self.assertEqual(_sample_signature(msg), _sample_signature(msg))
        self.assertEqual(len(_sample_signature(msg)), 16)

    def test_empty_image_sample_blocks(self):
        p = RecordingProfile()
        p.consume("image", B, _message("image", B, payload=b""))
        self.assertIn("INVALID_IMAGE_SAMPLED_PAYLOAD", p.failures)

    def test_missing_imu_stream_blocks(self):
        p = RecordingProfile()
        for i in range(3):
            t = B + i * 125_000_000
            p.consume("image", t, _message("image", t))
            p.consume("info", t + 1, _message("info", t))
        result = p.finish(preflight(counts={"image": 3, "info": 3, "imu": 0}),
                          declared_origin="synthetic")
        self.assertEqual(result["verdict"], "BLOCKED_RECORDING_PROFILE")
        self.assertIn("MISSING_STREAM", result["faults"])

    def test_short_header_overlap_blocks(self):
        p = RecordingProfile()
        for k in ("image", "info", "imu"):
            p.consume(k, B, _message(k, B))
            p.consume(k, B + 100_000_000,
                      _message(k, B + 100_000_000))
        result = p.finish(
            preflight(counts={"image": 2, "info": 2, "imu": 2}),
            declared_origin="synthetic")
        self.assertIn("SHORT_HEADER_TIME_OVERLAP", result["faults"])

    def test_m23a_blocker_remains_blocker(self):
        result = good_profile().finish(
            preflight(verdict="BLOCKED_REPLAY_INPUT"),
            declared_origin="synthetic")
        self.assertIn("M23A_FORMAT_PREFLIGHT_FAILED", result["faults"])

    def test_mono_only_preflight_does_not_become_vio_ready(self):
        result = good_profile().finish(
            preflight(mode="monocular"), declared_origin="synthetic")
        self.assertIn("VIO_MODE_REQUIRED", result["faults"])

    def test_second_pass_count_mismatch_blocks(self):
        result = good_profile().finish(
            preflight(counts={"image": 15, "info": 16, "imu": 104}),
            declared_origin="synthetic")
        self.assertIn("PROFILE_COUNTS_DISAGREE_WITH_M23A",
                      result["faults"])

    def test_backwards_imu_header_is_flagged(self):
        p = RecordingProfile()
        p.consume("imu", B, _message("imu", B))
        p.consume("imu", B + 1, _message("imu", B - 2))
        self.assertIn("NONMONOTONIC_IMU_STAMP", p.failures)

    def test_backwards_bag_time_is_flagged(self):
        p = RecordingProfile()
        p.consume("imu", B, _message("imu", B))
        p.consume("imu", B - 1, _message("imu", B + 1))
        self.assertIn("BACKWARD_BAG_TIME", p.failures)

    def test_invalid_bag_time_is_flagged(self):
        p = RecordingProfile()
        p.consume("imu", 0, _message("imu", B))
        self.assertIn("INVALID_BAG_TIME", p.failures)

    def test_image_gap_reports_count_and_warning(self):
        p = RecordingProfile()
        p.consume("image", B, _message("image", B))
        p.consume("image", B + 400_000_000,
                  _message("image", B + 400_000_000))
        self.assertEqual(p.gaps_over_250ms["image"], 1)
        self.assertEqual(p.max_gap_ns["image"], 400_000_000)

    def test_wrong_stream_does_not_get_counted(self):
        p = RecordingProfile()
        p.consume("wheel", B, _message("imu", B))
        self.assertIn("UNKNOWN_STREAM", p.failures)
        self.assertEqual(sum(p.count.values()), 0)

    def test_invalid_stamp_does_not_create_fake_overlap(self):
        p = RecordingProfile()
        p.consume("image", B, S(header=S(stamp=S(sec=-1, nanosec=0))))
        self.assertIn("INVALID_IMAGE_STAMP", p.failures)
        self.assertNotIn("image", p.first)

    def test_source_does_not_start_ros_or_devices(self):
        src = (Path(__file__).resolve().parent /
               "m23d_recorded_bag_readiness.py").read_text()
        for unsafe in ("rclpy.init(", "serial.Serial(", "robot.launch.py",
                       "create_publisher(", "create_subscription(",
                       "set_wheels ", "/dev/ttyACM", "subprocess.Popen"):
            self.assertNotIn(unsafe, src)


if __name__ == "__main__":
    outcome = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            M23DRecordedReadinessContract))
    if outcome.wasSuccessful():
        print("M23D_RECORDED_READINESS_STATIC_CONTRACT=PASS", flush=True)
        print("M23D_VERDICT=OFFLINE_SYNTHETIC_PROFILE_CONTRACT_ONLY", flush=True)
        print("M23D_REAL_RECORDED_BAG_VERIFIED=FALSE", flush=True)
        print("M23D_XRSLAM_TRACKING_VALIDATED=FALSE", flush=True)
        print("M23D_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    print("M23D_RECORDED_READINESS_STATIC_CONTRACT=FAIL", flush=True)
    raise SystemExit(2)
