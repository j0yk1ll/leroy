"""M23F offline Python inventory contracts: no ROS runtime or hardware.

All fake metadata paths are TEMP, not real sensor recordings. M23E's
actual CDR reader is only invoked by a separately gated Jazzy smoke test.
"""
from pathlib import Path
import tempfile
from unittest import mock
import unittest

import m23f_existing_recording_inventory as sut

GOOD = {
    "candidate_verdict": "REFERENCED_STORAGE_FILES_PRESENT_ONLY",
    "counts": None, "faults": [], "sha256_manifest": None,
}


def fake_bag(root, name):
    path = root / name
    path.mkdir()
    (path / "metadata.yaml").write_text("fake: true\n", encoding="utf8")
    return path


class M23FExistingRecordingContract(unittest.TestCase):
    def test_empty_data_root_is_blocked(self):
        with tempfile.TemporaryDirectory() as temp:
            report = sut.inventory(Path(temp))
        self.assertEqual(report["verdict"], "BLOCKED_NO_EXISTING_ROSBAG")
        self.assertEqual(report["candidate_count"], 0)
        self.assertEqual(report["candidates"], [])

    def test_missing_explicit_root_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaises(ValueError):
                sut.inventory(Path(temp) / "unavailable")

    def test_file_is_not_a_directory(self):
        with tempfile.TemporaryDirectory() as temp:
            f = Path(temp) / "fake"
            f.write_text("x")
            with self.assertRaises(ValueError):
                sut.discover_bags(f)

    def test_root_as_single_bag_candidate(self):
        with tempfile.TemporaryDirectory() as temp:
            (Path(temp) / "metadata.yaml").write_text("placeholder")
            self.assertEqual(
                [(name, path) for name, path
                 in sut.discover_bags(Path(temp))],
                [(".", Path(temp))])

    def test_shallow_discovery_only(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            a = root / "outer"
            a.mkdir()
            fake_bag(a, "nested")
            self.assertEqual(sut.discover_bags(root), [])

    def test_parent_discovers_direct_bags_sorted(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            fake_bag(root, "z_bag")
            fake_bag(root, "a_bag")
            (root / "random.txt").write_text("ignore me")
            self.assertEqual(
                [name for name, _ in sut.discover_bags(root)],
                ["a_bag", "z_bag"])

    def test_directory_symlink_is_not_traversed(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            actual = fake_bag(root, "actual")
            (root / "alias").symlink_to(actual, target_is_directory=True)
            self.assertEqual(
                [name for name, _ in sut.discover_bags(root)], ["actual"])

    def test_symlinked_root_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            real = root / "bags"
            real.mkdir()
            alias = root / "linked"
            alias.symlink_to(real, target_is_directory=True)
            with self.assertRaises(ValueError):
                sut.discover_bags(alias)

    def test_single_candidate_reports_references_only(self):
        with tempfile.TemporaryDirectory() as temp:
            fake_bag(Path(temp), "candidate")
            with mock.patch.object(sut, "_audit_one", return_value=GOOD):
                report = sut.inventory(Path(temp))
        self.assertEqual(report["verdict"],
                         "EXISTING_ROSBAG_REFERENCES_INVENTORIED_ONLY")
        self.assertEqual(report["candidate_count"], 1)

    def test_explicit_cdr_audit_claim_is_still_unverified(self):
        result = dict(GOOD)
        result["candidate_verdict"] = "OFFLINE_EVIDENCE_SNAPSHOT_ONLY"
        with tempfile.TemporaryDirectory() as temp:
            fake_bag(Path(temp), "candidate")
            with mock.patch.object(sut, "_audit_one", return_value=result):
                report = sut.inventory(Path(temp), full_cdr=True)
        self.assertEqual(report["verdict"],
                         "EXISTING_CDR_INPUTS_PROFILED_UNVERIFIED")
        self.assertEqual(report["audit_mode"], "READ_ONLY_CDR")
        for name in (
                "recorded_sensor_origin_verified",
                "recorded_sensor_calibration_verified",
                "real_xrslam_tracking_validated",
                "safe_for_metric_vio",
                "physical_motion_authorized",
                "device_access", "root_scanned_recursively",
                "other_host_filesystems_searched"):
            self.assertIs(report[name], False)

    def test_one_bad_candidate_blocks_mixed_inventory(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            fake_bag(root, "good")
            fake_bag(root, "bad")
            def audit(folder, **_kw):
                if folder.name == "bad":
                    raise ValueError("test-only invalid metadata")
                return dict(GOOD)
            with mock.patch.object(sut, "_audit_one", side_effect=audit):
                report = sut.inventory(root)
        self.assertEqual(report["verdict"],
                         "BLOCKED_CANDIDATE_STORAGE_OR_CDR")
        self.assertEqual([x["candidate_verdict"] for x
                          in report["candidates"]],
                         ["BLOCKED_CANDIDATE",
                          "REFERENCED_STORAGE_FILES_PRESENT_ONLY"])

    def test_unsafe_input_fault_is_bounded(self):
        with tempfile.TemporaryDirectory() as temp:
            fake_bag(Path(temp), "bad")
            with mock.patch.object(sut, "_audit_one",
                                   side_effect=ValueError("x" * 1000)):
                report = sut.inventory(Path(temp))
        self.assertLessEqual(
            len(report["candidates"][0]["faults"][0]), 260)

    def test_candidate_cap_blocks(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            for i in range(sut.MAX_BAGS + 1):
                fake_bag(root, "b%03d" % i)
            with self.assertRaisesRegex(ValueError, "candidate limit"):
                sut.discover_bags(root)

    def test_root_entries_cap_blocks(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            for i in range(sut.MAX_ROOT_ENTRIES + 1):
                (root / ("x%03d" % i)).write_text("")
            with self.assertRaisesRegex(ValueError, "entry limit"):
                sut.discover_bags(root)

    def test_origin_and_calibration_are_never_assumed(self):
        with tempfile.TemporaryDirectory() as temp:
            report = sut.inventory(Path(temp))
        for flag in ("recorded_sensor_origin_verified",
                     "recorded_sensor_calibration_verified",
                     "safe_for_metric_vio",
                     "physical_motion_authorized", "device_access"):
            self.assertIs(report[flag], False)

    def test_real_ordinal_not_confused_with_synthetic(self):
        with tempfile.TemporaryDirectory() as temp:
            fake_bag(Path(temp), "real_2026_10_18")
            with mock.patch.object(sut, "_audit_one", return_value=GOOD):
                report = sut.inventory(Path(temp))
        self.assertIs(report["recorded_sensor_origin_verified"], False)
        self.assertEqual(report["candidates"][0]["relative_directory"],
                         "real_2026_10_18")

    def test_source_does_not_open_hardware_or_start_ros(self):
        source = (Path(__file__).resolve().parent /
                  "m23f_existing_recording_inventory.py").read_text()
        for forbidden in ("rclpy.init(", "serial.Serial(",
                          "subprocess.Popen(", "create_publisher(",
                          "robot.launch.py", "os.system("):
            self.assertNotIn(forbidden, source)
        self.assertIn("MAX_ROOT_ENTRIES = 256", source)
        self.assertIn("MAX_BAGS = 32", source)
        self.assertIn("_ensure_regular_bag_files(path)", source)

    def test_bag_without_metadata_not_discovered(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            empty = root / "notabag"
            empty.mkdir()
            (empty / "file.db3").write_bytes(b"fake")
            self.assertEqual(sut.discover_bags(root), [])


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            M23FExistingRecordingContract))
    if result.wasSuccessful():
        print("M23F_OFFLINE_INVENTORY_CONTRACT=PASS", flush=True)
        print("M23F_REAL_RECORDED_DATA_FOUND=FALSE", flush=True)
        print("M23F_CALIBRATION_VALIDATED=FALSE", flush=True)
        print("M23F_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    print("M23F_OFFLINE_INVENTORY_CONTRACT=FAIL", flush=True)
    raise SystemExit(2)
