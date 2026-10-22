"""M23BG: device-free source-only stationary pipeline readiness test suite."""
import contextlib
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import m23bg_stationary_integration_contract as m


def fixtures():
    return {name: "\n".join(anchors) + "\n"
            for name, anchors in m.REQUIRED.items()}


class Tests(unittest.TestCase):
    def test_default_inert_without_reading_source(self):
        with patch.object(m, "load_sources", side_effect=AssertionError("read")):
            with contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(m.main([]), 0)
        self.assertIn("M23BG_STATUS=INERT_NO_ROS_NO_DEVICE_NO_WRITES",
                      out.getvalue())

    def test_explicit_check_and_root_required(self):
        with contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(m.main(["--check"]), 2)
        self.assertIn("M23BG_REAL_ROBOT_INTEGRATION=BLOCKED", out.getvalue())

    def test_source_interface_audit_is_not_hardware_pass(self):
        report = m.audit_sources(fixtures())
        self.assertEqual(report["verdict"], "OFFLINE_SOURCE_INTERFACES_AUDITED_ONLY")
        self.assertEqual(report["component_count"], 7)
        self.assertFalse(report["physical_camera_retest_authorized"])
        self.assertEqual(report["production_acceptance"], "BLOCKED")

    def test_reports_costmap_topic_mismatch_without_normalizing(self):
        report = m.audit_sources(fixtures())
        self.assertEqual(report["m4_fixture_costmap_topic"], "/costmap")
        self.assertEqual(report["m15_real_gate_costmap_topic"],
                         "/local_costmap/costmap")
        self.assertFalse(report["actual_costmap_endpoint_alignment_verified"])

    def test_reports_xrslam_tracking_and_scale_unverified(self):
        report = m.audit_sources(fixtures())
        self.assertFalse(report["xrslam_metric_pose_verified_on_robot"])
        self.assertFalse(report["real_vio_tracking_verified"])
        self.assertFalse(report["camera_imu_physical_offset_verified"])
        self.assertFalse(report["persistent_mapping_verified"])

    def test_reports_depth_cloud_disabled_until_calibrated(self):
        report = m.audit_sources(fixtures())
        self.assertTrue(report["depth_obstacles_source_topic_matches_costmap"])
        self.assertFalse(report["depth_obstacle_publication_enabled"])
        self.assertFalse(report["depth_ground_and_extrinsics_verified"])

    def test_reports_bridge_in_normal_launch(self):
        report = m.audit_sources(fixtures())
        self.assertTrue(report["normal_robot_launch_includes_motor_capable_bridge"])
        self.assertFalse(report["bridge_free_stationary_live_launch_present"])

    def test_rejects_unexpected_component(self):
        src = fixtures()
        src["other"] = "fake"
        with self.assertRaisesRegex(m.Blocked, "seven exact"):
            m.audit_sources(src)

    def test_rejects_missing_component(self):
        src = fixtures()
        src.pop("costmap_config")
        with self.assertRaisesRegex(m.Blocked, "seven exact"):
            m.audit_sources(src)

    def test_rejects_ungated_xrslam_pose(self):
        src = fixtures()
        src["xrslam"] = src["xrslam"].replace(
            'declare_parameter<bool>("pose_convention_validated",false)',
            'declare_parameter<bool>("pose_convention_validated",true)')
        with self.assertRaisesRegex(m.Blocked, "xrslam"):
            m.audit_sources(src)

    def test_rejects_depth_obstacle_flag_enabled(self):
        src = fixtures()
        src["depth_obstacles"] = src["depth_obstacles"].replace(
            "'publish_obstacles': False", "'publish_obstacles': True")
        with self.assertRaisesRegex(m.Blocked, "depth_obstacles"):
            m.audit_sources(src)

    def test_rejects_depth_calibration_bypass(self):
        src = fixtures()
        src["depth_obstacles"] = src["depth_obstacles"].replace(
            "'ground_reference_validated': False",
            "'ground_reference_validated': True")
        with self.assertRaisesRegex(m.Blocked, "ground_reference"):
            m.audit_sources(src)

    def test_rejects_costmap_unsafe_clearing(self):
        src = fixtures()
        src["costmap_config"] = src["costmap_config"].replace(
            "clearing: false", "clearing: true")
        with self.assertRaisesRegex(m.Blocked, "costmap_config"):
            m.audit_sources(src)

    def test_rejects_synthetic_fake_odom_removed(self):
        src = fixtures()
        src["synthetic_probe"] = src["synthetic_probe"].replace(
            "'/leroy_nav2/diagnostics/synthetic_odom'", "'/odom'")
        with self.assertRaisesRegex(m.Blocked, "synthetic_probe"):
            m.audit_sources(src)

    def test_rejects_altered_real_costmap_endpoint(self):
        src = fixtures()
        src["real_input_gate"] = src["real_input_gate"].replace(
            "'map': '/local_costmap/costmap'", "'map': '/costmap'")
        with self.assertRaisesRegex(m.Blocked, "real_input_gate"):
            m.audit_sources(src)

    def test_rejects_motor_bridge_exposure_claim(self):
        src = fixtures()
        src["robot_launch"] = src["robot_launch"].replace(
            "FindPackageShare('leroy_bridge')", "FindPackageShare('safe_fake')")
        with self.assertRaisesRegex(m.Blocked, "robot_launch"):
            m.audit_sources(src)

    def test_rejects_unexpected_live_command_output(self):
        src = fixtures()
        src["real_input_gate"] = src["real_input_gate"].replace(
            "OUTPUT = '/leroy_nav2/diagnostics/m15_gated_cmd_vel'",
            "OUTPUT = '/cmd_vel'")
        with self.assertRaisesRegex(m.Blocked, "real_input_gate"):
            m.audit_sources(src)

    def test_rejects_passive_imu_topic_mismatch(self):
        src = fixtures()
        src["passive_observer"] = src["passive_observer"].replace(
            "'imu': ('/imu/data_raw'", "'imu': ('/imu/unverified'")
        with self.assertRaisesRegex(m.Blocked, "passive_observer"):
            m.audit_sources(src)

    def test_file_loader_only_reads_required_sources(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            for name, path in m.PATHS.items():
                f = root / path
                f.parent.mkdir(parents=True, exist_ok=True)
                f.write_text(fixtures()[name])
            loaded = m.load_sources(root)
            self.assertEqual(loaded, fixtures())
            report = m.audit_sources(loaded)
            self.assertEqual(report["component_count"], 7)

    def test_missing_source_file_is_blocked(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaisesRegex(m.Blocked, "absent"):
                m.load_sources(Path(d))

    def test_symlinked_checkout_root_blocked(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            target = root / "target"
            target.mkdir()
            (root / "link").symlink_to(target)
            with self.assertRaisesRegex(m.Blocked, "symlinked"):
                m.load_sources(root / "link")

    def test_oversize_rejected_without_reading(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            path = root / m.PATHS["xrslam"]
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("wb") as f:
                f.truncate(m.MAX_FILE_BYTES + 1)
            with self.assertRaisesRegex(m.Blocked, "size outside"):
                m.load_sources(root)

    def test_source_check_does_not_write_anything(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            for name, path in m.PATHS.items():
                f = root / path
                f.parent.mkdir(parents=True, exist_ok=True)
                f.write_text(fixtures()[name])
            old = {str(p): p.read_bytes() for p in root.rglob("*")
                   if p.is_file()}
            with contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(m.main(["--check", "--root", str(root)]), 0)
            new = {str(p): p.read_bytes() for p in root.rglob("*")
                   if p.is_file()}
            self.assertEqual(old, new)
            self.assertIn("M23BG_SOURCE_INTERFACE_GATE=PASS", out.getvalue())
            self.assertIn("M23BG_REAL_ROBOT_INTEGRATION=BLOCKED",
                          out.getvalue())


if __name__ == "__main__":
    unittest.main()
