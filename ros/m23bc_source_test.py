"""M23BC: synthetic stdlib source/adapter unit tests; no ROS, device, Podman."""
import contextlib
import io
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import m23bc_offline_source_stage as stage
import m23bc_synthetic_persistence as persist
import m23bc_binary_audit as elf


ROOT = Path(__file__).resolve().parent


class Tests(unittest.TestCase):
    def test_stage_default_is_inert(self):
        with patch.object(stage, "stage", side_effect=AssertionError("wrote source")):
            with contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(stage.main([]), 0)
        self.assertIn("M23BC_STAGE_STATUS=INERT_NO_WRITES_NO_DEVICE", out.getvalue())

    def test_synthetic_default_is_inert(self):
        with patch.object(persist, "write_only_synthetic",
                          side_effect=AssertionError("wrote host evidence")):
            with contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(persist.main([]), 0)
        self.assertIn("M23BC_STATUS=INERT_NO_ROS_NO_DEVICE", out.getvalue())

    def test_binary_default_is_inert(self):
        with patch.object(elf, "audit", side_effect=AssertionError("read binaries")):
            with contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(elf.main([]), 0)
        self.assertIn("M23BC_ELF_STATUS=INERT_NO_ROS_NO_DEVICE", out.getvalue())

    def test_source_contract_with_pinned_m23k(self):
        s = (ROOT / "m23k_camera_path_probe.py").read_text()
        o = (ROOT / "m23k_cpp/src/m23k_camera_header_counter.cpp").read_text()
        changed_s, changed_o = stage.contract(s, o)
        self.assertIn("from m23bb_host_evidence_adapter import staged_paths", changed_s)
        self.assertIn("CAMERA_LAUNCH_LOG, COUNTER_LOG = staged_paths()", changed_s)
        self.assertIn(stage.BINARY_ROOT, changed_s)
        self.assertNotIn("/opt/m23m-cpp-install/lib/", changed_s)
        self.assertNotIn("/opt/m23k-cpp-install/lib/", changed_s)
        self.assertIn('receipt_ns() << std::endl;', changed_o)
        self.assertIn('CAMERA_LAUNCH_LOG.open("xb")', changed_s)
        self.assertIn('COUNTER_LOG.open("xb")', changed_s)

    def test_source_refuses_changed_m23n_counter_root(self):
        s = (ROOT / "m23k_camera_path_probe.py").read_text()
        o = (ROOT / "m23k_cpp/src/m23k_camera_header_counter.cpp").read_text()
        with self.assertRaisesRegex(stage.Blocked, "M23N_OBSERVER_BINARY"):
            stage.contract(s.replace("/opt/m23m-cpp-install/",
                                     "/opt/changed-counter/"), o)

    def test_source_refuses_wrong_m23k_launch_anchor(self):
        s = (ROOT / "m23k_camera_path_probe.py").read_text()
        o = (ROOT / "m23k_cpp/src/m23k_camera_header_counter.cpp").read_text()
        with self.assertRaisesRegex(stage.Blocked, "launch contract"):
            stage.contract(s.replace('["ros2", "launch", camera_launch_path]',
                                     '["ros2", "launch", "unreviewed.launch.py"]'), o)

    def test_stage_requires_explicit_gate(self):
        with patch.dict(os.environ, {"M23BC_OFFLINE_BUILD_ONLY": ""}):
            with self.assertRaisesRegex(stage.Blocked, "build gate"):
                stage.stage()

    def test_payloads_are_synthetic_not_m23ax_or_m23k(self):
        camera, counter = persist.payloads()
        for data in (camera, counter):
            self.assertLess(len(data), 500)
            self.assertNotIn(b"M23AX_PUBLISH", data)
            self.assertNotIn(b"M23K_SAMPLE", data)
            self.assertEqual(data.count(b"\n"), 2)

    def test_synthetic_gate_required(self):
        with patch.object(persist, "no_devices_visible",
                          side_effect=AssertionError("device probe attempted")):
            with self.assertRaisesRegex(persist.Blocked, "approval gate"):
                persist.write_only_synthetic(env={})

    def test_no_video_devices_allowed(self):
        with self.assertRaisesRegex(persist.Blocked, "camera/serial"):
            persist.no_devices_visible(
                globber=lambda p: ["/dev/video3"] if "video" in p else [],
                bus=lambda: False)

    def test_no_serial_devices_allowed(self):
        with self.assertRaisesRegex(persist.Blocked, "camera/serial"):
            persist.no_devices_visible(
                globber=lambda p: ["/dev/ttyACM0"] if "ttyACM" in p else [],
                bus=lambda: False)

    def test_no_usb_bus_allowed(self):
        with self.assertRaisesRegex(persist.Blocked, "USB bus"):
            persist.no_devices_visible(globber=lambda p: [], bus=lambda: True)

    def test_synthetic_normal_files_independent_and_fsynced(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            files = root / "camera", root / "observer"
            with patch.object(persist, "no_devices_visible", return_value=True):
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(persist.write_only_synthetic(
                        paths=files, env={persist.GATE: persist.GATE_VALUE}), 0)
            self.assertEqual(files[0].read_bytes(), persist.CAM)
            self.assertEqual(files[1].read_bytes(), persist.OBS)

    def test_synthetic_crash_has_written_both_files_before_exit(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            files = root / "camera", root / "observer"
            def fake_exit(status):
                self.assertEqual(status, persist.CRASH_EXIT)
                self.assertEqual(files[0].read_bytes(), persist.CAM)
                self.assertEqual(files[1].read_bytes(), persist.OBS)
                raise RuntimeError("simulated nonzero termination")
            with patch.object(persist, "no_devices_visible", return_value=True), \
                 patch.object(persist.os, "_exit", side_effect=fake_exit):
                with self.assertRaisesRegex(RuntimeError, "simulated nonzero"):
                    persist.write_only_synthetic(
                        crash=True, paths=files,
                        env={persist.GATE: persist.GATE_VALUE})

    def test_synthetic_existing_file_never_overwritten(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            files = root / "camera", root / "observer"
            files[0].write_bytes(b"original")
            with patch.object(persist, "no_devices_visible", return_value=True):
                with self.assertRaises(FileExistsError):
                    persist.write_only_synthetic(
                        paths=files, env={persist.GATE: persist.GATE_VALUE})
            self.assertEqual(files[0].read_bytes(), b"original")
            self.assertFalse(files[1].exists())

    def test_binary_rejects_missing_libs(self):
        with self.assertRaisesRegex(elf.Blocked, "dependency"):
            elf.validate_ldd("libfoo.so => not found")

    def test_binary_rejects_tbb(self):
        with self.assertRaisesRegex(elf.Blocked, "TBB"):
            elf.validate_ldd("librclcpp.so => /opt/ros/jazzy/lib/librclcpp.so\n"
                             "libtbb.so.12 => /usr/lib/libtbb.so.12")

    def test_binary_good_rclcpp_only(self):
        self.assertTrue(elf.validate_ldd(
            "librclcpp.so => /opt/ros/jazzy/lib/librclcpp.so\n"
            "libsensor_msgs.so => /opt/ros/jazzy/lib/libsensor_msgs.so\n"))

    def test_binary_audit_gate_required(self):
        with patch.dict(os.environ, {"M23BC_OFFLINE_BINARY_ONLY": ""}):
            with self.assertRaisesRegex(elf.Blocked, "audit gate"):
                elf.audit()


if __name__ == "__main__":
    unittest.main()
