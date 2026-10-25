#!/usr/bin/env python3
"""Necessary source-only safety preflight for physical Stage 2B (NO devices)."""
import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parent


class Stage2BContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.host = (ROOT / "m23bv_v3_stage2b_host_run.sh").read_text()
        cls.probe = (ROOT / "m23bv_v3_stage2b_physical_probe.py").read_text()
        cls.ast = ast.parse(cls.probe)
        cls.stage1 = (ROOT / "m23bv_v3_stage1_camera_observer.py").read_text()
        cls.m20 = (ROOT / "m20_combined_observer.py").read_text()

    def test_exact_unchanged_m20_and_v3_source_pins(self):
        for required in (
            "a0ae75a62da39235b113e3d2c324efda5fcfc6e1",
            "93320b985c5dda3fdf0d1f8def001ba06480e7b6",
            "5cf2f88cb2f79be80821888401b57a6393ee5e3b",
            "66111d8da486dc1e0c78057c1948f5e1aad9a06e",
            "3c5fb37ca291533e2702a9159f3e8db11b076d22",
            "443aa82066e88d50cb88859304e9197e3f459ee0",
            "f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c",
        ):
            self.assertIn(required, self.probe + self.host)
        self.assertIn("git hash-object", self.host)
        self.assertIn('git status --porcelain', self.host)
        self.assertIn('BRANCH=', self.host)

    def test_real_mcu_is_only_started_through_protected_m20(self):
        self.assertIn('["python3", "-u", M20_OBSERVER]', self.probe)
        self.assertIn('M20_OBSERVER = "/opt/m20/m20_combined_observer.py"', self.probe)
        self.assertNotIn("serial.Serial(", self.probe)
        self.assertNotIn("device.write(", self.probe)
        self.assertIn("send_only_whitelisted_serial_command(device, START)", self.m20)
        self.assertIn("send_only_whitelisted_serial_command(device, STOP)", self.m20)
        self.assertIn("M20_IMU_STREAM_STOP_ACK=PASS", self.host)
        self.assertIn("M20_COMBINED_VERDICT=OBSERVED_CONCURRENT_CAMERA_IMU_ROS_CONTRACT",
                      self.host)
        self.assertIn("M20_COMBINED_VERDICT=OBSERVED_CONCURRENT_CAMERA_IMU_ROS_CONTRACT",
                      self.probe)

    def test_human_gate_before_two_device_mappings(self):
        for token in (
            "MOTOR RAILS OFF MCU SENSOR ONLY",
            "CALIBRATION VERIFIED FOR CAMERA",
            "AUTHORIZE V3 CAMERA AND M20 IMU NO RECORDING",
            "M23BV_STAGE2B_MCU_BY_ID",
            "M23BV_STAGE2B_MCU_ID_SERIAL",
            "/dev/serial/by-id/",
            "MCU_SERIAL_ID_MISMATCH",
            "CAMERA_SERIAL_CHANGED",
            "ESTOP",
        ):
            # E-stop instructions are case-sensitive text and must be present.
            if token == "ESTOP":
                self.assertIn("emergency stop", self.host)
            else:
                self.assertIn(token, self.host)
        self.assertLess(self.host.index("MOTOR RAILS OFF MCU SENSOR ONLY"),
                        self.host.index("podman run"))
        self.assertEqual(self.host.count('  --device "'), 2)
        self.assertIn('--device "$cam:/dev/video2:rw"', self.host)
        self.assertIn('--device "$mcu:/dev/ttyACM0:rw"', self.host)
        self.assertIn("m23bv_v3_stage2b_approval_", self.host)
        self.assertIn("chmod 0600", self.host)

    def test_pre_serial_gate_and_original_m20_lifecycle(self):
        self.assertLess(self.probe.index("graph_pre_serial(node)"),
                        self.probe.index('m20_proc = subprocess.Popen'))
        self.assertIn("MIN_PRECAM_STAMPS = 90", self.probe)
        self.assertIn('M23BV_V3_STAGE2B_PRE_SERIAL_GRAPH_QOS=PASS', self.probe)
        self.assertIn("stage1.info_matches_calibration", self.probe)
        self.assertIn("stage1.ALLOWED_TOPICS", self.probe)
        self.assertIn("PRE_SERIAL_UNKNOWN_TOPICS", self.probe)
        self.assertIn("PRE_SERIAL_UNKNOWN_NODES", self.probe)
        self.assertIn("M20_MAX_S = 155", self.probe)
        self.assertIn("M20_TERM_GRACE_S = 28", self.probe)

    def test_stop_imu_before_camera_and_no_recording(self):
        self.assertLess(self.probe.index("m20_proc.send_signal(signal.SIGTERM)"),
                        self.probe.index("camera_clean = stop_camera(camera_proc)"))
        self.assertIn("BLOCKED_M20_STOP_UNVERIFIED_MANUAL_POWER_ISOLATION",
                      self.probe)
        self.assertIn("M23BV_V3_STAGE2B_CAMERA_LAUNCH_REAPED=", self.probe)
        self.assertIn('timeout --foreground', self.host)
        self.assertIn("sha256sum", self.host)
        self.assertIn("--network none", self.host)
        self.assertIn("--cap-drop all", self.host)
        self.assertIn("--read-only", self.host)
        for src in (self.host, self.probe):
            self.assertNotIn("ros2 bag record", src)
            self.assertNotIn("rosbag2_py", src)
            self.assertNotIn("cmd_vel", src)
            self.assertNotIn("leroy_bridge_node", src)

    def test_probe_has_no_direct_serial_api(self):
        roots = set()
        for node in ast.walk(self.ast):
            if isinstance(node, ast.Import):
                roots.update(x.name.split(".")[0] for x in node.names)
            if isinstance(node, ast.ImportFrom) and node.module:
                roots.add(node.module.split(".")[0])
        self.assertNotIn("serial", roots)
        self.assertNotIn("rosbag2_py", roots)
        self.assertNotIn("cv2", roots)


if __name__ == "__main__":
    unittest.main(verbosity=2)
