"""Focused no-device regression for the persistent live sensor boot service."""
import ast
from pathlib import Path
import subprocess
import unittest

import live_boot_imu_service as source

ROOT = Path(__file__).resolve().parent


class PersistentBootService(unittest.TestCase):
    def test_exclusive_source_environment(self):
        env = dict(source.REQUIRED_ENV)
        self.assertTrue(source.preflight(env, lambda p: p == source.DEVICE))
        for key in env:
            with self.subTest(key=key):
                broken = dict(env)
                del broken[key]
                with self.assertRaisesRegex(RuntimeError, "BOOT_IMU_ENV_"):
                    source.preflight(broken, lambda p: p == source.DEVICE)

    def test_source_preflight_denies_missing_or_extra_devices(self):
        with self.assertRaisesRegex(RuntimeError, "BOOT_IMU_MCU_DEVICE_MISSING"):
            source.preflight(source.REQUIRED_ENV, lambda p: False)
        for unexpected in ("/dev/video2", "/dev/gpiochip0"):
            with self.subTest(unexpected=unexpected):
                with self.assertRaisesRegex(
                        RuntimeError, "BOOT_IMU_UNEXPECTED_DEVICE"):
                    source.preflight(source.REQUIRED_ENV,
                                     lambda p: p in (source.DEVICE, unexpected))

    def test_single_source_only_whitelisted_commands(self):
        s = (ROOT / "live_boot_imu_service.py").read_text()
        tree = ast.parse(s)
        imports = {a.name for node in ast.walk(tree)
                   if isinstance(node, ast.Import)
                   for a in node.names}
        self.assertNotIn("subprocess", imports)
        self.assertNotIn("leroy_bridge.node", s)
        self.assertNotIn("set_wheels", s)
        self.assertNotIn("/cmd_vel", s.split("def run():")[-1])
        self.assertIn("send_only_whitelisted_serial_command(device, START)", s)
        self.assertIn("send_only_whitelisted_serial_command(device, STOP)", s)
        self.assertIn("exclusive=True", s)
        self.assertIn("BOOT_IMU_SOURCE_STOP_ACK=", s)

    def test_persistent_yaw_still_publishes_status_only(self):
        s = (ROOT / "live_boot_yaw.py").read_text()
        self.assertIn('LEROY_BOOT_YAW_PERSISTENT', s)
        self.assertIn('if node.heading.state == "FAULT_LATCHED"', s)
        self.assertIn('"motor_motion_authorized": False', s)
        tree = ast.parse(s)
        publishers = [n for n in ast.walk(tree)
                      if isinstance(n, ast.Call)
                      and isinstance(n.func, ast.Attribute)
                      and n.func.attr == "create_publisher"]
        self.assertEqual(len(publishers), 1)
        self.assertEqual(ast.unparse(publishers[0].args[1]), "STATUS_TOPIC")

    def test_single_command_shell_is_syntactically_valid_and_cleans_up(self):
        script = ROOT / "live_boot_session.sh"
        subprocess.run(["bash", "-n", str(script)], check=True)
        s = script.read_text()
        self.assertIn("LEROY_STAGE4_MOTOR_ISOLATION_ATTESTED", s)
        self.assertIn("BOOT_IMU_ONLY", s)
        self.assertIn('trap cleanup EXIT', s)
        self.assertIn('podman stop -t 7 "$source"', s)
        self.assertIn("BOOT_IMU_SOURCE_STOP_ACK=PASS", s)
        self.assertIn("LEROY_BOOT_YAW_PERSISTENT=1 bash", s)
        self.assertIn("--device=", s)
        self.assertIn("--read-only", s)
        self.assertIn("source=leroy-boot-imu-source", s)
        self.assertNotIn("--privileged", s)

    def test_accepted_short_live_yaw_mode_not_replaced(self):
        s = (ROOT / "live_boot_yaw.sh").read_text()
        subprocess.run(["bash", "-n", str(ROOT/"live_boot_yaw.sh")], check=True)
        self.assertIn("source=leroy-stage4-imu", s)
        self.assertIn("sidecar=leroy-live-boot-yaw", s)
        self.assertIn("source=leroy-boot-imu-source", s)
        self.assertIn('LEROY_BOOT_YAW_PERSISTENT', s)


if __name__ == "__main__":
    unittest.main()
