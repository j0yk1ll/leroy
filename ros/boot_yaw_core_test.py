"""No-device regression for live MCU-relative yaw and read-only bridge."""
import ast
import math
from pathlib import Path
import unittest

from boot_yaw_core import BootYaw
import live_boot_yaw as live

ROOT = Path(__file__).resolve().parent


def samples(bias=0.003, dt_ns=20_000_000, count=140):
    est = BootYaw()
    stamp = 1_800_000_000_000_000_000
    mono = 5_000_000_000
    for i in range(count):
        e = est.update(stamp+i*dt_ns, mono+i*dt_ns,
                       accel=(0.35, 0.1, 9.9),
                       gyro=(0.0, 0.0, bias))
    return est, stamp+(count-1)*dt_ns, mono+(count-1)*dt_ns


class BootYawTests(unittest.TestCase):
    def test_default_no_heading_and_no_authority(self):
        e = BootYaw().estimate()
        self.assertEqual(e.state, "WAITING_IMU")
        self.assertIsNone(e.relative_yaw_rad)
        self.assertFalse(hasattr(e, "motor_enable"))

    def test_stable_imu_bias_acquisition(self):
        est, stamp, mono = samples()
        e = est.estimate()
        self.assertEqual(e.state, "HEADING_ACTIVE")
        self.assertAlmostEqual(e.gyro_z_bias_rad_s, 0.003, places=6)
        self.assertAlmostEqual(e.relative_yaw_rad, 0, places=8)
        self.assertEqual(e.accepted_samples, 140)

    def test_real_time_integration_of_relative_yaw(self):
        est, stamp, mono = samples()
        rate = math.pi / 10  # 90 degrees over 5 seconds
        for i in range(1, 251):
            est.update(stamp + i*20_000_000,
                       mono + i*20_000_000,
                       accel=(0.35, 0.1, 9.9),
                       gyro=(0.0, 0.0, 0.003 + rate))
        self.assertEqual(est.state, "HEADING_ACTIVE")
        self.assertAlmostEqual(math.degrees(est.yaw), 90, delta=0.5)

    def test_motion_during_stationary_bias_latches(self):
        est = BootYaw()
        e = est.update(100_000_000, 100_000_000,
                       accel=(0.1, 0.1, 9.8),
                       gyro=(0.0, 0.0, 0.4))
        self.assertEqual((e.state, e.reason),
                         ("FAULT_LATCHED", "MOTION_DURING_BIAS_ESTIMATE"))
        self.assertIsNone(est.yaw)

    def test_imu_stamp_gap_and_backwards_latch(self):
        for change in (0, -20_000_000, 250_000_000):
            with self.subTest(change=change):
                est, stamp, mono = samples()
                e = est.update(stamp+change, mono+20_000_000,
                               accel=(0, 0, 9.8),
                               gyro=(0, 0, 0.003))
                self.assertEqual(e.state, "FAULT_LATCHED")
                self.assertEqual(e.reason, "IMU_STAMP_GAP_OR_REGRESSION")

    def test_no_stale_yaw_after_loss_or_reboot(self):
        est, stamp, mono = samples()
        e = est.check_freshness(mono + 400_000_000)
        self.assertEqual((e.state, e.reason), ("FAULT_LATCHED", "IMU_STALE"))
        self.assertIsNone(e.relative_yaw_rad)
        self.assertEqual(est.update(
            stamp+20_000_000, mono+20_000_000,
            accel=(0, 0, 9.8), gyro=(0, 0, 0.003)).state,
            "FAULT_LATCHED")
        self.assertEqual(BootYaw().state, "WAITING_IMU")

    def test_bad_acceleration_and_bad_gyro(self):
        for acc, gyro in (
            ((0, 0, 1), (0, 0, 0)),
            ((0, 0, 9.8), (0, 0, float("nan"))),
            ((0, 0, 9.8), (0, 0, float("inf"))),
        ):
            with self.subTest(acc=acc, gyro=gyro):
                est = BootYaw()
                e = est.update(100_000_000, 100_000_000,
                               accel=acc, gyro=gyro)
                self.assertEqual(e.state, "FAULT_LATCHED")

    def test_yaw_rate_out_of_range_latches(self):
        est, stamp, mono = samples()
        e = est.update(stamp+20_000_000, mono+20_000_000,
                       accel=(0, 0, 9.8),
                       gyro=(0, 0, 2.0))
        self.assertEqual(e.reason, "YAW_RATE_EXCEEDS_ESTIMATOR_ENVELOPE")

    def test_isolation_env_and_visible_devices(self):
        env = dict(live.REQUIRED_ENV)
        self.assertTrue(live.preflight(env, lambda _: False))
        for key in env:
            with self.subTest(key=key):
                bad = dict(env)
                bad.pop(key)
                with self.assertRaisesRegex(RuntimeError, "BOOT_YAW_ENV_"):
                    live.preflight(bad, lambda _: False)
        for path in live.FORBIDDEN_DEVICES:
            with self.subTest(path=path):
                with self.assertRaisesRegex(RuntimeError,
                                            "BOOT_YAW_DEVICE_VISIBLE"):
                    live.preflight(env, lambda p: p == path)

    def test_ros_module_can_only_publish_relative_yaw_status(self):
        src = (ROOT/"live_boot_yaw.py").read_text()
        tree = ast.parse(src)
        pubs = [node for node in ast.walk(tree)
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "create_publisher"]
        self.assertEqual(len(pubs), 1)
        self.assertEqual(ast.unparse(pubs[0].args[1]), "STATUS_TOPIC")
        self.assertIn("EXPECTED_OWNER", src)
        self.assertIn("physical_heading_accuracy_validated", src)
        self.assertIn('"motor_motion_authorized": False', src)
        self.assertNotIn("serial.Serial(", src)
        self.assertNotIn("set_wheels", src)

    def test_yaw_launcher_cannot_get_device_or_motor_access(self):
        shell = (ROOT/"live_boot_yaw.sh").read_text()
        self.assertIn('source=leroy-stage4-imu', shell)
        self.assertIn('--network "container:$parent"', shell)
        self.assertIn('--ipc "container:$parent"', shell)
        self.assertIn('--pid "container:$parent"', shell)
        self.assertIn('--read-only', shell)
        self.assertIn('--cap-drop=all', shell)
        self.assertNotIn("--device", shell)
        self.assertNotIn("--privileged", shell)
        self.assertNotIn("podman stop", shell)


if __name__ == "__main__":
    unittest.main()
