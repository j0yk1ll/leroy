"""No-hardware verification of autonomous stationary gyro calibration."""
import ast
import math
from pathlib import Path
import subprocess
import unittest

from leroy_localization.auto_calibration_core import AutoImuCalibration
from leroy_localization import heading_node as live_boot_yaw

ROOT = Path(__file__).resolve().parents[2]
START = 1_800_000_000_000_000_000
RECEIPT = 5_000_000_000
DT = 20_000_000


def feed(cal, count=411, *, gyro_z=0.003, heading_bias=0.003,
         yaw_end=0.0, accel=(0.0, 0.0, 9.81)):
    latest = None
    for i in range(count):
        z = gyro_z(i) if callable(gyro_z) else gyro_z
        latest = cal.update(
            START + i*DT, RECEIPT + i*DT,
            accel=accel, gyro=(0.0, 0.0, z),
            heading_state="HEADING_ACTIVE",
            heading_yaw_rad=yaw_end * i / max(1, count-1),
            heading_bias_rad_s=heading_bias)
        if latest.state == "FAULT_LATCHED":
            break
    return latest


class AutomaticInertialCalibrationTests(unittest.TestCase):
    def test_never_claims_hardware_motion_or_absolute_heading(self):
        c = AutoImuCalibration()
        s = c.status()
        self.assertEqual((s.state, s.samples),
                         ("WAITING_HEADING", 0))
        self.assertIsNone(s.independent_bias_rad_s)
        self.assertFalse(s.motor_motion_authorized)
        self.assertFalse(s.physical_heading_accuracy_validated)
        self.assertFalse(s.camera_imu_extrinsics_validated)

    def test_realistic_50hz_holdout_auto_verifies_after_eight_seconds(self):
        c = AutoImuCalibration()
        s = feed(c)
        self.assertEqual(s.state, "VERIFIED_STATIONARY_CANDIDATE")
        self.assertEqual(s.reason, "STATIONARY_GYRO_CONSISTENCY_PASSED_ONLY")
        self.assertEqual(s.samples, 401)
        self.assertAlmostEqual(s.window_elapsed_s, 8.0)
        self.assertAlmostEqual(s.independent_bias_rad_s, 0.003)
        self.assertAlmostEqual(s.heading_bias_rad_s, 0.003)
        self.assertAlmostEqual(s.gyro_z_noise_std_rad_s, 0)
        self.assertAlmostEqual(s.yaw_drift_deg, 0)
        self.assertFalse(s.motor_motion_authorized)

    def test_gyro_bias_stability_checks_two_independent_halves(self):
        c = AutoImuCalibration()
        s = feed(c, gyro_z=lambda i: 0.0 if i < 200 else 0.020,
                 heading_bias=0.010)
        self.assertEqual(s.state, "FAULT_LATCHED")
        self.assertEqual(s.reason, "STATIONARY_BIAS_OR_YAW_DRIFT_UNSTABLE")
        self.assertIsNone(s.independent_bias_rad_s)

    def test_drift_disqualifies_even_when_gyro_bias_looks_fine(self):
        c = AutoImuCalibration()
        s = feed(c, yaw_end=math.radians(1.0))
        self.assertEqual(s.reason, "STATIONARY_BIAS_OR_YAW_DRIFT_UNSTABLE")
        self.assertEqual(s.state, "FAULT_LATCHED")

    def test_motion_resets_window_then_new_stationary_holdout_passes(self):
        c = AutoImuCalibration()
        for i in range(20):
            c.update(
                START+i*DT, RECEIPT+i*DT,
                accel=(0, 0, 9.81), gyro=(0, 0, 0.003),
                heading_state="HEADING_ACTIVE", heading_yaw_rad=0.0,
                heading_bias_rad_s=0.003)
        s = c.update(
            START+20*DT, RECEIPT+20*DT,
            accel=(0, 0, 9.81), gyro=(0, 0, 0.5),
            heading_state="HEADING_ACTIVE", heading_yaw_rad=0.0,
            heading_bias_rad_s=0.003)
        self.assertEqual(s.state, "WAITING_HEADING")
        self.assertEqual(s.samples, 0)
        self.assertEqual(s.window_restarts, 1)
        self.assertFalse(s.motor_motion_authorized)
        for i in range(21, 434):
            s = c.update(
                START+i*DT, RECEIPT+i*DT,
                accel=(0, 0, 9.81), gyro=(0, 0, 0.003),
                heading_state="HEADING_ACTIVE", heading_yaw_rad=0.0,
                heading_bias_rad_s=0.003)
        self.assertEqual(s.state, "VERIFIED_STATIONARY_CANDIDATE")
        self.assertEqual(s.window_restarts, 1)

    def test_excessive_movement_cannot_certify_calibration(self):
        c = AutoImuCalibration()
        for i in range(6):
            s = c.update(
                START+i*DT, RECEIPT+i*DT,
                accel=(0, 0, 9.81), gyro=(0, 0, 0.5),
                heading_state="HEADING_ACTIVE", heading_yaw_rad=0.0,
                heading_bias_rad_s=0.003)
        self.assertEqual(s.state, "FAULT_LATCHED")
        self.assertEqual(s.reason, "STATIONARY_CALIBRATION_TOO_FRAGMENTED")

    def test_stale_samples_restart_then_backward_timestamp_latches(self):
        c = AutoImuCalibration()
        c.update(START, RECEIPT,
                 accel=(0, 0, 9.81), gyro=(0, 0, 0.003),
                 heading_state="HEADING_ACTIVE", heading_yaw_rad=0.0,
                 heading_bias_rad_s=0.003)
        s = c.on_timeout(RECEIPT + 450_000_000)
        self.assertEqual(s.state, "WAITING_HEADING")
        self.assertEqual(s.samples, 0)
        c.update(START+DT, RECEIPT+DT,
                 accel=(0, 0, 9.81), gyro=(0, 0, 0.003),
                 heading_state="HEADING_ACTIVE", heading_yaw_rad=0.0,
                 heading_bias_rad_s=0.003)
        s = c.update(
            START+DT, RECEIPT+2*DT,
            accel=(0, 0, 9.81), gyro=(0, 0, 0.003),
            heading_state="HEADING_ACTIVE", heading_yaw_rad=0.0,
            heading_bias_rad_s=0.003)
        self.assertEqual(s.reason, "CALIBRATION_TIMESTAMP_REGRESSION")

    def test_live_yaw_publishes_separate_status_not_authority(self):
        s = (ROOT/"src/leroy_localization/leroy_localization/heading_node.py").read_text()
        tree = ast.parse(s)
        pubs = [n for n in ast.walk(tree)
                if isinstance(n, ast.Call)
                and isinstance(n.func, ast.Attribute)
                and n.func.attr == "create_publisher"]
        self.assertEqual(len(pubs), 2)
        self.assertEqual({ast.unparse(n.args[1]) for n in pubs},
                         {"STATUS_TOPIC", "CALIBRATION_TOPIC"})
        self.assertIn('"absolute_heading_validated": False', s)
        self.assertIn('"physical_motion_authorized": False', s)
        self.assertNotIn("serial.Serial(", s)
        self.assertNotIn("set_wheels", s)

    def test_existing_launcher_maps_core_without_devices(self):
        s = (ROOT/"src/leroy_bringup/scripts/heading.sh").read_text()
        subprocess.run(["bash", "-n", str(ROOT/"src/leroy_bringup/scripts/heading.sh")], check=True)
        self.assertIn("src/leroy_localization/leroy_localization/auto_calibration_core.py", s)
        self.assertIn("--read-only", s)
        self.assertNotIn("--device=", s)
        self.assertNotIn("--privileged", s)


if __name__ == "__main__":
    unittest.main()
