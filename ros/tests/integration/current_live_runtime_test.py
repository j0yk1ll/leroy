"""Narrow source guard for deprecating inactive bridge placeholders.

This does NOT validate hardware stopping or authorize motor output.
"""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "src/leroy_bridge/config/leroy_bridge.yaml"
BRIDGE = ROOT / "src/leroy_bridge/leroy_bridge/node.py"
MANIFEST = ROOT / "src/leroy_nav2/config/hardware.disabled.yaml"
GUIDE = ROOT / "CURRENT_LIVE_RUNTIME.md"


class CurrentLivePath(unittest.TestCase):
    def test_unused_deprecated_bridge_parameters_not_in_active_yaml(self):
        data = CONFIG.read_text()
        for removed in (
            "motor_gpio_chip:",
            "motor_estop_feedback_offset:",
            "motor_power_feedback_offset:",
            "front_right_encoder_sign:",
            "rear_right_encoder_sign:",
        ):
            with self.subTest(parameter=removed):
                self.assertNotIn(removed, data)

    def test_signed_motor_and_degraded_odom_default_disabled(self):
        data = CONFIG.read_text()
        self.assertIn("motor_motion_key_file: ''", data)
        self.assertIn("motor_operator_key_file: ''", data)
        self.assertIn("degraded_odometry_enabled: false", data)
        self.assertIn("publish_odom_tf: false", data)
        manifest = MANIFEST.read_text()
        self.assertIn("enabled: false", manifest)
        self.assertIn("bridge_command_output_enabled: false", manifest)

    def test_legacy_interlock_still_fails_closed_in_bridge_code(self):
        source = BRIDGE.read_text()
        self.assertIn("PhysicalMotorFeedback", source)
        self.assertIn("self._motor_hardware_release_verified = False", source)
        self.assertIn("'motor_gpio_chip', ''", source)
        self.assertIn("'motor_estop_feedback_offset', -1", source)
        self.assertIn("'motor_power_feedback_offset', -1", source)
        self.assertIn("self._motor_authority = None", source)

    def test_single_root_readme_preserves_motion_inhibition(self):
        source = (ROOT.parent / "README.md").read_text()
        self.assertIn("single README", source)
        self.assertIn("hardware.disabled.yaml", source)
        self.assertIn("motor", source.lower())
        self.assertIn("not an operative deployment path", source)
        self.assertNotIn("The bridge subscribes to absolute", source)

    def test_live_guide_distinguishes_active_from_historical(self):
        guide = GUIDE.read_text()
        self.assertIn("Stage 3", guide)
        self.assertIn("M22", guide)
        self.assertIn("Deprecated assumptions", guide)
        self.assertIn("read-only", guide)
        self.assertIn("STOP", guide.upper())


if __name__ == "__main__":
    unittest.main()
