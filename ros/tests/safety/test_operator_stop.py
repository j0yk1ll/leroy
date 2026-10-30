"""Operator-facing shutdown safety contract (no device or Podman required)."""
import pathlib
import unittest

SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "leroy.sh"


class OperatorShutdownContract(unittest.TestCase):
    def test_entrypoint_exposes_explicit_stop(self):
        text = SCRIPT.read_text()
        self.assertIn("check|build|run|status|stop|help", text)
        self.assertIn("  stop)", text)

    def test_missing_stop_ack_is_not_success(self):
        text = SCRIPT.read_text()
        self.assertIn("BOOT_IMU_SOURCE_STOP_ACK=PASS", text)
        self.assertIn("IMU_STOP_UNVERIFIED_DISCONNECT_POWER", text)
        self.assertIn("STOP_FAILED_OR_IMU_UNVERIFIED", text)

    def test_no_forced_removal_on_failed_stop(self):
        text = SCRIPT.read_text()
        self.assertIn('if [[ "$rc" == 0 ]]; then\n        podman rm "$source"', text)
        self.assertNotIn("podman rm -f \"$source\"", text)


if __name__ == "__main__":
    unittest.main()
