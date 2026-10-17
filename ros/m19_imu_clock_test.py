"""M19 offline policy and clock-audit acceptance tests; no MCU device."""
import ast
import copy
from pathlib import Path
import unittest

from m19_imu_clock_audit import audit_acceptance, DURATION_S, MIN_PUBLISHED


def fixture():
    metrics = {
        "clock_rate_estimate_ready": True,
        "clock_rate_estimate_ppm": -41.5,
        "measured_publication_rate_hz": 50.0,
        "terminal_fault": None,
        "counts": {
            "published_imu_messages": 6000,
            "calibration_withheld": 0,
            "malformed_imu_packets": 0,
            "sequence_gap_samples": 0,
            "sequence_faults": 0,
            "timestamp_faults": 0,
            "stale_or_future_withheld": 0,
            "invalid_ascii_lines": 0,
            "unexpected_serial_lines": 0,
        },
    }
    imu = {
        "accepted": 5999, "invalid": 0, "fresh_valid_receipt": True,
        "publishers": [{
            "node": "m18_imu_source", "namespace": "/",
            "type": "sensor_msgs/msg/Imu",
        }],
    }
    return metrics, imu


class M19AuditContract(unittest.TestCase):
    def test_accept_when_mapper_has_observations(self):
        a, b = fixture()
        self.assertEqual(DURATION_S, 120)
        self.assertGreaterEqual(MIN_PUBLISHED, 5700)
        self.assertTrue(audit_acceptance(
            complete=True, start_ack=True, stop_ack=True,
            metrics=a, imu=b))

    def test_incomplete_without_rate_fit_or_stop_ack(self):
        a, b = fixture()
        a["clock_rate_estimate_ready"] = False
        self.assertFalse(audit_acceptance(
            complete=True, start_ack=True, stop_ack=True,
            metrics=a, imu=b))
        a["clock_rate_estimate_ready"] = True
        self.assertFalse(audit_acceptance(
            complete=True, start_ack=True, stop_ack=False,
            metrics=a, imu=b))

    def test_faults_or_wrong_graph_never_pass(self):
        a, b = fixture()
        for key in ("timestamp_faults", "sequence_gap_samples",
                    "calibration_withheld", "malformed_imu_packets"):
            metrics = copy.deepcopy(a)
            metrics["counts"][key] = 1
            self.assertFalse(audit_acceptance(
                complete=True, start_ack=True, stop_ack=True,
                metrics=metrics, imu=b))
        metrics = copy.deepcopy(a)
        metrics["clock_rate_estimate_ppm"] = 210.0
        self.assertFalse(audit_acceptance(
            complete=True, start_ack=True, stop_ack=True,
            metrics=metrics, imu=b))
        imu = copy.deepcopy(b)
        imu["publishers"][0]["node"] = "leroy_bridge"
        self.assertFalse(audit_acceptance(
            complete=True, start_ack=True, stop_ack=True,
            metrics=a, imu=imu))

    def test_no_new_serial_write_or_motor_ros_authority(self):
        m19 = (Path(__file__).parent / "m19_imu_clock_audit.py").read_text()
        m18 = (Path("/opt/m18") / "m18_imu_stream_source.py").read_text()
        tree = ast.parse(m19)
        self.assertFalse(any(
            isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute)
            and n.func.attr in ("write", "create_publisher",
                                "create_subscription", "create_client")
            for n in ast.walk(tree)))
        self.assertEqual(m19.count(
            "send_only_whitelisted_serial_command(device, START)"), 1)
        self.assertEqual(m19.count(
            "send_only_whitelisted_serial_command(device, STOP)"), 1)
        self.assertIn("if payload not in (START, STOP)", m18)
        self.assertEqual(m18.count("device.write(payload)"), 1)
        self.assertNotIn("leroy_bridge.node", m19)
        self.assertNotIn("cmd_vel", m19)
        self.assertNotIn("keepalive", m19)
        self.assertNotIn("set_wheels", m19)


if __name__ == "__main__":
    unittest.main()
