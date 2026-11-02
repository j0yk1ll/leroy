"""No-device tests for source arbitration and fail-closed gateway."""
import unittest

from leroy_safety.motion_supervisor import Mode, MotionSupervisor
from leroy_safety.command_gateway import CommandGateway, MotionRequest


class MotionSafetyTests(unittest.TestCase):
    def setUp(self):
        self.supervisor = MotionSupervisor()
        self.gateway = CommandGateway(self.supervisor)
        self.t = 1_000_000_000

    def select(self):
        return self.supervisor.select(
            Mode.AUTONOMOUS, now_ns=self.t,
            local_operator_confirmed=True, physical_stop_verified=True,
            environmental_clearance_verified=True, localization_verified=True)

    def request(self, source="nav2", seq=1, stamp=None):
        return MotionRequest(source, seq, self.t + 1 if stamp is None else stamp,
                             0.04, 0.0, 0.07)

    def test_default_denial(self):
        d = self.gateway.admit(self.request(), now_ns=self.t + 2)
        self.assertFalse(d.accepted)
        self.assertEqual(d.velocity, (0.0, 0.0, 0.0))

    def test_mode_selection_is_not_motor_permission(self):
        d = self.select()
        self.assertEqual(d.mode, Mode.AUTONOMOUS)
        self.assertFalse(d.allowed)

    def test_missing_physical_evidence_rejects_mode(self):
        d = self.supervisor.select(
            Mode.AUTONOMOUS, now_ns=self.t, local_operator_confirmed=True)
        self.assertFalse(d.allowed)
        self.assertEqual(self.supervisor.mode, Mode.DISARMED)

    def test_admit_only_with_independent_evidence(self):
        self.select()
        d = self.gateway.admit(self.request(), now_ns=self.t + 2,
                               prerequisites_verified=True,
                               single_publisher_verified=True)
        self.assertTrue(d.accepted)
        self.assertEqual(d.reason, "ADMITTED_NOT_MOTOR_AUTHORIZED")

    def test_bad_source_fault_latches(self):
        self.select()
        d = self.gateway.admit(self.request("teleop"), now_ns=self.t + 2,
                               prerequisites_verified=True,
                               single_publisher_verified=True)
        self.assertFalse(d.accepted)
        self.assertEqual(self.supervisor.mode, Mode.FAULT)
        self.assertFalse(self.supervisor.reset_fault().allowed)
        self.assertEqual(self.supervisor.mode, Mode.FAULT)
        self.supervisor.reset_fault(local_operator_confirmed=True)
        self.assertEqual(self.supervisor.mode, Mode.DISARMED)

    def test_missing_readiness_faults(self):
        self.select()
        d = self.gateway.admit(self.request(), now_ns=self.t + 2,
                               single_publisher_verified=True)
        self.assertFalse(d.accepted)
        self.assertEqual(self.supervisor.mode, Mode.FAULT)

    def test_replay_faults(self):
        self.select()
        self.gateway.admit(self.request(), now_ns=self.t + 2,
                           prerequisites_verified=True,
                           single_publisher_verified=True)
        d = self.gateway.admit(self.request(), now_ns=self.t + 3,
                               prerequisites_verified=True,
                               single_publisher_verified=True)
        self.assertFalse(d.accepted)
        self.assertEqual(self.supervisor.mode, Mode.FAULT)

    def test_future_or_old_request_faults(self):
        self.select()
        d = self.gateway.admit(self.request(stamp=self.t + 9999),
                               now_ns=self.t + 2, prerequisites_verified=True,
                               single_publisher_verified=True)
        self.assertFalse(d.accepted)
        self.assertEqual(self.supervisor.mode, Mode.FAULT)

    def test_wrong_publisher_count_faults(self):
        self.select()
        d = self.gateway.admit(self.request(), now_ns=self.t + 2,
                               prerequisites_verified=True)
        self.assertFalse(d.accepted)
        self.assertEqual(self.supervisor.mode, Mode.FAULT)


if __name__ == "__main__":
    unittest.main()
