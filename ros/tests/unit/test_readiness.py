"""No-device tests for readiness evidence and fail-closed transitions."""
import unittest

from leroy_bringup.readiness import HealthEvidence, ReadinessAggregator, COMPONENTS


class ReadinessTest(unittest.TestCase):
    def setUp(self):
        self.agg = ReadinessAggregator(max_age_s=0.5)
        self.t = 1_000_000_000

    def report(self, component, *, stamp=None, healthy=True, verified=True,
               source=None):
        self.agg.record(HealthEvidence(
            component, healthy, verified,
            self.t if stamp is None else stamp, source or component + "_node"))

    def test_missing_evidence_never_ready(self):
        state = self.agg.evaluate(now_ns=self.t)
        self.assertFalse(state.navigation_prerequisites_met)
        self.assertEqual(len(state.failure_reasons), len(COMPONENTS))

    def test_fresh_verified_evidence_is_only_a_prerequisite(self):
        for name in COMPONENTS:
            self.report(name)
        state = self.agg.evaluate(now_ns=self.t + 1)
        self.assertTrue(state.navigation_prerequisites_met)
        self.assertTrue(state.physical_stop_verified)
        self.assertEqual(state.failure_reasons, ())
        self.assertFalse(hasattr(state, "motor_motion_authorized"))

    def test_evidence_stales_without_a_callback(self):
        for name in COMPONENTS:
            self.report(name)
        state = self.agg.evaluate(now_ns=self.t + 500_000_001)
        self.assertFalse(state.navigation_prerequisites_met)
        self.assertIn("stop:stale_or_future", state.failure_reasons)

    def test_unverified_and_unhealthy_are_rejected(self):
        self.report("sensors", verified=False)
        self.report("mcu", healthy=False)
        state = self.agg.evaluate(now_ns=self.t)
        self.assertIn("sensors:unverified", state.failure_reasons)
        self.assertIn("mcu:unhealthy", state.failure_reasons)

    def test_future_timestamp_is_not_trusted(self):
        self.report("sensors", stamp=self.t + 1)
        self.assertFalse(self.agg.evaluate(now_ns=self.t).sensors_healthy)

    def test_duplicate_or_changed_source_latches_fault(self):
        self.report("sensors")
        self.report("sensors", source="second_owner", stamp=self.t + 1)
        self.assertIn("sensors:provenance_fault",
                      self.agg.evaluate(now_ns=self.t + 1).failure_reasons)
        self.report("sensors", stamp=self.t + 2)
        self.assertFalse(self.agg.evaluate(now_ns=self.t + 2).sensors_healthy)
        self.agg.reset()
        self.assertIn("sensors:missing",
                      self.agg.evaluate(now_ns=self.t + 2).failure_reasons)

    def test_replayed_timestamp_latches_fault(self):
        self.report("sensors")
        self.report("sensors")
        self.assertIn("sensors:provenance_fault",
                      self.agg.evaluate(now_ns=self.t).failure_reasons)

    def test_strict_types_and_provenance(self):
        with self.assertRaises(TypeError):
            self.agg.record({"component": "sensors"})
        with self.assertRaises(TypeError):
            self.report("sensors", healthy=1)
        with self.assertRaises(ValueError):
            self.report("unknown")
        with self.assertRaises(ValueError):
            self.report("sensors", source=" ")
        with self.assertRaises(ValueError):
            ReadinessAggregator(max_age_s=float("nan"))
        with self.assertRaises(ValueError):
            self.agg.evaluate(now_ns=-1)


if __name__ == "__main__":
    unittest.main()
