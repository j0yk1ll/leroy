"""Safety extraction and compatibility contract tests (no motor hardware)."""
import unittest

from leroy_safety.health_contract import decode, encode
from leroy_safety.readiness import HealthEvidence, ReadinessAggregator
from leroy_safety.readiness import ReadinessAggregator as LegacyAggregator
from leroy_safety.health_contract import decode as legacy_decode


class SafetyMiddlewareBoundaryTest(unittest.TestCase):
    def test_compatibility_uses_exact_safety_implementation(self):
        self.assertEqual(ReadinessAggregator.__module__, 'leroy_safety.readiness')

    def test_ros_health_cannot_claim_verified(self):
        report = encode("mcu", observed=True, reason="live_stream")
        self.assertFalse(decode(report, "mcu")["verified"])
        self.assertIsNone(decode(report.replace('"verified": false', '"verified": true'), "mcu"))

    def test_observations_cannot_authorize_motion(self):
        agg = ReadinessAggregator(max_age_s=0.5)
        for component in ("sensors", "perception", "localization", "mcu"):
            agg.record(HealthEvidence(component, True, False, 1_000_000_000, component))
        decision = agg.evaluate(now_ns=1_000_000_001)
        self.assertFalse(decision.navigation_prerequisites_met)
        self.assertIn("stop:missing", decision.failure_reasons)
        self.assertFalse(hasattr(decision, "motion_authorized"))

    def test_stale_stop_evidence_fails_closed(self):
        agg = ReadinessAggregator(max_age_s=0.5)
        agg.record(HealthEvidence("stop", True, True, 1_000_000_000, "hardware"))
        self.assertFalse(agg.evaluate(now_ns=1_500_000_001).physical_stop_verified)


if __name__ == "__main__":
    unittest.main()
