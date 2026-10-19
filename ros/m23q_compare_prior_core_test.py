"""Offline M23Q prior-core comparison: synthetic metadata only, no core export."""
import unittest
from pathlib import Path

from m23q_compare_prior_core import (
    PRIOR_PC, PRIOR_NEARBY, LATEST_PC, LATEST_NEARBY, summarize,
)


class PriorCoreCompareContracts(unittest.TestCase):
    def test_export_script_uses_real_shell_parameter_expansions(self):
        script = Path(__file__).with_name("m23q_extract_prior_core.sh")
        source = script.read_text()
        self.assertIn('BASE="${HOME}/.local/state/leroy-m23q"', source)
        self.assertIn('"${BASH_SOURCE[0]}"', source)
        self.assertNotIn(r'\${', source)
        self.assertIn('coredumpctl --no-pager info "$PID"', source)
        self.assertIn('coredumpctl --no-pager --output="$CORE" dump "$PID"', source)

    def test_recurring_relative_address_signature(self):
        self.assertEqual(PRIOR_PC & 0xFFF, LATEST_PC & 0xFFF)
        self.assertEqual(PRIOR_NEARBY - PRIOR_PC, 0x801A)
        self.assertEqual(LATEST_NEARBY - LATEST_PC, 0x801A)
        out = summarize(1, [], [], [(11, 0, 1, PRIOR_PC)])
        self.assertIn("M23Q_CROSS_CORE_PAGE_OFFSET_MATCH=TRUE", out)
        self.assertIn("M23Q_CROSS_CORE_RELATIVE_FRAME_DELTA_MATCH=TRUE", out)

    def test_prior_unmapped_fault_can_be_supported_independently(self):
        out = summarize(1, [], [], [(11, 0, 1, PRIOR_PC)])
        self.assertIn("M23Q_PRIOR_SIGINFO_EQUALS_PC=TRUE", out)
        self.assertIn("M23Q_PRIOR_UNMAPPED_EXECUTION_HYPOTHESIS=SUPPORTED", out)
        self.assertIn("M23Q_PRIOR_CAUSE_VERIFIED=FALSE", out)

    def test_different_signal_fault_address_never_claims_unmapped_execution(self):
        out = summarize(1, [], [], [(11, 0, 1, PRIOR_PC + 8)])
        self.assertIn("M23Q_PRIOR_SIGINFO_EQUALS_PC=FALSE", out)
        self.assertIn(
            "M23Q_PRIOR_UNMAPPED_EXECUTION_HYPOTHESIS=NOT_ESTABLISHED", out)

    def test_mapped_prior_pc_never_claims_unmapped_execution(self):
        mapped = [(PRIOR_PC - 4096, PRIOR_PC + 4096,
                   0, "/opt/ros/jazzy/lib/libexample.so")]
        out = summarize(1, mapped, [], [(11, 0, 1, PRIOR_PC)])
        self.assertIn("M23Q_PRIOR_PC_FILE_MATCHES=1", out)
        self.assertIn(
            "M23Q_PRIOR_UNMAPPED_EXECUTION_HYPOTHESIS=NOT_ESTABLISHED", out)

    def test_missing_siginfo_remains_unknown(self):
        out = summarize(1, [], [], [])
        self.assertIn("M23Q_PRIOR_SIGINFO_FAULT_ADDRESS=UNAVAILABLE", out)
        self.assertIn("M23Q_PRIOR_UNMAPPED_EXECUTION_HYPOTHESIS=UNKNOWN", out)

    def test_different_signal_or_si_code_does_not_establish_mechanism(self):
        for record in ((6, 0, 1, PRIOR_PC), (11, 0, 2, PRIOR_PC)):
            with self.subTest(record=record):
                out = summarize(1, [], [], [record])
                self.assertIn(
                    "M23Q_PRIOR_UNMAPPED_EXECUTION_HYPOTHESIS=NOT_ESTABLISHED",
                    out)


if __name__ == "__main__":
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(
        PriorCoreCompareContracts)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    if result.wasSuccessful():
        print("M23Q_PRIOR_CORE_COMPARISON_CONTRACT=PASS")
        print("M23Q_DEVICE_ACCESS=NONE")
        raise SystemExit(0)
    raise SystemExit(2)
