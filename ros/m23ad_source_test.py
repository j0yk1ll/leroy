"""Offline-only M23AD runner validation; no Podman/ROS invoked."""
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("m23ad_repeat", ROOT / "m23ad_repeat.py")
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def synthetic(policy):
    import json
    code = -11 if policy == "control" else 0
    parent = 2 if policy == "control" else 0
    verdict = "SIGSEGV_STOCK_RECTIFIER_900_OF_900" if policy == "control" else "CLEAN_STOCK_RECTIFIER_900_OF_900"
    base = {"arm": "component", "child_exit": code, "parent_exit": parent,
            "verdict": verdict, "counts": {"raw": 900, "rect": 900, "info": 900},
            "graph_verified": True, "stock_rectifier": True,
            "synthetic_only": True, "production_acceptance": "BLOCKED"}
    hook = {"policy": policy, "factory_hook_verified_before_frames": True,
            "diagnostic_verdict": "INHERITED_RUN_COMPLETE",
            "inherited_m23aa_exit": parent, "production_acceptance": "BLOCKED",
            "root_cause_proven": False}
    return "\n".join(("M23AA_RESULT_JSON=" + json.dumps(base),
                      "M23AA_VERDICT=" + verdict,
                      "M23AC_RESULT_JSON=" + json.dumps(hook),
                      "M23AC_DEVICE_ACCESS=NONE",
                      "M23AC_PRODUCTION_ACCEPTANCE=BLOCKED")) + "\n"


class SourceTests(unittest.TestCase):
    def test_fixed_abba_and_identity(self):
        self.assertEqual(runner.TRIALS, (("01", "control"), ("02", "nodelete"),
                                         ("03", "nodelete"), ("04", "control")))
        self.assertEqual(len(runner.PINNED_IMAGE), 64)
        self.assertTrue(all(c in "0123456789abcdef" for c in runner.PINNED_IMAGE))

    def test_good_both_policies(self):
        for policy, rc in (("control", 2), ("nodelete", 0)):
            self.assertEqual(runner.validate(synthetic(policy), policy, rc)["exit"], rc)

    def test_reject_wrong_outcome_and_policy(self):
        with self.assertRaises(runner.EvidenceError):
            runner.validate(synthetic("nodelete"), "control", 0)
        with self.assertRaises(runner.EvidenceError):
            runner.validate(synthetic("control"), "nodelete", 2)

    def test_reject_duplicate_or_missing_markers(self):
        valid = synthetic("control")
        with self.assertRaises(runner.EvidenceError):
            runner.validate(valid + valid, "control", 2)
        with self.assertRaises(runner.EvidenceError):
            runner.validate(valid.replace("M23AC_DEVICE_ACCESS=NONE", ""), "control", 2)

    def test_reject_missing_delivery_or_hook(self):
        valid = synthetic("nodelete")
        with self.assertRaises(runner.EvidenceError):
            runner.validate(valid.replace('"rect": 900', '"rect": 899'), "nodelete", 0)
        with self.assertRaises(runner.EvidenceError):
            runner.validate(valid.replace('"factory_hook_verified_before_frames": true',
                                          '"factory_hook_verified_before_frames": false'),
                            "nodelete", 0)

    def test_default_does_not_run(self):
        self.assertEqual(runner.main([]), 0)


if __name__ == "__main__":
    unittest.main()
