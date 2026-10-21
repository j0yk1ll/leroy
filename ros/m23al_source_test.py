"""M23AL offline contract tests. Importing does not execute ROS/Podman."""
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import m23al_repeatability as m


def example_log(nonce="a" * 32, child_exit=0, frames=900):
    counts = {"raw": frames, "rect": frames, "info": frames}
    maps = {
        phase: {"observed_opencv_prefix": m.PREFIX,
                "one_tbb_executable_mappings": 0,
                "rectifier_library_mapped": True,
                "opencv_modules": sorted(m.MODULES),
                "executable_mapping_count": 110}
        for phase in ("post_factory", "on_shutdown")
    }
    base = {"arm": "component",
            "verdict": "CLEAN_STOCK_RECTIFIER_900_OF_900",
            "parent_exit": 0, "child_exit": child_exit,
            "counts": counts, "graph_verified": True,
            "stock_rectifier": True, "synthetic_only": True,
            "production_acceptance": "BLOCKED"}
    hook = {"policy": "control", "factory_hook_verified_before_frames": True,
            "inherited_m23aa_exit": 0,
            "diagnostic_verdict": "INHERITED_RUN_COMPLETE"}
    result = {
        "verdict": "TBB_FREE_PTHREADS_COMPOSABLE_SYNTHETIC_CLEAN",
        "run_id": nonce, "pid_in_container": 27,
        "child_exit": child_exit, "parent_exit": 0,
        "inherited_900_frame_counts": counts,
        "stock_composable_rectifier": True,
        "factory_policy": "control", "opencv_prefix": m.PREFIX,
        "tbb_absent_from_two_executable_map_snapshots": True,
        "opencv_mapping_audit": maps,
        "physical_device_validation": "NONE",
        "production_acceptance": "BLOCKED",
    }
    parts = [
        "M23AK_RUN_ID=" + nonce,
        "M23AA_VERDICT=CLEAN_STOCK_RECTIFIER_900_OF_900",
        "M23AK_VERDICT=TBB_FREE_PTHREADS_COMPOSABLE_SYNTHETIC_CLEAN",
        "M23AK_DEVICE_ACCESS=NONE",
        "M23AK_PRODUCTION_ACCEPTANCE=BLOCKED",
        "M23AA_RESULT_JSON=" + json.dumps(base),
        "M23AC_RESULT_JSON=" + json.dumps(hook),
        "M23AK_RESULT_JSON=" + json.dumps(result),
    ]
    return "\n".join(parts)


class OfflineTests(unittest.TestCase):
    def test_accept_full_mock_results(self):
        self.assertEqual(m.validate(example_log(), 0, set()), "a"*32)

    def test_reject_duplicate_nonce(self):
        with self.assertRaisesRegex(m.Blocked, "duplicate nonce"):
            m.validate(example_log(), 0, {"a"*32})

    def test_reject_nonzero_exit(self):
        with self.assertRaisesRegex(m.Blocked, "wrapper exit"):
            m.validate(example_log(), 2, set())

    def test_reject_child_crash(self):
        with self.assertRaisesRegex(m.Blocked, "frame, child"):
            m.validate(example_log(child_exit=-11), 0, set())

    def test_reject_frame_loss(self):
        with self.assertRaisesRegex(m.Blocked, "frame, child"):
            m.validate(example_log(frames=899), 0, set())

    def test_reject_duplicate_verdict_marker(self):
        value = example_log()
        value += "\nM23AK_VERDICT=TBB_FREE_PTHREADS_COMPOSABLE_SYNTHETIC_CLEAN"
        with self.assertRaisesRegex(m.Blocked, "exactly one"):
            m.validate(value, 0, set())

    def test_reject_mixed_opencv(self):
        value = example_log().replace(
            '"observed_opencv_prefix": "/opt/m23aj-opencv/lib/"',
            '"observed_opencv_prefix": "/usr/lib/x86_64-linux-gnu/"', 1)
        with self.assertRaisesRegex(m.Blocked, "mapping"):
            m.validate(value, 0, set())

    def test_reject_tbb_mapping(self):
        value = example_log().replace('"one_tbb_executable_mappings": 0',
                                     '"one_tbb_executable_mappings": 1', 1)
        with self.assertRaisesRegex(m.Blocked, "mapping"):
            m.validate(value, 0, set())

    def test_reject_incomplete_mapping(self):
        value = example_log().replace('"on_shutdown":', '"wrong_phase":', 1)
        with self.assertRaisesRegex(m.Blocked, "snapshots"):
            m.validate(value, 0, set())

    def test_default_no_command_execution(self):
        with patch.object(m, "run", side_effect=AssertionError("runtime called")):
            self.assertEqual(m.main([]), 0)

    def test_exactly_five(self):
        self.assertEqual(m.COUNT, 5)

    def test_runtime_security_contract(self):
        src = (Path(__file__).resolve().parent /
               "m23al_repeatability.py").read_text()
        for required in (
            '"--network=none"', '"--read-only"', '"--pull=never"',
            '"--cap-drop=all"', '"--rm"', 'notmpcopyup',
            'M23P_SYNTHETIC_ONLY=1', 'ROS_DOMAIN_ID=170',
            'M23AK_ARM=pthreads', 'M23AA_VERDICT=',
            'M23AK_RESULT_JSON=', 'M23AL_STOP=NO_MORE_TRIALS',
            '"xb"', 'if STATE.exists() or STATE.is_symlink():',
        ):
            self.assertIn(required, src)
        self.assertNotIn('"--device"', src)
        self.assertNotIn('"--privileged"', src)


if __name__ == "__main__":
    unittest.main()
