"""M23AK source-only tests: synthetic map dictionaries; no ROS/Podman."""
import json
from pathlib import Path
import unittest

from m23ak_maps_gate import Blocked, PHASES, REQUIRED, inspect_maps

PREFIX = "/opt/m23aj-opencv/lib/"


def fixtures():
    recs = [{"path": "/opt/ros/jazzy/lib/librectify.so"}]
    recs += [{"path": PREFIX + "libopencv_" + m + ".so.4.6.0"}
             for m in sorted(REQUIRED)]
    return {
        "run_id": "0" * 32,
        "pid_in_container": 42,
        "snapshots": {p: [dict(r) for r in recs] for p in PHASES},
    }


class MapsGateTests(unittest.TestCase):
    def test_accept_synthetic_clean_maps(self):
        result = inspect_maps(fixtures())
        self.assertEqual(set(result), set(PHASES))
        self.assertEqual(result["post_factory"]["one_tbb_executable_mappings"], 0)

    def test_reject_tbb_core(self):
        data = fixtures()
        data["snapshots"]["post_factory"].append(
            {"path": "/usr/lib/x86_64-linux-gnu/libtbb.so.12.11"})
        with self.assertRaisesRegex(Blocked, "oneTBB"):
            inspect_maps(data)

    def test_reject_tbb_malloc(self):
        data = fixtures()
        data["snapshots"]["on_shutdown"].append(
            {"path": "/usr/lib/x86_64-linux-gnu/libtbbmalloc.so.2"})
        with self.assertRaisesRegex(Blocked, "oneTBB"):
            inspect_maps(data)

    def test_reject_mixed_distro_opencv(self):
        data = fixtures()
        data["snapshots"]["post_factory"].append(
            {"path": "/usr/lib/x86_64-linux-gnu/libopencv_core.so.4.6.0"})
        with self.assertRaisesRegex(Blocked, "foreign OpenCV"):
            inspect_maps(data)

    def test_reject_missing_required_module(self):
        data = fixtures()
        for p in PHASES:
            data["snapshots"][p] = [
                r for r in data["snapshots"][p]
                if "libopencv_flann.so" not in r["path"]
            ]
        with self.assertRaisesRegex(Blocked, "required alternate"):
            inspect_maps(data)

    def test_reject_incomplete_mapping_phase(self):
        data = fixtures()
        del data["snapshots"]["on_shutdown"]
        with self.assertRaisesRegex(Blocked, "both M23AF"):
            inspect_maps(data)

    def test_reject_missing_rectifier(self):
        data = fixtures()
        for phase in PHASES:
            data["snapshots"][phase] = [
                r for r in data["snapshots"][phase]
                if "librectify" not in r["path"]]
        with self.assertRaisesRegex(Blocked, "rectifier"):
            inspect_maps(data)

    def test_reject_foreign_opencv_abi(self):
        data = fixtures()
        data["snapshots"]["on_shutdown"].append(
            {"path": PREFIX + "libopencv_video.so.4.9.0"})
        with self.assertRaisesRegex(Blocked, "ABI"):
            inspect_maps(data)

    def test_source_only_runner_contract(self):
        src = (Path(__file__).resolve().parent /
               "m23ak_composable_compare.py").read_text()
        for required in (
            'm23ac.run()', 'm23af.parse_maps(log, nonce)',
            'validate_clean(buffer.getvalue(), status)',
            '"M23AA_RESULT_JSON="', '"M23AC_RESULT_JSON="',
            '"child_exit") == 0', '"rect": 900',
            'inspect_maps(parsed)', "M23AK_PRODUCTION_ACCEPTANCE=BLOCKED",
        ):
            self.assertIn(required, src)
        self.assertNotIn("LD_PRELOAD=", src)


if __name__ == "__main__":
    unittest.main()
