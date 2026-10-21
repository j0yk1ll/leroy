"""M23AF offline contract tests, synthetic map lines only, no ROS/Podman."""
import json
from pathlib import Path
import unittest

import m23af_compare as af

ROOT = Path(__file__).resolve().parent
RID = "a" * 32


def fixture(*, pid=32, with_shutdown=True):
    def phase(name, p):
        return "\n".join((
            f"[component_container-1] M23AF_MAP_BEGIN phase={name} run_id={RID} pid={p}",
            f"[component_container-1] M23AF_MAP_ENTRY phase={name} run_id={RID} pid={p} text="
            "7f1234000000-7f1234010000 r-xp 00001000 00:45 12345 "
            "/opt/ros/jazzy/lib/librectify.so",
            f"[component_container-1] M23AF_MAP_END phase={name} run_id={RID} pid={p} count=1",
        ))
    out = phase("post_factory", pid)
    if with_shutdown:
        out += "\n" + phase("on_shutdown", pid)
    return out + "\n"


def inherited():
    base = {
        "arm": "component", "child_exit": -11, "parent_exit": 2,
        "counts": {"raw": 900, "rect": 900, "info": 900},
        "verdict": "SIGSEGV_STOCK_RECTIFIER_900_OF_900",
        "graph_verified": True, "stock_rectifier": True,
        "synthetic_only": True, "production_acceptance": "BLOCKED",
    }
    hook = {
        "policy": "control", "factory_hook_verified_before_frames": True,
        "inherited_m23aa_exit": 2, "diagnostic_verdict": "INHERITED_RUN_COMPLETE",
        "root_cause_proven": False,
    }
    return ("M23AA_RESULT_JSON=" + json.dumps(base) + "\n"
            + "M23AC_RESULT_JSON=" + json.dumps(hook) + "\n")


class M23AFSourceTests(unittest.TestCase):
    def test_exact_two_snapshots(self):
        result = af.parse_maps(fixture(), RID)
        self.assertEqual(result["pid_in_container"], 32)
        self.assertEqual(tuple(result["snapshots"]), af.PHASES)
        self.assertEqual(result["snapshots"]["on_shutdown"][0]["file_offset"], 4096)

    def test_wrong_nonce_rejected(self):
        with self.assertRaises(ValueError):
            af.parse_maps(fixture(), "b" * 32)

    def test_duplicate_or_incomplete_phase_rejected(self):
        with self.assertRaises(ValueError):
            af.parse_maps(fixture() + fixture(), RID)
        with self.assertRaises(ValueError):
            af.parse_maps(fixture(with_shutdown=False), RID)

    def test_reversed_phase_order_rejected(self):
        chunks = fixture().strip().split("\n")
        self.assertEqual(len(chunks), 6)
        reversed_log = "\n".join(chunks[3:] + chunks[:3]) + "\n"
        with self.assertRaises(ValueError):
            af.parse_maps(reversed_log, RID)

    def test_cross_pid_or_map_count_rejected(self):
        with self.assertRaises(ValueError):
            af.parse_maps(fixture().replace("pid=32", "pid=33", 1), RID)
        with self.assertRaises(ValueError):
            af.parse_maps(fixture().replace("count=1", "count=2"), RID)

    def test_absent_rectifier_or_error_rejected(self):
        with self.assertRaises(ValueError):
            af.parse_maps(fixture().replace("librectify.so", "libother.so"), RID)
        with self.assertRaises(ValueError):
            af.parse_maps(fixture() + "M23AF_MAP_ERROR phase=on_shutdown\n", RID)

    def test_inherited_crashing_control_acceptance(self):
        self.assertTrue(af.validate_inherited_output(inherited(), 2))
        with self.assertRaises(ValueError):
            af.validate_inherited_output(inherited(), 0)
        with self.assertRaises(ValueError):
            af.validate_inherited_output(inherited().replace('"rect": 900', '"rect": 899'), 2)

    def test_cpp_stock_factory_and_shutdown_callback(self):
        source = (ROOT / "m23af_cpp/src/observed_rectify_container.cpp").read_text()
        self.assertIn("ComponentManager::create_component_factory(resource)", source)
        self.assertIn("const int flags = RTLD_NOW | RTLD_NOLOAD;", source)
        self.assertIn("rclcpp::on_shutdown(", source)
        self.assertIn('std::ifstream maps("/proc/self/maps")', source)
        self.assertIn('snapshot_executable_maps("post_factory", run_id_);', source)
        self.assertIn('snapshot_executable_maps("on_shutdown", run_id);', source)
        self.assertNotIn("LD_PRELOAD=", source)

    def test_launch_and_image_contract(self):
        launch = (ROOT / "m23af_rectifier_container.launch.py").read_text()
        docker = (ROOT / "Dockerfile.m23af-shutdown-maps").read_text()
        self.assertIn("m23af_run_id:=", launch)
        self.assertIn("image_proc::RectifyNode", launch)
        self.assertIn("localhost/leroy-sensors:m23ac-loaded-rectify-v1", docker)
        self.assertIn("M23AF_BUILD_CONTRACT=PASS", docker)
        self.assertIn("--packages-select leroy_m23af_shutdown_maps", docker)


if __name__ == "__main__":
    unittest.main()
