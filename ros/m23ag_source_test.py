"""M23AG source-only tests. No coredumpctl, ROS, Podman, or core export."""
import json
import unittest
from unittest.mock import patch

import m23ag_core_join as ag


def metadata(uid="1000 (kevin)", gid="1000 (kevin)", run_id=ag.RUN_ID):
    return "\n".join([
        "           PID: 32869 (component_conta)",
        "           UID: " + uid,
        "           GID: " + gid,
        "     Timestamp: Wed 2026-10-21 12:45:08 CEST",
        "        Signal: 11 (SEGV) si_code: SEGV_MAPERR",
        "    Executable: " + ag.EXECUTABLE,
        "  Command Line: " + ag.EXECUTABLE
        + " --ros-args -p m23af_run_id:=" + run_id,
        "       Storage: /var/lib/systemd/coredump/core.test (present)",
    ])


def snapshots(pc=0x7f56a0000f5b, include=True):
    hit = {
        "start": pc - 0xf5b, "end": pc - 0xf5b + 0x4000,
        "file_offset": 4096,
        "path": "/opt/ros/jazzy/lib/librectify.so",
        "permissions": "r-xp",
    }
    other = {
        "start": pc + 0x10000, "end": pc + 0x12000,
        "file_offset": 4096,
        "path": "/opt/ros/jazzy/lib/librectify.so",
        "permissions": "r-xp",
    }
    return {
        "post_factory": [hit if include else other],
        "on_shutdown": [hit if include else other],
    }


def prior_log():
    base = {
        "child_exit": -11, "parent_exit": 2,
        "counts": {"raw": 900, "rect": 900, "info": 900},
        "graph_verified": True, "synthetic_only": True,
        "verdict": "SIGSEGV_STOCK_RECTIFIER_900_OF_900",
    }
    af = {
        "run_id": ag.RUN_ID, "pid_in_container": 34,
        "control_child_exit": -11,
        "mapping_phases": ["post_factory", "on_shutdown"],
        "executable_mappings": {"post_factory": 1, "on_shutdown": 1},
        "same_process_core_correlated": False,
        "faulting_dso_identified": False,
        "production_acceptance": "BLOCKED",
    }
    ac = {"factory_hook_verified_before_frames": True,
          "policy": "control"}
    parts = [
        "M23AF_RUN_ID=" + ag.RUN_ID,
        "M23AA_RESULT_JSON=" + json.dumps(base),
        "M23AF_RESULT_JSON=" + json.dumps(af),
        "M23AC_RESULT_JSON=" + json.dumps(ac),
        "M23AF_VERDICT=CRASH_REPRODUCED_WITH_SAME_PROCESS_PRE_TEARDOWN_MAPS",
        "M23AA_VERDICT=SIGSEGV_STOCK_RECTIFIER_900_OF_900",
        "M23AF_DEVICE_ACCESS=NONE",
        "M23AF_PRODUCTION_ACCEPTANCE=BLOCKED",
    ]
    for phase, values in snapshots().items():
        parts.append("M23AF_MAP_SNAPSHOT_JSON=" + json.dumps({
            "phase": phase, "run_id": ag.RUN_ID, "pid_in_container": 34,
            "segments": values,
        }))
    return "\n".join(parts)


class M23AGSourceOnly(unittest.TestCase):
    def test_numeric_uid_formats(self):
        self.assertEqual(ag.numeric_id("1000"), 1000)
        self.assertEqual(ag.numeric_id("1000 (kevin)"), 1000)
        self.assertIsNone(ag.numeric_id("1000 (kevin) (extra)"))
        self.assertIsNone(ag.numeric_id("1000x"))

    def test_metadata_accepts_verified_format(self):
        got = ag.parse_metadata(metadata())
        self.assertTrue(got["identity_verified"])
        self.assertEqual(got["host_pid"], 32869)

    def test_metadata_rejects_mismatch(self):
        for value in (
            metadata(uid="0 (root)"),
            metadata(gid="0"),
            metadata(run_id="b" * 32),
            metadata(run_id=ag.RUN_ID + "f"),
            metadata().replace("SEGV_MAPERR", "SEGV_ACCERR").replace(
                "11 (SEGV)", "6 (ABRT)"),
        ):
            with self.subTest(value=value[:80]):
                with self.assertRaises(ag.Stop):
                    ag.parse_metadata(value)

    def test_duplicate_metadata_key_rejected(self):
        with self.assertRaises(ag.Stop):
            ag.parse_metadata(metadata() + "\nUID: 1000")

    def test_accepted_control_and_maps(self):
        maps = ag.parse_existing_log(prior_log())
        self.assertEqual(set(maps), set(ag.PHASES))
        self.assertEqual(maps["on_shutdown"][0]["path"],
                         "/opt/ros/jazzy/lib/librectify.so")

    def test_reject_wrong_nonce_or_missing_map(self):
        with self.assertRaises(ag.Stop):
            ag.parse_existing_log(prior_log().replace(
                '"pid_in_container": 34', '"pid_in_container": 35', 1))
        with self.assertRaises(ag.Stop):
            ag.parse_existing_log(prior_log().split(
                "\nM23AF_MAP_SNAPSHOT_JSON=")[0])

    def test_core_rip_join_present_at_both_snapshots(self):
        pc = 0x7f56a0000f5b
        with patch.object(ag, "inspect_elf", return_value=(
            1, [], [], [(11, 0, 1, pc)])), patch.object(
                ag, "prstatus_threads",
                return_value=[{"tid": 42, "rip": pc}]):
            result = ag.join_core("unused-synthetic", snapshots(pc))
        self.assertEqual(result["map_correspondence"],
                         "MATCHED_SHUTDOWN_AND_FACTORY")
        self.assertEqual(result["prior_mapping_file_offset"], 4096 + 0xf5b)
        self.assertFalse(result["root_cause_proven"])
        self.assertFalse(result["actual_dlclose_unmap_observed"])

    def test_core_rip_join_without_prior_mapping(self):
        pc = 0x7f56a0000f5b
        with patch.object(ag, "inspect_elf", return_value=(
            1, [], [], [(11, 0, 1, pc)])), patch.object(
                ag, "prstatus_threads",
                return_value=[{"tid": 42, "rip": pc}]):
            result = ag.join_core("unused-synthetic", snapshots(pc, include=False))
        self.assertEqual(result["map_correspondence"],
                         "NO_PRIOR_EXECUTABLE_MAP_MATCH")
        self.assertIsNone(result["prior_mapping_path"])

    def test_core_join_rejects_mapped_fault_or_wrong_signal(self):
        pc = 0x7f56a0000f5b
        with patch.object(ag, "inspect_elf", return_value=(
            1, [(pc - 2, pc + 3, 0, "still-loaded")],
            [], [(11, 0, 1, pc)])), patch.object(
                ag, "prstatus_threads",
                return_value=[{"tid": 42, "rip": pc}]):
            with self.assertRaises(ag.Stop):
                ag.join_core("unused-synthetic", snapshots(pc))
        with patch.object(ag, "inspect_elf", return_value=(
            1, [], [], [(11, 0, 2, pc)])), patch.object(
                ag, "prstatus_threads",
                return_value=[{"tid": 42, "rip": pc}]):
            with self.assertRaises(ag.Stop):
                ag.join_core("unused-synthetic", snapshots(pc))

    def test_default_does_not_access_core(self):
        with patch.object(ag, "host_preflight", side_effect=AssertionError(
                "default must not open existing logs")):
            self.assertEqual(ag.main.__name__, "main")
            # No flags means the CLI skips host_preflight entirely.
            with patch("sys.argv", ["m23ag_core_join.py"]):
                self.assertEqual(ag.main(), 0)


if __name__ == "__main__":
    unittest.main()
