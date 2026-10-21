"""M23AO offline tests only. No ROS/Podman/process access."""
from unittest.mock import patch
import unittest

import m23ao_component_resources as m


def stat_line(utime=150, stime=75, starttime=12345):
    # /proc/PID/stat fields 1,2 followed by 3..24; include tricky comm.
    fields = ["S"] + ["0"] * 21
    fields[11], fields[12], fields[19] = str(utime), str(stime), str(starttime)
    return "27 (observed (ROS) container) " + " ".join(fields)


class OfflineResourceTests(unittest.TestCase):
    def test_parse_proc_stat_cpu_and_starttime(self):
        self.assertEqual(m.parse_stat(stat_line()), (12345, 225))

    def test_parse_status_vm_hwm(self):
        self.assertEqual(m.parse_hwm("Name:\tros\nVmHWM:\t 92160 kB\n"), 92160)

    def test_missing_rss_fails(self):
        with self.assertRaisesRegex(m.Blocked, "VmHWM"):
            m.parse_hwm("VmRSS:\t 100 kB\n")

    def test_bad_starttime_fails(self):
        with self.assertRaisesRegex(m.Blocked, "invalid CPU"):
            m.parse_stat(stat_line(starttime=0))

    def test_short_proc_stat_fails(self):
        with self.assertRaisesRegex(m.Blocked, "short proc stat"):
            m.parse_stat("27 (foo) R 1 2")

    def test_cpu_math(self):
        rows = [(0, 100, 81920), (15_000_000_000, 700, 92160),
                (30_000_000_000, 1300, 102400)]
        result = m.compute(rows * 1 if len(rows) >= m.MIN_SAMPLES else
                           [rows[0]] * 24 + [rows[1]] * 25 + [rows[2]], 100)
        self.assertEqual(result["sample_count"], 50)
        self.assertEqual(result["component_process_cpu_seconds"], 12)
        self.assertEqual(result["component_cpu_percent_of_one_logical_core"], 40)
        self.assertEqual(result["component_peak_rss_mib"], 100)

    def test_too_short_rejected(self):
        rows = [(i * 200_000_000, i * 8 + 10, 51200) for i in range(120)]
        # 23.8s satisfies, so deliberately shorten to ~12s.
        rows = [(i * 100_000_000, i * 8 + 10, 51200) for i in range(120)]
        with self.assertRaisesRegex(m.Blocked, "duration"):
            m.compute(rows, 100)

    def test_too_few_samples_rejected(self):
        rows = [(i * 1_000_000_000, i + 10, 51200) for i in range(5)]
        with self.assertRaisesRegex(m.Blocked, "insufficient CPU/RSS"):
            m.compute(rows, 100)

    def test_duplicate_inherited_line_rejected(self):
        with self.assertRaisesRegex(m.Blocked, "duplicate inherited"):
            m.one("M23AK_RUN_ID=abc\nM23AK_RUN_ID=def", "M23AK_RUN_ID=")

    def test_default_no_ros(self):
        with patch.object(m, "run", side_effect=AssertionError("runtime invoked")):
            self.assertEqual(m.main([]), 0)

    def test_same_container_pid_and_nonce_marker(self):
        match = m.MARKER.search(
            "[component_container-1] M23AF_MAP_BEGIN phase=post_factory "
            "run_id=" + "a"*32 + " pid=27")
        self.assertEqual(match.groups(), ("a"*32, "27"))

    def test_no_hardware_tools_in_source(self):
        from pathlib import Path
        content = (Path(__file__).resolve().parent /
                   "m23ao_component_resources.py").read_text()
        self.assertIn("m23an.run()", content)
        self.assertIn("m23af_run_id:=", content)
        self.assertIn('monitor.pid != inherited.get("pid_in_container")', content)
        self.assertNotIn("podman run", content)
        self.assertNotIn("cv2.VideoCapture", content)


if __name__ == "__main__":
    unittest.main()
