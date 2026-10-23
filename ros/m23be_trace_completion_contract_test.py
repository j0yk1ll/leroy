"""M23BE synthetic real-trace format closure contracts, no ROS or devices."""
import contextlib
import io
from pathlib import Path
import tempfile
import unittest

import m23be_trace_completion_contract as m
import m23ba_event_correlation as ba


STAMPS = (1_700_000_000_000_000_000,
          1_700_000_000_033_333_333,
          1_700_000_000_066_666_666)


def pub(i, stamp):
    return ("[usb_cam_node_exe-1] M23AX_PUBLISH pub_index=" + str(i)
            + " fresh=1 status=mmap_dequeued dequeue_count=" + str(i)
            + " v4l2_seq=" + str(i)
            + " v4l2_timestamp_us=" + str(i * 33333)
            + " ros_stamp_ns=" + str(stamp))


def camera(*, stamps=STAMPS, epilogue=True, cam_clean=True, rect_clean=True):
    lines = [pub(i, st) for i, st in enumerate(stamps, 1)]
    if epilogue:
        lines.append("[usb_cam_node_exe-1] M23BE_PUBLISHER_FINAL "
                     "attempts=" + str(len(stamps))
                     + " publish_returns=" + str(len(stamps))
                     + " shutdown=clean")
    if cam_clean:
        lines.append("[INFO] [usb_cam_node_exe-1]: process has finished cleanly")
    if rect_clean:
        lines.append("[INFO] [component_container-2]: process has finished cleanly")
    return "\n".join(lines) + "\n"


def observer(*, raw=STAMPS, rect=STAMPS, info=STAMPS):
    lines = ["M23M_COUNTER_QOS_PROFILE=reliable_100",
             "M23K_COUNTER_VARIANT=FULL"]
    for kind, stamps in (("raw", raw), ("rect", rect), ("info", info)):
        for i, st in enumerate(stamps):
            lines.append("M23K_SAMPLE " + kind + " " + str(st)
                         + " " + str(100_000_000_000 + i * 33_333_333))
    lines.append("M23K_COUNTER_COUNTS="
                 + ",".join(str(len(v)) for v in (raw, rect, info)))
    return "\n".join(lines) + "\n"


class Tests(unittest.TestCase):
    def test_default_inert(self):
        with contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(m.main([]), 0)
        self.assertIn("M23BE_STATUS=INERT_NO_ROS_NO_DEVICE", out.getvalue())

    def test_positive_structural_completion_only(self):
        report = m.assess(camera(), observer())
        self.assertEqual(report["verdict"], "OFFLINE_STRUCTURAL_TRACE_CONTRACT_ONLY")
        self.assertFalse(report["publisher_completion_marker_implemented_in_current_image"])
        self.assertFalse(report["observer_and_publisher_capture_epochs_proven_equal"])
        self.assertEqual(report["production_acceptance"], "BLOCKED")

    def test_existing_instrumentation_without_final_blocks(self):
        with self.assertRaisesRegex(m.Blocked, "authoritative publisher"):
            m.assess(camera(epilogue=False), observer())

    def test_synthetic_persistence_data_never_certified(self):
        import m23bc_synthetic_persistence as synthetic
        with self.assertRaisesRegex(m.Blocked, "authoritative publisher|no M23AX"):
            m.assess(synthetic.CAM.decode(), synthetic.OBS.decode())

    def test_epilogue_attempts_disagree_with_samples(self):
        # Keep both reported summary counters internally consistent, so
        # the mismatched event count (3 entries versus 4 reported) is
        # the ONLY intended failing predicate in this negative fixture.
        src = camera().replace(
            "attempts=3 publish_returns=3",
            "attempts=4 publish_returns=4")
        with self.assertRaisesRegex(m.Blocked, "completion count differs"):
            m.assess(src, observer())

    def test_epilogue_returns_less_than_attempts(self):
        src = camera().replace("publish_returns=3", "publish_returns=2")
        with self.assertRaisesRegex(m.Blocked, "completion totals invalid"):
            m.assess(src, observer())

    def test_epilogue_duplicate(self):
        src = camera()
        line = src.splitlines()[3]
        with self.assertRaisesRegex(m.Blocked, "duplicate authoritative"):
            m.assess(src + line + "\n", observer())

    def test_epilogue_malformed(self):
        src = camera().replace("shutdown=clean", "shutdown=crashed")
        with self.assertRaisesRegex(m.Blocked, "malformed publisher"):
            m.assess(src, observer())

    def test_publisher_event_after_final(self):
        src = camera() + pub(4, STAMPS[2] + 33333333) + "\n"
        with self.assertRaisesRegex(m.Blocked, "pre-send event after"):
            m.assess(src, observer())

    def test_camera_child_exit_missing(self):
        with self.assertRaisesRegex(m.Blocked, "usb_cam_node_exe"):
            m.assess(camera(cam_clean=False), observer())

    def test_component_child_exit_missing(self):
        with self.assertRaisesRegex(m.Blocked, "component_container"):
            m.assess(camera(rect_clean=False), observer())

    def test_duplicate_child_exit(self):
        line = "[INFO] [component_container-2]: process has finished cleanly\n"
        with self.assertRaisesRegex(m.Blocked, "duplicated child clean"):
            m.assess(camera() + line, observer())

    def test_ros_launch_crash_despite_other_exits(self):
        with self.assertRaisesRegex(m.Blocked, "process crash"):
            m.assess(camera() + "process has died [component_container-2]\n",
                     observer())

    def test_publisher_metadata_overflow(self):
        with self.assertRaises((m.Blocked, ba.Blocked)):
            m.assess(camera() + "M23AX_FATAL=METADATA_LOG_LIMIT\n", observer())

    def test_publisher_index_gap(self):
        src = camera().replace("pub_index=2", "pub_index=99")
        with self.assertRaisesRegex(ba.Blocked, "index"):
            m.assess(src, observer())

    def test_observer_summary_missing(self):
        src = observer().rsplit("M23K_COUNTER_COUNTS=", 1)[0]
        with self.assertRaisesRegex(m.Blocked, "observer completion"):
            m.assess(camera(), src)

    def test_observer_count_mismatch(self):
        src = observer().replace("M23K_COUNTER_COUNTS=3,3,3",
                                 "M23K_COUNTER_COUNTS=4,3,3")
        with self.assertRaisesRegex(m.Blocked, "observer completion"):
            m.assess(camera(), src)

    def test_wrong_qos(self):
        src = observer().replace("reliable_100", "best_effort_5")
        with self.assertRaisesRegex(m.Blocked, "observer completion"):
            m.assess(camera(), src)

    def test_raw_header_repeat_is_flagged_not_accepted_as_monotonic(self):
        raw = (STAMPS[0], STAMPS[1], STAMPS[1])
        report = m.assess(camera(), observer(raw=raw))
        self.assertEqual(report["raw_nonpositive_header_intervals"], 1)
        self.assertFalse(report["data_continuity_or_stamp_monotonicity_accepted"])

    def test_missing_half_cli(self):
        with contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(m.main(["--camera-log", "/does-not-exist"]), 2)
        self.assertIn("M23BE_BLOCKED=", out.getvalue())

    def test_same_file_cli(self):
        with tempfile.TemporaryDirectory() as d:
            f = Path(d) / "trace"
            f.write_text(camera())
            with contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(m.main(["--camera-log", str(f),
                                         "--observer-log", str(f)]), 2)
        self.assertIn("trace files must differ", out.getvalue())

    def test_symlink_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "trace"
            p.write_text(camera())
            link = Path(d) / "link"
            link.symlink_to(p)
            with self.assertRaisesRegex(m.Blocked, "non-symlink"):
                m.read_source(link)

    def test_oversize_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "huge"
            with p.open("wb") as f:
                f.truncate(m.MAX_BYTES + 1)
            with self.assertRaisesRegex(m.Blocked, "exceeds"):
                m.read_source(p)

    def test_end_to_end_two_existing_text_files(self):
        with tempfile.TemporaryDirectory() as d:
            cam, obs = Path(d) / "cam.log", Path(d) / "obs.log"
            cam.write_text(camera())
            obs.write_text(observer())
            with contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(m.main(["--camera-log", str(cam),
                                         "--observer-log", str(obs)]), 0)
            self.assertIn("M23BE_VERDICT=OFFLINE_STRUCTURAL_TRACE_CONTRACT_ONLY",
                          out.getvalue())
            self.assertIn("M23BE_CAMERA_RETEST_AUTHORIZED=FALSE",
                          out.getvalue())


if __name__ == "__main__":
    unittest.main()
