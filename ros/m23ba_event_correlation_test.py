"""M23BA: pure stdlib, synthetic, read-only event-correlator regression tests."""
import contextlib
import io
from pathlib import Path
import tempfile
import unittest

import m23ba_event_correlation as m

STAMPS = (1_700_000_000_000_000_000,
          1_700_000_000_033_333_333,
          1_700_000_000_066_666_666,
          1_700_000_000_099_999_999)


def pub(index, stamp, *, fresh=1, dequeue=None, sequence=None,
        status=None, v4l2_time=None, prefix=""):
    if dequeue is None:
        dequeue = index
    if sequence is None:
        sequence = dequeue
    if status is None:
        status = "mmap_dequeued" if fresh else "mmap_dqbuf_eagain"
    if v4l2_time is None:
        v4l2_time = dequeue * 33333
    return (
        prefix + "M23AX_PUBLISH pub_index=" + str(index) +
        " fresh=" + str(fresh) + " status=" + status +
        " dequeue_count=" + str(dequeue) +
        " v4l2_seq=" + str(sequence) +
        " v4l2_timestamp_us=" + str(v4l2_time) +
        " ros_stamp_ns=" + str(stamp)
    )


def publisher_lines(stamps=STAMPS[:3]):
    return "\n".join(pub(i, stamp) for i, stamp in enumerate(stamps, 1)) + "\n"


def observer_lines(raw=STAMPS[:3], rect=STAMPS[:3], info=STAMPS[:3],
                   qos="reliable_100", full="FULL"):
    lines = [
        "M23M_COUNTER_QOS_PROFILE=" + qos,
        "M23K_COUNTER_VARIANT=" + full,
    ]
    for kind, sequence in (("raw", raw), ("rect", rect), ("info", info)):
        for i, stamp in enumerate(sequence):
            lines.append("M23K_SAMPLE " + kind + " " + str(stamp) +
                         " " + str(200_000_000_000 + i * 33_333_333))
    lines.append("M23K_COUNTER_COUNTS=" +
                 ",".join(str(len(v)) for v in (raw, rect, info)))
    return "\n".join(lines) + "\n"


def publisher_stale():
    return "\n".join([
        pub(1, STAMPS[0], dequeue=1, sequence=11, v4l2_time=10001),
        pub(2, STAMPS[1], dequeue=2, sequence=12, v4l2_time=20002),
        pub(3, STAMPS[1], fresh=0, dequeue=2, sequence=12,
            v4l2_time=20002, status="mmap_dqbuf_eagain"),
        pub(4, STAMPS[2], dequeue=3, sequence=13, v4l2_time=30003),
    ]) + "\n"


class M23BATests(unittest.TestCase):
    def test_default_inert(self):
        with contextlib.redirect_stdout(io.StringIO()) as f:
            self.assertEqual(m.main([]), 0)
        self.assertIn("M23BA_STATUS=INERT_NO_ROS_NO_DEVICE", f.getvalue())

    def test_unique_normal(self):
        result = m.correlate(publisher_lines(), observer_lines())
        self.assertEqual(result["observed_pattern"], "NO_REPEAT_IN_PRESERVED_EVENTS")
        self.assertEqual(result["raw_repeated_stamp_excess"], 0)
        self.assertFalse(result["dds_redelivery_or_reordering_proven"])
        self.assertEqual(result["production_acceptance"], "BLOCKED")

    def test_matching_publisher_stale_and_raw_duplicate(self):
        raw = (STAMPS[0], STAMPS[1], STAMPS[1], STAMPS[2])
        result = m.correlate(publisher_stale(), observer_lines(raw=raw))
        self.assertEqual(result["observed_pattern"],
                         "PRESEND_AND_RAW_REPEATED_STAMP_OBSERVED")
        self.assertEqual(result["pre_send_nonfresh_attempts"], 1)
        self.assertEqual(result["pre_send_nonfresh_indexes"], [3])
        self.assertEqual(result["raw_repeated_stamp_excess"], 1)
        self.assertFalse(result["usb_cam_duplicate_delivered_to_raw_subscriber_proven"])

    def test_raw_duplicate_not_automatically_blamed_on_dds(self):
        raw = (STAMPS[0], STAMPS[1], STAMPS[1], STAMPS[2])
        result = m.correlate(publisher_lines(), observer_lines(raw=raw))
        self.assertEqual(result["observed_pattern"],
                         "RAW_OBSERVER_REPEAT_WITHOUT_RECORDED_PRESEND_REPEAT")
        self.assertFalse(result["dds_redelivery_or_reordering_proven"])
        self.assertEqual(result["raw_stamp_excess_record_count"], 1)

    def test_presend_duplicate_without_raw_duplicate(self):
        result = m.correlate(publisher_stale(), observer_lines())
        self.assertEqual(result["observed_pattern"],
                         "PRESEND_REPEAT_WITHOUT_MATCHED_RAW_REPEAT")

    def test_pre_send_log_with_ros_launch_prefix(self):
        events = m.parse_publisher(pub(1, STAMPS[0],
            prefix="[usb_cam_node_exe-1] ") + "\n")
        self.assertEqual(len(events), 1)

    def test_publisher_initial_nonfresh_blocked(self):
        with self.assertRaisesRegex(m.Blocked, "initial nonfresh"):
            m.parse_publisher(pub(1, STAMPS[0], fresh=0, dequeue=0,
                                  sequence=0, v4l2_time=-1) + "\n")

    def test_publisher_gap_index_blocked(self):
        src = publisher_lines().replace("pub_index=2", "pub_index=3")
        with self.assertRaisesRegex(m.Blocked, "index not contiguous"):
            m.parse_publisher(src)

    def test_wrong_dequeue_count_blocked(self):
        src = publisher_lines().replace("dequeue_count=2", "dequeue_count=55")
        with self.assertRaisesRegex(m.Blocked, "dequeue counter"):
            m.parse_publisher(src)

    def test_nonfresh_stored_stamp_changed_blocked(self):
        src = publisher_stale().replace(
            "pub_index=3 fresh=0", "pub_index=3 fresh=0")
        src = src.replace("ros_stamp_ns=" + str(STAMPS[1]),
                          "ros_stamp_ns=" + str(STAMPS[3]), 1)
        # This alters fresh publisher record 2; later stale stamp differs.
        with self.assertRaisesRegex(m.Blocked, "nonfresh event"):
            m.parse_publisher(src)

    def test_wrong_fresh_status_blocked(self):
        src = publisher_lines().replace("fresh=1", "fresh=0", 1)
        with self.assertRaisesRegex(m.Blocked, "invalid pre-send"):
            m.parse_publisher(src)

    def test_kernel_sequence_out_of_range_blocked(self):
        src = publisher_lines().replace("v4l2_seq=1", "v4l2_seq=4294967296", 1)
        with self.assertRaisesRegex(m.Blocked, "invalid pre-send"):
            m.parse_publisher(src)

    def test_malformed_pub_line_blocked(self):
        with self.assertRaisesRegex(m.Blocked, "malformed pre-send"):
            m.parse_publisher("M23AX_PUBLISH pub_index=1 incomplete\n")

    def test_duplicate_event_marker_line_blocked(self):
        line = pub(1, STAMPS[0])
        with self.assertRaisesRegex(m.Blocked, "ambiguous"):
            m.parse_publisher(line + " " + line + "\n")

    def test_publisher_fatal_blocked(self):
        with self.assertRaisesRegex(m.Blocked, "publisher fatal"):
            m.parse_publisher(publisher_lines() + "M23AX_FATAL=METADATA_LOG_LIMIT\n")

    def test_missing_publisher_events_blocked(self):
        with self.assertRaisesRegex(m.Blocked, "no M23AX"):
            m.parse_publisher("usb_cam launched\n")

    def test_wrong_qos_blocked(self):
        with self.assertRaisesRegex(m.Blocked, "observer marker"):
            m.parse_observer(observer_lines(qos="best_effort_5"))

    def test_wrong_variant_blocked(self):
        with self.assertRaisesRegex(m.Blocked, "observer marker"):
            m.parse_observer(observer_lines(full="RAW_ONLY"))

    def test_missing_final_counts_blocked(self):
        src = observer_lines().rsplit("M23K_COUNTER_COUNTS=", 1)[0]
        with self.assertRaisesRegex(m.Blocked, "completion summary"):
            m.parse_observer(src)

    def test_counts_mismatch_blocked(self):
        src = observer_lines().replace("M23K_COUNTER_COUNTS=3,3,3",
                                       "M23K_COUNTER_COUNTS=4,3,3")
        with self.assertRaisesRegex(m.Blocked, "counts mismatch"):
            m.parse_observer(src)

    def test_nonmonotonic_receipt_blocked(self):
        src = observer_lines().replace(
            "M23K_SAMPLE raw " + str(STAMPS[1]) + " 200033333333",
            "M23K_SAMPLE raw " + str(STAMPS[1]) + " 200000000000")
        # Generated receipts in fixture are 200000000000 + 33333333 = 200033333333
        with self.assertRaisesRegex(m.Blocked, "nonincreasing steady"):
            m.parse_observer(src)

    def test_nonmonotonic_header_is_reported_not_hidden(self):
        raw = (STAMPS[0], STAMPS[2], STAMPS[1])
        result = m.correlate(publisher_lines(), observer_lines(raw=raw))
        self.assertEqual(result["raw_nonpositive_header_intervals"], 1)

    def test_malformed_sample_blocked(self):
        src = observer_lines().replace("M23K_SAMPLE raw", "M23K_SAMPLE weird", 1)
        with self.assertRaisesRegex(m.Blocked, "malformed subscriber"):
            m.parse_observer(src)

    def test_counter_fatal_blocked(self):
        src = observer_lines() + "M23K_COUNTER_FATAL=BOUNDED_SAMPLE_CAP:raw\n"
        with self.assertRaisesRegex(m.Blocked, "observer fatal"):
            m.parse_observer(src)

    def test_duplicate_qos_marker_blocked(self):
        src = observer_lines() + "M23M_COUNTER_QOS_PROFILE=reliable_100\n"
        with self.assertRaisesRegex(m.Blocked, "observer marker"):
            m.parse_observer(src)

    def test_both_paths_required(self):
        with contextlib.redirect_stdout(io.StringIO()) as f:
            self.assertEqual(m.main(["--publisher", "/irrelevant"]), 2)
        self.assertIn("M23BA_BLOCKED=", f.getvalue())

    def test_bounded_file_and_symlink_block(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "small.log"
            p.write_text("data")
            self.assertEqual(m.read_bounded_file(p), "data")
            linked = Path(d) / "linked.log"
            linked.symlink_to(p)
            with self.assertRaisesRegex(m.Blocked, "symlinked"):
                m.read_bounded_file(linked)
            with self.assertRaisesRegex(m.Blocked, "above bound"):
                with p.open("wb") as f:
                    f.truncate(m.MAX_LOG_BYTES + 1)
                m.read_bounded_file(p)

    def test_end_to_end_two_existing_text_files(self):
        with tempfile.TemporaryDirectory() as d:
            publisher = Path(d) / "camera.log"
            observer = Path(d) / "counter.log"
            publisher.write_text(publisher_stale())
            observer.write_text(observer_lines(
                raw=(STAMPS[0], STAMPS[1], STAMPS[1], STAMPS[2])))
            with contextlib.redirect_stdout(io.StringIO()) as f:
                self.assertEqual(m.main(["--publisher", str(publisher),
                                         "--observer", str(observer)]), 0)
            self.assertIn("M23BA_VERDICT=OFFLINE_EVENT_CORRELATION_DIAGNOSTIC_ONLY",
                          f.getvalue())
            self.assertIn('"pre_send_nonfresh_attempts": 1', f.getvalue())


if __name__ == "__main__":
    unittest.main()
