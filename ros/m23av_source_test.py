"""M23AV: offline incident audit tests; no ROS, camera, Podman or hardware."""
import json
from pathlib import Path
from unittest.mock import patch
import unittest
import m23av_log_audit as m


def log_fixture():
    streams = {}
    for kind, count, rate in (("raw", 1404, 29.829),
                              ("rect", 1403, 29.827),
                              ("info", 1403, 29.817)):
        streams[kind] = {
            "count": count, "observed_rate_hz": rate,
            "header_intervals": {
                "gaps_over_250ms": 0,
                "nonpositive_gaps": 1 if kind == "raw" else 0
            },
            "receipt_intervals": {
                "gaps_over_250ms": 0, "nonpositive_gaps": 0
            },
        }
    report = {
        "faults": ["NONMONOTONIC_RAW"], "shutdown_clean": True,
        "graph_qos_verified": True, "receiver_qos_verified": True,
        "actual_publisher_offered_qos_verified": True,
        "camera_probe_variant": "full_mjpeg", "rectifier_enabled": True,
        "device_access": "CAMERA_ONLY_BY_OPERATOR",
        "stm32_serial_access": False,
        "camera_publisher_rate_verified": False,
        "rectified_image_continuity_validated": False,
        "counts": {"raw": 1404, "rect": 1403, "info": 1403},
        "raw_rect_identical_header_stamp_pairs": 1403,
        "raw_only_observed_header_stamps": 0,
        "rect_only_observed_header_stamps": 0,
        "streams": streams,
    }

    def wrap(p):
        payload = json.dumps(p, sort_keys=True)
        return "\n".join([
            "M23K_CAMERA_PROBE_JSON=" + payload,
            "M23K_VERDICT=INCOMPLETE_CAMERA_PATH_OBSERVATION",
            "M23O_RECT_QOS_JSON=" + payload,
            "M23O_RECT_QOS_VERDICT=INCOMPLETE_CAMERA_PATH_OBSERVATION",
            "M23AU_BLOCKED=Blocked:inherited M23O rectified continuity and/or graph failed",
            "M23AU_PRODUCTION_ACCEPTANCE=BLOCKED",
        ]) + "\n"

    return report, wrap


class OfflineTests(unittest.TestCase):
    def test_inert_default(self):
        with patch.object(m.Path, "read_text", side_effect=AssertionError("unexpected read")):
            self.assertEqual(m.main([]), 0)

    def test_accepted_diagnosis_remains_blocked(self):
        d, wrap = log_fixture()
        result = m.verify_log(wrap(d))
        self.assertEqual(result["raw_repeated_header_stamp_excess"], 1)
        self.assertEqual(result["live_test_acceptance"], "BLOCKED")
        self.assertIs(result["camera_publisher_duplicate_proven"], False)

    def test_cannot_overclaim_publisher(self):
        d, wrap = log_fixture()
        d["camera_publisher_rate_verified"] = True
        with self.assertRaisesRegex(m.Blocked, "publication-side"):
            m.verify_log(wrap(d))

    def test_fail_extra_raw_fault(self):
        d, wrap = log_fixture()
        d["faults"].append("CAMERA_CHILD_PROCESS_CRASHED")
        with self.assertRaisesRegex(m.Blocked, "raw timestamp anomaly"):
            m.verify_log(wrap(d))

    def test_fail_duplicate_raw_json_marker(self):
        d, wrap = log_fixture()
        text = wrap(d)
        text += [s for s in text.splitlines() if s.startswith("M23K_CAMERA_PROBE_JSON=")][0] + "\n"
        with self.assertRaisesRegex(m.Blocked, "duplicate marker"):
            m.verify_log(text)

    def test_fail_inconsistent_m23o_report(self):
        d, wrap = log_fixture()
        text = wrap(d).replace("M23O_RECT_QOS_JSON=", 'M23O_RECT_QOS_JSON={"mutated":true}', 1)
        with self.assertRaises((m.Blocked, json.JSONDecodeError)):
            m.verify_log(text)

    def test_fail_undercount_cardinality(self):
        d, wrap = log_fixture()
        d["raw_rect_identical_header_stamp_pairs"] = 1402
        with self.assertRaisesRegex(m.Blocked, "cardinality"):
            m.verify_log(wrap(d))

    def test_fail_wrong_raw_order_count(self):
        d, wrap = log_fixture()
        d["streams"]["raw"]["header_intervals"]["nonpositive_gaps"] = 0
        with self.assertRaisesRegex(m.Blocked, "header order"):
            m.verify_log(wrap(d))

    def test_fail_unverified_shutdown(self):
        d, wrap = log_fixture()
        d["shutdown_clean"] = False
        with self.assertRaisesRegex(m.Blocked, "shutdown"):
            m.verify_log(wrap(d))

    def test_fail_graph_qos(self):
        d, wrap = log_fixture()
        d["graph_qos_verified"] = False
        with self.assertRaisesRegex(m.Blocked, "graph"):
            m.verify_log(wrap(d))

    def test_fail_camera_below_25hz(self):
        d, wrap = log_fixture()
        d["streams"]["raw"]["observed_rate_hz"] = 9.0
        with self.assertRaisesRegex(m.Blocked, "rate outside"):
            m.verify_log(wrap(d))

    def test_fail_gap_over_250ms(self):
        d, wrap = log_fixture()
        d["streams"]["rect"]["header_intervals"]["gaps_over_250ms"] = 1
        with self.assertRaisesRegex(m.Blocked, "gap"):
            m.verify_log(wrap(d))

    def test_fail_production_accepted(self):
        d, wrap = log_fixture()
        with self.assertRaisesRegex(m.Blocked, "production"):
            m.verify_log(wrap(d).replace(
                "M23AU_PRODUCTION_ACCEPTANCE=BLOCKED",
                "M23AU_PRODUCTION_ACCEPTANCE=PASS"))

    def test_log_gate_path_explicit(self):
        self.assertEqual(m.main([]), 0)
        self.assertEqual(m.main.__defaults__, (None,))


if __name__ == "__main__":
    unittest.main()
