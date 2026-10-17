"""No ROS, Podman, serial, or camera dependencies."""
import json
from pathlib import Path
import tempfile
import unittest

from m21_offline_timing_audit import VERDICT, INCOMPLETE, audit, load_log, main


def fixture():
    topics = {}
    for name, frame, node, ns, ty, count in (
        ("camera_image", "camera_optical_frame", "rectify_node", "/camera", "sensor_msgs/msg/Image", 1000),
        ("camera_info", "camera_optical_frame", "usb_cam", "/camera", "sensor_msgs/msg/CameraInfo", 1000),
        ("imu", "imu_link", "m18_imu_source", "/", "sensor_msgs/msg/Imu", 6000),
    ):
        topics[name] = {"accepted": count, "received": count, "invalid": 0,
                        "fresh_valid_receipt": True, "frame": frame,
                        "publishers": [{"node": node, "namespace": ns, "type": ty}],
                        "mean_receipt_hz": 50, "last_acquisition_age_s": 0.01}
    m16 = {"observation_only": True, "topics": topics,
           "verdict": "INCOMPLETE_OR_UNVERIFIED_REAL_STREAMS"}
    m20 = {"verdict": "OBSERVED_CONCURRENT_CAMERA_IMU_ROS_CONTRACT",
           "imu_stream_start_acknowledged": True,
           "imu_stream_stop_acknowledged": True,
           "duration_requested_s": 120, "observation_completed": True,
           "sensor_receipt_overlap_s": 119.886, "clock_rate_ready": True,
           "clock_rate_estimate_ppm": -41.08,
           "error": None, "forbidden_graph_fault": None,
           "observer_thread_faults": [], "motor_commands_sent": 0,
           "imu_source": {"counts": {"published_imu_messages": 6001},
                          "clock_rate_estimate_ready": True,
                          "motor_commands_sent": 0}}
    markers = {"M20_COMBINED_VERDICT": m20["verdict"],
               "M20_IMU_STREAM_START_ACK": "PASS",
               "M20_IMU_STREAM_STOP_ACK": "PASS",
               "M16_VERDICT": m16["verdict"]}
    return {"m16": m16, "m20": m20}, markers


class M21OfflineAuditTests(unittest.TestCase):
    def test_accepts_m20_only_as_readiness_not_physical_calibration(self):
        records, markers = fixture()
        result = audit(records, markers)
        self.assertEqual(result["verdict"], VERDICT)
        self.assertIsNone(result["physical_camera_imu_offset_s"])
        self.assertFalse(result["physical_offset_validated"])
        self.assertEqual(result["mcu_clock_rate_ppm_host_relative"], -41.08)

    def test_rejects_missing_camera_stream(self):
        records, markers = fixture()
        del records["m16"]["topics"]["camera_image"]
        self.assertEqual(audit(records, markers)["verdict"], INCOMPLETE)

    def test_rejects_wrong_imu_publisher(self):
        records, markers = fixture()
        records["m16"]["topics"]["imu"]["publishers"][0]["node"] = "leroy_bridge"
        self.assertEqual(audit(records, markers)["verdict"], INCOMPLETE)

    def test_rejects_unacknowledged_stop(self):
        records, markers = fixture()
        markers["M20_IMU_STREAM_STOP_ACK"] = "UNVERIFIED"
        self.assertEqual(audit(records, markers)["verdict"], INCOMPLETE)

    def test_rejects_incomplete_duration_or_overlap(self):
        records, markers = fixture()
        records["m20"]["sensor_receipt_overlap_s"] = 40
        self.assertEqual(audit(records, markers)["verdict"], INCOMPLETE)
        records, markers = fixture()
        records["m20"]["observation_completed"] = False
        self.assertEqual(audit(records, markers)["verdict"], INCOMPLETE)

    def test_rejects_runtime_faults_and_motor_command(self):
        records, markers = fixture()
        records["m20"]["forbidden_graph_fault"] = "forbidden_ros_graph"
        self.assertEqual(audit(records, markers)["verdict"], INCOMPLETE)
        records, markers = fixture()
        records["m20"]["motor_commands_sent"] = 1
        self.assertEqual(audit(records, markers)["verdict"], INCOMPLETE)

    def test_read_only_log_roundtrip(self):
        records, markers = fixture()
        content = ["M20_IMU_STREAM_START_ACK=PASS", "M20_IMU_STREAM_STOP_ACK=PASS",
                   "M16_VERDICT=" + markers["M16_VERDICT"],
                   "M16_PASSIVE_REPORT_JSON=" + json.dumps(records["m16"]),
                   "M20_COMBINED_REPORT_JSON=" + json.dumps(records["m20"]),
                   "M20_COMBINED_VERDICT=" + markers["M20_COMBINED_VERDICT"]]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "capture.log"
            path.write_text("\n".join(content) + "\n")
            self.assertEqual(load_log(path), (records, markers))
            self.assertEqual(main([str(path)]), 0)

    def test_duplicate_report_fails_closed(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "capture.log"
            path.write_text('M20_COMBINED_REPORT_JSON={}\nM20_COMBINED_REPORT_JSON={}\n')
            with self.assertRaises(ValueError):
                load_log(path)

    def test_missing_reports_fails_closed(self):
        self.assertEqual(audit({}, {})["verdict"], INCOMPLETE)


if __name__ == "__main__":
    unittest.main()
