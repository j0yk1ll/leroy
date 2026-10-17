"""M20 offline concurrency contract / non-actuation source audit."""
import ast
import copy
from pathlib import Path
import unittest

from m20_combined_contract import (
    DURATION_S, EXPECTED_PUBLISHERS, MIN_OVERLAP_S, evaluate,
)


ROOT = Path(__file__).parent


def fixture():
    src = {
        "clock_rate_estimate_ready": True,
        "clock_rate_estimate_ppm": -39.0,
        "measured_publication_rate_hz": 50.0,
        "terminal_fault": None,
        "counts": {
            "published_imu_messages": 6000,
            "malformed_imu_packets": 0, "sequence_faults": 0,
            "sequence_gap_samples": 0, "timestamp_faults": 0,
            "stale_or_future_withheld": 0, "calibration_withheld": 0,
            "invalid_ascii_lines": 0, "unexpected_serial_lines": 0,
        },
    }
    topics = {}
    for name, accepted, frame in (
        ("camera_image", 1300, "camera_optical_frame"),
        ("camera_info", 2000, "camera_optical_frame"),
        ("imu", 6000, "imu_link"),
    ):
        topics[name] = {
            "accepted": accepted, "received": accepted, "invalid": 0,
            "fresh_valid_receipt": True, "frame": frame,
            "publisher_count": 1, "publishers": [EXPECTED_PUBLISHERS[name]],
        }
    topics["camera_image"]["encoding"] = "rgb8"
    topics["camera_image"]["dimensions"] = [640, 480]
    topics["camera_info"]["dimensions"] = [640, 480]
    for name in ("depth_image", "xrslam_tracking", "xrslam_valid",
                 "odom", "tf", "obstacle_cloud"):
        topics[name] = {"publisher_count": 0, "accepted": 0}
    return src, {"topics": topics}


def passed(src, report, **updates):
    opts = dict(completed=True, start_ack=True, stop_ack=True,
                source=src, observer=report, overlap_s=111.0,
                forbidden_graph_seen=False, observer_thread_error=False)
    opts.update(updates)
    return evaluate(**opts)


class M20ContractTest(unittest.TestCase):
    def test_accepts_concurrent_camera_and_real_imu_report(self):
        src, report = fixture()
        self.assertEqual(DURATION_S, 120)
        self.assertGreaterEqual(MIN_OVERLAP_S, 90)
        self.assertTrue(passed(src, report))

    def test_rejects_missing_overlap_ack_or_mapper(self):
        src, report = fixture()
        for key, val in (
            ("overlap_s", None), ("overlap_s", 5.0),
            ("start_ack", False), ("stop_ack", False),
            ("completed", False), ("forbidden_graph_seen", True),
            ("observer_thread_error", True),
        ):
            with self.subTest(key=key, val=val):
                self.assertFalse(passed(src, report, **{key: val}))
        src["clock_rate_estimate_ready"] = False
        self.assertFalse(passed(src, report))

    def test_rejects_any_invalid_or_wrong_publisher(self):
        src, report = fixture()
        for field, val in (("invalid", 1), ("accepted", 20),
                           ("fresh_valid_receipt", False),
                           ("frame", "other_frame")):
            altered = copy.deepcopy(report)
            altered["topics"]["camera_image"][field] = val
            self.assertFalse(passed(src, altered))
        altered = copy.deepcopy(report)
        altered["topics"]["imu"]["publishers"][0]["node"] = "leroy_bridge"
        self.assertFalse(passed(src, altered))
        altered = copy.deepcopy(report)
        altered["topics"]["depth_image"]["publisher_count"] = 1
        self.assertFalse(passed(src, altered))
        for key in ("sequence_gap_samples", "timestamp_faults"):
            altered_source = copy.deepcopy(src)
            altered_source["counts"][key] = 1
            self.assertFalse(passed(altered_source, report))

    def test_no_new_serial_write_or_motor_authority(self):
        src = (ROOT / "m20_combined_observer.py").read_text()
        tree = ast.parse(src)
        forbidden = {"write", "create_publisher", "create_client",
                     "create_subscription"}
        self.assertFalse(any(
            isinstance(n, ast.Call) and
            isinstance(n.func, ast.Attribute) and
            n.func.attr in forbidden for n in ast.walk(tree)))
        self.assertEqual(src.count(
            "send_only_whitelisted_serial_command(device, START)"), 1)
        self.assertEqual(src.count(
            "send_only_whitelisted_serial_command(device, STOP)"), 1)
        for word in ("set_wheels", "keepalive",
                     "LeroyBridgeNode", "leroy_xrslam_node"):
            self.assertNotIn(word, src)
        self.assertIn("ImuStreamSource()", src)
        self.assertIn("PassiveSensorObserver()", src)
        self.assertIn("SingleThreadedExecutor()", src)

    def test_launch_has_only_camera_and_rectifier(self):
        launch = (ROOT / "m20_camera_only.launch.py").read_text()
        tree = ast.parse(launch)
        nodes = [
            n for n in ast.walk(tree)
            if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Name)
            and n.func.id in ("Node", "ComposableNodeContainer")]
        self.assertEqual(len(nodes), 2)
        self.assertIn("usb_cam", launch)
        self.assertIn("image_proc::RectifyNode", launch)
        self.assertIn("ROS_DOMAIN_ID", launch)
        self.assertIn('"170"', launch)
        self.assertNotIn('package="leroy_depthart"', launch)
        self.assertNotIn("LeroyBridgeNode", launch)
        self.assertNotIn("cmd_vel", launch)

    def test_container_and_runner_are_device_limited(self):
        docker = (ROOT / "Dockerfile.m20-sensors").read_text()
        runner = Path("/usr/local/bin/leroy-m20-sensor-run").read_text()
        self.assertIn("FROM ${LEROY_M20_BASE_IMAGE}", docker)
        self.assertIn("localhost/leroy-imu:m19-clock-audit", docker)
        self.assertIn("M20_ROS_IMPORT_SELFTEST=PASS", docker)
        self.assertIn("LEROY_M20_SELFTEST=1", docker)
        self.assertIn('CMD ["/usr/local/bin/leroy-m20-sensor-run"]', docker)
        self.assertIn("ros2 launch /opt/m20/m20_camera_only.launch.py", runner)
        self.assertIn("trap cleanup EXIT", runner)
        self.assertIn('kill -TERM "$pid"', runner)
        self.assertNotIn("podman", runner)
        self.assertNotIn("leroy_bridge.launch", runner)
        self.assertNotIn("robot.launch", runner)


if __name__ == "__main__":
    unittest.main()
