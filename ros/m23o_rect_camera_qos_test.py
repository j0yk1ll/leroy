"""M23O isolated rectified-camera QoS contracts; no devices opened."""
import importlib
from pathlib import Path
import unittest

from m23o_rect_qos_graph import REQUIRED, validate_graph
from m23k_camera_path_probe import (
    camera_qos_request, rectified_qos_request, camera_crash_context,
    PROBE_VARIANTS,
)


def valid_graph():
    topics = {}
    for topic, role, name in REQUIRED:
        endpoints = topics.setdefault(
            topic, {"publishers": [], "subscriptions": []})
        endpoints[role].append({
            "node_name": name, "node_namespace": "/camera",
            "qos": {"reliability": "RELIABLE", "durability": "VOLATILE",
                    "history": "KEEP_LAST", "depth": 100},
        })
    return {"rmw_implementation": "rmw_fastrtps_cpp", "topics": topics}


class M23ORectifiedQoSContracts(unittest.TestCase):
    def test_valid_runtime_endpoint_inventory(self):
        self.assertTrue(validate_graph(valid_graph()))

    def test_missing_endpoints_block(self):
        for topic, role, name in REQUIRED:
            with self.subTest(topic=topic, role=role, name=name):
                graph = valid_graph()
                graph["topics"][topic][role] = [
                    endpoint for endpoint in graph["topics"][topic][role]
                    if endpoint["node_name"] != name]
                with self.assertRaises(ValueError):
                    validate_graph(graph)

    def test_best_effort_endpoint_blocks(self):
        for topic, role, name in REQUIRED:
            with self.subTest(topic=topic, role=role, name=name):
                graph = valid_graph()
                for endpoint in graph["topics"][topic][role]:
                    if endpoint["node_name"] == name:
                        endpoint["qos"]["reliability"] = "BEST_EFFORT"
                with self.assertRaises(ValueError):
                    validate_graph(graph)

    def test_nonvolatile_endpoint_blocks(self):
        graph = valid_graph()
        graph["topics"]["/camera/image_rect"]["publishers"][0]["qos"][
            "durability"] = "TRANSIENT_LOCAL"
        with self.assertRaises(ValueError):
            validate_graph(graph)

    def test_unknown_rmw_blocks(self):
        for identifier in (None, "", "unspecified_default", "FastDDS"):
            graph = valid_graph()
            graph["rmw_implementation"] = identifier
            with self.assertRaises(ValueError):
                validate_graph(graph)

    def test_m23o_only_allows_reliable_full_rectifier(self):
        self.assertEqual(rectified_qos_request({
            "M23O_RECT_QOS_ONLY": "1", "M23O_SUB_QOS": "reliable_100",
        }, "full_mjpeg"), "reliable_100")
        for profile, variant in (
            ("best_effort_100", "full_mjpeg"),
            ("reliable_100", "raw_only_mjpeg"),
            ("", "full_mjpeg"),
        ):
            with self.subTest(profile=profile, variant=variant):
                with self.assertRaises(ValueError):
                    rectified_qos_request({
                        "M23O_RECT_QOS_ONLY": "1", "M23O_SUB_QOS": profile,
                    }, variant)

    def test_m23n_separate_opt_in_unchanged(self):
        self.assertIsNone(rectified_qos_request({}, "full_mjpeg"))
        self.assertIsNone(camera_qos_request({}, "full_mjpeg"))
        self.assertEqual(camera_qos_request({
            "M23N_CAMERA_QOS_ONLY": "1",
            "M23N_SUB_QOS": "reliable_100",
        }, "raw_only_mjpeg"), "reliable_100")
        with self.assertRaises(ValueError):
            rectified_qos_request({
                "M23N_CAMERA_QOS_ONLY": "1",
                "M23O_RECT_QOS_ONLY": "1",
                "M23O_SUB_QOS": "reliable_100",
            }, "full_mjpeg")

    def test_full_rectifier_and_strict_continuity_source(self):
        source = Path(importlib.import_module(
            "m23k_camera_path_probe").__file__).read_text()
        self.assertTrue(PROBE_VARIANTS["full_mjpeg"][1])
        for token in (
            'M23O_RECT_QOS_ONLY', 'M23O_SUB_QOS',
            '["python3", "/opt/m23o/m23o_rect_qos_graph.py"]',
            'validate_graph(graph_report)',
            '"RECTIFIED_CAMERA_QOS_DIAGNOSTIC_ONLY"',
            '"BLOCKED_RECTIFIED_IMAGE_CONTINUITY"',
            '"M23O_RECTIFIED_CONTINUITY_GATE_FAILED"',
            'stream["header_intervals"]["gaps_over_250ms"] == 0',
            'stream["observed_rate_hz"] >= 25',
            'stats_report["rect_only_observed_header_stamps"] == 0',
            'stats_report["rectified_image_continuity_validated"] = False',
            'M23O_RECT_QOS_JSON=',
            'if requested_qos and not m23o_qos:',
            'M23O_CAMERA_CRASH_CONTEXT=',
        ):
            self.assertIn(token, source)
        # Preserve bounded crash context instead of only the ROS exit line.
        log = "\\n".join(
            ["older log " + str(i) for i in range(40)] +
            ["process has died: component_container exit code -11"] +
            ["later log " + str(i) for i in range(30)])
        excerpt = camera_crash_context(log)
        self.assertEqual(len(excerpt), 21)
        self.assertTrue(any("exit code -11" in line for line in excerpt))
        self.assertEqual(camera_crash_context("clean shutdown"), [])
        for forbidden in ("serial.Serial(", 'create_publisher(', "set_wheels("):
            self.assertNotIn(forbidden, source)

    def test_graph_probe_is_read_only(self):
        source = Path(importlib.import_module(
            "m23o_rect_qos_graph").__file__).read_text()
        for token in ("get_publishers_info_by_topic",
                      "get_subscriptions_info_by_topic",
                      "get_rmw_implementation_identifier",
                      "M23O_GRAPH_JSON=", "M23O_GRAPH_VERDICT="):
            self.assertIn(token, source)
        for forbidden in ("create_publisher(", "create_subscription(",
                          "/dev/video", "/dev/ttyACM", "serial.Serial("):
            self.assertNotIn(forbidden, source)

    def test_operator_and_docker_isolation(self):
        root = Path(__file__).resolve().parent
        operator = (Path("/opt/m23o/m23o_operator_camera_rect_qos.sh")
                    if Path("/opt/m23o/m23o_operator_camera_rect_qos.sh").exists()
                    else root / "m23o_operator_camera_rect_qos.sh").read_text()
        image = (Path("/opt/m23o/Dockerfile.m23o-rect-camera-qos")
                 if Path("/opt/m23o/Dockerfile.m23o-rect-camera-qos").exists()
                 else root / "Dockerfile.m23o-rect-camera-qos").read_text()
        for token in ("AUTHORIZE_M23O", "--network none",
                      '--device "$CAMERA_DEV":/dev/video2',
                      "usb-Wed_Camera_Wed_Camera_20240420112206-video-index0",
                      "M23K_PROBE_VARIANT=full_mjpeg",
                      'M23O_SUB_QOS=reliable_100',
                      'podman inspect leroy-ros',
                      'fuser "$CAMERA_DEV"'):
            self.assertIn(token, operator)
        for forbidden in ("/dev/ttyACM0", "--privileged",
                          "--network host", "m23h_operator_host_capture"):
            self.assertNotIn(forbidden, operator)
        self.assertIn(
            "FROM localhost/leroy-sensors:m23n-real-camera-qos", image)
        self.assertIn("M23O_JAZZY_RECT_QOS_IMPORT=PASS", image)
        for forbidden in ("apt-get ", "curl ", "wget ", "--device "):
            self.assertNotIn(forbidden, image)


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            M23ORectifiedQoSContracts))
    if result.wasSuccessful():
        print("M23O_RECT_QOS_SOURCE_CONTRACT=PASS", flush=True)
        print("M23O_LIVE_CAMERA_RUN=NOT_PERFORMED", flush=True)
        print("M23O_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    raise SystemExit(2)
