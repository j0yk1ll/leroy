"""No-device simulated-node tests for Capture ROS metadata publisher adapter."""
import json
from pathlib import Path
import unittest

from leroy_capture.contract import CaptureError, METADATA_TOPIC, source_metadata
from leroy_capture.metadata_ros import SourceMetadataRosAdapter

HERE = Path(__file__).resolve().parents[2]


class FakeString:
    def __init__(self):
        self.data = ""


class FakePublisher:
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.messages = []
        self.reject = False

    def publish(self, message):
        if self.reject:
            raise RuntimeError("simulated_metadata_publish_failure")
        self.messages.append(message)


class FakeImuSource:
    def __init__(self, name="m18_imu_source", namespace="/", remap=False):
        self.name = name
        self.namespace = namespace
        self.remap = remap
        self.pub = None
        self.calls = []

    def get_name(self):
        return self.name

    def get_namespace(self):
        return self.namespace

    def create_publisher(self, message_type, topic, qos):
        self.calls.append((message_type, topic, qos))
        self.pub = FakePublisher(
            "/unexpected_remapped_metadata" if self.remap else topic)
        return self.pub


class RosAdapterSimulation(unittest.TestCase):
    @staticmethod
    def make(node, *, clock_values=(1_000_000, 2_000_000)):
        sequence = iter(clock_values)
        return SourceMetadataRosAdapter(
            node, message_type=FakeString, qos=object(),
            monotonic_ns=lambda: next(sequence))

    def test_source_metadata_preserves_mcu_fields(self):
        node = FakeImuSource()
        adapter = self.make(node)
        output = adapter.on_imu_published(
            {"seq": 701, "t_us": 124_000, "cal": 1},
            1_790_000_000_000_000_000,
            999_999)
        self.assertEqual(len(node.calls), 1)
        self.assertEqual(node.calls[0][1], METADATA_TOPIC)
        self.assertIs(node.calls[0][0], FakeString)
        self.assertEqual(len(node.pub.messages), 1)
        self.assertIsInstance(node.pub.messages[0], FakeString)
        self.assertEqual(output, node.pub.messages[0].data)
        msg = source_metadata(output)
        self.assertEqual(msg["seq"], 701)
        self.assertEqual(msg["t_us"], 124_000)
        self.assertEqual(msg["imu_stamp_ns"], 1_790_000_000_000_000_000)
        self.assertEqual(msg["publish_monotonic_ns"], 1_000_000)
        self.assertNotEqual(msg["publish_monotonic_ns"], 999_999)
        self.assertEqual(adapter.tap.count, 1)

    def test_wrong_source_identity_rejected_before_publisher_creation(self):
        for name, namespace in (("other", "/"), ("m18_imu_source", "/wrong")):
            node = FakeImuSource(name=name, namespace=namespace)
            with self.subTest(name=name, namespace=namespace), self.assertRaisesRegex(
                CaptureError, "source_node_identity_not_m18"
            ):
                self.make(node)
            self.assertEqual(node.calls, [])

    def test_topic_remap_rejected(self):
        node = FakeImuSource(remap=True)
        with self.assertRaisesRegex(CaptureError, "metadata_topic_remapped"):
            self.make(node)
        self.assertEqual(node.calls[0][1], METADATA_TOPIC)

    def test_sender_failure_latches_and_does_not_commit_count(self):
        node = FakeImuSource()
        adapter = self.make(node)
        node.pub.reject = True
        with self.assertRaisesRegex(RuntimeError, "simulated_metadata_publish_failure"):
            adapter.on_imu_published(
                {"seq": 1, "t_us": 20_000, "cal": 1}, 1_000_000_000, 12)
        self.assertEqual(adapter.tap.count, 0)
        self.assertIsNotNone(adapter.tap.terminal_fault)
        with self.assertRaisesRegex(CaptureError, "metadata_tap_fault_latched"):
            adapter.on_imu_published(
                {"seq": 2, "t_us": 40_000, "cal": 1}, 1_020_000_000, 34)

    def test_original_tick_discontinuity_rejected_before_second_publish(self):
        node = FakeImuSource()
        adapter = self.make(node)
        adapter.on_imu_published(
            {"seq": 4, "t_us": 100_000, "cal": 1}, 1_000_000_000, 55)
        with self.assertRaisesRegex(CaptureError, "source_mcu_tick_discontinuity"):
            adapter.on_imu_published(
                {"seq": 5, "t_us": 2_000_001, "cal": 1}, 1_020_000_000, 66)
        self.assertEqual(len(node.pub.messages), 1)

    def test_provenance_fields_not_backfilled_from_ros_header(self):
        node = FakeImuSource()
        adapter = self.make(node)
        with self.assertRaisesRegex(CaptureError, "missing_original_mcu_fields"):
            adapter.on_imu_published(
                {"cal": 1}, 1_000_000_000, 55)
        self.assertEqual(len(node.pub.messages), 0)

    def test_metadata_adapter_has_no_serial_ownership(self):
        wrapper = (HERE / "src/leroy_capture/leroy_capture/metadata_ros.py").read_text()
        m18 = (HERE / "src/leroy_imu_observer/leroy_imu_observer/imu_source.py").read_text()
        runner = (HERE / "live_stage4_imu_overlap.py").read_text()
        for token in ("serial.Serial(", "device.write(", "/cmd_vel", "rclpy.init("):
            self.assertNotIn(token, wrapper)
        self.assertNotIn("live_stage4c_source_meta_ros", m18)
        self.assertIn("live_stage4c_source_meta_ros", runner)
        self.assertIn("SourceMetadataRosAdapter(source)", runner)
        self.assertIn('os.environ.get("LEROY_STAGE4C_SOURCE_META", "0")', runner)
        self.assertIn("source_metadata_requires_capture_only_session", runner)
        self.assertIn("require_source_meta=metadata_enabled", runner)
        self.assertIn("source_node.create_publisher(", wrapper)
        self.assertIn("metadata_topic_remapped", wrapper)

    def test_guarded_real_capture_is_explicitly_commissioned(self):
        wrapper = (HERE / "live_stage4_imu_sidecar.sh").read_text()
        recorder = (HERE / "src/leroy_capture/leroy_capture/recorder.py").read_text()
        for token in (
            'record_mode="${LEROY_STAGE4C_RECORD:-0}"',
            "CAPTURE_REVIEW_ATTESTATION_REQUIRED",
            "RECORDER_REQUIRES_SOURCE_METADATA_IMAGE",
            "CAPTURE_OPERATOR_CONSENT_MISSING",
            'grep -Fqx "LIVE_CAPTURE_COMMISSIONED = True"',
            "RECORDER_FAILED_BEFORE_MCU_OPEN",
            "STAGE4C_REAL_FOUR_TOPIC_RECORDING=COMPLETE_UNCALIBRATED",
            "timeout 110s podman wait",
        ):
            self.assertIn(token, wrapper)
        self.assertIn("LIVE_CAPTURE_COMMISSIONED = True", recorder)
        runner = (HERE / "live_stage4_imu_overlap.py").read_text()
        self.assertIn('capture_not_armed_before_serial', runner)
        self.assertIn("CAPTURE_SOURCE_DURATION_S = 65.0", runner)
        self.assertIn("source_stop_ack_completion_not_received", recorder)
        self.assertIn("STAGE4C_SOURCE_DONE=STOP_ACK_AND_TRANSPORT_PASS", wrapper)
        self.assertIn("node.first_imu_received_at + 75.0", recorder)
        self.assertIn('STAGE4C_RECORDER_ARMED_BEFORE_SERIAL=PASS', runner)
        self.assertIn('STAGE4C_RECORDER_ARMED=1', recorder)
        self.assertIn('armed_path.open("x"', recorder)
        self.assertIn('armed_path.unlink()', recorder)
        self.assertIn('record_mount=(-v "$capture_root:/capture:ro")', wrapper)
        self.assertIn('"${record_mount[@]}"', wrapper)
        self.assertIn("if not LIVE_CAPTURE_COMMISSIONED:", recorder)
        # No hardware passthrough or motor command in the recorder segment.
        recorder_run = wrapper.split('    trap cleanup_capture EXIT', 1)[1].split(
            "printf 'STAGE4A_SELECTED_IMAGE", 1)[0]
        self.assertNotIn("--device=", recorder_run)
        self.assertNotIn("/cmd_vel", recorder_run)
        self.assertIn('test ! -e /dev/ttyACM0', recorder_run)



if __name__ == "__main__":
    unittest.main()
