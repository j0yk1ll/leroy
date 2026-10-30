"""Stage 4D no-device validation, fault injection and virtual 75-second load."""
import gc
import json
import weakref
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

from live_stage4c_capture_contract import (
    CaptureContract, CaptureError, METADATA_TOPIC, TOPICS,
)
from live_stage4d_capture_ingress import CaptureIngress

HERE = Path(__file__).resolve().parent
T0 = 1_790_000_000_000_000_000
R0 = 2_000_000_000_000
IMAGE_CDR = b"v" * (640 * 480)


def header(stamp, frame):
    sec, nanosec = divmod(stamp, 1_000_000_000)
    return NS(stamp=NS(sec=sec, nanosec=nanosec), frame_id=frame)


def image(stamp, data=b"0123"):
    return NS(
        header=header(stamp, "camera_optical_frame"), width=2,
        height=2, step=2, encoding="mono8", data=data, cdr=data)


def camera_info(stamp):
    return NS(
        header=header(stamp, "camera_optical_frame"), width=640,
        height=480, k=[500., 0., 320., 0., 500., 240., 0., 0., 1.],
        d=[0., 0., 0., 0., 0.], cdr=b"camera_info")


def imu(stamp):
    return NS(
        header=header(stamp, "imu_link"),
        angular_velocity=NS(x=0.0, y=0.0, z=0.1),
        linear_acceleration=NS(x=0.0, y=0.0, z=9.8),
        cdr=b"imu_cdr")


def meta(stamp, seq=1, ticks=20_000, published=R0):
    data = json.dumps({
        "schema": "leroy_stage4c_mcu_v1",
        "publisher_node": "/m18_imu_source",
        "seq": seq, "t_us": ticks, "imu_stamp_ns": stamp,
        "publish_monotonic_ns": published, "cal": 1,
    }, separators=(",", ":"), sort_keys=True)
    return NS(data=data, cdr=data.encode("ascii"))


class IngressNoDevice(unittest.TestCase):
    def setUp(self):
        self.calls = []
        self.policy = CaptureContract()
        self.ingress = CaptureIngress(
            contract=self.policy, serialize=lambda m: m.cdr,
            write=lambda t, raw, stamp: self.calls.append((t, len(raw), stamp)))

    def test_all_four_topics_and_matching_original_ticks(self):
        stamp = T0
        self.ingress.accept("/camera/image_rect", image(stamp), receipt_ns=R0+2)
        self.ingress.accept("/camera/camera_info", camera_info(stamp), receipt_ns=R0+3)
        self.ingress.accept("/imu/data_raw", imu(stamp), receipt_ns=R0+4)
        self.ingress.accept(METADATA_TOPIC, meta(stamp, published=R0), receipt_ns=R0+5)
        self.assertEqual(len(self.calls), 4)
        self.assertEqual(self.policy.report()["matched_imu_metadata_stamps"], 1)
        self.assertFalse(self.policy.report()["physical_source_authenticated"])

    def test_invalid_image_geometry_latches_before_write(self):
        bad = image(T0, data=b"short")
        with self.assertRaisesRegex(CaptureError, "invalid_image_geometry_or_payload"):
            self.ingress.accept("/camera/image_rect", bad, receipt_ns=R0+2)
        self.assertEqual(self.calls, [])
        with self.assertRaisesRegex(CaptureError, "ingress_terminal_fault_latched"):
            self.ingress.accept("/imu/data_raw", imu(T0), receipt_ns=R0+3)

    def test_invalid_camera_intrinsics_latches_before_write(self):
        bad = camera_info(T0)
        bad.k[0] = float("nan")
        with self.assertRaisesRegex(CaptureError, "invalid_camera_intrinsics"):
            self.ingress.accept("/camera/camera_info", bad, receipt_ns=R0+1)
        self.assertEqual(self.calls, [])

    def test_invalid_imu_latches_before_write(self):
        bad = imu(T0)
        bad.angular_velocity.z = float("inf")
        with self.assertRaisesRegex(CaptureError, "nonfinite_imu_measurement"):
            self.ingress.accept("/imu/data_raw", bad, receipt_ns=R0+1)
        self.assertEqual(self.calls, [])

    def test_metadata_forged_fields_latch_before_write(self):
        bad = meta(T0)
        record = json.loads(bad.data)
        record["t_us"] = -1
        bad.data = json.dumps(record)
        with self.assertRaisesRegex(CaptureError, "invalid_mcu_ticks"):
            self.ingress.accept(METADATA_TOPIC, bad, receipt_ns=R0+2)
        self.assertEqual(self.calls, [])

    def test_invalid_header_latches_before_serialization(self):
        calls = []
        ingress = CaptureIngress(
            serialize=lambda msg: calls.append("serialize") or msg.cdr,
            write=lambda *args: calls.append("write"))
        bad = image(T0)
        bad.header.stamp.sec = "invalid"
        with self.assertRaises(ValueError):
            ingress.accept("/camera/image_rect", bad, receipt_ns=R0+2)
        self.assertEqual(calls, [])
        self.assertIsNotNone(ingress.terminal_fault)

    def test_writer_disk_full_latches_and_keeps_accounting_atomic(self):
        n = [0]
        def fail_after_one(*args):
            n[0] += 1
            if n[0] == 2:
                raise OSError("simulated_disk_full")
        ingress = CaptureIngress(
            contract=self.policy, serialize=lambda m: m.cdr, write=fail_after_one)
        ingress.accept("/imu/data_raw", imu(T0), receipt_ns=R0+2)
        with self.assertRaisesRegex(OSError, "simulated_disk_full"):
            ingress.accept("/imu/data_raw", imu(T0+20_000_000), receipt_ns=R0+20_000_002)
        self.assertEqual(self.policy.counts["/imu/data_raw"], 1)
        self.assertIsNotNone(ingress.terminal_fault)
        with self.assertRaisesRegex(CaptureError, "ingress_terminal_fault_latched"):
            ingress.accept("/imu/data_raw", imu(T0+40_000_000), receipt_ns=R0+40_000_002)
        self.assertEqual(n[0], 2)

    def test_payload_limit_latches_before_write(self):
        large = image(T0)
        large.width = 1
        large.height = 1
        large.step = 1
        large.data = b"z"
        large.cdr = b"x" * 4_000_001
        with self.assertRaisesRegex(CaptureError, "invalid_serialized_payload"):
            self.ingress.accept("/camera/image_rect", large, receipt_ns=R0+2)
        self.assertFalse(self.calls)

    def test_serializer_failure_latches_before_write(self):
        ingress = CaptureIngress(
            contract=self.policy,
            serialize=lambda msg: (_ for _ in ()).throw(RuntimeError("serialize")),
            write=lambda *a: self.calls.append(a))
        with self.assertRaisesRegex(RuntimeError, "serialize"):
            ingress.accept("/imu/data_raw", imu(T0), receipt_ns=R0+1)
        self.assertEqual(self.calls, [])
        self.assertEqual(self.policy.counts["/imu/data_raw"], 0)

    def test_close_releases_bound_writer_reference_before_read_only_replay(self):
        class FakeBagWriter:
            def write(self, _topic, _raw, _stamp):
                return None

        writer = FakeBagWriter()
        ref = weakref.ref(writer)
        ingress = CaptureIngress(
            contract=CaptureContract(), serialize=lambda msg: msg.cdr,
            write=writer.write)
        ingress.accept("/imu/data_raw", imu(T0), receipt_ns=R0+1)
        del writer
        gc.collect()
        self.assertIsNotNone(
            ref(), "An ingress holding writer.write keeps the writer alive")
        ingress.close()
        gc.collect()
        self.assertIsNone(
            ref(), "Closing ingress must release its bound writer reference")
        with self.assertRaisesRegex(CaptureError, "ingress_terminal_fault_latched"):
            ingress.accept("/imu/data_raw", imu(T0+20_000_000),
                           receipt_ns=R0+20_000_001)
        ingress.close()  # idempotent

    def test_close_preserves_existing_failure_reason_and_blocks_writes(self):
        def fail(_topic, _raw, _stamp):
            raise OSError("disk_full")
        ingress = CaptureIngress(
            serialize=lambda msg: msg.cdr, write=fail)
        with self.assertRaisesRegex(OSError, "disk_full"):
            ingress.accept("/imu/data_raw", imu(T0), receipt_ns=R0+1)
        prior = ingress.terminal_fault
        ingress.close()
        self.assertEqual(ingress.terminal_fault, prior)
        self.assertIsNone(ingress._write)
        with self.assertRaisesRegex(CaptureError, "ingress_terminal_fault_latched"):
            ingress.accept("/imu/data_raw", imu(T0+20_000_000),
                           receipt_ns=R0+20_000_001)

    def test_virtual_75s_50hz_imu_and_30hz_camera_no_disk_or_dds(self):
        """12k callbacks/75s virtual clock; NOT a real DDS throughput test."""
        policy = CaptureContract()
        bytes_written = [0]
        def count_writer(_topic, payload, _stamp):
            bytes_written[0] += len(payload)
        ingress = CaptureIngress(
            contract=policy, serialize=lambda msg: msg.cdr,
            write=count_writer)
        events = []
        for i in range(3750):
            t = i * 20_000_000
            stamp = T0 + t
            events.append((t+100, "/imu/data_raw", imu(stamp)))
            events.append((t+200, METADATA_TOPIC, meta(
                stamp, seq=i, ticks=i*20_000, published=R0+t+150)))
        camera_cdr = IMAGE_CDR
        camera_data = camera_cdr
        for j in range(2250):
            t = j * 1_000_000_000 // 30
            stamp = T0 + t
            img = NS(header=header(stamp, "camera_optical_frame"),
                     width=640, height=480, step=640, encoding="mono8",
                     data=camera_data, cdr=camera_cdr)
            events.append((t+300, "/camera/image_rect", img))
            events.append((t+400, "/camera/camera_info", camera_info(stamp)))
        events.sort(key=lambda row: row[0])
        for delta, topic, msg in events:
            ingress.accept(topic, msg, receipt_ns=R0+delta)
        result = policy.report()
        self.assertEqual(sum(result["counts"].values()), 12_000)
        self.assertEqual(result["counts"]["/imu/data_raw"], 3750)
        self.assertEqual(result["counts"][METADATA_TOPIC], 3750)
        self.assertEqual(result["counts"]["/camera/image_rect"], 2250)
        self.assertEqual(result["counts"]["/camera/camera_info"], 2250)
        self.assertEqual(result["matched_imu_metadata_stamps"], 3750)
        self.assertEqual(result["readiness"], "INPUT_FORMAT_COMPLETE_UNVERIFIED")
        self.assertEqual(result["total_serialized_bytes"], bytes_written[0])
        self.assertLess(result["total_serialized_bytes"], 900_000_000)
        self.assertFalse(result["physical_source_authenticated"])
        self.assertFalse(result["physical_camera_imu_offset_validated"])


class IntegrationGuards(unittest.TestCase):
    def test_real_candidate_uses_same_ingress(self):
        recorder = (HERE / "live_stage4c_passive_recorder.py").read_text()
        self.assertIn("from live_stage4d_capture_ingress import CaptureIngress", recorder)
        self.assertIn("self.ingress = CaptureIngress(", recorder)
        self.assertIn("self.ingress.accept(topic, msg)", recorder)
        self.assertIn("node.ingress.close()", recorder)
        load = (HERE / "live_stage4d_synthetic_load.py").read_text()
        self.assertIn("ingress.close()", load)
        self.assertLess(load.index("ingress.close()"), load.index("del writer"))
        self.assertIn("LIVE_CAPTURE_COMMISSIONED = True", recorder)
        self.assertIn('STAGE4C=BLOCKED_LIVE_CAPTURE_NOT_COMMISSIONED', recorder)
        self.assertNotIn("serial.Serial(", recorder)
        self.assertNotIn("create_publisher(", recorder)
        docker = (HERE / "Dockerfile.live-stage4c-capture").read_text()
        self.assertIn(
            "COPY live_stage4d_capture_ingress.py "
            "/opt/stage4c/live_stage4d_capture_ingress.py", docker)
        self.assertIn(
            "/opt/stage4c/live_stage4d_capture_ingress.py", docker)

    def test_ingress_has_no_device_ros_or_filesystem_imports(self):
        code = (HERE / "live_stage4d_capture_ingress.py").read_text()
        for forbidden in ("import rclpy", "import serial", "from sensor_msgs",
                          "subprocess", "create_publisher(", "open(",
                          "/dev/ttyACM0", "/cmd_vel"):
            self.assertNotIn(forbidden, code)


if __name__ == "__main__":
    unittest.main()
