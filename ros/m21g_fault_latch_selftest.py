#!/usr/bin/env python3
"""M21G regression: simulated MCU bytes; NO serial, V4L2 or motor hardware.

Check terminal-fault rejection and preserve M18/M20 default no-hook path.
"""
import os
import unittest

import rclpy

from m18_imu_stream_source import ImuStreamSource
from m21d_metadata_capture import MetadataCapture
from m21f_imu_source_hook_selftest import fake_packet, assert_no_physical_devices

ENV = {
    "LEROY_M20_SENSOR_ONLY": "1", "LEROY_NAV2_DRY_RUN": "1",
    "ROS_DOMAIN_ID": "170", "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
}


class FaultLatchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        assert_no_physical_devices()
        for key, value in ENV.items():
            if os.environ.get(key) != value:
                raise RuntimeError("M21G refuses non-isolated ROS environment: " + key)
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def make_source(self, callback=None):
        node = ImuStreamSource(on_imu_published=callback)
        self.addCleanup(node.destroy_node)
        self.assertEqual(node._imu_pub.topic_name, "/imu/data_raw")
        return node

    def test_default_no_hook_maintains_publication(self):
        src = self.make_source()
        src.consume(fake_packet(0))
        src.consume(fake_packet(1))
        self.assertEqual(src.counts["published_imu_messages"], 2)
        self.assertIsNone(src.terminal_fault)

    def test_callback_failure_latches_before_subsequent_publish(self):
        calls = []
        def broken(*args):
            calls.append(args)
            raise ValueError("synthetic disk failure")
        src = self.make_source(broken)
        with self.assertRaisesRegex(RuntimeError, "imu_metadata_callback_failed"):
            src.consume(fake_packet(0))
        # First publish already happened. Hook is post-publish, so it
        # cannot retract that ROS message.
        self.assertEqual(src.counts["published_imu_messages"], 1)
        self.assertEqual(src.terminal_fault, "imu_metadata_callback_failed")
        self.assertEqual(len(calls), 1)
        original_seq = src.last_sequence
        for _ in range(2):
            with self.assertRaisesRegex(RuntimeError, "imu_source_terminal_fault_latched"):
                src.consume(fake_packet(1))
        self.assertEqual(src.last_sequence, original_seq)
        self.assertEqual(src.counts["published_imu_messages"], 1)
        self.assertEqual(len(calls), 1)

    def test_bounded_capture_overflow_latches_fault(self):
        recorder = MetadataCapture(max_events=6)
        # Metadata capture is already at capacity. Do not create fake
        # timestamps or packets to bypass the capacity check.
        for index in range(2):
            base = 1_900_000_000_000_000_000 + index * 100_000_000
            recv = 2_000_000_000 + index * 100_000_000
            recorder.image(frame_index=index, stamp_ns=base,
                           receipt_mono_ns=recv,
                           frame_id="camera_optical_frame", width=640, height=480)
            recorder.camera_info(stamp_ns=base + 1000,
                                 receipt_mono_ns=recv + 1000,
                                 frame_id="camera_optical_frame", width=640, height=480)
            recorder.imu_source(mcu_seq=index, mcu_t_us=index * 100000,
                                stamp_ns=base + 2000,
                                receipt_mono_ns=recv + 2000,
                                gyro_xyz_rad_s=[0.0, 0.0, 0.1])
        self.assertEqual(len(recorder.rows), 7)
        def overfill(fields, ros_stamp, steady_stamp):
            recorder.imu_source(mcu_seq=fields["seq"], mcu_t_us=fields["t_us"],
                                stamp_ns=ros_stamp, receipt_mono_ns=steady_stamp,
                                gyro_xyz_rad_s=[0.0, 0.0, 0.1])
        src = self.make_source(overfill)
        with self.assertRaisesRegex(RuntimeError, "imu_metadata_callback_failed"):
            src.consume(fake_packet(0))
        self.assertEqual(len(recorder.rows), 7)
        self.assertEqual(src.counts["published_imu_messages"], 1)
        with self.assertRaisesRegex(RuntimeError, "imu_source_terminal_fault_latched"):
            src.consume(fake_packet(1))
        self.assertEqual(src.counts["published_imu_messages"], 1)

    def test_withheld_and_malformed_packets_skip_hook(self):
        records = []
        src = self.make_source(lambda *args: records.append(args))
        src.consume(fake_packet(0, calibration=0))
        src.consume(b"telemetry imu incomplete\r\n")
        self.assertEqual(records, [])
        self.assertEqual(src.counts["published_imu_messages"], 0)
        src.consume(fake_packet(1))
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0][0]["seq"], 1)
        self.assertEqual(records[0][0]["t_us"], 20000)
        self.assertEqual(src.counts["published_imu_messages"], 1)

    def test_other_latched_fault_prevents_publication(self):
        src = self.make_source()
        src.terminal_fault = "synthetic_source_clock_fault"
        with self.assertRaisesRegex(RuntimeError, "imu_source_terminal_fault_latched"):
            src.consume(fake_packet(0))
        self.assertEqual(src.counts["received_imu_packets"], 0)
        self.assertEqual(src.counts["published_imu_messages"], 0)

    def test_noncallable_hook_rejected(self):
        with self.assertRaisesRegex(TypeError, "callable"):
            ImuStreamSource(on_imu_published=123)


if __name__ == "__main__":
    assert_no_physical_devices()
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(FaultLatchTests))
    if result.wasSuccessful():
        print("M21G_SOURCE_FAULT_SELFTEST=PASS", flush=True)
        print("M21G_VERDICT=SIMULATED_TERMINAL_FAULT_LATCH_CONTRACT", flush=True)
        print("M21G_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    print("M21G_SOURCE_FAULT_SELFTEST=FAIL", flush=True)
    raise SystemExit(2)
