"""Stage 4B pure-Python fail-closed callback budget and exact-stamp trace tests.

No rclpy, video, serial port or physical device required.
"""
from pathlib import Path
import unittest

from live_stage4b_delivery_trace import DeliveryTrace
from live_stage4b_spin_budget import service_callbacks

ROOT = Path(__file__).resolve().parent


class BoundedSpin(unittest.TestCase):
    def test_more_than_one_spin_per_serial_iteration(self):
        state = {"t": 0.0, "spins": 0, "checks": 0}
        def clock():
            return state["t"]
        def spin():
            state["spins"] += 1
            state["t"] += 0.002
        def check():
            state["checks"] += 1
        attempts, elapsed = service_callbacks(
            spin, check, monotonic=clock, max_spins=8, budget_s=0.006)
        self.assertEqual(attempts, 3)
        self.assertEqual(state["spins"], 3)
        self.assertEqual(state["checks"], 3)
        self.assertAlmostEqual(elapsed, 0.006)

    def test_limit_and_exception_propagation(self):
        state = {"t": 0.0, "spins": 0}
        def clock():
            return state["t"]
        def spin():
            state["spins"] += 1
        n, _ = service_callbacks(spin, lambda: None, monotonic=clock)
        self.assertEqual(n, 8)  # hard cap even if clock has not advanced
        self.assertEqual(state["spins"], 8)
        def fail():
            raise RuntimeError("unexpected_odom_message")
        with self.assertRaisesRegex(RuntimeError, "unexpected_odom_message"):
            service_callbacks(spin, fail, monotonic=clock)
        self.assertEqual(state["spins"], 9)
        for kwargs in (
            {"max_spins": 0}, {"max_spins": 9}, {"budget_s": 0},
            {"budget_s": 0.009}, {"budget_s": float("inf")},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                service_callbacks(spin, lambda: None, **kwargs)

    def test_serial_bounded_multi_spin_integration(self):
        script = (ROOT / "live_stage4_imu_overlap.py").read_text()
        image = (ROOT / "Dockerfile.live-stage4-imu").read_text()
        self.assertIn("from live_stage4b_spin_budget import service_callbacks", script)
        self.assertIn("lambda: rclpy.spin_once(observer, timeout_sec=0.0)", script)
        self.assertIn('spin_stats["spin_attempts"] += attempts', script)
        self.assertIn('"stage4b_executor_service": {', script)
        self.assertIn("send_only_whitelisted_serial_command(device, STOP)", script)
        self.assertIn("COPY live_stage4b_spin_budget.py", image)
        self.assertIn('CMD ["/bin/false"]', image)


class StampMatching(unittest.TestCase):
    def test_exact_matching_and_no_claim_of_transport_loss(self):
        t = DeliveryTrace()
        t.source(1000000000, 10_000_000_000)
        t.source(1020000000, 10_020_000_000)
        t.source(1040000000, 10_040_000_000)
        t.subscriber(1000000000, 10_005_000_000)
        t.subscriber(1040000000, 10_050_000_000)
        r = t.report()
        self.assertEqual(r["matched_exact_header_stamps"], 2)
        self.assertEqual(r["source_unmatched_stamps"], 1)
        self.assertEqual(r["subscriber_unmatched_stamps"], 0)
        self.assertEqual(r["callback_delivery_delay_ms_p50"], 7.5)
        self.assertTrue(r["not_proof_of_dds_packet_loss"])
        self.assertFalse(r["calibration_ready"])

    def test_malformed_and_out_of_order_fail(self):
        t = DeliveryTrace(max_samples=2)
        t.source(100, 1000)
        with self.assertRaises(ValueError):
            t.source(100, 1200)
        with self.assertRaises(ValueError):
            t.source(90, 1300)
        self.assertTrue(t.report()["source_out_of_order"])
        t2 = DeliveryTrace(max_samples=1)
        t2.source(100, 200)
        with self.assertRaises(ValueError):
            t2.source(200, 300)
        with self.assertRaises(ValueError):
            t2.subscriber(100, -1)

    def test_negative_delay_is_not_reported_as_valid_latency(self):
        t = DeliveryTrace()
        t.source(100, 2000)
        t.subscriber(100, 1900)
        r = t.report()
        self.assertEqual(r["negative_host_monotonic_delays"], 1)
        self.assertIsNone(r["callback_delivery_delay_ms_p95"])

    def test_launcher_explicitly_selects_verified_stage4b_image(self):
        launcher = (ROOT / "live_stage4_imu_sidecar.sh").read_text()
        for token in (
            "LEROY_STAGE4_IMU_IMAGE",
            "localhost/leroy-ros:stage4-imu",
            "localhost/leroy-ros:stage4b-imu",
            "UNAPPROVED_STAGE4_IMAGE",
            "STAGE4B_IMAGE_TRACE_NOT_INSTALLED",
            "/opt/stage4/live_stage4b_spin_budget.py",
            'grep -Fq "service_callbacks("',
            "STAGE4B_IMAGE_TRACE=VERIFIED_OFFLINE",
            "--network=none",
            "--read-only",
            'printf \'STAGE4A_SELECTED_IMAGE=%s',
            "--device=\"$mcu:/dev/ttyACM0:rw\"",
            "STAGE4_IMU_ONLY",
            "LIVE_STAGE3_COSTMAP_NOT_ACCEPTED",
        ):
            self.assertIn(token, launcher)
        self.assertNotIn("--network=host", launcher)
        self.assertNotIn("--privileged", launcher)

    def test_integrated_image_stays_mcu_isolated(self):
        script = (ROOT / "live_stage4_imu_overlap.py").read_text()
        image = (ROOT / "Dockerfile.live-stage4-imu").read_text()
        for token in ("on_imu_published=", "trace.source(stamp, time.monotonic_ns())",
                      "trace.subscriber(", '"stage4b_delivery_trace": trace.report()'):
            self.assertIn(token, script)
        self.assertIn("COPY live_stage4b_delivery_trace.py", image)
        self.assertIn('CMD ["/bin/false"]', image)
        self.assertNotIn("device.write(", script)
        self.assertNotIn("XRSLAM_SENSOR_", script)


if __name__ == "__main__":
    unittest.main()
