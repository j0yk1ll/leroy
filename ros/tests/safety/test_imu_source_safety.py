"""No-device source and explicit IMU safety regression tests.

Does NOT open a serial port or import ROS runtime. Executes only parser logic.
"""
import ast
import pathlib
import sys
import unittest

HERE = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(HERE))
from leroy_imu_observer import imu_protocol as m18


class ImuSourceSafetyContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.supervisor = (HERE / "live_stage4_imu_overlap.py").read_text()
        cls.sidecar = (HERE / "src/leroy_bringup/scripts/live_stage4_imu_sidecar.sh").read_text()
        cls.dockerfile = (HERE / "Dockerfile.live-stage4-imu").read_text()
        cls.run_ast = ast.parse(cls.supervisor)

    def test_all_observed_streams_have_timestamp_tracker(self):
        """Catch CameraInfo KeyError in the callback before opening serial."""
        klass = next(n for n in self.run_ast.body
                     if isinstance(n, ast.ClassDef) and n.name == "PassiveOverlap")
        constructor = next(n for n in klass.body
                           if isinstance(n, ast.FunctionDef) and n.name == "__init__")
        literal = {}
        for n in ast.walk(constructor):
            if isinstance(n, ast.Assign):
                for target in n.targets:
                    if (isinstance(target, ast.Attribute)
                            and isinstance(target.value, ast.Name)
                            and target.value.id == "self"
                            and target.attr in ("counts", "last_stamp")):
                        literal[target.attr] = ast.literal_eval(n.value)
        self.assertEqual(set(literal["last_stamp"]), {"image", "info", "imu"})
        self.assertTrue(set(literal["last_stamp"]).issubset(literal["counts"]))
        for name in ("image", "info", "imu"):
            self.assertIn(f'self._take("{name}"', self.supervisor)

    def test_run_cannot_suppress_shutdown_exceptions(self):
        """Returning from finally could hide STOP/ROS cleanup failures."""
        runner = next(n for n in self.run_ast.body
                      if isinstance(n, ast.FunctionDef) and n.name == "run")
        for node in ast.walk(runner):
            if isinstance(node, ast.Try):
                self.assertFalse(
                    any(isinstance(child, ast.Return)
                        for statement in node.finalbody
                        for child in ast.walk(statement)),
                    "return_inside_finally_suppresses_cleanup_exception")
        self.assertTrue(
            any(isinstance(n, ast.Return) for n in runner.body),
            "run_must_return_a_verdict_after_cleanup")

    def test_serial_commands_are_allowlisted(self):
        self.assertEqual(m18.START, b"set_imu_stream_hz 50\r\n")
        self.assertEqual(m18.STOP, b"set_imu_stream_hz 0\r\n")
        self.assertEqual(m18.PORT, "/dev/ttyACM0")
        s = self.supervisor
        self.assertIn("send_only_whitelisted_serial_command(device, START)", s)
        self.assertIn("send_only_whitelisted_serial_command(device, STOP)", s)
        self.assertIn("stopped = wait_ack(device", s)
        self.assertIn("STAGE4A_IMU_STOP_ACK=", s)
        self.assertIn("STAGE4A_MCU_STOP_STATE=UNVERIFIED_PHYSICALLY_ISOLATE", s)
        # No alternate serial writer or motor bridge node.
        self.assertNotIn("device.write(", s)
        self.assertNotIn("leroy_bridge.node", s)
        self.assertNotIn("subprocess.", s)
        self.assertNotIn("os.system(", s)
        self.assertIn("if not stopped:", s)

    def test_fail_closed_runtime_and_auditing(self):
        s = self.supervisor
        for token in (
            "FORBIDDEN_TOPICS", "FORBIDDEN_NODES",
            "self.count_publishers(\"/imu/data_raw\") > 1",
            'pub[0].node_name == "m18_imu_source"',
            "unexpected_odom_message", "unexpected_localization_authority",
            "stage3_graph_or_false_validity_not_ready",
            "camera_imu_absolute_offset_validated",
            "camera_imu_extrinsics_validated",
            "xrslam_vio_started", "metric_odometry_authorized",
            'result["counts"]["stale_or_future_withheld"] == 0',
            'result["counts"]["calibration_withheld"] == 0',
            'result["counts"]["invalid_ascii_lines"] == 0',
            'result["counts"]["unexpected_serial_lines"] == 0',
            "STAGE4A_IMU_OVERLAP=",
            "PASS_TRANSPORT_ONLY",
        ):
            self.assertIn(token, s)
        for token in (
            "ros2 lifecycle set", "XRSLAM_SENSOR", "/cmd_vel", "/odom",
        ):
            # Except deliberate denial/monitoring in source, no commands.
            if token in ("ros2 lifecycle set", "XRSLAM_SENSOR"):
                self.assertNotIn(token, s)

    def test_guarded_serial_is_in_separate_contained_process(self):
        s = self.sidecar
        for token in (
            "LEROY_STAGE4_MOTOR_ISOLATION_ATTESTED",
            "LEROY_STAGE4_MCU_BY_ID",
            "/dev/serial/by-id/",
            "STAGE4_IMU_ONLY",
            "STAGE3_PARENT_NOT_RUNNING",
            "MOTOR_CAPABLE_MCU_VISIBLE_TO_PARENT",
            "MOTOR_CAPABLE_MCU_VISIBLE_TO_XRSLAM",
            "--network \"container:$parent\"",
            "--ipc \"container:$parent\"",
            "--pid \"container:$parent\"",
            '--device="$mcu:/dev/ttyACM0:rw"',
            "--cap-drop=all", "--security-opt=no-new-privileges",
            "--read-only",
            "timeout --foreground", "--signal=INT",
            "LEROY_STAGE4_IMU_ONLY=1",
            "LEROY_NAV2_DRY_RUN=1",
            "ros2 run leroy_nav2_perception live_costmap_probe",
            "LIVE_STAGE3_COSTMAP_NOT_ACCEPTED",
        ):
            self.assertIn(token, s)
        for forbidden in (
            "--network=host", "--pid=host", "--ipc=host",
            "--privileged", "--device=/dev/video2", "--device=/dev/gpiochip",
            "ros2 launch leroy_bridge",
        ):
            self.assertNotIn(forbidden, s)

    def test_no_runtime_permission_from_image(self):
        s = self.dockerfile
        self.assertIn("FROM localhost/leroy-ros:costmap", s)
        self.assertIn("src/leroy_imu_observer", s)
        self.assertIn("leroy_imu_observer.imu_protocol", s)
        self.assertIn('CMD ["/bin/false"]', s)
        self.assertNotIn("ENTRYPOINT", s)
        self.assertNotIn("CMD [\"/usr/local/bin/leroy-entrypoint\"]", s)

    def test_imu_packet_sequence_contract(self):
        fields = {
            "seq": 17, "t_us": 1_000_000, "cal": 1,
            "af_mm_s2": 100, "al_mm_s2": 200, "au_mm_s2": 9900,
            "gr_mrad_s": 1, "gp_mrad_s": 2, "gy_mrad_s": 3,
            "mag_valid": 0, "mx_raw": 0, "my_raw": 0, "mz_raw": 0,
        }
        line = "telemetry imu " + " ".join(f"{k}={fields[k]}" for k in m18.FIELD_NAMES)
        self.assertEqual(m18.parse_packet(line), fields)
        self.assertEqual(m18.sequence_gap(17, 18), 0)
        with self.assertRaises(m18.PacketError):
            m18.sequence_gap(18, 18)
        with self.assertRaises(m18.PacketError):
            m18.parse_packet(line + " unexpected=3")


if __name__ == "__main__":
    unittest.main()
