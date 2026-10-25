#!/usr/bin/env python3
"""Necessary source-only safety preflight for physical Stage 2B (NO devices)."""
import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parent


class Stage2BContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.host = (ROOT / "m23bv_v3_stage2b_host_run.sh").read_text()
        cls.probe = (ROOT / "m23bv_v3_stage2b_physical_probe.py").read_text()
        cls.ast = ast.parse(cls.probe)
        cls.stage1 = (ROOT / "m23bv_v3_stage1_camera_observer.py").read_text()
        cls.m20 = (ROOT / "m20_combined_observer.py").read_text()
        cls.m18 = (ROOT / "m18_imu_stream_source.py").read_text()
        cls.preflight = (ROOT / "m23bv_v3_stage2b_preflight.sh").read_text()

    def test_exact_unchanged_m20_and_v3_source_pins(self):
        for required in (
            "a0ae75a62da39235b113e3d2c324efda5fcfc6e1",
            "93320b985c5dda3fdf0d1f8def001ba06480e7b6",
            "5cf2f88cb2f79be80821888401b57a6393ee5e3b",
            "66111d8da486dc1e0c78057c1948f5e1aad9a06e",
            "3c5fb37ca291533e2702a9159f3e8db11b076d22",
            "443aa82066e88d50cb88859304e9197e3f459ee0",
            "f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c",
        ):
            self.assertIn(required, self.probe + self.host)
        self.assertIn("git hash-object", self.host)
        self.assertIn('git status --porcelain', self.host)
        self.assertIn('BRANCH=', self.host)

    def test_real_mcu_is_only_started_through_protected_m20(self):
        self.assertIn('["python3", "-u", M20_OBSERVER]', self.probe)
        self.assertIn('M20_OBSERVER = "/opt/m20/m20_combined_observer.py"', self.probe)
        self.assertNotIn("serial.Serial(", self.probe)
        self.assertNotIn("device.write(", self.probe)
        self.assertIn("send_only_whitelisted_serial_command(device, START)", self.m20)
        self.assertIn("send_only_whitelisted_serial_command(device, STOP)", self.m20)
        self.assertIn("M20_IMU_STREAM_STOP_ACK=PASS", self.host)
        self.assertIn("M20_COMBINED_VERDICT=OBSERVED_CONCURRENT_CAMERA_IMU_ROS_CONTRACT",
                      self.host)
        self.assertIn("M20_COMBINED_VERDICT=OBSERVED_CONCURRENT_CAMERA_IMU_ROS_CONTRACT",
                      self.probe)

    def test_two_exact_source_overlays_and_protected_stop_region(self):
        # Historical V3 image has older M20/M18 source blobs. The reviewed
        # M20 cleanup fix and M18 terminal-fault latch must be mounted.
        mounts = (
            'type=bind,src=$PWD/m20_combined_observer.py,dst=/opt/m20/m20_combined_observer.py,readonly',
            'type=bind,src=$PWD/m18_imu_stream_source.py,dst=/opt/m18/m18_imu_stream_source.py,readonly',
        )
        for mount in mounts:
            self.assertIn(mount, self.host)
            self.assertIn(mount, self.preflight)
        self.assertIn("M23BV_V3_STAGE2B_CORRECTED_M20_M18_MOUNTS=PASS",
                      self.preflight)
        self.assertIn("BLOCKED_IMPORT_LOCATION", self.preflight)
        self.assertIn("CORRECTED_M20_PIN", self.preflight)
        self.assertIn("CORRECTED_M18_PIN", self.preflight)
        self.assertIn("device.reset_input_buffer()", self.m20)
        protected = self.m20.split("with serial.Serial(", 1)[1]
        self.assertLess(protected.index("try:"),
                        protected.index("device.reset_input_buffer()"))
        self.assertLess(protected.index("device.reset_input_buffer()"),
                        protected.index("finally:"))
        self.assertLess(protected.index("finally:"),
                        protected.index("send_only_whitelisted_serial_command(device, STOP)"))
        self.assertIn("if self.terminal_fault is not None:", self.m18)
        self.assertIn("imu_source_terminal_fault_latched", self.m18)

    def test_one_explicit_yes_covers_all_physical_conditions(self):
        # Exactly one interactive input after all passive source, ID,
        # calibration, container, and host-group preflight gates.
        self.assertEqual(self.host.count("IFS= read -r "), 1)
        self.assertEqual(self.host.count("Type YES"), 1)
        self.assertNotIn("Type exactly:", self.host)
        self.assertIn('IFS= read -r operator_confirmation </dev/tty', self.host)
        self.assertIn('[[ "$operator_confirmation" == YES ]]', self.host)
        self.assertIn('OPERATOR_CONFIRMATION_MISSING', self.host)
        self.assertIn('OPERATOR_CONFIRMATION_DECLINED', self.host)
        for token in (
            "BOTH motor-power rails are PHYSICALLY DISCONNECTED",
            "The chassis is secured and immobilized",
            "E-stop has been tested",
            "A supervisor is present",
            "immediately disconnect power",
            "real front-camera feed is $cam",
            "pinned camera calibration belongs to that verified physical camera",
            "NO motor motion, rosbag, Nav2 or XRSLAM",
            "missing STOP acknowledgment",
            "M23BV_STAGE2B_MCU_BY_ID",
            "M23BV_STAGE2B_MCU_ID_SERIAL",
            "MCU_SERIAL_ID_MISMATCH",
            "CAMERA_SERIAL_CHANGED",
            "emergency",
        ):
            if token == "emergency":
                continue
            self.assertIn(token, self.host)
        # Single opt-in remains after passive identity checks and before
        # fresh camera/MCU rechecks, approval log and exact two mappings.
        self.assertLess(self.host.index('KEEP_GROUPS_NEEDS_ROOTLESS_CRUN'),
                        self.host.index('IFS= read -r operator_confirmation'))
        self.assertLess(self.host.index('OPERATOR_CONFIRMATION_DECLINED'),
                        self.host.index('CAMERA_CHANGED_AFTER_SIGNOFF'))
        self.assertLess(self.host.index('OPERATOR_CONFIRMATION_DECLINED'),
                        self.host.index('podman run'))
        self.assertEqual(self.host.count('  --device "'), 2)
        self.assertIn('--device "$cam:/dev/video2:rw"', self.host)
        self.assertIn('--device "$mcu:/dev/ttyACM0:rw"', self.host)
        self.assertIn('m23bv_v3_stage2b_approval_', self.host)
        self.assertIn('chmod 0600', self.host)
        self.assertIn('Single operator consent: YES', self.host)

    def test_camera_hotplug_uses_only_verified_by_id_and_serial(self):
        self.assertIn('M23BV_STAGE2B_CAMERA_STABLE_LINK', self.host)
        self.assertIn('M23BV_STAGE2B_CAMERA_VISUALLY_VERIFIED_NODE', self.host)
        self.assertIn(
            "pci-0000:00:14.0-usb-0:3.4.2:1.0-video-index0", self.host)
        self.assertIn("CAMERA_APPROVED_ID_PATH='pci-0000:00:14.0-usb-0:3.4.2:1.0'",
                      self.host)
        self.assertIn("CAMERA_VENDOR_ID='32e6'", self.host)
        self.assertIn("CAMERA_MODEL_ID='9221'", self.host)
        self.assertIn('"$CAMERA_BY_ID" == "$CAMERA_APPROVED_LINK"', self.host)
        self.assertIn('CAMERA_APPROVED_USB_PORT_LINK_REQUIRED', self.host)
        self.assertIn('"$cam_id_path" == "$CAMERA_APPROVED_ID_PATH"', self.host)
        self.assertIn('"$cam_vendor" == "$CAMERA_VENDOR_ID"', self.host)
        self.assertIn('"$cam_model" == "$CAMERA_MODEL_ID"', self.host)
        self.assertIn('CAMERA_USB_PORT_CHANGED_AFTER_SIGNOFF', self.host)
        self.assertIn('CAMERA_USB_VENDOR_MODEL_CHANGED_AFTER_SIGNOFF', self.host)
        self.assertIn('CAMERA_LINK_NOT_VISUALLY_VERIFIED_STREAM', self.host)
        self.assertIn('"$cam" == "$CAMERA_VERIFIED_NODE"', self.host)
        self.assertIn('real front-camera feed is $cam', self.host)
        self.assertIn('OPERATOR_CONFIRMATION_DECLINED', self.host)
        self.assertIn('"$cam_v4l_caps" == *:capture:*', self.host)
        self.assertIn('CAMERA_CAPABILITY_CHANGED_AFTER_SIGNOFF', self.host)
        self.assertIn("CAMERA_SERIAL='Wed_Camera_Wed_Camera_20240420112206'",
                      self.host)
        self.assertNotIn('"$cam" == /dev/video8', self.host)
        self.assertIn('"$cam" =~ ^/dev/video[0-9]+$ && -c "$cam"', self.host)
        self.assertIn('cam="$(readlink -f -- "$CAMERA_BY_ID")"', self.host)
        self.assertIn('"$cam_serial" == "$CAMERA_SERIAL"', self.host)
        self.assertIn('cam_rdev="$(stat -Lc', self.host)
        self.assertIn('"$(readlink -f -- "$CAMERA_BY_ID")" == "$cam"',
                      self.host)
        self.assertIn('"$(stat -Lc \'%t:%T\' -- "$cam")" == "$cam_rdev"',
                      self.host)
        self.assertIn('CAMERA_SERIAL_CHANGED_AFTER_SIGNOFF', self.host)
        self.assertIn('MCU_CHANGED_AFTER_SIGNOFF', self.host)
        self.assertIn('MCU_SERIAL_CHANGED_AFTER_SIGNOFF', self.host)
        self.assertIn('--device "$cam:/dev/video2:rw"', self.host)
        # Hotplug checks occur after fresh signoff and before the first
        # camera/MCU --device mapping in Podman.
        self.assertLess(self.host.index("OPERATOR_CONFIRMATION_DECLINED"),
                        self.host.index("CAMERA_CHANGED_AFTER_SIGNOFF"))
        self.assertLess(self.host.index("CAMERA_LINK_NOT_VISUALLY_VERIFIED_STREAM"),
                        self.host.index("podman run"))
        self.assertLess(self.host.index("CAMERA_CHANGED_AFTER_SIGNOFF"),
                        self.host.index("podman run"))
        self.assertLess(self.host.index("MCU_SERIAL_CHANGED_AFTER_SIGNOFF"),
                        self.host.index("podman run"))

    def test_rootless_crun_retains_only_existing_host_mcu_group(self):
        self.assertIn("mcu_gid=\"$(stat -Lc '%g' -- \"$mcu\")\"", self.host)
        self.assertIn('[[ " $(id -G) " == *" $mcu_gid "* ]]', self.host)
        self.assertIn('[[ -r "$mcu" && -w "$mcu" ]]', self.host)
        self.assertIn('HOST_MCU_GROUP_MEMBERSHIP_MISSING', self.host)
        self.assertIn('HOST_MCU_ACCESS_DENIED', self.host)
        self.assertIn('KEEP_GROUPS_NEEDS_ROOTLESS_CRUN', self.host)
        self.assertIn('podman info --format \'{{.Host.Security.Rootless}}\'', self.host)
        self.assertIn('podman info --format \'{{.Host.OCIRuntime.Name}}\'', self.host)
        self.assertIn('  --group-add keep-groups \\', self.host)
        self.assertLess(self.host.index('KEEP_GROUPS_NEEDS_ROOTLESS_CRUN'),
                        self.host.index('podman run'))
        self.assertEqual(self.host.count('  --group-add keep-groups'), 1)
        self.assertNotIn('--privileged', self.host.split('podman run', 1)[1])
        self.assertNotIn('--userns=host', self.host.split('podman run', 1)[1])
        self.assertNotIn('--group-add uucp', self.host)
        self.assertEqual(self.host.count('  --device "'), 2)

    def test_pre_serial_gate_and_original_m20_lifecycle(self):
        self.assertLess(self.probe.index("graph_pre_serial(node)"),
                        self.probe.index('m20_proc = subprocess.Popen'))
        self.assertIn("MIN_PRECAM_STAMPS = 90", self.probe)
        self.assertIn('M23BV_V3_STAGE2B_PRE_SERIAL_GRAPH_QOS=PASS', self.probe)
        self.assertIn("stage1.info_matches_calibration", self.probe)
        self.assertIn("stage1.ALLOWED_TOPICS", self.probe)
        self.assertIn("PRE_SERIAL_UNKNOWN_TOPICS", self.probe)
        self.assertIn("PRE_SERIAL_UNKNOWN_NODES", self.probe)
        self.assertIn("M20_MAX_S = 155", self.probe)
        self.assertIn("M20_TERM_GRACE_S = 28", self.probe)

    def test_stop_imu_before_camera_and_no_recording(self):
        self.assertLess(self.probe.index("m20_proc.send_signal(signal.SIGTERM)"),
                        self.probe.index("camera_clean = stop_camera(camera_proc)"))
        self.assertIn("BLOCKED_M20_STOP_UNVERIFIED_MANUAL_POWER_ISOLATION",
                      self.probe)
        self.assertIn("M23BV_V3_STAGE2B_CAMERA_LAUNCH_REAPED=", self.probe)
        self.assertIn('timeout --foreground', self.host)
        self.assertIn("sha256sum", self.host)
        self.assertIn("--network none", self.host)
        self.assertIn("--cap-drop all", self.host)
        self.assertIn("--read-only", self.host)
        for src in (self.host, self.probe):
            self.assertNotIn("ros2 bag record", src)
            self.assertNotIn("rosbag2_py", src)
            self.assertNotIn("cmd_vel", src)
            self.assertNotIn("leroy_bridge_node", src)

    def test_probe_has_no_direct_serial_api(self):
        roots = set()
        for node in ast.walk(self.ast):
            if isinstance(node, ast.Import):
                roots.update(x.name.split(".")[0] for x in node.names)
            if isinstance(node, ast.ImportFrom) and node.module:
                roots.add(node.module.split(".")[0])
        # Inert pyserial dependency import is permitted; the unchanged M20
        # observer alone may instantiate or use the motor-capable port.
        self.assertIn("serial", roots)
        self.assertNotIn("serial.Serial(", self.probe)
        self.assertNotIn("device.write(", self.probe)
        self.assertNotIn("rosbag2_py", roots)
        self.assertNotIn("cv2", roots)


if __name__ == "__main__":
    unittest.main(verbosity=2)
