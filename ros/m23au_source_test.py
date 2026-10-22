"""M23AU: offline safety and shutdown parser tests; no ROS, camera or Podman."""
import os
from pathlib import Path
from unittest.mock import patch
import unittest

import m23au_bounded_camera_probe as m


def approved():
    return {
        **m.M23AQ_ARGS, **m.ISOLATION,
        "LD_LIBRARY_PATH": "/opt/m23aj-opencv/lib:/opt/ros/jazzy/lib",
        "LD_PRELOAD": "",
    }


def clean_log():
    return (
        "[INFO] [usb_cam_node_exe-1]: process has finished cleanly [pid 11]\n"
        "[INFO] [component_container-2]: process has finished cleanly [pid 12]\n"
    )


class OfflineM23AUTests(unittest.TestCase):
    def test_default_inert(self):
        with patch.object(m, "run", side_effect=AssertionError("live start")):
            self.assertEqual(m.main([]), 0)

    def test_approved_gate(self):
        self.assertTrue(m.gate(approved(), exists=lambda _: True))

    def test_power_isolation_required(self):
        env = approved()
        env.pop("M23AQ_MOTOR_POWER_ISOLATION_ATTESTED")
        with self.assertRaisesRegex(m.Blocked, "POWER_ISOLATION"):
            m.gate(env, exists=lambda _: True)

    def test_camera_only_operator_authority_required(self):
        env = approved()
        env.pop("M23AU_CAMERA_ONLY_OPERATOR_APPROVAL")
        with self.assertRaisesRegex(m.Blocked, "M23AU_CAMERA_ONLY"):
            m.gate(env, exists=lambda _: True)

    def test_refuse_other_device(self):
        env = approved()
        env["M23AQ_VIDEO_DEVICE"] = "/dev/ttyACM0"
        with self.assertRaisesRegex(m.Blocked, "M23AQ_VIDEO_DEVICE"):
            m.gate(env, exists=lambda _: True)

    def test_calibration_required(self):
        env = approved()
        env["M23AQ_CALIBRATION_VERIFIED"] = "NO"
        with self.assertRaisesRegex(m.Blocked, "CALIBRATION"):
            m.gate(env, exists=lambda _: True)

    def test_wrong_opencv_precedence_rejected(self):
        env = approved()
        env["LD_LIBRARY_PATH"] = "/usr/lib/x86_64-linux-gnu:/opt/m23aj-opencv/lib"
        with self.assertRaisesRegex(m.Blocked, "pthreads OpenCV"):
            m.gate(env, exists=lambda _: True)

    def test_no_preload(self):
        env = approved()
        env["LD_PRELOAD"] = "/usr/lib/libhack.so"
        with self.assertRaisesRegex(m.Blocked, "LD_PRELOAD"):
            m.gate(env, exists=lambda _: True)

    def test_missing_launch_blocks(self):
        with self.assertRaisesRegex(m.Blocked, "two-node"):
            m.gate(approved(), exists=lambda _: False)

    def test_legacy_qos_variant_rejected(self):
        env = approved()
        env["M23N_CAMERA_QOS_ONLY"] = "1"
        with self.assertRaisesRegex(m.Blocked, "M23N"):
            m.gate(env, exists=lambda _: True)

    def test_both_ros_children_shutdown_clean(self):
        self.assertTrue(m.clean_exit_markers(clean_log()))

    def test_missing_ros_child_exit_blocks(self):
        with self.assertRaisesRegex(m.Blocked, "component_container"):
            m.clean_exit_markers(clean_log().splitlines()[0] + "\n")

    def test_child_segfault_blocks(self):
        with self.assertRaisesRegex(m.Blocked, "crashed"):
            m.clean_exit_markers(clean_log() +
                                 "[ERROR] process has died [pid 12, exit code -11]")

    def test_no_unreviewed_camera_stack_startup(self):
        self.assertEqual(m.CAMERA_LAUNCH, "/opt/m23au/m23aq_camera_only.launch.py")
        self.assertEqual(m.ISOLATION["M23O_SUB_QOS"], "reliable_100")
        self.assertEqual(m.ISOLATION["M23K_PROBE_VARIANT"], "full_mjpeg")
        source = (Path(__file__).resolve().parent /
                  "m23au_bounded_camera_probe.py").read_text()
        self.assertNotIn("robot.launch.py", source)
        self.assertNotIn("leroy_bridge_node", source)
        self.assertNotIn("VideoCapture", source)
        self.assertNotIn("rosbag", source)


if __name__ == "__main__":
    unittest.main()
