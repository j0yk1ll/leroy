"""M23AQ completely offline camera-only source checks, no ROS or hardware."""
import ast
import importlib.util
import os
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "m23aq_camera_only.launch.py"
PERMITTED = {
    "M23AQ_CAMERA_ONLY_OPERATOR_APPROVAL": "REVIEWED_AND_APPROVED",
    "M23AQ_MOTOR_POWER_ISOLATION_ATTESTED": "YES",
    "M23AQ_VIDEO_DEVICE": "/dev/video2",
    "M23AQ_CALIBRATION_VERIFIED": "YES",
    "M23AQ_CALIBRATION_PATH": "/config/camera/front_camera.yaml",
    "LD_LIBRARY_PATH": "/opt/m23aj-opencv/lib:/opt/ros/jazzy/lib",
    "LD_PRELOAD": "",
}


def fake_launch_module():
    def stub(label):
        def make(*args, **kwargs):
            return {"action": label, "args": args, "kwargs": kwargs}
        return make

    mods = {
        key: types.ModuleType(key) for key in
        ("launch", "launch_ros", "launch_ros.actions",
         "launch_ros.descriptions")
    }
    mods["launch"].LaunchDescription = stub("LaunchDescription")
    mods["launch_ros.actions"].Node = stub("Node")
    mods["launch_ros.actions"].ComposableNodeContainer = stub(
        "ComposableNodeContainer")
    mods["launch_ros.descriptions"].ComposableNode = stub("ComposableNode")

    spec = importlib.util.spec_from_file_location("m23aq_launch_stub", SOURCE)
    obj = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, mods):
        spec.loader.exec_module(obj)
    return obj


class CameraOnlySourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.obj = fake_launch_module()
        cls.tree = ast.parse(SOURCE.read_text())

    def test_no_hardware_by_default(self):
        with patch.dict(os.environ, {}, clear=True):
            with self.assertRaisesRegex(RuntimeError, "EXPLICIT_CAMERA_ONLY"):
                self.obj.generate_launch_description()

    def test_explicit_two_node_launch_only(self):
        with patch.dict(os.environ, PERMITTED, clear=True):
            launch = self.obj.generate_launch_description()
        self.assertEqual(launch["action"], "LaunchDescription")
        nodes = launch["args"][0]
        self.assertEqual(len(nodes), 2)
        self.assertEqual([n["action"] for n in nodes],
                         ["Node", "ComposableNodeContainer"])
        cam, rect = nodes
        self.assertEqual(cam["kwargs"]["package"], "usb_cam")
        self.assertEqual(cam["kwargs"]["executable"], "usb_cam_node_exe")
        self.assertEqual(cam["kwargs"]["parameters"][0]["video_device"],
                         "/dev/video2")
        self.assertEqual(cam["kwargs"]["parameters"][0]["camera_info_url"],
                         "file:///config/camera/front_camera.yaml")
        self.assertEqual(rect["kwargs"]["package"], "rclcpp_components")
        self.assertEqual(rect["kwargs"]["executable"], "component_container")
        component = rect["kwargs"]["composable_node_descriptions"]
        self.assertEqual(len(component), 1)
        self.assertEqual(component[0]["kwargs"]["package"], "image_proc")
        self.assertEqual(component[0]["kwargs"]["plugin"],
                         "image_proc::RectifyNode")

    def test_no_bridge_or_other_launch_imports(self):
        text = SOURCE.read_text()
        for forbidden in ("leroy_bridge", "robot.launch.py", "exec.sh",
                          "DepthART", "xrslam", "nav2", "IncludeLaunchDescription"):
            self.assertNotIn(forbidden, text)
        imported = [
            n.module for n in ast.walk(self.tree)
            if isinstance(n, ast.ImportFrom)
        ]
        self.assertEqual(sorted(imported),
                         sorted(["launch", "launch_ros.actions",
                                 "launch_ros.descriptions"]))

    def assert_denied(self, **changed):
        env = dict(PERMITTED)
        env.update(changed)
        with patch.dict(os.environ, env, clear=True):
            with self.assertRaises(RuntimeError):
                self.obj.generate_launch_description()

    def test_denies_no_power_isolation(self):
        self.assert_denied(M23AQ_MOTOR_POWER_ISOLATION_ATTESTED="")

    def test_denies_no_calibration_identity(self):
        self.assert_denied(M23AQ_CALIBRATION_VERIFIED="")

    def test_denies_foreign_calibration_path(self):
        self.assert_denied(M23AQ_CALIBRATION_PATH="/tmp/other.yaml")

    def test_denies_serial_device(self):
        self.assert_denied(M23AQ_VIDEO_DEVICE="/dev/ttyACM0")

    def test_denies_empty_camera(self):
        self.assert_denied(M23AQ_VIDEO_DEVICE="")

    def test_denies_global_preload(self):
        self.assert_denied(LD_PRELOAD="/tmp/libinjected.so")

    def test_denies_distro_opencv_precedence(self):
        self.assert_denied(
            LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/opt/m23aj-opencv/lib")

    def test_matches_existing_camera_geometry_only(self):
        yaml_text = (ROOT / "config/camera/front_camera.yaml").read_text()
        self.assertIn("image_width: 640", yaml_text)
        self.assertIn("image_height: 480", yaml_text)
        self.assertIn("distortion_model: plumb_bob", yaml_text)
        self.assertIn("0.1401151595", yaml_text)
        # Source geometry is NOT a physical serial-to-calibration attestation.


if __name__ == "__main__":
    unittest.main()
