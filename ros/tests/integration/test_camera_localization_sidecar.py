"""Camera Fast DDS sidecar uses isolated parent's network, IPC and PID."""
import pathlib
import unittest

HERE = pathlib.Path(__file__).resolve().parents[2]


class CameraLocalizationSidecarContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.script = (HERE / "src/leroy_bringup/scripts/xrslam_diagnostic.sh").read_text()

    def test_parent_shareable_ipc_contract(self):
        s = self.script
        self.assertIn("config/xrslam_monocular_diagnostic.yaml", s)
        self.assertIn("config/xrslam_device_template_UNCALIBRATED.yaml", s)
        self.assertNotIn("m23bv_v3_stage3", s)
        self.assertIn("{{.HostConfig.IpcMode}}", s)
        self.assertIn('== shareable', s)
        self.assertIn(
            "fail COSTMAP_PARENT_IPC_NOT_SHAREABLE_RECREATE_WITH_IPC_SHAREABLE",
            s)
        self.assertIn('--network "container:$parent" --ipc "container:$parent"', s)
        self.assertIn('--pid "container:$parent"', s)
        self.assertIn("{{.HostConfig.PidMode}}", s)
        self.assertIn("COSTMAP_PARENT_HOST_PID_MODE_DISALLOWED", s)
        self.assertNotIn('--pid private', s)
        self.assertNotIn('--pid host', s)
        self.assertIn('RMW_IMPLEMENTATION=rmw_fastrtps_cpp', s)
        self.assertNotIn('FASTDDS_BUILTIN_TRANSPORTS=UDPv4', s)
        self.assertNotIn('--ipc host', s)
        self.assertNotIn('--network host', s)

    def test_no_hardware_or_motion_permissions(self):
        s = self.script
        self.assertIn('test ! -e /dev/ttyACM0', s)
        self.assertIn('test ! -e /dev/video2', s)
        self.assertIn('--read-only --cap-drop=all', s)
        self.assertIn('-p pose_convention_validated:=false', s)
        self.assertIn('-p enable_map_tf:=false', s)
        self.assertNotIn('--device=', s)
        self.assertNotIn('--privileged', s)
        self.assertNotIn('ros2 launch leroy_description', s)
        self.assertNotIn('leroy_bridge/robot.launch.py', s)


if __name__ == "__main__":
    unittest.main()
