"""Source-only regression: Stage 3 sidecar shares network, never donor IPC."""
import pathlib
import unittest

HERE = pathlib.Path(__file__).resolve().parent


class LiveStage3SidecarContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.script = (HERE / "live_stage3_monocular_sidecar.sh").read_text()

    def test_private_ipc_udp_transport(self):
        s = self.script
        self.assertIn('--network "container:$parent" --ipc private', s)
        self.assertIn('FASTDDS_BUILTIN_TRANSPORTS=UDPv4', s)
        self.assertIn('RMW_IMPLEMENTATION=rmw_fastrtps_cpp', s)
        self.assertNotIn('--ipc "container:$parent"', s)
        self.assertNotIn('--ipc host', s)

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
