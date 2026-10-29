"""Source-only safety and reproducibility contract for the Stage 3 runner.

No ROS installation, camera, STM32 or motor connection is required.
"""
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent


class LiveStage3ReproducibilityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.start_script = (HERE / "live_stage3_start.sh").read_text()
        cls.build = (HERE / "live_stage3_build.sh").read_text()
        cls.sidecar = (HERE / "live_stage3_monocular_sidecar.sh").read_text()
        cls.costmap = (HERE / "src/leroy_nav2_perception/config/live_diagnostic_costmap.yaml").read_text()

    def test_isolated_parent_owns_only_camera(self):
        s = self.start_script
        for token in (
            "--network=none", "--ipc=shareable", "--pid=private",
            "--shm-size=256m", '--device="$camera:/dev/video2:rw"',
            "--cap-drop=all", "--security-opt=no-new-privileges",
            "ROS_DOMAIN_ID=170", "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST",
            "LEROY_MOTOR_POWER_DISCONNECTED",
            "camera=/dev/video8",
            "CAMERA_CHANGED_BEFORE_START",
            "LIVE_STAGE3_CAMERA_MAPPING=HOST_/dev/video8_TO_CONTAINER_/dev/video2",
            "LIVE_STAGE3_INSTALLED_COSTMAP_CONFIG=PASS_BOTH_HEIGHT_WINDOWS",
            'ros2 lifecycle set /costmap configure',
            'ros2 lifecycle set /costmap activate',
            'ros2 run leroy_nav2_perception live_costmap_probe',
        ):
            self.assertIn(token, s)
        for forbidden in (
            "--network=host", "--ipc=host", "--pid=host",
            "--privileged", "--device=/dev/ttyACM0",
            "ros2 launch leroy_bringup",
            "ros2 launch nav2_bringup",
            "ros2 run leroy_bridge",
        ):
            self.assertNotIn(forbidden, s)

    def test_video8_is_only_host_device_and_video2_remains_internal(self):
        s = self.start_script
        self.assertIn('camera=/dev/video8', s)
        self.assertIn('--device="$camera:/dev/video2:rw"', s)
        self.assertIn('[[ -c "$camera" && ! -L "$camera" ]]', s)
        self.assertIn('camera_rdev="$(stat -Lc', s)
        self.assertIn('CAMERA_CHANGED_BEFORE_START', s)
        self.assertNotIn('--device=/dev/video2 ', s)
        self.assertNotIn('--device=/dev/video8 ', s)
        self.assertNotIn('--device=/dev/ttyACM0', s)

    def test_both_height_filter_windows_are_required(self):
        s = self.start_script
        self.assertIn('layer["min_obstacle_height"]', s)
        self.assertIn('layer["max_obstacle_height"]', s)
        self.assertIn('source["min_obstacle_height"]', s)
        self.assertIn('source["max_obstacle_height"]', s)
        self.assertIn('ros2 param get /costmap obstacle_layer.min_obstacle_height', s)
        self.assertIn('ros2 param get /costmap obstacle_layer.max_obstacle_height', s)
        self.assertGreaterEqual(self.costmap.count("min_obstacle_height: -3.0"), 2)
        self.assertGreaterEqual(self.costmap.count("max_obstacle_height: 3.0"), 2)

    def test_xrslam_stays_monocular_gated(self):
        s = self.start_script
        self.assertIn('live_stage3_monocular_sidecar.sh --detach', s)
        self.assertIn("Submitted 1 camera frames to XRSLAM (monocular)", s)
        self.assertIn("/xrslam/localization_valid", s)
        self.assertIn("data: false", s)
        self.assertIn("timeout 6s ros2 topic echo /odom", s)
        self.assertIn("code", s)
        self.assertIn("--network \"container:$parent\"", self.sidecar)
        self.assertIn("--ipc \"container:$parent\"", self.sidecar)
        self.assertIn("--pid \"container:$parent\"", self.sidecar)
        self.assertIn("pose_convention_validated:=false", self.sidecar)
        self.assertIn("enable_map_tf:=false", self.sidecar)
        self.assertIn("run_flags=(--rm --detach)", self.sidecar)
        self.assertNotIn("--device=", self.sidecar)

    def test_build_family_and_fail_closed_rollback(self):
        for f in (
            "Dockerfile", "Dockerfile.live-perception",
            "Dockerfile.live-costmap", "Dockerfile.xrslam",
        ):
            self.assertIn("-f " + f + " -t localhost/leroy-ros:", self.build)
        self.assertIn("cleanup_if_failed", self.start_script)
        self.assertIn("LIVE_STAGE3_START=FAILED_ROLLING_BACK", self.start_script)
        self.assertIn("podman stop leroy-live-xrslam", self.start_script)
        self.assertIn("podman stop leroy-live-costmap", self.start_script)
        self.assertIn("EXISTING_CONTAINER_NAME_IN_USE", self.start_script)


if __name__ == "__main__":
    unittest.main()
