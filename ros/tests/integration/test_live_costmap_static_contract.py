"""Source-only guard regression for provisional live depth Nav2 integration."""
import pathlib
import unittest

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[2] / 'src/leroy_perception'
ROS = ROOT.parents[1]


class LiveCostmapStaticTest(unittest.TestCase):
    def test_launch_has_no_actuation_or_synthetic_sources(self):
        source = (ROOT / "launch/live_depth_costmap.launch.py").read_text()
        self.assertIn("LEROY_LIVE_COSTMAP_DIAGNOSTIC", source)
        self.assertIn('os.path.exists("/dev/ttyACM0")', source)
        self.assertIn('"live_depth_pipeline.launch.py"', source)
        self.assertIn('"description.launch.py"', source)
        self.assertIn('"sensor_mounts_file": "/etc/leroy/sensor_mounts.yaml"', source)
        self.assertIn('package="leroy_perception"', source)
        self.assertIn('"costmap_fixture"', source)
        for denied in ('package="leroy_bridge"',
                       'package="nav2_controller"',
                       'package="leroy_nav2_odometry"'):
            self.assertNotIn(denied, source)

    def test_real_raw_cloud_only_and_nonclearing(self):
        config = (ROOT / "config/live_diagnostic_costmap.yaml").read_text()
        parsed = yaml.safe_load(config)["costmap"]["ros__parameters"]
        # Nav2 declares integer width/height; floating YAML scalars abort
        # construction with InvalidParameterTypeException before configure.
        self.assertIs(type(parsed["width"]), int)
        self.assertIs(type(parsed["height"]), int)
        self.assertGreater(parsed["width"], 0)
        self.assertGreater(parsed["height"], 0)
        # Jazzy applies a source ObservationBuffer Z filter AND a second
        # layer-wide ObstacleLayer Z filter before marking cells. A default
        # 0..2 m layer filter discards negative-Z projected camera points.
        layer = parsed["obstacle_layer"]
        source = layer["raw_depth_surfaces"]
        self.assertIs(type(layer["min_obstacle_height"]), float)
        self.assertIs(type(layer["max_obstacle_height"]), float)
        self.assertEqual(layer["min_obstacle_height"], -3.0)
        self.assertEqual(layer["max_obstacle_height"], 3.0)
        self.assertLessEqual(layer["min_obstacle_height"],
                             source["min_obstacle_height"])
        self.assertGreaterEqual(layer["max_obstacle_height"],
                                source["max_obstacle_height"])
        self.assertTrue(source["marking"])
        self.assertFalse(source["clearing"])
        self.assertIn("topic: /leroy_nav2/diagnostics/raw_depth_points",
                      config)
        self.assertIn("global_frame: base_link", config)
        self.assertIn("robot_base_frame: base_link", config)
        self.assertIn("track_unknown_space: true", config)
        self.assertIn("clearing: false", config)
        self.assertIn("footprint_clearing_enabled: false", config)
        self.assertNotIn("topic: /leroy_nav2/diagnostics/obstacle_points",
                         config)

    def test_current_static_tf_package_is_built(self):
        recipe = (ROS / "Dockerfile.live-costmap").read_text()
        self.assertIn("COPY src/leroy_description", recipe)
        self.assertIn("--packages-select leroy_description", recipe)
        self.assertIn("static_transform_publisher", recipe)
        self.assertIn("COPY src/leroy_description/config/sensor_mounts.yaml /etc/leroy/sensor_mounts.yaml", recipe)
        self.assertIn("test ! -L /etc/leroy/sensor_mounts.yaml", recipe)
        # Build must fail when the tested source YAML and installed
        # package-share YAML (actually passed to launch) do not match.
        self.assertIn(
            "cmp -s /leroy_ws/src/leroy_perception/config/live_diagnostic_costmap.yaml",
            recipe)
        self.assertIn(
            "/leroy_ws/install/leroy_perception/share/leroy_perception/config/live_diagnostic_costmap.yaml",
            recipe)

    def test_probe_fails_on_motor_topics(self):
        probe = (ROOT / "leroy_perception/live_costmap_probe.py").read_text()
        self.assertIn('"/cmd_vel"', probe)
        self.assertIn('"/cmd_vel_nav"', probe)
        self.assertIn("LIVE_DEPTH_COSTMAP=PASS", probe)
        self.assertIn("unknown_cells=true", probe)
        self.assertIn("not_for_motion", probe)


if __name__ == "__main__":
    unittest.main()
