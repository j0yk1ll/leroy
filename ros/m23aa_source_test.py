"""Pure offline M23AA contract tests: no ROS launch or hardware."""
import ast
from pathlib import Path
import unittest
from m23aa_compare import child_outcome

ROOT = Path(__file__).resolve().parent


class Contracts(unittest.TestCase):
    def test_launch_difference_and_no_tracer(self):
        a = (ROOT / "m23aa_component_rectifier.launch.py").read_text()
        b = (ROOT / "m23aa_standalone_rectifier.launch.py").read_text()
        ast.parse(a)
        ast.parse(b)
        self.assertIn('plugin="image_proc::RectifyNode"', a)
        self.assertIn('executable="component_container"', a)
        self.assertIn('executable="rectify_node"', b)
        for token in ('name="rectify_node"', 'namespace="/camera"',
                      '("image", "image_raw")', '("camera_info", "camera_info")'):
            self.assertIn(token, a)
            self.assertIn(token, b)
        for s in (a, b):
            self.assertNotIn("LD_PRELOAD", s)
            self.assertNotIn("usb_cam", s)

    def test_exact_exit_classes(self):
        for arm, process in (("component", "component_container"),
                             ("standalone", "rectify_node")):
            self.assertEqual(child_outcome(
                f"[{process}-1] process has finished cleanly", arm), ("CLEAN", 0))
            self.assertEqual(child_outcome(
                f"[{process}-1] process has died [pid 42, exit code -11, cmd x]", arm),
                ("SIGSEGV", -11))
            self.assertEqual(child_outcome(
                f"[{process}-1] process has died [pid 42, exit code -9, cmd x]", arm),
                ("OTHER_EXIT", -9))
            with self.assertRaises(ValueError):
                child_outcome("ros2 launch exit 0", arm)
            with self.assertRaises(ValueError):
                child_outcome(
                    f"[{process}-1] process has finished cleanly\n"
                    f"[{process}-1] process has died [exit code -11]", arm)

    def test_runner_inherits_real_publisher_and_guards(self):
        src = (ROOT / "m23aa_compare.py").read_text()
        self.assertIn("m23y.PUBLISHER", src)
        self.assertIn("m23y.validate_drain", src)
        self.assertIn("m23p.prohibit_devices()", src)
        self.assertIn("m23p.run()", src)
        self.assertIn('== 900', src)
        self.assertIn('M23AA_PRODUCTION_ACCEPTANCE=BLOCKED', src)
        for banned in ("subprocess.Popen", "os.system(", "usb_cam"):
            self.assertNotIn(banned, src)

    def test_build_source_layout(self):
        docker = (ROOT / "Dockerfile.m23aa-standalone").read_text()
        self.assertIn("FROM localhost/leroy-sensors:m23y-synthetic-drain-v2", docker)
        self.assertIn("M23AA_BUILD_CONTRACT=PASS", docker)
        self.assertIn("test -x /opt/ros/jazzy/lib/image_proc/rectify_node", docker)
        for file in ("m23aa_compare.py", "m23aa_source_test.py",
                     "m23aa_component_rectifier.launch.py",
                     "m23aa_standalone_rectifier.launch.py",
                     "Dockerfile.m23aa-standalone",
                     "M23AA_STANDALONE_RECTIFIER_TEST.md"):
            self.assertIn("COPY " + file + " ", docker)
        self.assertNotIn("ros2 launch", docker)


if __name__ == "__main__":
    unittest.main()
