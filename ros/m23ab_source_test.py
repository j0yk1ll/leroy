"""M23AB offline-only source and process-classification contracts."""
import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parent


class SourceContracts(unittest.TestCase):
    def test_cpp_direct_construction_and_teardown(self):
        cpp = (ROOT / "m23ab_cpp/src/direct_rectify.cpp").read_text()
        # Ignore prose comments in string-presence contracts.
        code = "\n".join(line.split("//", 1)[0] for line in cpp.splitlines())
        self.assertIn("#include \"image_proc/rectify.hpp\"", code)
        self.assertIn("std::make_shared<image_proc::RectifyNode>", code)
        self.assertIn("rclcpp::executors::SingleThreadedExecutor", code)
        self.assertIn("executor.spin()", code)
        self.assertIn("executor.remove_node(node)", code)
        self.assertIn("rclcpp::shutdown()", code)
        self.assertLess(code.index("executor.remove_node(node)"),
                        code.index("rclcpp::shutdown()"))
        for forbidden in ("ClassLoader", "NodeFactory", "dlopen(", "dlclose(",
                          "rclcpp_components", "usb_cam"):
            self.assertNotIn(forbidden, code)

    def test_package_links_installed_rectifier(self):
        cmake = (ROOT / "m23ab_cpp/CMakeLists.txt").read_text()
        manifest = (ROOT / "m23ab_cpp/package.xml").read_text()
        self.assertIn("find_package(image_proc REQUIRED)", cmake)
        self.assertIn("find_library(IMAGE_PROC_RECTIFY_LIBRARY", cmake)
        self.assertIn("target_link_libraries(rectify_node PRIVATE", cmake)
        self.assertIn("IMAGE_PROC_RECTIFY_LIBRARY", cmake)
        self.assertIn("<depend>image_proc</depend>", manifest)
        self.assertIn("install(TARGETS rectify_node", cmake)

    def test_launch_same_camera_topics_and_new_executable(self):
        source = (ROOT / "m23ab_direct_rectifier.launch.py").read_text()
        tree = ast.parse(source)
        self.assertIn('package="leroy_m23ab_direct_rectifier"', source)
        for item in ('executable="rectify_node"', 'name="rectify_node"',
                     'namespace="/camera"', '("image", "image_raw")',
                     '("camera_info", "camera_info")',
                     "M23AA_LAUNCH_EVENT=RECTIFIER_PROCESS_START"):
            self.assertIn(item, source)
        # Check executable import/call structure, not docstrings.
        calls = [
            node.func.id if isinstance(node.func, ast.Name) else
            node.func.attr if isinstance(node.func, ast.Attribute) else ""
            for node in ast.walk(tree) if isinstance(node, ast.Call)
        ]
        self.assertNotIn("ComposableNodeContainer", calls)
        self.assertNotIn("SetEnvironmentVariable", calls)

    def test_comparison_reuses_existing_strict_runner(self):
        source = (ROOT / "m23ab_compare.py").read_text()
        ast.parse(source)
        for expected in ('ARMS = ("component", "direct")',
                         'import m23aa_compare as m23aa',
                         'm23aa.LAUNCHES["direct"] = DIRECT',
                         'status = m23aa.run()',
                         'M23AB_ARM', 'M23AB_PRODUCTION_ACCEPTANCE=BLOCKED'):
            self.assertIn(expected, source)
        for forbidden in ("subprocess.Popen", "ros2 launch", "/dev/video"):
            self.assertNotIn(forbidden, source)

    def test_disconnected_image_source_contract(self):
        docker = (ROOT / "Dockerfile.m23ab-direct-linked").read_text()
        self.assertIn("FROM localhost/leroy-sensors:m23aa-standalone-v1", docker)
        self.assertIn("--packages-select leroy_m23ab_direct_rectifier", docker)
        self.assertIn("M23AB_BUILD_CONTRACT=PASS", docker)
        self.assertIn("readelf -d", docker)
        for name in ("m23ab_cpp", "m23ab_compare.py", "m23ab_source_test.py",
                     "m23ab_direct_rectifier.launch.py",
                     "Dockerfile.m23ab-direct-linked",
                     "M23AB_DIRECT_LINKED_RECTIFY_TEST.md"):
            self.assertIn("COPY " + name + " ", docker)
        for forbidden in ("ros2 launch", "podman run", "usb_cam", "LD_PRELOAD"):
            self.assertNotIn(forbidden, docker)


if __name__ == "__main__":
    unittest.main()
