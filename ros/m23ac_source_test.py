"""Pure offline M23AC source and hook-contract checks. Never starts ROS."""
import ast
from pathlib import Path
import unittest

from m23ac_compare import check_hook

ROOT = Path(__file__).resolve().parent


class M23ACSourceTests(unittest.TestCase):
    def test_managed_rectifier_load_happens_before_noload_promotion(self):
        cpp = (ROOT / "m23ac_cpp/src/loaded_rectify_container.cpp").read_text()
        code = "\n".join(row.split("//", 1)[0] for row in cpp.splitlines())
        self.assertIn("rclcpp_components::ComponentManager::create_component_factory(resource)", code)
        self.assertIn("RTLD_NOLOAD", code)
        self.assertIn("RTLD_NODELETE", code)
        self.assertIn("RTLD_NOW", code)
        self.assertIn("::dlopen(path.c_str(), flags)", code)
        self.assertIn("::dlclose(handle)", code)
        self.assertIn('nodelete_ ? RTLD_NODELETE : 0', code)
        self.assertIn('M23AC_FACTORY_HOOK=VERIFIED policy=%s', code)
        self.assertLess(code.index("ComponentManager::create_component_factory(resource)"),
                        code.index("::dlopen(path.c_str(), flags)"))
        for forbidden in ("std::make_shared<image_proc::RectifyNode>", "LD_PRELOAD",
                          "image_transport::create_publisher", "usb_cam"):
            self.assertNotIn(forbidden, code)

    def test_same_component_binary_both_arms(self):
        launch = (ROOT / "m23ac_rectifier_container.launch.py").read_text()
        ast.parse(launch)
        self.assertIn('policy = os.environ.get("M23AC_ARM")', launch)
        self.assertIn('policy not in ("control", "nodelete")', launch)
        self.assertIn('package="leroy_m23ac_loaded_rectify_lifetime"', launch)
        self.assertIn('executable="component_container"', launch)
        self.assertIn('plugin="image_proc::RectifyNode"', launch)
        self.assertIn('namespace="/camera"', launch)
        self.assertIn('("image", "image_raw")', launch)
        self.assertIn('("camera_info", "camera_info")', launch)
        self.assertIn('name="M23AC_LIBRARY_POLICY", value=policy', launch)
        self.assertNotIn("LD_PRELOAD", launch)

    def test_inherited_strict_fixture_and_no_hardware(self):
        source = (ROOT / "m23ac_compare.py").read_text()
        ast.parse(source)
        for s in (
            "import m23aa_compare as m23aa",
            'm23aa.LAUNCHES["component"] = LAUNCH',
            'm23p.inspect_synthetic_graph = inspect',
            'audited["hook"] = check_hook(log, policy)',
            'result = m23aa.run()',
            'M23AC_DEVICE_ACCESS=NONE',
            'M23AC_PRODUCTION_ACCEPTANCE=BLOCKED',
        ):
            self.assertIn(s, source)
        self.assertNotIn("subprocess.Popen", source)
        self.assertNotIn("os.system(", source)

    def test_hook_is_exactly_once_and_has_expected_policy(self):
        for mode in ("control", "nodelete"):
            check_hook(
                "[component_container-1] M23AC_FACTORY_HOOK=VERIFIED policy=" +
                mode + "\n", mode)
            with self.assertRaises(ValueError):
                check_hook("no hook in process log", mode)
            with self.assertRaises(ValueError):
                check_hook("M23AC_FACTORY_HOOK=VERIFIED policy=control\n"
                           "M23AC_FACTORY_HOOK=VERIFIED policy=nodelete\n", mode)
            other = "nodelete" if mode == "control" else "control"
            with self.assertRaises(ValueError):
                check_hook("M23AC_FACTORY_HOOK=VERIFIED policy=" + other, mode)

    def test_ament_plain_linking(self):
        cmake = (ROOT / "m23ac_cpp/CMakeLists.txt").read_text()
        manifest = (ROOT / "m23ac_cpp/package.xml").read_text()
        self.assertIn("ament_target_dependencies(component_container rclcpp rclcpp_components)", cmake)
        self.assertIn("target_link_libraries(component_container dl)", cmake)
        self.assertNotIn("target_link_libraries(component_container PRIVATE", cmake)
        self.assertIn("<name>leroy_m23ac_loaded_rectify_lifetime</name>", manifest)

    def test_disconnected_build_contract(self):
        docker = (ROOT / "Dockerfile.m23ac-loaded-rectify").read_text()
        self.assertIn("FROM localhost/leroy-sensors:m23ab-direct-linked-v2", docker)
        self.assertIn("M23AC_BUILD_CONTRACT=PASS", docker)
        self.assertIn("--packages-select leroy_m23ac_loaded_rectify_lifetime", docker)
        for name in ("m23ac_cpp", "m23ac_compare.py", "m23ac_source_test.py",
                     "m23ac_rectifier_container.launch.py",
                     "Dockerfile.m23ac-loaded-rectify",
                     "M23AC_LOADED_RECTIFY_NODELETE_AB.md"):
            self.assertIn("COPY " + name + " ", docker)
        for forbidden in ("ros2 launch", "podman run", "usb_cam", "LD_PRELOAD"):
            self.assertNotIn(forbidden, docker)


if __name__ == "__main__":
    unittest.main()
