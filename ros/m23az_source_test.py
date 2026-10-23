"""M23AZ: offline source tests; never dlopen or access devices here."""
import unittest
from pathlib import Path
from unittest.mock import patch

import m23az_component_dlopen_probe as m


def approved():
    return {
        **m.ENVIRONMENT,
        "LD_PRELOAD": "",
        "AMENT_PREFIX_PATH": m.AMENT_PREFIX_PATH,
        "LD_LIBRARY_PATH": "/opt/m23ay-install/lib:/opt/m23aj-opencv/lib:/opt/ros/jazzy/lib",
    }


def maps():
    paths = [
        "/opt/m23ay-install/lib/libusb_cam_node.so",
        "/opt/m23ay-install/lib/libusb_cam.so",
        "/opt/m23aj-opencv/lib/libopencv_core.so.4.6.0",
        "/opt/m23aj-opencv/lib/libopencv_imgproc.so.4.6.0",
    ]
    return "\n".join(
        "7f4900000000-7f4900011000 r-xp 00000000 00:01 1023 " + p for p in paths
    ) + "\n"


def gate(env, **overrides):
    args = {
        "exists": lambda path: True,
        "is_dir": lambda path: True,
        "globs": lambda pattern: [],
        "usb_bus_exists": lambda: False,
    }
    args.update(overrides)
    return m.gate(env, **args)


class Tests(unittest.TestCase):
    def test_default_inert(self):
        with patch.object(m, "run", side_effect=AssertionError("dlopen started")):
            self.assertEqual(m.main([]), 0)

    def test_valid_guards(self):
        self.assertTrue(gate(approved()))

    def test_missing_approval(self):
        env = approved()
        env.pop("M23AZ_OFFLINE_DLOPEN_APPROVED")
        with self.assertRaisesRegex(m.Blocked, "M23AZ_OFFLINE"):
            gate(env)

    def test_missing_ament_prefix_fails_before_dlopen(self):
        env = approved()
        env.pop("AMENT_PREFIX_PATH")
        with self.assertRaisesRegex(m.Blocked, "AMENT_PREFIX_PATH"):
            gate(env)

    def test_empty_ament_prefix_rejected(self):
        env = approved()
        env["AMENT_PREFIX_PATH"] = ""
        with self.assertRaisesRegex(m.Blocked, "AMENT_PREFIX_PATH"):
            gate(env)

    def test_legacy_ament_prefix_first_rejected(self):
        env = approved()
        env["AMENT_PREFIX_PATH"] = (
            "/leroy_ws/install/usb_cam:" + m.AMENT_PREFIX_PATH)
        with self.assertRaisesRegex(m.Blocked, "wrong order"):
            gate(env)

    def test_missing_ament_prefix_directory_rejected(self):
        with self.assertRaisesRegex(m.Blocked, "prefix directory"):
            gate(approved(), is_dir=lambda p: str(p) != "/opt/m23ay-install")

    def test_missing_diagnostic_package_index_rejected(self):
        with self.assertRaisesRegex(m.Blocked, "diagnostic usb_cam ament"):
            gate(approved(), exists=lambda p: not str(p).endswith(
                "/resource_index/packages/usb_cam"))

    def test_missing_ros_components_index_rejected(self):
        with self.assertRaisesRegex(m.Blocked, "rclcpp_components ament"):
            gate(approved(), exists=lambda p: not str(p).endswith(
                "/resource_index/packages/rclcpp_components"))

    def test_missing_ament_prefix_does_not_enter_ctypes(self):
        env = approved()
        env.pop("AMENT_PREFIX_PATH")
        with patch.dict(m.os.environ, env, clear=True), \
             patch.object(m.ctypes, "CDLL",
                          side_effect=AssertionError("native initializer entered")):
            self.assertEqual(m.main(["--run"]), 2)

    def test_wrong_ros_domain(self):
        env = approved()
        env["ROS_DOMAIN_ID"] = "0"
        with self.assertRaisesRegex(m.Blocked, "ROS_DOMAIN_ID"):
            gate(env)

    def test_wrong_loader_order(self):
        env = approved()
        env["LD_LIBRARY_PATH"] = "/usr/lib:/opt/m23ay-install/lib"
        with self.assertRaisesRegex(m.Blocked, "precedence"):
            gate(env)

    def test_ld_preload_forbidden(self):
        env = approved()
        env["LD_PRELOAD"] = "/tmp/libexample.so"
        with self.assertRaisesRegex(m.Blocked, "LD_PRELOAD"):
            gate(env)

    def test_camera_device_forbidden(self):
        with self.assertRaisesRegex(m.Blocked, "device exposed"):
            gate(approved(), globs=lambda p: ["/dev/video2"] if "video" in p else [])

    def test_mcu_serial_forbidden(self):
        with self.assertRaisesRegex(m.Blocked, "device exposed"):
            gate(approved(), globs=lambda p: ["/dev/ttyACM0"] if "ttyACM" in p else [])

    def test_usb_bus_forbidden(self):
        with self.assertRaisesRegex(m.Blocked, "USB bus"):
            gate(approved(), usb_bus_exists=lambda: True)

    def test_missing_installed_node_forbidden(self):
        # Let the Ament resource indexes and diagnostic core exist.
        # The only absent file is the node component itself; otherwise
        # an earlier Ament-index preflight masks this negative control.
        node_path = m.Path(m.NODE)
        with self.assertRaisesRegex(m.Blocked, "libraries missing"):
            gate(approved(), exists=lambda p: p != node_path)

    def test_valid_actual_mapping(self):
        result = m.mapped_libraries(maps())
        self.assertEqual(result["tbb_mappings"], 0)
        self.assertEqual(len(result["mapped_diagnostic_cam_libraries"]), 2)
        self.assertEqual(len(result["mapped_pthreads_opencv_libraries"]), 2)

    def test_missing_camera_core_blocked(self):
        with self.assertRaisesRegex(m.Blocked, "core DSO"):
            m.mapped_libraries(maps().replace(
                "7f4900000000-7f4900011000 r-xp 00000000 00:01 1023 " +
                m.CORE + "\n", ""))

    def test_missing_component_blocked(self):
        with self.assertRaisesRegex(m.Blocked, "component DSO"):
            m.mapped_libraries(maps().replace(
                "7f4900000000-7f4900011000 r-xp 00000000 00:01 1023 " +
                m.NODE + "\n", ""))

    def test_mixed_legacy_usb_cam_blocked(self):
        with self.assertRaisesRegex(m.Blocked, "unexpected camera DSO"):
            m.mapped_libraries(maps() +
                "7f000000-7f111111 r-xp 0 00:01 123 /leroy_ws/install/usb_cam/lib/libusb_cam.so\n")

    def test_mixed_system_opencv_blocked(self):
        with self.assertRaisesRegex(m.Blocked, "mixed system OpenCV"):
            m.mapped_libraries(maps() +
                "7f000000-7f111111 r-xp 0 00:01 123 /usr/lib/libopencv_core.so.406\n")

    def test_tbb_is_fatal(self):
        with self.assertRaisesRegex(m.Blocked, "TBB mapped"):
            m.mapped_libraries(maps() +
                "7f000000-7f111111 r-xp 0 00:01 123 /usr/lib/libtbb.so.12\n")

    def test_tbb_malloc_is_fatal(self):
        with self.assertRaisesRegex(m.Blocked, "TBB mapped"):
            m.mapped_libraries(maps() +
                "7f000000-7f111111 r-xp 0 00:01 123 /usr/lib/libtbbmalloc.so.2\n")

    def test_empty_maps_is_fatal(self):
        with self.assertRaisesRegex(m.Blocked, "no process loader"):
            m.mapped_libraries("")

    def test_no_ros_or_camera_invocation_in_source(self):
        source = (Path(__file__).resolve().parent /
                  "m23az_component_dlopen_probe.py").read_text()
        for forbidden in ("rclpy.init(", "rclcpp::init(", "ros2 launch ",
                          "VideoCapture(", "open('/dev/video", "rosbag record",
                          "robot.launch.py"):
            self.assertNotIn(forbidden, source)
        self.assertIn("ctypes.CDLL(NODE", source)


if __name__ == "__main__":
    unittest.main()
