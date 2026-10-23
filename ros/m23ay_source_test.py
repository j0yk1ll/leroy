"""M23AY: offline-only static loader-contract and Dockerfile tests, no device."""
from pathlib import Path
from unittest.mock import patch
import unittest
import m23ay_binary_audit as m


def good_loader(with_core=True):
    value = (
        "    linux-vdso.so.1 (0x7f0000000000)\n"
        "    libopencv_core.so.406 => /opt/m23aj-opencv/lib/libopencv_core.so.406\n"
    )
    if with_core:
        value += (
            "    libusb_cam.so => /opt/m23ay-install/lib/libusb_cam.so "
            "(0x7f0000000000)\n"
        )
    return value + "    libc.so.6 => /lib/x86_64-linux-gnu/libc.so.6\n"


def good_launcher_strings():
    return "libusb_cam_node.so\nusb_cam::UsbCamNode\n"


def good_strings():
    return ("M23AX_PUBLISH pub_index=\n"
            "M23AX_FATAL=METADATA_LOG_LIMIT\n"
            " dequeue_count=\n v4l2_seq=\n ros_stamp_ns=\n")


class OfflineM23AYTests(unittest.TestCase):
    def test_default_safe(self):
        with patch.object(m, "audit", side_effect=AssertionError("binary audit invoked")):
            self.assertEqual(m.main([]), 0)

    def test_good_overlay_library_resolution(self):
        self.assertTrue(m.verify_link_output(good_loader(), require_core=True,
                                             require_diagnostic=True))

    def test_detect_missing_core(self):
        with self.assertRaisesRegex(m.Blocked, "did not load isolated core"):
            m.verify_link_output(good_loader(False), require_core=True)

    def test_detect_legacy_underlay(self):
        payload = good_loader() + (
            "libusb_cam_node.so => "
            "/leroy_ws/install/usb_cam/lib/libusb_cam_node.so\n"
        )
        with self.assertRaisesRegex(m.Blocked, "legacy"):
            m.verify_link_output(payload, require_diagnostic=True)

    def test_detect_missing_dependency(self):
        with self.assertRaisesRegex(m.Blocked, "unresolved"):
            m.verify_link_output(good_loader() + " libfoo.so => not found")

    def test_detect_undefined_symbol(self):
        with self.assertRaisesRegex(m.Blocked, "unresolved"):
            m.verify_link_output(good_loader() + "undefined symbol: _ZN3fooE")

    def test_detect_tbb(self):
        with self.assertRaisesRegex(m.Blocked, "oneTBB"):
            m.verify_link_output(good_loader() +
                                 "libtbb.so.12 => /usr/lib/libtbb.so.12")

    def test_validate_bounded_publisher_markers(self):
        self.assertTrue(m.verify_symbols(
            good_strings(), "mmap_dequeued\nselect_eintr\nmmap_dqbuf_eagain\n",
            good_launcher_strings()))

    def test_missing_publisher_marker_blocks(self):
        with self.assertRaisesRegex(m.Blocked, "missing publisher"):
            m.verify_symbols("M23AX_PUBLISH pub_index=", "mmap_dequeued", good_launcher_strings())

    def test_missing_acquisition_marker_blocks(self):
        with self.assertRaisesRegex(m.Blocked, "missing acquisition"):
            m.verify_symbols(good_strings(), "mmap_dequeued\nselect_eintr", good_launcher_strings())

    def test_launcher_dlopen_component_without_direct_core(self):
        self.assertTrue(m.verify_link_output(good_loader(False),
                                             require_diagnostic=True,
                                             require_core=False))
        self.assertTrue(m.verify_symbols(
            good_strings(), "mmap_dequeued\nselect_eintr\nmmap_dqbuf_eagain\n",
            good_launcher_strings()))

    def test_launcher_missing_component_filename_is_blocked(self):
        with self.assertRaisesRegex(m.Blocked, "component loader reference"):
            m.verify_symbols(
                good_strings(), "mmap_dequeued\nselect_eintr\nmmap_dqbuf_eagain\n",
                "usb_cam::UsbCamNode")

    def test_launcher_wrong_plugin_class_is_blocked(self):
        with self.assertRaisesRegex(m.Blocked, "component loader reference"):
            m.verify_symbols(
                good_strings(), "mmap_dequeued\nselect_eintr\nmmap_dqbuf_eagain\n",
                "libusb_cam_node.so\nwrong_plugin")

    def test_audit_still_requires_component_core(self):
        src = (Path(__file__).resolve().parent /
               "m23ay_binary_audit.py").read_text()
        self.assertIn('require_core=key == "node"', src)
        self.assertIn("libusb_cam_node.so", src)
        self.assertNotIn('require_core=key in ("node", "executable")', src)

    def test_mocked_audit_accepts_ros_jazzy_dynamic_executable(self):
        def fake_command(argv):
            target = argv[-1]
            if argv[0] == "readelf":
                return "Class: ELF64\\nMachine: Advanced Micro Devices X86-64\\n"
            if argv[0] == "ldd":
                return good_loader(target.endswith("/libusb_cam_node.so"))
            if argv[0] == "strings":
                if target.endswith("/libusb_cam_node.so"):
                    return good_strings()
                if target.endswith("/libusb_cam.so"):
                    return "mmap_dequeued\\nselect_eintr\\nmmap_dqbuf_eagain\\n"
                if target.endswith("/usb_cam_node_exe"):
                    return good_launcher_strings()
            raise AssertionError("unexpected ELF command: " + repr(argv))

        env = {"M23AY_OFFLINE_BINARY_ONLY": "YES",
               "LD_LIBRARY_PATH": "/opt/m23ay-install/lib:/opt/m23aj-opencv/lib"}
        with patch.dict(m.os.environ, env), \
             patch.object(m.Path, "is_file", return_value=True), \
             patch.object(m, "run_command", side_effect=fake_command):
            result = m.audit()
        self.assertTrue(result["component_dso_requires_diagnostic_core"])
        self.assertTrue(result["generated_launcher_uses_component_dlopen"])
        self.assertFalse(result["generated_launcher_static_core_link_required"])

    def test_mocked_audit_blocks_unresolved_component_core(self):
        def fake_command(argv):
            if argv[0] == "readelf":
                return "Class: ELF64\\nMachine: Advanced Micro Devices X86-64\\n"
            if argv[0] == "ldd":
                return good_loader(False)
            raise AssertionError("should fail before strings audit")

        env = {"M23AY_OFFLINE_BINARY_ONLY": "YES",
               "LD_LIBRARY_PATH": "/opt/m23ay-install/lib:/opt/m23aj-opencv/lib"}
        with patch.dict(m.os.environ, env), \
             patch.object(m.Path, "is_file", return_value=True), \
             patch.object(m, "run_command", side_effect=fake_command):
            with self.assertRaisesRegex(m.Blocked, "did not load isolated core"):
                m.audit()

    def test_explicit_offline_gate_required(self):
        with patch.dict(m.os.environ, {"M23AY_OFFLINE_BINARY_ONLY": ""}):
            with self.assertRaisesRegex(m.Blocked, "offline build"):
                m.audit()


    def test_colcon_cli_does_not_require_optional_override_extension(self):
        doc = (Path(__file__).resolve().parent /
               "Dockerfile.m23ay-usb-cam-diagnostic").read_text()
        self.assertIn("--base-paths /opt/m23ax-source/usb_cam", doc)
        self.assertIn("--install-base /opt/m23ay-install", doc)
        self.assertIn("--packages-select usb_cam", doc)
        self.assertNotIn("--allow-overriding", doc)
        self.assertIn("M23AY_OFFLINE_BINARY_ONLY=YES", doc)

    def test_build_file_no_runtime_launch(self):
        doc = (Path(__file__).resolve().parent /
               "Dockerfile.m23ay-usb-cam-diagnostic").read_text()
        self.assertIn("FROM localhost/leroy-sensors:m23aj-pthreads-opencv-v2", doc)
        self.assertIn("M23AX_OFFLINE_BUILD_ONLY=YES", doc)
        self.assertIn("--packages-select usb_cam", doc)
        self.assertNotIn("--allow-overriding", doc)
        self.assertIn("--network", "podman build --network none")
        self.assertIn('CMD ["/bin/false"]', doc)
        self.assertNotIn("robot.launch.py", doc)
        self.assertNotIn("leroy-entrypoint", doc)
        self.assertNotIn("V4L2_CID_", doc)


if __name__ == "__main__":
    unittest.main()
