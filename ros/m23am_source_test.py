"""M23AM offline ELF-resolution parser tests; no ROS, Podman or hardware."""
from unittest.mock import patch
import unittest

import m23am_camera_stack_elf_audit as gate


def sample(bind="libopencv_core.so.406 => /opt/m23aj-opencv/lib/libopencv_core.so.406"):
    return """linux-vdso.so.1 (0x0000)
    libcv_bridge.so => /opt/ros/jazzy/lib/libcv_bridge.so (0x0100)
    %s (0x0200)
    libstdc++.so.6 => /lib/x86_64-linux-gnu/libstdc++.so.6 (0x0300)
    """ % bind


class OfflineTests(unittest.TestCase):
    def test_accept_alt_opencv_406(self):
        self.assertEqual(
            gate.interpret_ldd(sample(), 0, "rectify"),
            {"libopencv_core.so.406":
             "/opt/m23aj-opencv/lib/libopencv_core.so.406"})

    def test_reject_nonzero_ldd_exit(self):
        with self.assertRaisesRegex(gate.Blocked, "returned 1"):
            gate.interpret_ldd(sample(), 1, "rectify")

    def test_reject_unresolved_symbol(self):
        with self.assertRaisesRegex(gate.Blocked, "unresolved"):
            gate.interpret_ldd(sample() + "\nundefined symbol: ABC\n", 0, "rectify")

    def test_reject_missing_needed(self):
        with self.assertRaisesRegex(gate.Blocked, "missing needed"):
            gate.interpret_ldd(sample() + "\n  libfoo.so => not found\n", 0, "rectify")

    def test_reject_tbb_runtime(self):
        for library in ("libtbb.so.12", "libtbbmalloc.so.2",
                        "libtbbmalloc_proxy.so.2"):
            with self.subTest(library=library):
                with self.assertRaisesRegex(gate.Blocked, "oneTBB"):
                    gate.interpret_ldd(sample() + "\n  " + library +
                                       " => /usr/lib/" + library + "\n", 0, "rectify")

    def test_reject_distro_opencv(self):
        distro = "libopencv_core.so.406 => /usr/lib/x86_64-linux-gnu/libopencv_core.so.406"
        with self.assertRaisesRegex(gate.Blocked, "mixed OpenCV"):
            gate.interpret_ldd(sample(distro), 0, "rectify")

    def test_reject_foreign_opencv_soname(self):
        binding = "libopencv_core.so.410 => /opt/m23aj-opencv/lib/libopencv_core.so.410"
        with self.assertRaisesRegex(gate.Blocked, "SONAME"):
            gate.interpret_ldd(sample(binding), 0, "rectify")

    def test_reject_unparsed_opencv(self):
        with self.assertRaisesRegex(gate.Blocked, "unparsed OpenCV"):
            gate.interpret_ldd("libopencv_core.so.406 (0x123)\n", 0, "rectify")

    def test_no_audit_by_default(self):
        with patch.object(gate, "audit", side_effect=AssertionError("audit invoked")):
            self.assertEqual(gate.main([]), 0)

    def test_project_camera_paths_and_permissions(self):
        self.assertEqual(len(gate.TARGETS), 6)
        self.assertEqual(gate.TARGETS["usb_cam_component"],
                         "/leroy_ws/install/usb_cam/lib/libusb_cam_node.so")
        self.assertEqual(gate.TARGETS["usb_cam_node_exe"],
                         "/leroy_ws/install/usb_cam/lib/usb_cam/usb_cam_node_exe")
        self.assertIn("cv_bridge", gate.TARGETS)
        self.assertIn("image_geometry", gate.TARGETS)
        self.assertIn("rectify", gate.TARGETS)

    def test_reject_static_ldd(self):
        with self.assertRaisesRegex(gate.Blocked, "not dynamically"):
            gate.interpret_ldd("statically linked", 0, "rectify")


if __name__ == "__main__":
    unittest.main()
