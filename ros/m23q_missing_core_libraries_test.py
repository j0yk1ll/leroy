"""M23Q missing library selection tests: synthetic GDB text only."""
import unittest
from m23q_missing_core_libraries import parse_missing_libraries

PREFIX = "warning: Can't open file "
SUFFIX = " during file-backed mapping note processing"


def warn(path):
    return PREFIX + path + SUFFIX


class M23QMissingLibraryContracts(unittest.TestCase):
    def test_ros_and_ubuntu_shared_libraries_are_selected(self):
        text = "\n".join((warn("/opt/ros/jazzy/lib/libimage_transport.so"),
                          warn("/usr/lib/x86_64-linux-gnu/libtbbmalloc.so.2.11")))
        self.assertEqual(parse_missing_libraries(text), [
            "/opt/ros/jazzy/lib/libimage_transport.so",
            "/usr/lib/x86_64-linux-gnu/libtbbmalloc.so.2.11",
        ])

    def test_ephemeral_fastrtps_device_path_refused(self):
        self.assertEqual(parse_missing_libraries(
            warn("/dev/shm/fastrtps_c636313735765e7f")), [])

    def test_no_arbitrary_host_path_or_file(self):
        self.assertEqual(parse_missing_libraries("\n".join((
            warn("/home/private/user/private.so"),
            warn("/etc/shadow"),
            warn("/dev/ttyACM0"),
            warn("/proc/self/mem"),
            warn("/opt/ros/jazzy/lib/component_container"),
        ))), [])

    def test_deduplicate_and_sort(self):
        text = "\n".join((warn("/opt/ros/jazzy/lib/libz.so.1"),
                          warn("/opt/ros/jazzy/lib/liba.so"),
                          warn("/opt/ros/jazzy/lib/libz.so.1")))
        self.assertEqual(parse_missing_libraries(text), [
            "/opt/ros/jazzy/lib/liba.so",
            "/opt/ros/jazzy/lib/libz.so.1",
        ])

    def test_path_traversal_refused(self):
        for path in (
            "/opt/ros/jazzy/lib/../../etc/shadow.so",
            "/usr/lib/x86_64-linux-gnu/../outside.so",
            "/opt/ros/jazzy/lib/a//bad.so",
        ):
            with self.subTest(path=path):
                self.assertEqual(parse_missing_libraries(warn(path)), [])

    def test_non_warning_lines_are_ignored(self):
        self.assertEqual(parse_missing_libraries(
            "Can't open file /opt/ros/jazzy/lib/librectify.so\n"
            "warning: librectify.so missing"), [])

    def test_basename_with_whitespace_is_blocked(self):
        self.assertEqual(parse_missing_libraries(
            warn("/opt/ros/jazzy/lib/secret libs.so")), [])

    def test_no_warn_returns_empty_without_fabricated_modules(self):
        self.assertEqual(parse_missing_libraries("M23Q_CORE_BYTES=110616576"), [])


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            M23QMissingLibraryContracts))
    if result.wasSuccessful():
        print("M23Q_MISSING_LIBRARY_ALLOWLIST_CONTRACT=PASS", flush=True)
        print("M23Q_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    raise SystemExit(2)
