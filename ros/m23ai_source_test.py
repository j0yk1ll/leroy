"""M23AI offline source tests. Synthetic files; never invoke ROS or Podman."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import m23ai_dependency_audit as ai

ELF_TEXT = """
 0x0000000000000001 (NEEDED) Shared library: [libtbb.so.12]
 0x0000000000000001 (NEEDED) Shared library: [libc.so.6]
"""


def synthetic_root(with_tbb=True):
    work = tempfile.TemporaryDirectory()
    base = Path(work.name)
    libs, ros = base / "usr", base / "ros"
    libs.mkdir()
    ros.mkdir()
    (libs / "libopencv_core.so.4.6.0").touch()
    (libs / "libopencv_imgproc.so.4.6.0").touch()
    (ros / "librectify.so").touch()

    def fake_readelf(path):
        if path.name.startswith("libopencv_core"):
            return ["libtbb.so.12", "libc.so.6"] if with_tbb else ["libc.so.6"]
        if path.name.startswith("libopencv_imgproc"):
            return ["libopencv_core.so.406", "libc.so.6"]
        return ["libopencv_core.so.406"]
    return work, libs, ros, fake_readelf


class M23AISourceTests(unittest.TestCase):
    def test_parse_needed(self):
        self.assertEqual(ai.parse_needed(ELF_TEXT), ["libtbb.so.12", "libc.so.6"])

    def test_detect_tbb_variants(self):
        for item in ("libtbb.so.12", "libtbbmalloc.so.2",
                     "libtbbmalloc_proxy.so.2"):
            self.assertTrue(ai.TBB.match(item))
        self.assertIsNone(ai.TBB.match("libtbbmismatch.so.12"))

    def test_missing_or_duplicate_needed_fails(self):
        with self.assertRaises(ai.Blocked):
            ai.parse_needed(ELF_TEXT + ELF_TEXT)

    def test_detect_opencv_tbb_consumer(self):
        work, libs, ros, getter = synthetic_root()
        with work:
            result = ai.collect((libs, ros), getter)
        self.assertTrue(result["core_direct_tbb"])
        self.assertEqual(len(result["opencv_direct_tbb_consumers"]), 1)
        self.assertEqual(result["opencv_shared_objects_inspected"], 2)
        self.assertFalse(result["runtime_tbb_absence_verified"])

    def test_tbb_missing_yields_no_positive_attribution(self):
        work, libs, ros, getter = synthetic_root(with_tbb=False)
        with work:
            result = ai.collect((libs, ros), getter)
        self.assertFalse(result["core_direct_tbb"])
        self.assertEqual(result["opencv_direct_tbb_consumers"], [])

    def test_missing_rectifier_rejected(self):
        work, libs, ros, getter = synthetic_root()
        with work:
            (ros / "librectify.so").unlink()
            with self.assertRaises(ai.Blocked):
                ai.collect((libs, ros), getter)

    def test_default_does_not_invoke_static_elf_audit(self):
        with patch.object(ai, "collect", side_effect=AssertionError("no read")):
            self.assertEqual(ai.main([]), 0)


if __name__ == "__main__":
    unittest.main()
