"""M23AS pure-source tests. No hardware, camera, ROS or image decoding."""
import json
from pathlib import Path
import tempfile
from unittest.mock import patch
import unittest

import m23as_calibration_provenance as m


def fixture():
    # Source (generated ChArUco YAML) uses one '-' item per data row;
    # ROS deployed camera YAML uses one inline [..] sequence.
    vals = {
        "camera_matrix": (3, 3, [806.095673, 0, 302.420813, 0,
                                 804.064887, 234.430158, 0, 0, 1]),
        "distortion_coefficients": (1, 5, [0.1401151595, -0.4397031719,
                                         -0.0033891926, -0.0062540552, 1.2071193294]),
        "rectification_matrix": (3, 3, [1, 0, 0, 0, 1, 0, 0, 0, 1]),
        "projection_matrix": (3, 4, [826.276538, 0, 300.49696, 0, 0,
                                   825.326674, 233.509453, 0, 0, 0, 1, 0]),
    }
    a = b = "image_width: 640\nimage_height: 480\ndistortion_model: plumb_bob\n"
    for key, (rows, cols, values) in vals.items():
        a += f"{key}:\n  rows: {rows}\n  cols: {cols}\n  data:\n"
        a += "".join("  - " + str(v) + "\n" for v in values)
        b += f"{key}:\n  rows: {rows}\n  cols: {cols}\n  data: ["
        b += ", ".join(str(v) for v in values) + "]\n"
    a += "views_used: 92\nviews_rejected: 0\nrms_reprojection_error_px: 0.31844348\n"
    names = [f"calib_{n:03d}.png" for n in range(1, 93)]
    dims = [(640,480)]*92
    return a, b, dims, names


class OfflineCalibrationTests(unittest.TestCase):
    def test_source_matches_rounded_deployed_values(self):
        a,b,ds,ns = fixture()
        report = m.validate(a,b,ds,ns)
        self.assertEqual(report["source_charuco_image_count"], 92)
        self.assertTrue(report["deployed_matches_source_matrices"])
        self.assertFalse(report["physical_camera_which_acquired_images_proven"])

    def test_reject_matrix_mutation(self):
        a,b,ds,ns = fixture()
        with self.assertRaisesRegex(m.Blocked, "differs"):
            m.validate(a,b.replace("806.095673", "900.095673"),ds,ns)

    def test_reject_wrong_resolution(self):
        a,b,ds,ns = fixture()
        with self.assertRaisesRegex(m.Blocked, "not 640x480"):
            m.validate(a,b.replace("image_height: 480", "image_height: 720"),ds,ns)

    def test_reject_wrong_lens_model(self):
        a,b,ds,ns = fixture()
        with self.assertRaisesRegex(m.Blocked, "distortion"):
            m.validate(a,b.replace("plumb_bob", "equidistant"),ds,ns)

    def test_reject_insufficient_calibration_views(self):
        a,b,ds,ns = fixture()
        with self.assertRaisesRegex(m.Blocked, "view count"):
            m.validate(a.replace("views_used: 92", "views_used: 17"),b,ds,ns)

    def test_reject_modified_png_dimensions(self):
        a,b,ds,ns = fixture()
        ds[3] = (800,600)
        with self.assertRaisesRegex(m.Blocked, "not 640x480"):
            m.validate(a,b,ds,ns)

    def test_reject_missing_image_reference(self):
        a,b,ds,ns = fixture()
        with self.assertRaisesRegex(m.Blocked, "reference count"):
            m.validate(a,b,ds[:-1],ns[:-1])

    def test_reject_bad_rms(self):
        a,b,ds,ns = fixture()
        with self.assertRaisesRegex(m.Blocked, "RMS"):
            m.validate(a.replace("0.31844348", "3.31844348"),b,ds,ns)

    def test_read_png_ihdr_dimensions_without_decoding(self):
        png = (m.PNG_MAGIC + b"\x00\x00\x00\x0dIHDR"
               + b"\x00\x00\x02\x80\x00\x00\x01\xe0")
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/"calib_001.png"
            p.write_bytes(png)
            self.assertEqual(m.png_dimensions(p), (640,480))

    def test_bad_png_header_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/"calib_001.png"
            p.write_bytes(b"not a png")
            with self.assertRaisesRegex(m.Blocked, "PNG IHDR"):
                m.png_dimensions(p)

    def test_default_is_inert(self):
        with patch.object(m, "run", side_effect=AssertionError("opened data")):
            self.assertEqual(m.main([]), 0)

    def test_actual_paths_only_offline(self):
        self.assertEqual(m.SOURCE.relative_to(m.ROOT).as_posix(),
                         "utils/charuco/calibration.yaml")
        self.assertEqual(m.DEPLOYED.relative_to(m.ROOT).as_posix(),
                         "ros/config/camera/front_camera.yaml")


if __name__ == "__main__":
    unittest.main()
