import unittest
import numpy as np
from leroy_navigation.feature_overlay import draw_corners


class FeatureOverlayTests(unittest.TestCase):
    def test_blank_camera_image(self):
        frame = np.zeros((40, 40, 3), dtype=np.uint8)
        overlay = draw_corners(frame)
        self.assertEqual(overlay.shape, frame.shape)
        self.assertTrue((overlay == frame).all())

    def test_detectable_corner_creates_overlay(self):
        frame = np.zeros((80, 80, 3), dtype=np.uint8)
        frame[20:60, 20:60] = 255
        overlay = draw_corners(frame)
        self.assertGreater(np.count_nonzero(overlay != frame), 0)
        self.assertTrue((frame[0, 0] == 0).all())

    def test_rejects_unexpected_encoding(self):
        with self.assertRaises(ValueError):
            draw_corners(np.zeros((40, 40), dtype=np.uint8))


if __name__ == "__main__":
    unittest.main()
