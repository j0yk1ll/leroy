"""OpenCV ChArUco geometry/detection tests (safe on legacy OpenCV 4.6).

Run: python3 -m unittest discover -s utils/charuco -p 'test_*.py'
"""
import unittest
from pathlib import Path

import numpy as np

from calibrate import charuco_object_points, load_board, detect_charuco


CONFIG = Path(__file__).resolve().parent / "calibration_board" / "board_config.json"


class TestCharucoCompatibility(unittest.TestCase):
    def test_grid_geometry_and_corner_ids(self):
        cfg, _, _ = load_board(CONFIG)
        corners = charuco_object_points(cfg)
        self.assertEqual(corners.shape, (24, 3))
        self.assertEqual(corners.dtype, np.float32)
        np.testing.assert_allclose(corners[0], (0.035, 0.035, 0), atol=1e-7)
        np.testing.assert_allclose(corners[3], (0.140, 0.035, 0), atol=1e-7)
        np.testing.assert_allclose(corners[4], (0.035, 0.070, 0), atol=1e-7)
        np.testing.assert_allclose(corners[23], (0.140, 0.210, 0), atol=1e-7)

    def test_geometry_rejects_invalid_dimensions(self):
        with self.assertRaises(ValueError):
            charuco_object_points({
                "squares_x": 1, "squares_y": 7, "square_length_m": 0.035
            })
        with self.assertRaises(ValueError):
            charuco_object_points({
                "squares_x": 5, "squares_y": 7, "square_length_m": -0.035
            })

    def test_real_board_and_synthetic_detection(self):
        cfg, dictionary, board = load_board(CONFIG)
        # Board geometry is sourced exclusively from JSON, never from the
        # native .chessboardCorners property (segfault on OpenCV 4.6.0).
        self.assertEqual(len(charuco_object_points(cfg)), 24)
        image = (
            board.generateImage((700, 980), marginSize=35)
            if hasattr(board, "generateImage")
            else board.draw((700, 980), marginSize=35)
        )
        corners, ids, _, _ = detect_charuco(image, board, dictionary)
        self.assertIsNotNone(ids)
        self.assertGreaterEqual(len(ids), 8)
        self.assertEqual(len(corners), len(ids))


if __name__ == "__main__":
    unittest.main()
