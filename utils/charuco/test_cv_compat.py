"""OpenCV Python binding compatibility regression tests for Leroy's ChArUco board.

Run: python3 -m unittest discover -s utils/charuco -p 'test_*.py'
"""
import unittest
from pathlib import Path

import cv2
import numpy as np

from calibrate import board_chessboard_corners, load_board, detect_charuco


class TestCharucoCompatibility(unittest.TestCase):
    def test_getter_binding(self):
        class Board:
            def getChessboardCorners(self):
                return [[0, 0, 0], [1, 0, 0]]
        np.testing.assert_array_equal(
            board_chessboard_corners(Board()),
            np.array([[0, 0, 0], [1, 0, 0]], dtype=np.float32),
        )

    def test_attribute_binding(self):
        class Board:
            chessboardCorners = [[0, 0, 0], [0, 1, 0]]
        np.testing.assert_array_equal(
            board_chessboard_corners(Board()),
            np.array([[0, 0, 0], [0, 1, 0]], dtype=np.float32),
        )

    def test_real_board_and_synthetic_detection(self):
        config = Path(__file__).resolve().parent / "calibration_board" / "board_config.json"
        _, dictionary, board = load_board(config)
        self.assertEqual(len(board_chessboard_corners(board)), 24)
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
