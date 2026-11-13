"""Synthetic tests for diagnostic-only relative-depth boundaries."""

import unittest
import numpy as np

from leroy_perception.relative_depth_structure import (
    UNKNOWN,
    POSSIBLE_DEPTH_BOUNDARY,
    detect_relative_boundaries,
)


class RelativeDepthStructureTests(unittest.TestCase):

    def test_uniform_surface_is_unknown(self):
        image = np.full((20, 20), 3.0, dtype=np.float32)
        result = detect_relative_boundaries(image)
        self.assertTrue(np.all(result == UNKNOWN))

    def test_depth_step_detected(self):
        image = np.ones((20, 20), dtype=np.float32)
        image[:, 10:] = 4.0

        result = detect_relative_boundaries(image)

        self.assertTrue(
            np.all(result[:, 9:11] == POSSIBLE_DEPTH_BOUNDARY)
        )
        self.assertTrue(np.all(result[:, :8] == UNKNOWN))
        self.assertTrue(np.all(result[:, 12:] == UNKNOWN))

    def test_reversed_depth_order_same_boundaries(self):
        image = np.ones((20, 20), dtype=np.float32)
        image[:, 10:] = 4.0

        forward = detect_relative_boundaries(image)
        reversed_values = detect_relative_boundaries(-image)

        np.testing.assert_array_equal(forward, reversed_values)

    def test_insufficient_valid_data_is_unknown(self):
        image = np.full((20, 20), np.nan, dtype=np.float32)
        image[:5, :] = 1.0

        result = detect_relative_boundaries(image)

        self.assertTrue(np.all(result == UNKNOWN))

    def test_invalid_pixels_are_not_obstacles(self):
        image = np.ones((20, 20), dtype=np.float32)
        image[:, 10:] = 4.0
        image[5:15, 9:11] = np.nan

        result = detect_relative_boundaries(image)

        self.assertTrue(
            np.all(result[5:15, 9:11] == UNKNOWN)
        )

    def test_invalid_parameters_rejected(self):
        image = np.ones((20, 20), dtype=np.float32)

        for threshold in (-1.0, 0.0, 1.0, np.nan):
            with self.subTest(threshold=threshold):
                with self.assertRaises(ValueError):
                    detect_relative_boundaries(
                        image, contrast_threshold=threshold
                    )


if __name__ == "__main__":
    unittest.main()
