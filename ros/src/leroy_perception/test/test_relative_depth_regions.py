import unittest
import numpy as np

from leroy_perception.relative_depth_regions import regional_contrast


class RegionalContrastTests(unittest.TestCase):

    def test_uniform_is_uninformative(self):
        scores, valid = regional_contrast(np.ones((24, 32)))
        self.assertFalse(np.isfinite(scores).any())
        self.assertFalse(valid.any())

    def test_step_produces_regional_contrast(self):
        image = np.ones((24, 32))
        image[:, 16:] = 4
        scores, valid = regional_contrast(image, rows=6, columns=8)
        self.assertTrue(valid.all())
        self.assertTrue(np.nanmax(scores) > 0.9)
        self.assertEqual(scores[2, 0], 0)

    def test_affine_and_reversed_order_invariant(self):
        image = np.tile(np.linspace(1, 5, 32), (24, 1))
        baseline, _ = regional_contrast(image)
        transformed, _ = regional_contrast(-3 * image + 17)
        np.testing.assert_allclose(
            baseline, transformed, equal_nan=True
        )

    def test_invalid_regions_remain_unsupported(self):
        image = np.ones((24, 32))
        image[:, 16:] = 4
        image[:4, :4] = np.nan
        scores, valid = regional_contrast(image, rows=6, columns=8)
        self.assertFalse(valid[0, 0])
        self.assertTrue(np.isnan(scores[0, 0]))

    def test_invalid_configuration_rejected(self):
        with self.assertRaises(ValueError):
            regional_contrast(np.ones((24, 32)), rows=0)


if __name__ == "__main__":
    unittest.main()
