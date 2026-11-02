import unittest
from leroy_navigation.depth_grid import MapSettings, rasterize


class DepthGridTests(unittest.TestCase):
    def setUp(self):
        self.s = MapSettings(resolution=1.0, width=12, height=12,
                             max_depth=10.0)

    def make(self, points):
        return rasterize(points, (0., 0., 0.5), (0., 0.), self.s)

    def cell(self, result, x, y):
        ox, oy, cells = result
        col, row = int((x-ox)), int((y-oy))
        return cells[row*self.s.width+col]

    def test_unknown_without_measurement(self):
        a = self.make([])
        self.assertTrue(all(v == -1 for v in a[2]))

    def test_ray_marks_observed_only(self):
        a = self.make([(3., 0., 0.0)])
        self.assertEqual(self.cell(a, 1., 0.), 0)
        self.assertEqual(self.cell(a, 3., 0.), 0)
        self.assertEqual(self.cell(a, 4., 0.), -1)

    def test_obstacle_endpoints_occupied(self):
        a = self.make([(3., 0., 0.5)])
        self.assertEqual(self.cell(a, 3., 0.), 100)

    def test_obstacle_wins_over_free_ray(self):
        a = self.make([(2., 0., 0.5), (4., 0., 0.0)])
        self.assertEqual(self.cell(a, 2., 0.), 100)

    def test_snapshot_discards_stale_obstacles(self):
        self.assertEqual(self.cell(self.make([(2., 0., 0.5)]), 2., 0.), 100)
        self.assertEqual(self.cell(self.make([]), 2., 0.), -1)

    def test_invalid_sample_cannot_clear(self):
        a = self.make([(float("nan"), 0., 0.)])
        self.assertTrue(all(x == -1 for x in a[2]))


if __name__ == "__main__":
    unittest.main()
