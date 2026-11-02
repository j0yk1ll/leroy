import unittest
from leroy_navigation.grid_planner import Grid, plan


class GridPlannerTests(unittest.TestCase):
    def grid(self, blocked=(), inflation=0.0):
        data = [0] * 100
        for x, y in blocked:
            data[y*10+x] = 100
        return Grid(10, 10, 1.0, 0.0, 0.0, tuple(data),
                    inflation_radius_m=inflation)

    def test_direct_route(self):
        route = plan(self.grid(), (1.5, 1.5), (7.5, 1.5))
        self.assertEqual(route[0], (1.5, 1.5))
        self.assertEqual(route[-1], (7.5, 1.5))

    def test_avoid_wall(self):
        wall = [(4, y) for y in range(8)]
        route = plan(self.grid(wall), (2.5, 2.5), (7.5, 2.5))
        self.assertIsNotNone(route)
        self.assertTrue(all(x != 4.5 or y >= 8.5 for x, y in route))

    def test_unknown_is_blocked(self):
        data = [-1] * 100
        grid = Grid(10, 10, 1., 0., 0., tuple(data), inflation_radius_m=0.)
        self.assertIsNone(plan(grid, (1.5, 1.5), (8.5, 8.5)))

    def test_inflation(self):
        grid = self.grid([(5, 5)], inflation=1.0)
        self.assertIn((5, 4), grid.blocked())
        self.assertNotIn((5, 3), grid.blocked())

    def test_blocked_destination(self):
        self.assertIsNone(plan(self.grid([(7, 7)]), (1.5, 1.5), (7.5, 7.5)))

    def test_corner_cut_forbidden(self):
        grid = self.grid([(2, 1), (1, 2)])
        route = plan(grid, (1.5, 1.5), (2.5, 2.5))
        self.assertIsNotNone(route)
        self.assertNotEqual(route[1], (2.5, 2.5))


if __name__ == "__main__":
    unittest.main()
