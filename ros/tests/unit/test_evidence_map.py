import unittest
from leroy_navigation.depth_grid import MapSettings
from leroy_navigation.evidence_map import EvidenceMap


class EvidenceMapTests(unittest.TestCase):
    def setUp(self):
        self.settings = MapSettings(resolution=1., width=4, height=4)
        self.m = EvidenceMap(self.settings, evidence_ttl_ns=100)
        self.empty = [-1]*16

    def frame(self, x=0., y=0., *, index=None, value=100, stamp=1000):
        obs = list(self.empty)
        if index is not None:
            obs[index] = value
        return self.m.update(origin_x=x, origin_y=y,
                             observed=obs, now_ns=stamp)

    def test_retains_unobserved_cells(self):
        self.frame(index=5, stamp=1000)
        self.assertEqual(self.frame(index=6, stamp=1010)[2][5], 100)
        self.assertEqual(self.m.snapshot(now_ns=1010)[2][6], 100)

    def test_new_free_observation_clears_old_obstacle(self):
        self.frame(index=5, stamp=1000)
        self.assertEqual(self.frame(index=5, value=0, stamp=1010)[2][5], 0)

    def test_old_evidence_expires(self):
        self.frame(index=5, stamp=1000)
        self.assertEqual(self.frame(stamp=1101)[2][5], -1)

    def test_rolling_window_carries_world_cells(self):
        self.frame(index=6, stamp=1000)  # world x=2, y=1
        shifted = self.frame(x=1., stamp=1010)
        self.assertEqual(shifted[2][5], 100)  # new window x=1 + col=1

    def test_reset_on_clock_regression(self):
        self.frame(index=5, stamp=1000)
        with self.assertRaises(ValueError):
            self.frame(stamp=1000)
        self.assertEqual(len(self.m.cells), 0)

    def test_invalid_frame_does_not_remain_live(self):
        self.frame(index=5, stamp=1000)
        with self.assertRaises(ValueError):
            self.m.update(origin_x=0., origin_y=0.,
                          observed=[42]*16, now_ns=1010)
        self.assertEqual(len(self.m.cells), 0)


if __name__ == "__main__":
    unittest.main()
