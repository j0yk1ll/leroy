import unittest
from leroy_navigation.mission import Mission, Waypoint


class MissionTests(unittest.TestCase):
    def test_sequential_waypoints(self):
        mission = Mission()
        a, b = Waypoint(0., 0., 0.), Waypoint(1., 2., 0.3)
        self.assertEqual(mission.load((a, b)), a)
        self.assertEqual(mission.arrived(), b)
        self.assertIsNone(mission.arrived())
        self.assertEqual(mission.state, "SUCCEEDED")

    def test_cancel_discards_targets(self):
        mission = Mission()
        mission.load((Waypoint(1., 1., 0.),))
        mission.cancel()
        self.assertIsNone(mission.current)
        self.assertEqual(mission.state, "CANCELLED")

    def test_unreachable_exhausts_retry_budget(self):
        mission = Mission(max_replans=2)
        mission.load((Waypoint(1., 0., 0.),))
        self.assertTrue(mission.unreachable())
        self.assertTrue(mission.unreachable())
        self.assertFalse(mission.unreachable())
        self.assertEqual(mission.state, "FAILED")
        self.assertIsNone(mission.current)

    def test_new_waypoint_resets_retry_counter(self):
        mission = Mission(max_replans=1)
        mission.load((Waypoint(1., 0., 0.), Waypoint(2., 0., 0.)))
        mission.unreachable()
        mission.arrived()
        self.assertTrue(mission.unreachable())

    def test_reject_invalid_mission_atomically(self):
        mission = Mission(max_waypoints=1)
        with self.assertRaises(ValueError):
            mission.load((Waypoint(0., 0., 0.), Waypoint(1., 0., 0.)))
        with self.assertRaises(ValueError):
            Waypoint(float("nan"), 0., 0.)


if __name__ == "__main__":
    unittest.main()
