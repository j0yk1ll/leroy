import math
import unittest
from leroy_navigation.goal_controller import GoalController, Pose


class TestGoalController(unittest.TestCase):
    def setUp(self):
        self.controller = GoalController()

    def test_forward(self):
        result = self.controller.calculate(Pose(0.,0.,0.), Pose(1.,0.,0.))
        self.assertGreater(result.vx, 0)
        self.assertEqual(result.vy, 0)
        self.assertLessEqual(result.vx, 0.10)

    def test_strafe(self):
        result = self.controller.calculate(Pose(0.,0.,0.), Pose(0.,1.,0.))
        self.assertGreater(result.vy, 0)
        self.assertEqual(result.vx, 0)

    def test_world_to_body(self):
        result = self.controller.calculate(Pose(0.,0.,math.pi/2), Pose(1.,0.,math.pi/2))
        self.assertLess(result.vy, 0)
        self.assertAlmostEqual(result.vx, 0., places=7)

    def test_turn_and_arrival(self):
        result = self.controller.calculate(Pose(0.,0.,0.), Pose(0.,0.,math.pi/2))
        self.assertEqual(result.vx, 0)
        self.assertGreater(result.wz, 0)
        self.assertTrue(self.controller.calculate(
            Pose(0.,0.,0.), Pose(0.,0.,0.)).arrived)

    def test_invalid_pose(self):
        with self.assertRaises(ValueError):
            self.controller.calculate(Pose(float("nan"),0,0), Pose(0,0,0))
