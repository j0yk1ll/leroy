"""No-device tests: exact stamped real-data transport contract."""
import unittest
from leroy_nav2_perception.live_pipeline_contract import LiveChain


P = [100.0, 0.0, 320.0, 0.0, 0.0, 100.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]


class ChainTest(unittest.TestCase):
    def test_matches_same_stamped_info_depth_and_cloud(self):
        c = LiveChain()
        c.on_depth(12345, "camera_optical_frame", 640, 480,
                   "32FC1", 2560, 1228800, 0.74)
        c.on_cloud(12345, "camera_optical_frame", 4800, 1, 16, 76800)
        self.assertEqual(len(c.matched), 0)
        c.on_info(12345, "camera_optical_frame", 640, 480, P)
        self.assertEqual(len(c.matched), 1)
        self.assertEqual(c.ages, [0.74])
        c.on_info(12345, "camera_optical_frame", 640, 480, P)
        self.assertEqual(len(c.matched), 1)

    def test_different_stamps_cannot_join(self):
        c = LiveChain()
        c.on_info(42, "camera_optical_frame", 640, 480, P)
        c.on_depth(43, "camera_optical_frame", 640, 480,
                   "32FC1", 2560, 1228800, 0.70)
        c.on_cloud(43, "camera_optical_frame", 4800, 1, 16, 76800)
        self.assertEqual(len(c.matched), 0)

    def test_stale_or_malformed_depth_is_not_accepted(self):
        c = LiveChain()
        for age in (-0.1, 2.01):
            with self.assertRaises(ValueError):
                c.on_depth(2, "camera_optical_frame", 640, 480,
                           "32FC1", 2560, 1228800, age)
        with self.assertRaises(ValueError):
            c.on_depth(2, "camera_optical_frame", 640, 480,
                       "rgb8", 2560, 1228800, 0.72)
        with self.assertRaises(ValueError):
            c.on_cloud(2, "camera_optical_frame", 0, 1, 16, 0)
        self.assertEqual(len(c.matched), 0)

    def test_mismatched_camera_frame_rejected(self):
        c = LiveChain()
        c.on_info(1, "camera_optical_frame", 640, 480, P)
        c.on_depth(1, "camera_optical_frame", 640, 480,
                   "32FC1", 2560, 1228800, 0.72)
        c.on_cloud(1, "wrong_frame", 4800, 1, 16, 76800)
        self.assertEqual(len(c.matched), 0)
        self.assertIn("camera-depth-cloud frame/dimension mismatch", c.faults)


if __name__ == "__main__":
    unittest.main()
