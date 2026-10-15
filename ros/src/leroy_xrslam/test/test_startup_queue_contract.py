"""Fail-closed source contract for XRSLAM startup and camera queue.

This is a lightweight guard against accidentally restoring the startup race
or weakening runtime image queue safety. Real ROS integration is still required.
"""
from pathlib import Path
import re
import unittest


SOURCE = (Path(__file__).resolve().parents[1]
          / "src" / "leroy_xrslam_node.cpp").read_text(encoding="utf-8")


class XrslamStartupQueueContract(unittest.TestCase):
    def test_waiting_for_tf_does_not_enqueue_camera_frames(self):
        image = SOURCE.split("  void image(", 1)[1].split("  void poll(", 1)[0]
        self.assertRegex(
            image,
            r"if\(!initialized_ \|\| first_forwarded_imu_time_<0 \|\|\s*"
            r"t<=first_forwarded_imu_time_\)return;",
        )
        self.assertLess(
            image.index("if(!initialized_ || first_forwarded_imu_time_<0"),
            image.index("pending_.push_back(m)"),
        )

    def test_only_actual_forwarded_imu_unblocks_camera(self):
        imu = SOURCE.split("  void imu(", 1)[1].split("  void image(", 1)[0]
        self.assertIn("if(!initialized_||failed_)return;", imu)
        self.assertLess(
            imu.index("XRSLAMPushSensorData(XRSLAM_SENSOR_ACCELERATION,&accel)"),
            imu.index("if(first_forwarded_imu_time_<0)"),
        )
        self.assertIn("double first_forwarded_imu_time_=-1;", SOURCE)

    def test_runtime_queue_still_fails_closed(self):
        image = SOURCE.split("  void image(", 1)[1].split("  void poll(", 1)[0]
        self.assertIn(
            'if(pending_.size()>=8){ fail("camera_queue_overflow");return; }',
            image,
        )

    def test_unvalidated_pose_still_suppresses_odometry(self):
        pose = SOURCE.split("  void read_pose(", 1)[1].split("  void fail(", 1)[0]
        self.assertLess(
            pose.index('if(!pose_validated_)'),
            pose.index('odom_pub_->publish(odom);'),
        )
        self.assertIn('enable_map_tf_=declare_parameter<bool>("enable_map_tf",false)', SOURCE)


if __name__ == "__main__":
    unittest.main()
