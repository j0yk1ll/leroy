"""Fail-closed source contract for XRSLAM startup and camera queue.

This is a lightweight guard against accidentally restoring the startup race
or weakening runtime image queue safety. Real ROS integration is still required.
"""
from pathlib import Path
import unittest


PACKAGE = Path(__file__).resolve().parents[2] / 'src' / 'leroy_localization'
SOURCE = (PACKAGE / "src" / "leroy_xrslam_node.cpp").read_text(encoding="utf-8")
TIMING_HEADER = (PACKAGE / "include" / "leroy_xrslam" / "startup_timing.hpp"
                 ).read_text(encoding="utf-8")
MODE_HEADER = (PACKAGE / "include" / "leroy_xrslam" / "sensor_mode.hpp"
               ).read_text(encoding="utf-8")


class XrslamStartupQueueContract(unittest.TestCase):
    def test_waiting_for_tf_does_not_enqueue_camera_frames(self):
        image = SOURCE.split("  void image(", 1)[1].split("  void poll(", 1)[0]
        self.assertRegex(
            image,
            r"if\(!leroy_xrslam::can_queue_camera\(\s*"
            r"initialized_, use_imu_, t, offset_, first_forwarded_imu_time_\)\)return;",
        )
        self.assertLess(
            image.index("!leroy_xrslam::can_queue_camera("),
            image.index("pending_.push_back(m)"),
        )
        # The compiled helper uses the same effective timestamp (t+offset)
        # as the runtime drain, without requiring IMU in monocular mode.
        self.assertIn(
            "camera_header_time + camera_imu_offset_sec > first_forwarded_imu_time",
            TIMING_HEADER,
        )
        self.assertIn("camera_after_first_forwarded_imu(", MODE_HEADER)
        self.assertIn("initialized && (!requires_imu ||", MODE_HEADER)
        drain = SOURCE.split("  void drain(", 1)[1].split("  void read_pose(", 1)[0]
        self.assertIn("await_imu_for_camera(", drain)
        self.assertIn("latest_imu_stamp < image_stamp + camera_imu_offset + 0.001",
                      MODE_HEADER)

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

    def test_monocular_mode_is_imu_free_and_never_publishes_metric_odometry(self):
        ctor = SOURCE.split("  LeroyXrslam()", 1)[1].split("  ~LeroyXrslam()", 1)[0]
        self.assertIn("if(use_imu_) {", ctor)
        self.assertIn("imu_sub_=create_subscription", ctor)
        self.assertIn("if(pose_validated_ || enable_map_tf_)", ctor)
        self.assertIn("sensor_mode_requires_imu(mode)", ctor)
        init = SOURCE.split("  bool init_locked()", 1)[1].split("  void info(", 1)[0]
        self.assertIn("if(use_imu_)", init)
        self.assertIn("XRSLAMRequiresIMU(config)", init)
        self.assertIn("!=use_imu_", init)

    def test_unvalidated_pose_still_suppresses_odometry(self):
        pose = SOURCE.split("  void read_pose(", 1)[1].split("  void fail(", 1)[0]
        self.assertLess(
            pose.index('if(!pose_validated_)'),
            pose.index('odom_pub_->publish(odom);'),
        )
        self.assertIn('enable_map_tf_=declare_parameter<bool>("enable_map_tf",false)', SOURCE)


if __name__ == "__main__":
    unittest.main()
