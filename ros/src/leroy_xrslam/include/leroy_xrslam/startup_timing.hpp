#pragma once

// Keep this predicate ROS-free so the camera/IMU startup time contract can be
// tested with actual compiled C++ without launching XRSLAM or any robot nodes.
namespace leroy_xrslam {

inline bool camera_after_first_forwarded_imu(
    double camera_header_time,
    double camera_imu_offset_sec,
    double first_forwarded_imu_time) {
  // XRSLAM's configured camera time is t_camera + offset. Do not hand XRSLAM
  // any frame whose effective time precedes (or equals) its first IMU sample.
  return first_forwarded_imu_time >= 0.0 &&
         camera_header_time + camera_imu_offset_sec > first_forwarded_imu_time;
}

}  // namespace leroy_xrslam
