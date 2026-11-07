#pragma once

#include "leroy_xrslam/startup_timing.hpp"
#include <string_view>

namespace leroy_xrslam {

// RGB-D is admitted only with the explicit depth subscriber and exact-stamp pairing.
// RGB-D plus IMU is experimental and requires a separate guarded adapter flag.
inline bool supported_sensor_mode(std::string_view mode) {
  return mode == "monocular" || mode == "monocular_imu" || mode == "rgbd" || mode == "rgbd_imu";
}

inline bool sensor_mode_requires_depth(std::string_view mode) {
  return mode == "rgbd" || mode == "rgbd_imu";
}

inline bool sensor_mode_requires_imu(std::string_view mode) {
  return mode == "monocular_imu" || mode == "rgbd_imu";
}

// Pure-monocular images must not be blocked on an IMU that is deliberately
// neither subscribed to nor pushed into XRSLAM. VIO retains the startup guard
// from the TF queue-overflow fix and uses effective camera time t + offset.
inline bool can_queue_camera(
    bool initialized, bool requires_imu, double image_stamp,
    double camera_imu_offset, double first_forwarded_imu) {
  return initialized && (!requires_imu ||
      camera_after_first_forwarded_imu(
          image_stamp, camera_imu_offset, first_forwarded_imu));
}

inline bool await_imu_for_camera(
    bool requires_imu, double latest_imu_stamp, double image_stamp,
    double camera_imu_offset) {
  return requires_imu &&
      latest_imu_stamp < image_stamp + camera_imu_offset + 0.001;
}

}  // namespace leroy_xrslam
