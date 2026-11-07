// Executable mode tests (no ROS graph, XRSLAM instance, sensors or motors).
#include "leroy_xrslam/sensor_mode.hpp"

#include <cstdio>
#include <string_view>

namespace {
bool check(const char* name, bool actual, bool expected) {
  if(actual != expected) {
    std::fprintf(stderr,"FAILED %s: expected=%d got=%d\n",
                 name,expected,actual);
    return false;
  }
  return true;
}
}

int main() {
  using leroy_xrslam::supported_sensor_mode;
  using leroy_xrslam::sensor_mode_requires_imu;
  using leroy_xrslam::sensor_mode_requires_depth;
  using leroy_xrslam::can_queue_camera;
  using leroy_xrslam::await_imu_for_camera;
  bool ok=true;

  ok &= check("mono supported",supported_sensor_mode("monocular"),true);
  ok &= check("vio supported",supported_sensor_mode("monocular_imu"),true);
  ok &= check("rgbd supported",supported_sensor_mode("rgbd"),true);
  ok &= check("rgbd requires depth",sensor_mode_requires_depth("rgbd"),true);
  ok &= check("mono no depth",sensor_mode_requires_depth("monocular"),false);
  ok &= check("rgbd no IMU",sensor_mode_requires_imu("rgbd"),false);
  ok &= check("reject stereo",supported_sensor_mode("stereo"),false);
  ok &= check("experimental rgbd imu parsed",supported_sensor_mode("rgbd_imu"),true);
  ok &= check("rgbd imu needs depth",sensor_mode_requires_depth("rgbd_imu"),true);
  ok &= check("rgbd imu needs imu",sensor_mode_requires_imu("rgbd_imu"),true);
  ok &= check("reject empty",supported_sensor_mode(""),false);
  ok &= check("mono does not need IMU",sensor_mode_requires_imu("monocular"),false);
  ok &= check("vio requires IMU",sensor_mode_requires_imu("monocular_imu"),true);

  // Frames must be dropped during TF-dependent XRSLAM initialization.
  ok &= check("mono before initialization",can_queue_camera(false,false,11.0,0.0,-1),false);
  ok &= check("vio before initialization",can_queue_camera(false,true,11.0,0.0,10),false);
  // Monocular must accept camera data without any IMU ROS topic, even though
  // the first-forwarded IMU timestamp remains its sentinel value.
  ok &= check("mono needs no IMU",can_queue_camera(true,false,11.0,0.0,-1),true);
  ok &= check("mono does not wait IMU",await_imu_for_camera(false,-1,11.0,0.0),false);

  // The VIO path must still enforce exactly the PR #26 effective time gate.
  ok &= check("vio needs first IMU",can_queue_camera(true,true,11.0,0.0,-1),false);
  ok &= check("vio rejects earlier effective camera",can_queue_camera(true,true,10.01,-0.020,10.0),false);
  ok &= check("vio accepts later effective camera",can_queue_camera(true,true,10.03,-0.020,10.0),true);
  ok &= check("vio waits for IMU",await_imu_for_camera(true,10.0,10.03,-0.020),true);
  ok &= check("vio consumes with sufficient IMU",await_imu_for_camera(true,10.02,10.03,-0.020),false);
  return ok?0:1;
}
