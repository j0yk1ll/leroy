#include <xrslam_ros2/device_config.hpp>
#include <cassert>
#include <iostream>
#include <stdexcept>
int main() {
    xrslam_ros2::SensorConfig c;
    c.width=640; c.height=480;
    c.fx=525; c.fy=525; c.cx=319.5; c.cy=239.5;
    c.base_from_camera.quaternion_xyzw={0.5,-0.5,0.5,-0.5};
    c.base_from_imu.quaternion_xyzw={0,0,0,1};
    c.base_from_imu.translation={0.095,-0.095,0};
    c.gyro_variance={1e-6,2e-6,3e-6};
    c.accel_variance={1e-4,1e-4,1e-4};
    c.gyro_bias_variance={1e-8,1e-8,1e-8};
    c.accel_bias_variance={1e-6,1e-6,1e-6};
    const std::string s=xrslam_ros2::render_device_config(c);
    assert(s.find("resolution: [640, 480]")!=std::string::npos);
    assert(s.find("p_bi: [0.095")!=std::string::npos);
    assert(s.find("q_bc: [0.5, -0.5, 0.5, -0.5]")!=std::string::npos);
    assert(s.find("cov_g: [9.9999999999999995e-07, 0")!=std::string::npos || s.find("cov_g: [1e-06, 0")!=std::string::npos);
    assert(s.find("camera_distortion_flag: 0")!=std::string::npos);
    c.base_from_imu.quaternion_xyzw={0,0,0,0};
    bool rejected=false;
    try { (void)xrslam_ros2::render_device_config(c); }
    catch(const std::invalid_argument&) { rejected=true; }
    assert(rejected);
    c.base_from_imu.quaternion_xyzw={0,0,0,1};
    c.gyro_variance[0]=0;
    rejected=false;
    try { (void)xrslam_ros2::render_device_config(c); }
    catch(const std::invalid_argument&) { rejected=true; }
    assert(rejected);
    std::cout << "PASS: TF/CameraInfo device YAML, quaternion and noise guards\n";
}
