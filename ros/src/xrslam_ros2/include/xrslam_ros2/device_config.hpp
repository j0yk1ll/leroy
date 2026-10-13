#ifndef XRSLAM_ROS2_DEVICE_CONFIG_HPP
#define XRSLAM_ROS2_DEVICE_CONFIG_HPP

#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace xrslam_ros2 {

// Values are collected from TF2 (/tf_static), rectified CameraInfo, and
// independently calibrated inertial noise. This header has no ROS dependency.
struct RigidExtrinsic {
    std::array<double, 3> translation{};  // position of sensor origin in base_link
    std::array<double, 4> quaternion_xyzw{}; // rotation sensor->base_link
};
struct SensorConfig {
    int width = 0, height = 0;
    double fx = 0, fy = 0, cx = 0, cy = 0;
    RigidExtrinsic base_from_camera;
    RigidExtrinsic base_from_imu;
    std::array<double, 3> gyro_variance{};
    std::array<double, 3> accel_variance{};
    std::array<double, 3> gyro_bias_variance{};
    std::array<double, 3> accel_bias_variance{};
    double pixel_variance = 0.5;
    double camera_time_offset_s = 0.0;
};

inline void validate(const RigidExtrinsic &t) {
    double sq = 0.0;
    for (double v : t.translation)
        if (!std::isfinite(v)) throw std::invalid_argument("nonfinite TF translation");
    for (double v : t.quaternion_xyzw) {
        if (!std::isfinite(v)) throw std::invalid_argument("nonfinite TF quaternion");
        sq += v * v;
    }
    if (std::abs(sq - 1.0) > 1e-3)
        throw std::invalid_argument("TF quaternion must be normalized");
}
inline std::string render_device_config(const SensorConfig &c) {
    if (c.width <= 0 || c.height <= 0 ||
        !std::isfinite(c.fx) || c.fx <= 0 ||
        !std::isfinite(c.fy) || c.fy <= 0 ||
        !std::isfinite(c.cx) || !std::isfinite(c.cy) ||
        !std::isfinite(c.pixel_variance) || c.pixel_variance <= 0 ||
        !std::isfinite(c.camera_time_offset_s))
        throw std::invalid_argument("invalid CameraInfo/calibration values");
    validate(c.base_from_camera);
    validate(c.base_from_imu);
    for (auto a : {c.gyro_variance, c.accel_variance,
                   c.gyro_bias_variance, c.accel_bias_variance}) {
        for (double v : a)
            if (!std::isfinite(v) || v <= 0)
                throw std::invalid_argument("calibrated IMU variances must be positive");
    }
    std::ostringstream out;
    out << std::setprecision(17);
    auto vec = [&out](const auto &a) {
        out << '[';
        for (std::size_t i = 0; i < a.size(); ++i) {
            if (i) out << ", ";
            out << a[i];
        }
        out << ']';
    };
    auto matrix = [&out](const std::array<double, 3> &d) {
        out << '[' << d[0] << ", 0, 0, 0, " << d[1] << ", 0, 0, 0, " << d[2] << ']';
    };
    out << "# GENERATED from /tf_static and rectified CameraInfo; do not edit.\n";
    out << "cam0:\n  resolution: [" << c.width << ", " << c.height << "]\n";
    out << "  camera_model: pinhole\n  camera_distortion_flag: 0\n";
    out << "  distortion: [0, 0, 0, 0]\n";
    out << "  intrinsics: [" << c.fx << ", " << c.fy << ", " << c.cx << ", " << c.cy << "]\n";
    out << "  time_offset: " << c.camera_time_offset_s << '\n';
    out << "  noise: [" << c.pixel_variance << ", 0, 0, " << c.pixel_variance << "]\n";
    out << "  extrinsic:\n    q_bc: "; vec(c.base_from_camera.quaternion_xyzw); out << '\n';
    out << "    p_bc: "; vec(c.base_from_camera.translation); out << '\n';
    out << "imu:\n  extrinsic:\n    q_bi: "; vec(c.base_from_imu.quaternion_xyzw); out << '\n';
    out << "    p_bi: "; vec(c.base_from_imu.translation); out << '\n';
    out << "  noise:\n    cov_g: "; matrix(c.gyro_variance); out << '\n';
    out << "    cov_a: "; matrix(c.accel_variance); out << '\n';
    out << "    cov_bg: "; matrix(c.gyro_bias_variance); out << '\n';
    out << "    cov_ba: "; matrix(c.accel_bias_variance); out << '\n';
    return out.str();
}
} // namespace xrslam_ros2
#endif
