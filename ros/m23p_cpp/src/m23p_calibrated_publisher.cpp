// M23P: fabricated *calibrated-shaped* CameraInfo and RGB8 images for
// a device-free RectifyNode lifecycle exercise. NOT a real calibration.
// M23L's accepted 900-image publisher must remain unchanged.
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace {
constexpr unsigned int kFrames = 900;
constexpr int kWidth = 640;
constexpr int kHeight = 480;
constexpr std::int64_t kPeriodNs = 33333333LL;

std::int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

class M23PSyntheticCalibratedPublisher final : public rclcpp::Node {
 public:
  M23PSyntheticCalibratedPublisher()
      : rclcpp::Node("m23p_synthetic_calibrated_publisher",
                     rclcpp::NodeOptions().enable_rosout(false)
                                          .start_parameter_services(false)) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(100))
                   .reliable().durability_volatile();
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "/camera/image_raw", qos);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info", qos);

    image_.header.frame_id = "synthetic_camera_optical_frame";
    image_.height = kHeight;
    image_.width = kWidth;
    image_.encoding = "rgb8";
    image_.step = kWidth * 3;
    image_.is_bigendian = false;
    image_.data.resize(kWidth * kHeight * 3, 0x42);

    info_.header.frame_id = image_.header.frame_id;
    info_.width = kWidth;
    info_.height = kHeight;
    info_.distortion_model = "plumb_bob";
    // Entirely fabricated, internally consistent pinhole camera model.
    // Unlike M23L's intentionally generic CameraInfo, K[0] is nonzero:
    // image_proc would reject an uncalibrated K and publish nothing.
    info_.k = {380.0, 0.0, 320.0,
               0.0, 380.0, 240.0,
               0.0, 0.0, 1.0};
    info_.r = {1.0, 0.0, 0.0,
               0.0, 1.0, 0.0,
               0.0, 0.0, 1.0};
    info_.p = {380.0, 0.0, 320.0, 0.0,
               0.0, 380.0, 240.0, 0.0,
               0.0, 0.0, 1.0, 0.0};
    // Nonzero D forces the actual OpenCV rectification path.
    info_.d = {-0.08, 0.01, 0.001, -0.001, 0.0};

    std::cout << "M23P_SYNTHETIC_CALIBRATION=FABRICATED_NONZERO_DISTORTION"
              << std::endl;
    timer_ = create_wall_timer(std::chrono::nanoseconds(kPeriodNs),
                               [this]() { on_frame(); });
  }

 private:
  void on_frame() {
    const std::int64_t started = steady_ns();
    if (!first_start_) first_start_ = started;
    last_start_ = started;
    const auto stamp = now();
    image_.header.stamp = stamp;
    info_.header.stamp = stamp;
    image_pub_->publish(image_);
    info_pub_->publish(info_);
    const auto elapsed = steady_ns() - started;
    if (elapsed > max_callback_ns_) max_callback_ns_ = elapsed;
    if (++count_ == kFrames) {
      std::cout << "M23P_PUBLISHER_COUNTS=" << count_ << "," << count_
                << std::endl;
      std::cout << "M23P_PUBLISHER_START_SPAN_NS="
                << (last_start_ - first_start_) << std::endl;
      std::cout << "M23P_PUBLISHER_MAX_CALLBACK_NS="
                << max_callback_ns_ << std::endl;
      std::cout << "M23P_PUBLISHER_PAYLOAD_BYTES="
                << image_.data.size() << std::endl;
      rclcpp::shutdown();
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image image_;
  sensor_msgs::msg::CameraInfo info_;
  unsigned int count_ = 0;
  std::int64_t first_start_ = 0;
  std::int64_t last_start_ = 0;
  std::int64_t max_callback_ns_ = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<M23PSyntheticCalibratedPublisher>());
  } catch (const std::exception &error) {
    std::cerr << "M23P_PUBLISHER_FATAL=" << error.what() << std::endl;
    if (rclcpp::ok()) rclcpp::shutdown();
    return 2;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
