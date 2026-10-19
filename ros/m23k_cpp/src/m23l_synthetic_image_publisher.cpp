// M23L: 900 synthetic 640x480 RGB8 Images at nominal 30 Hz.
// Separate ROS publisher process, NOT a physical camera or recording.
// Data are synthetic and must never be called a real sensor capture.
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

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

class SyntheticImagePublisher final : public rclcpp::Node {
 public:
  SyntheticImagePublisher() : rclcpp::Node("m23l_synthetic_rgb_publisher",
      rclcpp::NodeOptions().enable_rosout(false)
                           .start_parameter_services(false)) {
    // usb_cam/image_transport's camera publisher uses reliable QoS(100)
    // in the upstream code examined for this experiment. Independently
    // built publishers keep DDS message deserialization cost intact.
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile();
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/camera/image_raw", qos);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", qos);

    image_.header.frame_id = "camera_optical_frame";
    image_.height = kHeight;
    image_.width = kWidth;
    image_.encoding = "rgb8";
    image_.is_bigendian = false;
    image_.step = kWidth * 3;
    image_.data.resize(kWidth * kHeight * 3, 0x42);
    info_.header.frame_id = "camera_optical_frame";
    info_.width = kWidth;
    info_.height = kHeight;
    info_.p = {380.0, 0.0, 320.0, 0.0,
               0.0, 380.0, 240.0, 0.0,
               0.0, 0.0, 1.0, 0.0};

    timer_ = create_wall_timer(std::chrono::nanoseconds(kPeriodNs),
                              [this]() { on_timer(); });
  }

 private:
  void on_timer() {
    const auto start = steady_ns();
    if (!first_start_) first_start_ = start;
    last_start_ = start;
    // No pixel data are generated from any real sensor.
    // Jazzy rclcpp::Time implicitly converts to builtin_interfaces::msg::Time.
    // Take one ROS timestamp and reuse it for Image and CameraInfo.
    const auto stamp = now();
    image_.header.stamp = stamp;
    info_.header.stamp = stamp;
    image_pub_->publish(image_);
    info_pub_->publish(info_);
    const auto took = steady_ns() - start;
    if (took > max_publish_duration_ns_) max_publish_duration_ns_ = took;
    ++count_;
    if (count_ == kFrames) {
      std::cout << "M23L_PUBLISHER_COUNTS=" << count_ << "," << count_
                << std::endl;
      std::cout << "M23L_PUBLISHER_START_SPAN_NS="
                << (last_start_ - first_start_) << std::endl;
      std::cout << "M23L_PUBLISHER_MAX_CALLBACK_NS="
                << max_publish_duration_ns_ << std::endl;
      std::cout << "M23L_PUBLISHER_PAYLOAD_BYTES="
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
  std::int64_t max_publish_duration_ns_ = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SyntheticImagePublisher>());
  } catch (const std::exception &exc) {
    std::cerr << "M23L_PUBLISHER_FATAL=" << exc.what() << std::endl;
    if (rclcpp::ok()) rclcpp::shutdown();
    return 2;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
