// M23K: low-overhead header-only OBSERVER of sensor_msgs/Image.
// The pixels are not accessed, copied into Python, written to disk,
// published or used for image processing. No serial or robot motion.
#include <array>
#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace {
int64_t receipt_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
template<typename H>
int64_t header_ns(const H &header) {
  return static_cast<int64_t>(header.stamp.sec) * 1000000000LL
       + static_cast<int64_t>(header.stamp.nanosec);
}
}  // namespace

class HeaderCounter final : public rclcpp::Node {
 public:
  HeaderCounter() : rclcpp::Node("m23k_cpp_header_counter",
      rclcpp::NodeOptions().enable_rosout(false)
                           .start_parameter_services(false)) {
    auto qos = rclcpp::SensorDataQoS(); // best effort, volatile
    raw_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
        sample("raw", 0, m->header);
      });
    // In isolated raw-only trials, explicitly avoid subscribing to an
    // absent rectifier. The raw transport still undergoes full DDS
    // deserialization; this is NOT a publication-side frame counter.
    const char *raw_only = std::getenv("M23K_RAW_ONLY");
    if (raw_only && std::string(raw_only) == "1") {
      std::cout << "M23K_COUNTER_VARIANT=RAW_ONLY" << std::endl;
    } else {
      rect_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_rect", qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
          sample("rect", 1, m->header);
        });
      std::cout << "M23K_COUNTER_VARIANT=FULL" << std::endl;
    }
    info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
        sample("info", 2, m->header);
      });
    guard_ = create_wall_timer(std::chrono::seconds(2), [this]() {
      for (const auto &topic : {"/cmd_vel", "/odom", "/tf",
                                "/depthart/depth"}) {
        if (count_publishers(topic) != 0) {
          std::cerr << "M23K_COUNTER_FATAL=FORBIDDEN_GRAPH_TOPIC:" << topic
                    << std::endl;
          rclcpp::shutdown();
          return;
        }
      }
      std::cout.flush();
    });
  }

  void summary() {
    std::cout << "M23K_COUNTER_COUNTS="
              << counts_[0] << "," << counts_[1] << ","
              << counts_[2] << std::endl;
  }

 private:
  template<typename HeaderT>
  void sample(const char *kind, size_t index, const HeaderT &header) {
    if (counts_[index] >= 5000) {
      std::cerr << "M23K_COUNTER_FATAL=BOUNDED_SAMPLE_CAP:" << kind
                << std::endl;
      rclcpp::shutdown();
      return;
    }
    const auto stamp = header_ns(header);
    if (stamp <= 0) {
      std::cerr << "M23K_COUNTER_FATAL=INVALID_HEADER:" << kind << std::endl;
      rclcpp::shutdown();
      return;
    }
    ++counts_[index];
    std::cout << "M23K_SAMPLE " << kind << " " << stamp
              << " " << receipt_ns() << "\n";
  }

  std::array<uint32_t, 3> counts_{};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rect_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_;
  rclcpp::TimerBase::SharedPtr guard_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    auto counter = std::make_shared<HeaderCounter>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(counter);
    executor.spin();
    counter->summary();
  } catch (const std::exception &error) {
    std::cerr << "M23K_COUNTER_FATAL=EXCEPTION:" << error.what() << std::endl;
    code = 2;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return code;
}
