// M23AB synthetic-only experiment: directly instantiate the stock Jazzy
// RectifyNode without rclcpp_components::NodeFactory or class_loader calls.
// Linking librectify.so can still load its TRANSITIVE dependencies.
// No image processing, calibration or sensor implementation is copied here.
#include <memory>

#include "image_proc/rectify.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Match the stock generated standalone's node-wrapper scope as closely
  // as possible, with the executor kept alive until after node teardown.
  rclcpp::executors::SingleThreadedExecutor executor;
  {
    auto node = std::make_shared<image_proc::RectifyNode>(rclcpp::NodeOptions());
    executor.add_node(node);
    executor.spin();
    if (node->get_node_base_interface()->get_associated_with_executor_atomic().load()) {
      executor.remove_node(node);
    }
  }  // node and its OpenCV map cache are destroyed before rclcpp::shutdown().

  rclcpp::shutdown();
  return 0;
}
