// Copyright (c) 2026 Leroy contributors
// SPDX-License-Identifier: Apache-2.0
//
// Experimental OFFLINE fixture only. No motor or sensor authority.
// Tests whether releasing the callback group during lifecycle cleanup
// avoids the stock Jazzy Costmap2DROS destructor SIGSEGV.
#include <memory>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"

class CostmapFixture final : public nav2_costmap_2d::Costmap2DROS
{
public:
  using nav2_costmap_2d::Costmap2DROS::Costmap2DROS;

  nav2_util::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override
  {
    // Parent stops the dedicated executor thread and releases costmap,
    // subscriptions, layers, and TF resources.
    const auto result = nav2_costmap_2d::Costmap2DROS::on_cleanup(state);
    if (result != nav2_util::CallbackReturn::SUCCESS) {
      return result;
    }

    // This callback group was manually attached by on_configure().
    // Release the group while the ROS context is still valid.
    if (executor_ && callback_group_) {
      executor_->remove_callback_group(callback_group_);
    }
    executor_.reset();
    callback_group_.reset();

    RCLCPP_INFO(
      get_logger(),
      "FIXTURE: released executor and callback group during cleanup");
    return result;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CostmapFixture>();
  rclcpp::spin(node->get_node_base_interface());
  // Default SIGINT shutdown behavior matches Nav2 upstream.
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
