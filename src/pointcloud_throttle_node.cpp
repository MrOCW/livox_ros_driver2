#include "pointcloud_throttle_node.h"

#include <utility>

#include "qos_utils.h"

namespace livox_ros {

PointCloudThrottleNode::PointCloudThrottleNode(const rclcpp::NodeOptions & options)
: Node("livox_pointcloud_throttle", options),
  rate_gate_(declare_parameter<double>("max_rate_hz", 5.0)) {
  const auto qos = MakeVolatileQos(1, "best_effort");
  publisher_ = create_publisher<PointCloud2>("output", qos);
  subscription_ = create_subscription<PointCloud2>(
      "input", qos,
      [this](PointCloud2::UniquePtr message) {
        HandlePointCloud(std::move(message));
      });
}

void PointCloudThrottleNode::HandlePointCloud(PointCloud2::UniquePtr message) {
  if (rate_gate_.ShouldPublish(RateGate::Clock::now())) {
    publisher_->publish(std::move(message));
  }
}

}  // namespace livox_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::PointCloudThrottleNode)
