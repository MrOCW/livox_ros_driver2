#ifndef LIVOX_ROS_DRIVER2_POINTCLOUD_THROTTLE_NODE_H_
#define LIVOX_ROS_DRIVER2_POINTCLOUD_THROTTLE_NODE_H_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rate_gate.h"

namespace livox_ros {

class PointCloudThrottleNode final : public rclcpp::Node {
 public:
  explicit PointCloudThrottleNode(const rclcpp::NodeOptions & options);

 private:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  void HandlePointCloud(PointCloud2::UniquePtr message);

  RateGate rate_gate_;
  rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
};

}  // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER2_POINTCLOUD_THROTTLE_NODE_H_
