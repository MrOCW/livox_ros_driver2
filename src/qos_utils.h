#ifndef LIVOX_ROS_DRIVER2_QOS_UTILS_H_
#define LIVOX_ROS_DRIVER2_QOS_UTILS_H_

#include <stdexcept>
#include <string>

#include "rclcpp/qos.hpp"

namespace livox_ros {

inline rclcpp::QoS MakeVolatileQos(size_t depth, const std::string & reliability) {
  if (depth == 0) {
    throw std::invalid_argument("QoS depth must be greater than zero");
  }

  rclcpp::QoS qos{rclcpp::KeepLast(depth)};
  qos.durability_volatile();
  if (reliability == "best_effort") {
    qos.best_effort();
  } else if (reliability == "reliable") {
    qos.reliable();
  } else {
    throw std::invalid_argument(
        "QoS reliability must be 'best_effort' or 'reliable', got '" + reliability + "'");
  }
  return qos;
}

}  // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER2_QOS_UTILS_H_
