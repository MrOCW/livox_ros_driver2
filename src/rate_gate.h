#ifndef LIVOX_ROS_DRIVER2_RATE_GATE_H_
#define LIVOX_ROS_DRIVER2_RATE_GATE_H_

#include <chrono>
#include <cmath>
#include <stdexcept>

namespace livox_ros {

class RateGate {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit RateGate(double max_rate_hz)
  : interval_(std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(ValidateAndInvert(max_rate_hz)))) {}

  bool ShouldPublish(TimePoint now) {
    if (!has_published_ || now - last_publish_ >= interval_) {
      has_published_ = true;
      last_publish_ = now;
      return true;
    }
    return false;
  }

 private:
  static double ValidateAndInvert(double max_rate_hz) {
    if (!std::isfinite(max_rate_hz) || max_rate_hz <= 0.0) {
      throw std::invalid_argument("max_rate_hz must be finite and greater than zero");
    }
    return 1.0 / max_rate_hz;
  }

  Clock::duration interval_;
  TimePoint last_publish_{};
  bool has_published_ = false;
};

}  // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER2_RATE_GATE_H_
