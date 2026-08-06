//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef TIMESTAMP_GUARD_H_
#define TIMESTAMP_GUARD_H_

#include <cstdint>
#include <mutex>

namespace livox_ros {

enum class TimestampState : uint8_t {
  kNoSync,
  kValidSynchronized,
  kInvalidSynchronized,
};

struct TimestampDecision {
  bool accept;
  bool uses_sensor_timestamp;
  bool state_changed;
  bool recovered;
  uint64_t timestamp_ns;
  uint64_t absolute_offset_ns;
  TimestampState state;
};

class TimestampGuard {
 public:
  void Configure(uint64_t max_offset_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    max_offset_ns_ = max_offset_ns;
    initialized_ = false;
    state_ = TimestampState::kNoSync;
  }

  TimestampDecision Evaluate(
      bool has_synchronized_timestamp, uint64_t sensor_timestamp_ns,
      uint64_t system_timestamp_ns) {
    const uint64_t absolute_offset_ns =
        sensor_timestamp_ns >= system_timestamp_ns
            ? sensor_timestamp_ns - system_timestamp_ns
            : system_timestamp_ns - sensor_timestamp_ns;

    std::lock_guard<std::mutex> lock(mutex_);
    const TimestampState new_state = !has_synchronized_timestamp
        ? TimestampState::kNoSync
        : absolute_offset_ns <= max_offset_ns_
            ? TimestampState::kValidSynchronized
            : TimestampState::kInvalidSynchronized;
    const bool state_changed = initialized_ && new_state != state_;
    const bool recovered = initialized_ &&
        state_ == TimestampState::kInvalidSynchronized &&
        new_state == TimestampState::kValidSynchronized;
    initialized_ = true;
    state_ = new_state;

    const bool use_sensor_timestamp =
        new_state == TimestampState::kValidSynchronized;
    return TimestampDecision{
        new_state != TimestampState::kInvalidSynchronized,
        use_sensor_timestamp,
        state_changed,
        recovered,
        use_sensor_timestamp ? sensor_timestamp_ns : system_timestamp_ns,
        absolute_offset_ns,
        new_state,
    };
  }

  TimestampState state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }

 private:
  mutable std::mutex mutex_;
  uint64_t max_offset_ns_ = 1000000000ULL;
  bool initialized_ = false;
  TimestampState state_ = TimestampState::kNoSync;
};

}  // namespace livox_ros

#endif  // TIMESTAMP_GUARD_H_
