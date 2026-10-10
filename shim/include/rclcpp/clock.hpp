// ROS-free replacement of rclcpp::Clock.
//
// The clock never advances by itself: the current time is set explicitly with set_now().
// This lets Python drive the controllers with simulated time, like `use_sim_time` in ROS 2.
#ifndef SHIM__RCLCPP__CLOCK_HPP_
#define SHIM__RCLCPP__CLOCK_HPP_

#include "rclcpp/time.hpp"

#include <memory>

namespace rclcpp
{
class Clock
{
public:
  using SharedPtr = std::shared_ptr<Clock>;
  using ConstSharedPtr = std::shared_ptr<const Clock>;

  explicit Clock(rcl_clock_type_t clock_type = RCL_SYSTEM_TIME) : clock_type_(clock_type) {}

  Time now() const { return Time(now_ns_, clock_type_); }

  /// Set the current (simulated) time.
  void set_now(const Time & t) { now_ns_ = t.nanoseconds(); }

  rcl_clock_type_t get_clock_type() const { return clock_type_; }
  /// In rclcpp this is the underlying rcl clock; here the clock itself (see rcl/time.h).
  Clock * get_clock_handle() { return this; }

private:
  rcl_clock_type_t clock_type_;
  int64_t now_ns_{0};
};
}  // namespace rclcpp

#endif  // SHIM__RCLCPP__CLOCK_HPP_
