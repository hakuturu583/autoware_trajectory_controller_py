// ROS-free replacement of rclcpp::Clock.
//
// A clock is either driven by the wall clock (RCL_SYSTEM_TIME / RCL_STEADY_TIME)
// or manually (RCL_ROS_TIME, the default here), in which case the current time is
// set explicitly with set_now(). The manual mode lets Python drive the controllers
// with simulated time, exactly like `use_sim_time` in ROS 2.
#ifndef SHIM__RCLCPP__CLOCK_HPP_
#define SHIM__RCLCPP__CLOCK_HPP_

#include "rclcpp/time.hpp"

#include <chrono>
#include <memory>

namespace rclcpp
{
class Clock
{
public:
  using SharedPtr = std::shared_ptr<Clock>;
  using ConstSharedPtr = std::shared_ptr<const Clock>;

  explicit Clock(rcl_clock_type_t clock_type = RCL_SYSTEM_TIME) : clock_type_(clock_type) {}

  Time now() const
  {
    if (clock_type_ == RCL_ROS_TIME) {
      return Time(now_ns_, clock_type_);
    }
    if (clock_type_ == RCL_STEADY_TIME) {
      const auto t = std::chrono::steady_clock::now().time_since_epoch();
      return Time(std::chrono::duration_cast<std::chrono::nanoseconds>(t).count(), clock_type_);
    }
    const auto t = std::chrono::system_clock::now().time_since_epoch();
    return Time(std::chrono::duration_cast<std::chrono::nanoseconds>(t).count(), clock_type_);
  }

  /// Set the current time of a manually driven (RCL_ROS_TIME) clock.
  void set_now(const Time & t) { now_ns_ = t.nanoseconds(); }

  rcl_clock_type_t get_clock_type() const { return clock_type_; }
  /// In rclcpp this is the underlying rcl clock; here the clock itself (see rcl/time.h).
  Clock * get_clock_handle() { return this; }
  bool ros_time_is_active() const { return clock_type_ == RCL_ROS_TIME; }

private:
  rcl_clock_type_t clock_type_;
  int64_t now_ns_{0};
};
}  // namespace rclcpp

#endif  // SHIM__RCLCPP__CLOCK_HPP_
