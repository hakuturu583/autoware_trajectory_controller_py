// Minimal rcl time API used by upstream tests to drive the simulated clock.
#ifndef SHIM__RCL__TIME_H_
#define SHIM__RCL__TIME_H_

#include "rclcpp/clock.hpp"

#include <cstdint>

using rcl_ret_t = int;
constexpr rcl_ret_t RCL_RET_OK = 0;
using rcl_clock_t = rclcpp::Clock;
using rcl_time_point_value_t = int64_t;

inline rcl_ret_t rcl_enable_ros_time_override(rcl_clock_t *) { return RCL_RET_OK; }
inline rcl_ret_t rcl_disable_ros_time_override(rcl_clock_t *) { return RCL_RET_OK; }
inline rcl_ret_t rcl_set_ros_time_override(rcl_clock_t * clock, rcl_time_point_value_t ns)
{
  clock->set_now(rclcpp::Time(ns, RCL_ROS_TIME));
  return RCL_RET_OK;
}

#endif  // SHIM__RCL__TIME_H_
