#ifndef SHIM__TF2_ROS__TIME_HPP_
#define SHIM__TF2_ROS__TIME_HPP_

#include "builtin_interfaces/msg/time.hpp"
#include "tf2/time.hpp"

namespace tf2_ros
{
inline tf2::TimePoint fromMsg(const builtin_interfaces::msg::Time & t)
{
  return tf2::TimePoint(
    tf2::Duration(static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec)));
}
inline builtin_interfaces::msg::Time toMsg(const tf2::TimePoint & t)
{
  const auto ns = t.time_since_epoch().count();
  builtin_interfaces::msg::Time msg;
  msg.sec = static_cast<int32_t>(ns / 1000000000LL);
  msg.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return msg;
}
}  // namespace tf2_ros

#endif  // SHIM__TF2_ROS__TIME_HPP_
