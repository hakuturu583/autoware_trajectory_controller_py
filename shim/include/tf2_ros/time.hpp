#ifndef SHIM__TF2_ROS__TIME_HPP_
#define SHIM__TF2_ROS__TIME_HPP_

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/time.hpp"
#include "tf2/time.hpp"

namespace tf2_ros
{
inline tf2::TimePoint fromMsg(const builtin_interfaces::msg::Time & t)
{
  return tf2::TimePoint(tf2::Duration(rclcpp::Time(t).nanoseconds()));
}
}  // namespace tf2_ros

#endif  // SHIM__TF2_ROS__TIME_HPP_
