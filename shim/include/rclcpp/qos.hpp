#ifndef SHIM__RCLCPP__QOS_HPP_
#define SHIM__RCLCPP__QOS_HPP_

#include <cstddef>

namespace rclcpp
{
/// QoS settings are irrelevant without middleware; the class only keeps call sites compiling.
struct QoS
{
  QoS(size_t = 10) {}  // NOLINT
  QoS & transient_local() { return *this; }
};
}  // namespace rclcpp

#endif  // SHIM__RCLCPP__QOS_HPP_
