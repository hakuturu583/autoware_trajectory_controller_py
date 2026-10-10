#ifndef SHIM__RCLCPP__QOS_HPP_
#define SHIM__RCLCPP__QOS_HPP_

#include <cstddef>

namespace rclcpp
{
struct KeepLast
{
  explicit KeepLast(size_t d) : depth(d) {}
  size_t depth;
};
struct KeepAll
{
};

/// QoS settings are irrelevant without middleware; the class only keeps call sites compiling.
class QoS
{
public:
  QoS(size_t depth = 10) : depth_(depth) {}  // NOLINT
  QoS(const KeepLast & k) : depth_(k.depth) {}  // NOLINT
  QoS(const KeepAll &) : depth_(0) {}  // NOLINT
  QoS & transient_local() { return *this; }
  QoS & durability_volatile() { return *this; }
  QoS & reliable() { return *this; }
  QoS & best_effort() { return *this; }
  QoS & keep_last(size_t depth)
  {
    depth_ = depth;
    return *this;
  }
  size_t depth() const { return depth_; }

private:
  size_t depth_;
};
struct SensorDataQoS : public QoS
{
  SensorDataQoS() : QoS(5) {}
};
}  // namespace rclcpp

#endif  // SHIM__RCLCPP__QOS_HPP_
