// ROS-free replacement of rclcpp::Publisher.
//
// Published messages are not sent anywhere; the latest message of each topic is kept
// so that debug outputs (predicted trajectory, diagnostics, ...) can be read back.
#ifndef SHIM__RCLCPP__PUBLISHER_HPP_
#define SHIM__RCLCPP__PUBLISHER_HPP_

#include <memory>
#include <optional>
#include <utility>

namespace rclcpp
{
template <typename MessageT>
class Publisher
{
public:
  using SharedPtr = std::shared_ptr<Publisher<MessageT>>;
  using Slot = std::optional<MessageT>;

  explicit Publisher(std::shared_ptr<Slot> slot) : slot_(std::move(slot)) {}

  void publish(const MessageT & msg) { *slot_ = msg; }
  void publish(std::unique_ptr<MessageT> msg)
  {
    if (msg) *slot_ = std::move(*msg);
  }

private:
  std::shared_ptr<Slot> slot_;
};
}  // namespace rclcpp

#endif  // SHIM__RCLCPP__PUBLISHER_HPP_
