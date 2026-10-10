// ROS-free replacement of rclcpp::Publisher.
//
// Published messages are not sent anywhere; the latest message of each topic is kept
// so that debug outputs (predicted trajectory, diagnostics, ...) can be read back.
#ifndef SHIM__RCLCPP__PUBLISHER_HPP_
#define SHIM__RCLCPP__PUBLISHER_HPP_

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace rclcpp
{
template <typename MessageT>
class Publisher
{
public:
  using SharedPtr = std::shared_ptr<Publisher<MessageT>>;
  using ConstSharedPtr = std::shared_ptr<const Publisher<MessageT>>;
  using Slot = std::optional<MessageT>;

  Publisher(std::string topic, std::shared_ptr<Slot> slot)
  : topic_(std::move(topic)), slot_(std::move(slot))
  {
  }

  void publish(const MessageT & msg) { *slot_ = msg; }
  void publish(std::unique_ptr<MessageT> msg)
  {
    if (msg) *slot_ = std::move(*msg);
  }
  size_t get_subscription_count() const { return 1; }
  size_t get_intra_process_subscription_count() const { return 0; }
  const char * get_topic_name() const { return topic_.c_str(); }

private:
  std::string topic_;
  std::shared_ptr<Slot> slot_;
};
}  // namespace rclcpp

#endif  // SHIM__RCLCPP__PUBLISHER_HPP_
