// ROS-free replacement of rclcpp::Node.
//
// Only the parts used by the wrapped Autoware controllers are provided:
// parameters (declare/get/set + on-set callbacks), a manually driven clock,
// a logger and "publishers" that keep the last published message.
#ifndef SHIM__RCLCPP__NODE_HPP_
#define SHIM__RCLCPP__NODE_HPP_

#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"

#include <any>
#include <functional>
#include <list>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace rclcpp
{
class NodeOptions
{
public:
  NodeOptions() = default;
  NodeOptions & parameter_overrides(const std::vector<Parameter> & overrides)
  {
    parameter_overrides_ = overrides;
    return *this;
  }
  const std::vector<Parameter> & parameter_overrides() const { return parameter_overrides_; }
  std::vector<Parameter> & parameter_overrides() { return parameter_overrides_; }
  template <typename ValueT>
  NodeOptions & append_parameter_override(const std::string & name, const ValueT & value)
  {
    parameter_overrides_.emplace_back(name, ParameterValue(value));
    return *this;
  }
  /// Only `--params-file <yaml>` is interpreted (other ROS arguments are ignored).
  NodeOptions & arguments(const std::vector<std::string> & arguments)
  {
    arguments_ = arguments;
    return *this;
  }
  const std::vector<std::string> & arguments() const { return arguments_; }

private:
  std::vector<Parameter> parameter_overrides_;
  std::vector<std::string> arguments_;
};

/// Parse a ROS 2 parameter file (`<node>: ros__parameters: {...}`) into parameters.
/// Supports the YAML subset used by parameter files: nested maps, scalars and flow lists.
std::vector<Parameter> parameters_from_yaml_file(const std::string & path);

class Node;

namespace node_interfaces
{
class NodeParametersInterface
{
public:
  using SharedPtr = std::shared_ptr<NodeParametersInterface>;
  explicit NodeParametersInterface(Node * node) : node_(node) {}
  bool has_parameter(const std::string & name) const;
  Parameter get_parameter(const std::string & name) const;

private:
  Node * node_;
};
}  // namespace node_interfaces

class Node
{
public:
  using SharedPtr = std::shared_ptr<Node>;
  using OnSetParametersCallbackType =
    std::function<rcl_interfaces::msg::SetParametersResult(const std::vector<Parameter> &)>;
  struct OnSetParametersCallbackHandle
  {
    using SharedPtr = std::shared_ptr<OnSetParametersCallbackHandle>;
    OnSetParametersCallbackType callback;
  };

  explicit Node(const std::string & name, const NodeOptions & options = NodeOptions());
  virtual ~Node() = default;

  Node(const Node &) = delete;
  Node & operator=(const Node &) = delete;

  const char * get_name() const { return name_.c_str(); }

  // ---- parameters --------------------------------------------------------
  const ParameterValue & declare_parameter(
    const std::string & name, const ParameterValue & default_value,
    const rcl_interfaces::msg::ParameterDescriptor & descriptor = {},
    bool ignore_override = false);

  template <typename ParameterT>
  ParameterT declare_parameter(
    const std::string & name, const ParameterT & default_value,
    const rcl_interfaces::msg::ParameterDescriptor & descriptor = {},
    bool ignore_override = false)
  {
    (void)descriptor;
    return declare_typed(
             name, ParameterValue(default_value), ParameterValue(ParameterT{}).get_type(),
             ignore_override)
      .template get<ParameterT>();
  }

  /// Declare a parameter without default value: an override must have been given.
  template <typename ParameterT>
  ParameterT declare_parameter(
    const std::string & name, const rcl_interfaces::msg::ParameterDescriptor & descriptor = {},
    bool ignore_override = false)
  {
    (void)descriptor;
    return declare_typed(
             name, ParameterValue{}, ParameterValue(ParameterT{}).get_type(), ignore_override)
      .template get<ParameterT>();
  }

  bool has_parameter(const std::string & name) const;
  Parameter get_parameter(const std::string & name) const;
  /// All declared parameters, sorted by name.
  std::vector<Parameter> list_parameters() const;

  rcl_interfaces::msg::SetParametersResult set_parameters_atomically(
    const std::vector<Parameter> & parameters);

  OnSetParametersCallbackHandle::SharedPtr add_on_set_parameters_callback(
    OnSetParametersCallbackType callback);

  node_interfaces::NodeParametersInterface::SharedPtr get_node_parameters_interface()
  {
    return node_parameters_;
  }

  // ---- time / logging ----------------------------------------------------
  Clock::SharedPtr get_clock() { return clock_; }
  Clock::ConstSharedPtr get_clock() const { return clock_; }
  Time now() const { return clock_->now(); }
  Logger get_logger() const { return logger_; }

  // ---- publishers --------------------------------------------------------
  template <typename MessageT>
  typename Publisher<MessageT>::SharedPtr create_publisher(
    const std::string & topic, const QoS & qos = QoS())
  {
    (void)qos;
    auto slot = std::make_shared<typename Publisher<MessageT>::Slot>();
    topics_[topic] = slot;
    return std::make_shared<Publisher<MessageT>>(slot);
  }

  /// Latest message published on `topic` (std::nullopt if none / type mismatch).
  template <typename MessageT>
  std::optional<MessageT> get_last_message(const std::string & topic) const
  {
    const auto it = topics_.find(topic);
    if (it == topics_.end()) return std::nullopt;
    using SlotPtr = std::shared_ptr<typename Publisher<MessageT>::Slot>;
    if (const auto * slot = std::any_cast<SlotPtr>(&it->second)) {
      return **slot;
    }
    return std::nullopt;
  }
  std::vector<std::string> get_topic_names() const;

private:
  ParameterValue & declare_impl(
    const std::string & name, const ParameterValue & default_value, bool ignore_override);
  /// Declare and store the value converted to the declared type (see promoted_to()).
  const ParameterValue & declare_typed(
    const std::string & name, const ParameterValue & default_value, ParameterType type,
    bool ignore_override);

  std::string name_;
  std::map<std::string, ParameterValue> overrides_;
  std::map<std::string, ParameterValue> parameters_;
  std::list<std::weak_ptr<OnSetParametersCallbackHandle>> param_callbacks_;
  std::map<std::string, std::any> topics_;
  Clock::SharedPtr clock_;
  Logger logger_;
  node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
};

// --- misc free functions used by the sources -------------------------------
bool ok();
void init(int argc, char const * const * argv);
bool shutdown();
void sleep_for(const std::chrono::nanoseconds & duration);

/// Stub so that templates mentioning it still parse; it cannot be used.
class SyncParametersClient
{
public:
  template <typename... Args>
  explicit SyncParametersClient(Args &&...)
  {
    throw std::runtime_error("SyncParametersClient is not available without ROS 2");
  }
  template <typename... Args>
  bool wait_for_service(Args &&...)
  {
    return false;
  }
  bool has_parameter(const std::string &) { return false; }
  template <typename T>
  T get_parameter(const std::string &)
  {
    return T{};
  }
};
}  // namespace rclcpp

#endif  // SHIM__RCLCPP__NODE_HPP_
