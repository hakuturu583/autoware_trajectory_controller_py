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
  NodeOptions & automatically_declare_parameters_from_overrides(bool) { return *this; }
  NodeOptions & allow_undeclared_parameters(bool) { return *this; }

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
  const char * get_namespace() const { return "/"; }
  std::string get_fully_qualified_name() const { return "/" + name_; }

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
    return declare_parameter(name, ParameterValue(default_value), descriptor, ignore_override)
      .template get<ParameterT>();
  }

  /// Declare a parameter without default value: an override must have been given.
  template <typename ParameterT>
  ParameterT declare_parameter(
    const std::string & name, const rcl_interfaces::msg::ParameterDescriptor & descriptor = {},
    bool ignore_override = false)
  {
    (void)descriptor;
    (void)ignore_override;
    return declare_without_default(name).template get<ParameterT>();
  }

  bool has_parameter(const std::string & name) const;
  Parameter get_parameter(const std::string & name) const;
  template <typename ParameterT>
  bool get_parameter(const std::string & name, ParameterT & value) const
  {
    if (!has_parameter(name)) return false;
    value = get_parameter(name).template get_value<ParameterT>();
    return true;
  }
  template <typename ParameterT>
  bool get_parameter_or(
    const std::string & name, ParameterT & value, const ParameterT & alternative) const
  {
    if (get_parameter(name, value)) return true;
    value = alternative;
    return false;
  }
  std::vector<Parameter> get_parameters(const std::vector<std::string> & names) const;
  /// All declared parameters, sorted by name.
  std::vector<Parameter> list_parameters() const;

  std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters(
    const std::vector<Parameter> & parameters);
  rcl_interfaces::msg::SetParametersResult set_parameter(const Parameter & parameter);
  rcl_interfaces::msg::SetParametersResult set_parameters_atomically(
    const std::vector<Parameter> & parameters);

  OnSetParametersCallbackHandle::SharedPtr add_on_set_parameters_callback(
    OnSetParametersCallbackType callback);
  void remove_on_set_parameters_callback(const OnSetParametersCallbackHandle * handle);

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
    return std::make_shared<Publisher<MessageT>>(topic, slot);
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
  const ParameterValue & declare_without_default(const std::string & name);

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
