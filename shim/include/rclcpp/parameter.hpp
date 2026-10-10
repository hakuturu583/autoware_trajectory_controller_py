// ROS-free replacement of rclcpp::Parameter / rclcpp::ParameterValue.
#ifndef SHIM__RCLCPP__PARAMETER_HPP_
#define SHIM__RCLCPP__PARAMETER_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace rclcpp
{
enum ParameterType {
  PARAMETER_NOT_SET = 0,
  PARAMETER_BOOL,
  PARAMETER_INTEGER,
  PARAMETER_DOUBLE,
  PARAMETER_STRING,
  PARAMETER_BOOL_ARRAY,
  PARAMETER_INTEGER_ARRAY,
  PARAMETER_DOUBLE_ARRAY,
  PARAMETER_STRING_ARRAY,
};

namespace exceptions
{
class InvalidParameterTypeException : public std::runtime_error
{
public:
  InvalidParameterTypeException(const std::string & name, const std::string & message)
  : std::runtime_error("parameter '" + name + "' has invalid type: " + message)
  {
  }
};
class ParameterUninitializedException : public std::runtime_error
{
public:
  explicit ParameterUninitializedException(const std::string & name)
  : std::runtime_error("parameter '" + name + "' is not initialized (no value was given)")
  {
  }
};
class ParameterNotDeclaredException : public std::runtime_error
{
public:
  explicit ParameterNotDeclaredException(const std::string & name)
  : std::runtime_error("parameter '" + name + "' has not been declared")
  {
  }
};
class ParameterAlreadyDeclaredException : public std::runtime_error
{
public:
  explicit ParameterAlreadyDeclaredException(const std::string & name)
  : std::runtime_error("parameter '" + name + "' has already been declared")
  {
  }
};
}  // namespace exceptions

class ParameterTypeException : public std::runtime_error
{
public:
  ParameterTypeException(ParameterType expected, ParameterType actual)
  : std::runtime_error(
      "expected parameter type " + std::to_string(expected) + " but got " +
      std::to_string(actual))
  {
  }
};

class ParameterValue
{
public:
  using Storage = std::variant<
    std::monostate, bool, int64_t, double, std::string, std::vector<bool>, std::vector<int64_t>,
    std::vector<double>, std::vector<std::string>>;

  ParameterValue() = default;
  ParameterValue(bool v) : value_(v) {}                                  // NOLINT
  ParameterValue(int v) : value_(static_cast<int64_t>(v)) {}             // NOLINT
  ParameterValue(int64_t v) : value_(v) {}                               // NOLINT
  ParameterValue(float v) : value_(static_cast<double>(v)) {}            // NOLINT
  ParameterValue(double v) : value_(v) {}                                // NOLINT
  ParameterValue(const char * v) : value_(std::string(v)) {}             // NOLINT
  ParameterValue(std::string v) : value_(std::move(v)) {}                // NOLINT
  ParameterValue(std::vector<bool> v) : value_(std::move(v)) {}          // NOLINT
  ParameterValue(const std::vector<int> & v)                             // NOLINT
  : value_(std::vector<int64_t>(v.begin(), v.end()))
  {
  }
  ParameterValue(std::vector<int64_t> v) : value_(std::move(v)) {}       // NOLINT
  ParameterValue(const std::vector<float> & v)                           // NOLINT
  : value_(std::vector<double>(v.begin(), v.end()))
  {
  }
  ParameterValue(std::vector<double> v) : value_(std::move(v)) {}        // NOLINT
  ParameterValue(std::vector<std::string> v) : value_(std::move(v)) {}   // NOLINT

  ParameterType get_type() const { return static_cast<ParameterType>(value_.index()); }

  /// The value converted to `type` where rclcpp would accept it (integer -> double,
  /// integer array -> double array, as YAML often writes `1` for `1.0`); otherwise unchanged.
  ParameterValue promoted_to(ParameterType type) const
  {
    if (type == PARAMETER_DOUBLE) {
      if (auto p = std::get_if<int64_t>(&value_)) return ParameterValue(static_cast<double>(*p));
    }
    if (type == PARAMETER_DOUBLE_ARRAY) {
      if (auto p = std::get_if<std::vector<int64_t>>(&value_)) {
        return ParameterValue(std::vector<double>(p->begin(), p->end()));
      }
    }
    return *this;
  }

  /// Typed access; throws ParameterTypeException on a type mismatch (like rclcpp).
  template <typename T>
  T get() const
  {
    using U = std::decay_t<T>;
    if constexpr (std::is_same_v<U, bool>) {
      return expect<bool>(PARAMETER_BOOL);
    } else if constexpr (std::is_same_v<U, double> || std::is_same_v<U, float>) {
      return static_cast<U>(expect<double>(PARAMETER_DOUBLE));
    } else if constexpr (std::is_integral_v<U>) {
      return static_cast<U>(expect<int64_t>(PARAMETER_INTEGER));
    } else if constexpr (std::is_same_v<U, std::string>) {
      return expect<std::string>(PARAMETER_STRING);
    } else if constexpr (std::is_same_v<U, std::vector<bool>>) {
      return expect<std::vector<bool>>(PARAMETER_BOOL_ARRAY);
    } else if constexpr (std::is_same_v<U, std::vector<double>>) {
      return expect<std::vector<double>>(PARAMETER_DOUBLE_ARRAY);
    } else if constexpr (std::is_same_v<U, std::vector<float>>) {
      const auto v = get<std::vector<double>>();
      return std::vector<float>(v.begin(), v.end());
    } else if constexpr (std::is_same_v<U, std::vector<int64_t>>) {
      return expect<std::vector<int64_t>>(PARAMETER_INTEGER_ARRAY);
    } else if constexpr (std::is_same_v<U, std::vector<int>>) {
      const auto v = expect<std::vector<int64_t>>(PARAMETER_INTEGER_ARRAY);
      return std::vector<int>(v.begin(), v.end());
    } else if constexpr (std::is_same_v<U, std::vector<std::string>>) {
      return expect<std::vector<std::string>>(PARAMETER_STRING_ARRAY);
    } else {
      static_assert(!sizeof(U), "unsupported parameter type");
    }
  }

  const Storage & storage() const { return value_; }

private:
  template <typename V>
  const V & expect(ParameterType expected) const
  {
    if (auto p = std::get_if<V>(&value_)) return *p;
    throw ParameterTypeException(expected, get_type());
  }

  Storage value_{};
};

/// Build an array parameter from scalar values: all bools -> bool array, all integers ->
/// integer array, any mix of numbers -> double array, all strings -> string array.
/// An empty list becomes an empty double array.
inline ParameterValue make_array_value(const std::vector<ParameterValue> & values)
{
  bool all_bool = true, all_int = true, all_num = true, all_str = true;
  for (const auto & v : values) {
    const auto t = v.get_type();
    all_bool &= t == PARAMETER_BOOL;
    all_int &= t == PARAMETER_INTEGER;
    all_num &= t == PARAMETER_INTEGER || t == PARAMETER_DOUBLE;
    all_str &= t == PARAMETER_STRING;
  }
  if (values.empty() || (all_num && !all_int)) {
    std::vector<double> out;
    for (const auto & v : values) out.push_back(v.promoted_to(PARAMETER_DOUBLE).get<double>());
    return ParameterValue(out);
  }
  if (all_bool) {
    std::vector<bool> out;
    for (const auto & v : values) out.push_back(v.get<bool>());
    return ParameterValue(out);
  }
  if (all_int) {
    std::vector<int64_t> out;
    for (const auto & v : values) out.push_back(v.get<int64_t>());
    return ParameterValue(out);
  }
  if (all_str) {
    std::vector<std::string> out;
    for (const auto & v : values) out.push_back(v.get<std::string>());
    return ParameterValue(out);
  }
  throw std::invalid_argument("array parameters must not mix types");
}

class Parameter
{
public:
  Parameter() = default;
  Parameter(std::string name, ParameterValue value) : name_(std::move(name)), value_(std::move(value))
  {
  }
  template <typename ValueT>
  Parameter(std::string name, ValueT value)
  : name_(std::move(name)), value_(ParameterValue(value))
  {
  }

  const std::string & get_name() const { return name_; }
  ParameterType get_type() const { return value_.get_type(); }
  const ParameterValue & get_parameter_value() const { return value_; }

  template <typename T>
  T get_value() const
  {
    try {
      return value_.get<T>();
    } catch (const ParameterTypeException & e) {
      throw exceptions::InvalidParameterTypeException(name_, e.what());
    }
  }

  double as_double() const { return get_value<double>(); }
  std::string as_string() const { return get_value<std::string>(); }

private:
  std::string name_;
  ParameterValue value_;
};
}  // namespace rclcpp

namespace rcl_interfaces::msg
{
struct SetParametersResult
{
  bool successful{false};
  std::string reason{};
};
struct ParameterDescriptor
{
  std::string name{};
  uint8_t type{0};
  std::string description{};
  bool read_only{false};
  bool dynamic_typing{false};
};
}  // namespace rcl_interfaces::msg

#endif  // SHIM__RCLCPP__PARAMETER_HPP_
