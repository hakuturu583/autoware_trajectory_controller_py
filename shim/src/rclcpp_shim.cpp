// Implementation of the ROS-free rclcpp shim.
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <cstdarg>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

namespace rclcpp
{
namespace logging
{
namespace
{
std::atomic<int> g_level{static_cast<int>(LogLevel::Info)};
std::mutex g_sink_mutex;
LogSink g_sink;

const char * level_name(LogLevel level)
{
  switch (level) {
    case LogLevel::Debug:
      return "DEBUG";
    case LogLevel::Info:
      return "INFO";
    case LogLevel::Warn:
      return "WARN";
    case LogLevel::Error:
      return "ERROR";
    default:
      return "FATAL";
  }
}
}  // namespace

void set_global_level(LogLevel level) { g_level = static_cast<int>(level); }
void set_sink(LogSink sink)
{
  std::lock_guard<std::mutex> lock(g_sink_mutex);
  g_sink = std::move(sink);
}
bool is_enabled(LogLevel level) { return static_cast<int>(level) >= g_level.load(); }

void log_str(LogLevel level, const std::string & name, const std::string & msg)
{
  std::lock_guard<std::mutex> lock(g_sink_mutex);
  if (g_sink) {
    g_sink(level, name, msg);
    return;
  }
  std::cerr << "[" << level_name(level) << "] [" << name << "]: " << msg << std::endl;
}

void log(LogLevel level, const std::string & name, const char * fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  va_list args_copy;
  va_copy(args_copy, args);
  const int size = std::vsnprintf(nullptr, 0, fmt, args_copy);
  va_end(args_copy);
  std::string msg;
  if (size > 0) {
    std::vector<char> buf(static_cast<size_t>(size) + 1);
    std::vsnprintf(buf.data(), buf.size(), fmt, args);
    msg.assign(buf.data(), static_cast<size_t>(size));
  }
  va_end(args);
  log_str(level, name, msg);
}
}  // namespace logging

namespace node_interfaces
{
bool NodeParametersInterface::has_parameter(const std::string & name) const
{
  return node_->has_parameter(name);
}
Parameter NodeParametersInterface::get_parameter(const std::string & name) const
{
  return node_->get_parameter(name);
}
}  // namespace node_interfaces

namespace
{
std::string trim(const std::string & s)
{
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return "";
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

std::string strip_comment(const std::string & line)
{
  bool in_single = false, in_double = false;
  for (size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '\'' && !in_double) in_single = !in_single;
    if (c == '"' && !in_single) in_double = !in_double;
    if (c == '#' && !in_single && !in_double && (i == 0 || std::isspace(line[i - 1]))) {
      return line.substr(0, i);
    }
  }
  return line;
}

bool is_quoted(const std::string & s)
{
  return s.size() >= 2 && ((s.front() == '"' && s.back() == '"') ||
                           (s.front() == '\'' && s.back() == '\''));
}

// Scalar classification following YAML 1.2 core schema (as used by rcl_yaml_param_parser).
ParameterValue scalar_value(const std::string & raw)
{
  const std::string s = trim(raw);
  if (is_quoted(s)) return ParameterValue(s.substr(1, s.size() - 2));
  std::string lower;
  for (char c : s) lower.push_back(static_cast<char>(std::tolower(c)));
  if (lower == "true" || lower == "yes" || lower == "on") return ParameterValue(true);
  if (lower == "false" || lower == "no" || lower == "off") return ParameterValue(false);
  try {
    size_t pos = 0;
    const long long v = std::stoll(s, &pos, 0);
    if (pos == s.size()) return ParameterValue(static_cast<int64_t>(v));
  } catch (...) {
  }
  try {
    size_t pos = 0;
    const double v = std::stod(s, &pos);
    if (pos == s.size()) return ParameterValue(v);
  } catch (...) {
  }
  if (lower == ".inf" || lower == "+.inf") return ParameterValue(INFINITY);
  if (lower == "-.inf") return ParameterValue(-INFINITY);
  if (lower == ".nan") return ParameterValue(NAN);
  return ParameterValue(s);
}

std::vector<std::string> split_flow_list(const std::string & inner)
{
  std::vector<std::string> items;
  std::string cur;
  bool in_single = false, in_double = false;
  for (char c : inner) {
    if (c == '\'' && !in_double) in_single = !in_single;
    if (c == '"' && !in_single) in_double = !in_double;
    if (c == ',' && !in_single && !in_double) {
      items.push_back(trim(cur));
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  if (!trim(cur).empty()) items.push_back(trim(cur));
  return items;
}

ParameterValue parse_value(const std::string & raw, const std::string & name)
{
  const std::string s = trim(raw);
  if (s.size() >= 2 && s.front() == '[' && s.back() == ']') {
    const auto items = split_flow_list(s.substr(1, s.size() - 2));
    std::vector<ParameterValue> values;
    for (const auto & item : items) values.push_back(scalar_value(item));
    try {
      return make_array_value(values);
    } catch (const std::invalid_argument &) {
      throw std::runtime_error("mixed types in list parameter '" + name + "'");
    }
  }
  return scalar_value(s);
}
}  // namespace

std::vector<Parameter> parameters_from_yaml_file(const std::string & path)
{
  std::ifstream ifs(path);
  if (!ifs) throw std::runtime_error("failed to open parameter file: " + path);

  // Stack of (indent, key) describing the current map nesting.
  std::vector<std::pair<int, std::string>> stack;
  std::vector<Parameter> out;
  std::string line;
  while (std::getline(ifs, line)) {
    line = strip_comment(line);
    if (trim(line).empty() || trim(line) == "---") continue;
    const int indent = static_cast<int>(line.find_first_not_of(' '));
    const std::string content = trim(line);
    const auto colon = content.find(':');
    if (colon == std::string::npos) {
      throw std::runtime_error("unsupported YAML line in " + path + ": " + content);
    }
    std::string key = trim(content.substr(0, colon));
    if (is_quoted(key)) key = key.substr(1, key.size() - 2);
    const std::string value = trim(content.substr(colon + 1));
    while (!stack.empty() && stack.back().first >= indent) stack.pop_back();
    if (value.empty()) {
      stack.emplace_back(indent, key);
      continue;
    }
    // Build the parameter name from the keys below "ros__parameters".
    std::string name;
    bool under_params = false;
    for (const auto & [_, k] : stack) {
      if (under_params) name += k + ".";
      if (k == "ros__parameters") under_params = true;
    }
    if (!under_params) continue;
    name += key;
    out.emplace_back(name, parse_value(value, name));
  }
  return out;
}

Node::Node(const std::string & name, const NodeOptions & options)
: name_(name),
  clock_(std::make_shared<Clock>(RCL_ROS_TIME)),
  logger_(name),
  node_parameters_(std::make_shared<node_interfaces::NodeParametersInterface>(this))
{
  // Like rclcpp: parameter files first, then the explicit overrides win.
  const auto & args = options.arguments();
  for (size_t i = 0; i + 1 < args.size(); ++i) {
    if (args[i] == "--params-file") {
      for (const auto & p : parameters_from_yaml_file(args[i + 1])) {
        overrides_[p.get_name()] = p.get_parameter_value();
      }
    }
  }
  for (const auto & p : options.parameter_overrides()) {
    overrides_[p.get_name()] = p.get_parameter_value();
  }
}

ParameterValue & Node::declare_impl(
  const std::string & name, const ParameterValue & default_value, bool ignore_override)
{
  if (parameters_.count(name)) {
    throw exceptions::ParameterAlreadyDeclaredException(name);
  }
  const auto it = overrides_.find(name);
  const ParameterValue & value =
    (!ignore_override && it != overrides_.end()) ? it->second : default_value;
  if (value.get_type() == PARAMETER_NOT_SET) {
    throw exceptions::ParameterUninitializedException(name);
  }
  return parameters_[name] = value;
}

const ParameterValue & Node::declare_parameter(
  const std::string & name, const ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor &, bool ignore_override)
{
  return declare_impl(name, default_value, ignore_override);
}

const ParameterValue & Node::declare_typed(
  const std::string & name, const ParameterValue & default_value, ParameterType type,
  bool ignore_override)
{
  auto & value = declare_impl(name, default_value, ignore_override);
  value = value.promoted_to(type);
  return value;
}

bool Node::has_parameter(const std::string & name) const { return parameters_.count(name) > 0; }

Parameter Node::get_parameter(const std::string & name) const
{
  const auto it = parameters_.find(name);
  if (it == parameters_.end()) {
    throw exceptions::ParameterNotDeclaredException(name);
  }
  return Parameter(name, it->second);
}

std::vector<Parameter> Node::list_parameters() const
{
  std::vector<Parameter> out;
  for (const auto & [n, v] : parameters_) out.emplace_back(n, v);
  return out;
}

rcl_interfaces::msg::SetParametersResult Node::set_parameters_atomically(
  const std::vector<Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  // Values are converted to the declared type before the callbacks see them.
  std::vector<Parameter> converted;
  for (const auto & p : parameters) {
    const auto it = parameters_.find(p.get_name());
    if (it == parameters_.end()) {
      result.successful = false;
      result.reason = "parameter '" + p.get_name() + "' has not been declared";
      return result;
    }
    const auto value = p.get_parameter_value().promoted_to(it->second.get_type());
    if (value.get_type() != it->second.get_type()) {
      result.successful = false;
      result.reason = "parameter '" + p.get_name() + "' has a different type than declared";
      return result;
    }
    converted.emplace_back(p.get_name(), value);
  }
  for (auto it = param_callbacks_.begin(); it != param_callbacks_.end();) {
    if (auto handle = it->lock()) {
      result = handle->callback(converted);
      if (!result.successful) return result;
      ++it;
    } else {
      it = param_callbacks_.erase(it);
    }
  }
  for (const auto & p : converted) {
    parameters_[p.get_name()] = p.get_parameter_value();
  }
  return result;
}

Node::OnSetParametersCallbackHandle::SharedPtr Node::add_on_set_parameters_callback(
  OnSetParametersCallbackType callback)
{
  auto handle = std::make_shared<OnSetParametersCallbackHandle>();
  handle->callback = std::move(callback);
  param_callbacks_.emplace_front(handle);
  return handle;
}

std::vector<std::string> Node::get_topic_names() const
{
  std::vector<std::string> out;
  for (const auto & [n, _] : topics_) out.push_back(n);
  return out;
}

bool ok() { return true; }
void init(int, char const * const *) {}
bool shutdown() { return true; }
void sleep_for(const std::chrono::nanoseconds & duration) { std::this_thread::sleep_for(duration); }
}  // namespace rclcpp
