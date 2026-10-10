// ROS-free replacement of rclcpp::Logger.
#ifndef SHIM__RCLCPP__LOGGER_HPP_
#define SHIM__RCLCPP__LOGGER_HPP_

#include <functional>
#include <string>

namespace rclcpp
{
namespace logging
{
enum class LogLevel : int { Debug = 10, Info = 20, Warn = 30, Error = 40, Fatal = 50 };

using LogSink = std::function<void(LogLevel, const std::string & name, const std::string & msg)>;

/// Messages below this level are dropped. Default: Info.
void set_global_level(LogLevel level);
/// Replace the output sink (default: stderr). Passing an empty function restores the default.
void set_sink(LogSink sink);
bool is_enabled(LogLevel level);
void log(LogLevel level, const std::string & name, const char * fmt, ...)
#if defined(__GNUC__)
  __attribute__((format(printf, 3, 4)))
#endif
  ;
void log_str(LogLevel level, const std::string & name, const std::string & msg);
}  // namespace logging

class Logger
{
public:
  Logger() = default;
  explicit Logger(std::string name) : name_(std::move(name)) {}
  const char * get_name() const { return name_.c_str(); }
  Logger get_child(const std::string & suffix) const
  {
    return Logger(name_.empty() ? suffix : name_ + "." + suffix);
  }

private:
  std::string name_;
};

inline Logger get_logger(const std::string & name) { return Logger(name); }
}  // namespace rclcpp

#endif  // SHIM__RCLCPP__LOGGER_HPP_
