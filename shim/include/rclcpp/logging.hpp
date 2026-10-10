// ROS-free replacement of the RCLCPP_* logging macros.
#ifndef SHIM__RCLCPP__LOGGING_HPP_
#define SHIM__RCLCPP__LOGGING_HPP_

#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"

#include <cstdint>
#include <limits>
#include <sstream>

#define SHIM_RCLCPP_LOG(level, logger, ...)                                     \
  do {                                                                          \
    if (::rclcpp::logging::is_enabled(level)) {                                 \
      ::rclcpp::logging::log(level, (logger).get_name(), __VA_ARGS__);          \
    }                                                                           \
  } while (0)

#define SHIM_RCLCPP_LOG_ONCE(level, logger, ...)                                \
  do {                                                                          \
    static bool shim_logged_once_ = false;                                      \
    if (!shim_logged_once_ && ::rclcpp::logging::is_enabled(level)) {           \
      shim_logged_once_ = true;                                                 \
      ::rclcpp::logging::log(level, (logger).get_name(), __VA_ARGS__);          \
    }                                                                           \
  } while (0)

#define SHIM_RCLCPP_LOG_THROTTLE(level, logger, clock, duration_ms, ...)        \
  do {                                                                          \
    if (::rclcpp::logging::is_enabled(level)) {                                 \
      static int64_t shim_last_ns_ = std::numeric_limits<int64_t>::min();       \
      const int64_t shim_now_ns_ = (clock).now().nanoseconds();                 \
      const int64_t shim_period_ns_ = static_cast<int64_t>(duration_ms) * 1000000LL; \
      if (                                                                      \
        shim_last_ns_ == std::numeric_limits<int64_t>::min() ||                 \
        shim_now_ns_ - shim_last_ns_ >= shim_period_ns_ ||                      \
        shim_now_ns_ < shim_last_ns_) {                                         \
        shim_last_ns_ = shim_now_ns_;                                           \
        ::rclcpp::logging::log(level, (logger).get_name(), __VA_ARGS__);        \
      }                                                                         \
    }                                                                           \
  } while (0)

#define SHIM_RCLCPP_LOG_STREAM(level, logger, stream_arg)                       \
  do {                                                                          \
    if (::rclcpp::logging::is_enabled(level)) {                                 \
      std::stringstream shim_ss_;                                               \
      shim_ss_ << stream_arg;                                                   \
      ::rclcpp::logging::log_str(level, (logger).get_name(), shim_ss_.str());   \
    }                                                                           \
  } while (0)

#define SHIM_RCLCPP_LOG_EXPRESSION(level, logger, expr, ...)                    \
  do {                                                                          \
    if (expr) {                                                                 \
      SHIM_RCLCPP_LOG(level, logger, __VA_ARGS__);                              \
    }                                                                           \
  } while (0)

#define SHIM_LVL_DEBUG ::rclcpp::logging::LogLevel::Debug
#define SHIM_LVL_INFO ::rclcpp::logging::LogLevel::Info
#define SHIM_LVL_WARN ::rclcpp::logging::LogLevel::Warn
#define SHIM_LVL_ERROR ::rclcpp::logging::LogLevel::Error
#define SHIM_LVL_FATAL ::rclcpp::logging::LogLevel::Fatal

#define RCLCPP_DEBUG(logger, ...) SHIM_RCLCPP_LOG(SHIM_LVL_DEBUG, logger, __VA_ARGS__)
#define RCLCPP_INFO(logger, ...) SHIM_RCLCPP_LOG(SHIM_LVL_INFO, logger, __VA_ARGS__)
#define RCLCPP_WARN(logger, ...) SHIM_RCLCPP_LOG(SHIM_LVL_WARN, logger, __VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) SHIM_RCLCPP_LOG(SHIM_LVL_ERROR, logger, __VA_ARGS__)
#define RCLCPP_FATAL(logger, ...) SHIM_RCLCPP_LOG(SHIM_LVL_FATAL, logger, __VA_ARGS__)

#define RCLCPP_DEBUG_ONCE(logger, ...) SHIM_RCLCPP_LOG_ONCE(SHIM_LVL_DEBUG, logger, __VA_ARGS__)
#define RCLCPP_INFO_ONCE(logger, ...) SHIM_RCLCPP_LOG_ONCE(SHIM_LVL_INFO, logger, __VA_ARGS__)
#define RCLCPP_WARN_ONCE(logger, ...) SHIM_RCLCPP_LOG_ONCE(SHIM_LVL_WARN, logger, __VA_ARGS__)
#define RCLCPP_ERROR_ONCE(logger, ...) SHIM_RCLCPP_LOG_ONCE(SHIM_LVL_ERROR, logger, __VA_ARGS__)

#define RCLCPP_DEBUG_THROTTLE(logger, clock, ms, ...) \
  SHIM_RCLCPP_LOG_THROTTLE(SHIM_LVL_DEBUG, logger, clock, ms, __VA_ARGS__)
#define RCLCPP_INFO_THROTTLE(logger, clock, ms, ...) \
  SHIM_RCLCPP_LOG_THROTTLE(SHIM_LVL_INFO, logger, clock, ms, __VA_ARGS__)
#define RCLCPP_WARN_THROTTLE(logger, clock, ms, ...) \
  SHIM_RCLCPP_LOG_THROTTLE(SHIM_LVL_WARN, logger, clock, ms, __VA_ARGS__)
#define RCLCPP_ERROR_THROTTLE(logger, clock, ms, ...) \
  SHIM_RCLCPP_LOG_THROTTLE(SHIM_LVL_ERROR, logger, clock, ms, __VA_ARGS__)

#define RCLCPP_DEBUG_STREAM(logger, s) SHIM_RCLCPP_LOG_STREAM(SHIM_LVL_DEBUG, logger, s)
#define RCLCPP_INFO_STREAM(logger, s) SHIM_RCLCPP_LOG_STREAM(SHIM_LVL_INFO, logger, s)
#define RCLCPP_WARN_STREAM(logger, s) SHIM_RCLCPP_LOG_STREAM(SHIM_LVL_WARN, logger, s)
#define RCLCPP_ERROR_STREAM(logger, s) SHIM_RCLCPP_LOG_STREAM(SHIM_LVL_ERROR, logger, s)

#define RCLCPP_DEBUG_EXPRESSION(logger, e, ...) \
  SHIM_RCLCPP_LOG_EXPRESSION(SHIM_LVL_DEBUG, logger, e, __VA_ARGS__)
#define RCLCPP_WARN_EXPRESSION(logger, e, ...) \
  SHIM_RCLCPP_LOG_EXPRESSION(SHIM_LVL_WARN, logger, e, __VA_ARGS__)

#endif  // SHIM__RCLCPP__LOGGING_HPP_
