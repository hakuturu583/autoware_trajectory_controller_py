// ROS-free replacement of rclcpp::Time / rclcpp::Duration.
#ifndef SHIM__RCLCPP__TIME_HPP_
#define SHIM__RCLCPP__TIME_HPP_

#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>

enum rcl_clock_type_t { RCL_CLOCK_UNINITIALIZED = 0, RCL_ROS_TIME, RCL_SYSTEM_TIME, RCL_STEADY_TIME };

namespace rclcpp
{
class Duration
{
public:
  Duration() = default;
  Duration(int32_t seconds, uint32_t nanoseconds)
  : ns_(static_cast<int64_t>(seconds) * 1000000000LL + static_cast<int64_t>(nanoseconds))
  {
  }
  explicit Duration(std::chrono::nanoseconds ns) : ns_(ns.count()) {}
  template <class Rep, class Period>
  Duration(const std::chrono::duration<Rep, Period> & d)  // NOLINT
  : ns_(std::chrono::duration_cast<std::chrono::nanoseconds>(d).count())
  {
  }
  Duration(const builtin_interfaces::msg::Duration & msg)  // NOLINT
  : Duration(msg.sec, msg.nanosec)
  {
  }

  static Duration from_seconds(double seconds)
  {
    Duration d;
    d.ns_ = static_cast<int64_t>(std::llround(seconds * 1e9));
    return d;
  }
  static Duration from_nanoseconds(int64_t ns)
  {
    Duration d;
    d.ns_ = ns;
    return d;
  }
  static Duration max() { return from_nanoseconds(INT64_MAX); }

  double seconds() const { return static_cast<double>(ns_) * 1e-9; }
  int64_t nanoseconds() const { return ns_; }
  std::chrono::nanoseconds to_chrono() const { return std::chrono::nanoseconds(ns_); }
  template <class DurationT>
  DurationT to_chrono() const
  {
    return std::chrono::duration_cast<DurationT>(std::chrono::nanoseconds(ns_));
  }

  operator builtin_interfaces::msg::Duration() const  // NOLINT
  {
    builtin_interfaces::msg::Duration msg;
    int64_t sec = ns_ / 1000000000LL;
    int64_t nsec = ns_ % 1000000000LL;
    if (nsec < 0) {
      nsec += 1000000000LL;
      sec -= 1;
    }
    msg.sec = static_cast<int32_t>(sec);
    msg.nanosec = static_cast<uint32_t>(nsec);
    return msg;
  }

  Duration operator+(const Duration & rhs) const { return from_nanoseconds(ns_ + rhs.ns_); }
  Duration operator-(const Duration & rhs) const { return from_nanoseconds(ns_ - rhs.ns_); }
  Duration operator-() const { return from_nanoseconds(-ns_); }
  Duration operator*(double s) const
  {
    return from_nanoseconds(static_cast<int64_t>(static_cast<double>(ns_) * s));
  }
  Duration & operator+=(const Duration & rhs)
  {
    ns_ += rhs.ns_;
    return *this;
  }
  Duration & operator-=(const Duration & rhs)
  {
    ns_ -= rhs.ns_;
    return *this;
  }
  bool operator==(const Duration & rhs) const { return ns_ == rhs.ns_; }
  bool operator!=(const Duration & rhs) const { return ns_ != rhs.ns_; }
  bool operator<(const Duration & rhs) const { return ns_ < rhs.ns_; }
  bool operator<=(const Duration & rhs) const { return ns_ <= rhs.ns_; }
  bool operator>(const Duration & rhs) const { return ns_ > rhs.ns_; }
  bool operator>=(const Duration & rhs) const { return ns_ >= rhs.ns_; }

private:
  int64_t ns_{0};
};

class Time
{
public:
  Time() = default;
  Time(int32_t seconds, uint32_t nanoseconds, rcl_clock_type_t clock_type = RCL_SYSTEM_TIME)
  : ns_(static_cast<int64_t>(seconds) * 1000000000LL + static_cast<int64_t>(nanoseconds)),
    clock_type_(clock_type)
  {
    if (seconds < 0) {
      throw std::runtime_error("cannot store a negative time point in rclcpp::Time");
    }
  }
  explicit Time(int64_t nanoseconds, rcl_clock_type_t clock_type = RCL_SYSTEM_TIME)
  : ns_(nanoseconds), clock_type_(clock_type)
  {
  }
  Time(  // NOLINT
    const builtin_interfaces::msg::Time & msg, rcl_clock_type_t clock_type = RCL_ROS_TIME)
  : ns_(static_cast<int64_t>(msg.sec) * 1000000000LL + static_cast<int64_t>(msg.nanosec)),
    clock_type_(clock_type)
  {
  }

  static Time max(rcl_clock_type_t clock_type = RCL_SYSTEM_TIME)
  {
    return Time(INT64_MAX, clock_type);
  }

  double seconds() const { return static_cast<double>(ns_) * 1e-9; }
  int64_t nanoseconds() const { return ns_; }
  rcl_clock_type_t get_clock_type() const { return clock_type_; }

  operator builtin_interfaces::msg::Time() const  // NOLINT
  {
    builtin_interfaces::msg::Time msg;
    msg.sec = static_cast<int32_t>(ns_ / 1000000000LL);
    msg.nanosec = static_cast<uint32_t>(ns_ % 1000000000LL);
    return msg;
  }

  // NOTE: unlike rclcpp, clock types are not checked so that messages stamped by
  // different sources can be compared freely in this ROS-less environment.
  Duration operator-(const Time & rhs) const { return Duration::from_nanoseconds(ns_ - rhs.ns_); }
  Time operator+(const Duration & rhs) const { return Time(ns_ + rhs.nanoseconds(), clock_type_); }
  Time operator-(const Duration & rhs) const { return Time(ns_ - rhs.nanoseconds(), clock_type_); }
  Time & operator+=(const Duration & rhs)
  {
    ns_ += rhs.nanoseconds();
    return *this;
  }
  Time & operator-=(const Duration & rhs)
  {
    ns_ -= rhs.nanoseconds();
    return *this;
  }
  bool operator==(const Time & rhs) const { return ns_ == rhs.ns_; }
  bool operator!=(const Time & rhs) const { return ns_ != rhs.ns_; }
  bool operator<(const Time & rhs) const { return ns_ < rhs.ns_; }
  bool operator<=(const Time & rhs) const { return ns_ <= rhs.ns_; }
  bool operator>(const Time & rhs) const { return ns_ > rhs.ns_; }
  bool operator>=(const Time & rhs) const { return ns_ >= rhs.ns_; }

private:
  int64_t ns_{0};
  rcl_clock_type_t clock_type_{RCL_SYSTEM_TIME};
};

inline Time operator+(const Duration & lhs, const Time & rhs) { return rhs + lhs; }
}  // namespace rclcpp

#endif  // SHIM__RCLCPP__TIME_HPP_
