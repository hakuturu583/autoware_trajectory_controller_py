// ROS-free stand-in for diagnostic_updater. Statuses are collected on force_update()
// and can be read back with get_statuses().
#ifndef SHIM__DIAGNOSTIC_UPDATER__DIAGNOSTIC_UPDATER_HPP_
#define SHIM__DIAGNOSTIC_UPDATER__DIAGNOSTIC_UPDATER_HPP_

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"

#include <functional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace diagnostic_updater
{
class DiagnosticStatusWrapper : public diagnostic_msgs::msg::DiagnosticStatus
{
public:
  void summary(unsigned char lvl, const std::string & msg)
  {
    level = lvl;
    message = msg;
  }
  template <class T>
  void add(const std::string & key, const T & val)
  {
    std::ostringstream ss;
    ss << val;
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = ss.str();
    values.push_back(kv);
  }
};

class Updater
{
public:
  using TaskFunction = std::function<void(DiagnosticStatusWrapper &)>;

  Updater() = default;
  template <class NodeT>
  explicit Updater(NodeT, double period = 1.0)
  {
    (void)period;
  }

  void add(const std::string & name, TaskFunction f) { tasks_.emplace_back(name, std::move(f)); }
  template <class T>
  void add(const std::string & name, T * c, void (T::*f)(DiagnosticStatusWrapper &))
  {
    add(name, [c, f](DiagnosticStatusWrapper & s) { (c->*f)(s); });
  }
  void setHardwareID(const std::string & hwid) { hwid_ = hwid; }

  void force_update()
  {
    statuses_.clear();
    for (auto & [name, f] : tasks_) {
      DiagnosticStatusWrapper s;
      s.name = name;
      s.hardware_id = hwid_;
      f(s);
      statuses_.push_back(s);
    }
  }

  const std::vector<diagnostic_msgs::msg::DiagnosticStatus> & get_statuses() const
  {
    return statuses_;
  }

private:
  std::vector<std::pair<std::string, TaskFunction>> tasks_;
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> statuses_;
  std::string hwid_;
};
}  // namespace diagnostic_updater

#endif  // SHIM__DIAGNOSTIC_UPDATER__DIAGNOSTIC_UPDATER_HPP_
