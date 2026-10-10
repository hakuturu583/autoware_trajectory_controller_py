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
  void summary(const diagnostic_msgs::msg::DiagnosticStatus & src)
  {
    summary(src.level, src.message);
  }
  void mergeSummary(unsigned char lvl, const std::string & msg)
  {
    if ((lvl > 0) == (level > 0)) {
      if (!message.empty()) message += "; ";
      message += msg;
    } else if (lvl > level) {
      message = msg;
    }
    if (lvl > level) level = lvl;
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
  void add(const std::string & key, bool val) { add<std::string>(key, val ? "True" : "False"); }
  void clear()
  {
    values.clear();
    level = 0;
    message.clear();
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
  bool removeByName(const std::string & name)
  {
    for (auto it = tasks_.begin(); it != tasks_.end(); ++it) {
      if (it->first == name) {
        tasks_.erase(it);
        return true;
      }
    }
    return false;
  }
  void setHardwareID(const std::string & hwid) { hwid_ = hwid; }
  void setPeriod(double) {}

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
