// ROS-free hosts for the Autoware trajectory follower controllers.
//
// Each host owns a shim `rclcpp::Node` (parameters + manually driven clock) and a
// `diagnostic_updater::Updater`, and constructs the upstream controller classes with
// them exactly like `autoware_trajectory_follower_node` does.
#ifndef AUTOWARE_TRAJECTORY_CONTROLLER__TRAJECTORY_FOLLOWER_HPP_
#define AUTOWARE_TRAJECTORY_CONTROLLER__TRAJECTORY_FOLLOWER_HPP_

#include "autoware/trajectory_follower_base/lateral_controller_base.hpp"
#include "autoware/trajectory_follower_base/longitudinal_controller_base.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_control_msgs/msg/control_horizon.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware_trajectory_controller
{
namespace tf = ::autoware::motion::control::trajectory_follower;

/// Owns the shim node shared by one or more controllers.
class NodeHost
{
public:
  NodeHost(const std::string & name, const std::vector<rclcpp::Parameter> & parameters);
  virtual ~NodeHost() = default;

  rclcpp::Node & node() { return *node_; }
  const rclcpp::Node & node() const { return *node_; }
  diagnostic_updater::Updater & diag_updater() { return *diag_updater_; }

  /// Set the current time of the (simulated) clock in nanoseconds.
  void set_time_ns(int64_t ns) { node_->get_clock()->set_now(rclcpp::Time(ns, RCL_ROS_TIME)); }
  int64_t time_ns() const { return node_->now().nanoseconds(); }

  /// Run the diagnostic tasks and return their statuses.
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> update_diagnostics();

protected:
  std::unique_ptr<rclcpp::Node> node_;
  std::shared_ptr<diagnostic_updater::Updater> diag_updater_;
};

/// A lateral controller ("mpc" or "pure_pursuit").
class LateralController : public NodeHost
{
public:
  LateralController(const std::string & mode, const std::vector<rclcpp::Parameter> & parameters);
  bool is_ready(const tf::InputData & input) { return controller_->isReady(input); }
  tf::LateralOutput run(const tf::InputData & input) { return controller_->run(input); }
  void sync(const tf::LongitudinalSyncData & data) { controller_->sync(data); }
  void set_steering_offset(double offset) { controller_->set_steering_offset(offset); }
  const std::string & mode() const { return mode_; }

private:
  std::string mode_;
  std::shared_ptr<tf::LateralControllerBase> controller_;
};

/// A longitudinal controller ("pid").
class LongitudinalController : public NodeHost
{
public:
  LongitudinalController(
    const std::string & mode, const std::vector<rclcpp::Parameter> & parameters);
  bool is_ready(const tf::InputData & input) { return controller_->isReady(input); }
  tf::LongitudinalOutput run(const tf::InputData & input) { return controller_->run(input); }
  void sync(const tf::LateralSyncData & data) { controller_->sync(data); }
  void reset() { controller_->reset(); }
  const std::string & mode() const { return mode_; }

private:
  std::string mode_;
  std::shared_ptr<tf::LongitudinalControllerBase> controller_;
};

/// Result of one TrajectoryFollower control cycle.
struct ControlResult
{
  /// False when the cycle was skipped (controllers not ready or command timed out).
  bool is_published{false};
  /// Human readable reason when `is_published` is false.
  std::string skip_reason{};
  autoware_control_msgs::msg::Control control_cmd{};
  std::optional<autoware_control_msgs::msg::ControlHorizon> control_cmd_horizon{};
  std::optional<tf::LateralOutput> lateral_output{};
  std::optional<tf::LongitudinalOutput> longitudinal_output{};
  double lateral_processing_time_ms{0.0};
  double longitudinal_processing_time_ms{0.0};
};

/// ROS-free equivalent of `autoware::motion::control::trajectory_follower_node::Controller`:
/// one lateral and one longitudinal controller sharing one node, run in lock step.
class TrajectoryFollower : public NodeHost
{
public:
  explicit TrajectoryFollower(const std::vector<rclcpp::Parameter> & parameters);

  /// One control cycle (Controller::callbackTimerControl without the ROS I/O).
  ControlResult step(const tf::InputData & input);

  void set_steering_offset(double offset) { lateral_->set_steering_offset(offset); }
  void reset_longitudinal() { longitudinal_->reset(); }
  const std::string & lateral_mode() const { return lateral_mode_; }
  const std::string & longitudinal_mode() const { return longitudinal_mode_; }
  double ctrl_period() const { return ctrl_period_; }

private:
  std::string lateral_mode_;
  std::string longitudinal_mode_;
  double ctrl_period_{0.03};
  double timeout_thr_sec_{0.5};
  bool enable_control_cmd_horizon_pub_{false};
  std::shared_ptr<tf::LateralControllerBase> lateral_;
  std::shared_ptr<tf::LongitudinalControllerBase> longitudinal_;
};

/// Merge lateral and longitudinal horizons (copied from controller_node.cpp).
std::optional<autoware_control_msgs::msg::ControlHorizon> merge_lat_lon_horizon(
  const tf::LateralHorizon & lateral_horizon, const tf::LongitudinalHorizon & longitudinal_horizon,
  const rclcpp::Time & stamp);
}  // namespace autoware_trajectory_controller

#endif  // AUTOWARE_TRAJECTORY_CONTROLLER__TRAJECTORY_FOLLOWER_HPP_
