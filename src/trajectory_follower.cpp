#include "trajectory_follower.hpp"

#include "autoware/mpc_lateral_controller/mpc_lateral_controller.hpp"
#include "autoware/pid_longitudinal_controller/pid_longitudinal_controller_node.hpp"
#include "autoware/pure_pursuit/autoware_pure_pursuit_lateral_controller.hpp"

#include <autoware_utils_system/stop_watch.hpp>

#include <chrono>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware_trajectory_controller
{
namespace
{
std::shared_ptr<tf::LateralControllerBase> make_lateral(
  const std::string & mode, rclcpp::Node & node,
  const std::shared_ptr<diagnostic_updater::Updater> & diag)
{
  if (mode == "mpc") {
    return std::make_shared<autoware::motion::control::mpc_lateral_controller::MpcLateralController>(
      node, diag);
  }
  if (mode == "pure_pursuit") {
    return std::make_shared<autoware::pure_pursuit::PurePursuitLateralController>(node);
  }
  throw std::invalid_argument(
    "invalid lateral_controller_mode '" + mode + "' (expected \"mpc\" or \"pure_pursuit\")");
}

std::shared_ptr<tf::LongitudinalControllerBase> make_longitudinal(
  const std::string & mode, rclcpp::Node & node,
  const std::shared_ptr<diagnostic_updater::Updater> & diag)
{
  if (mode == "pid") {
    return std::make_shared<
      autoware::motion::control::pid_longitudinal_controller::PidLongitudinalControllerNode>(
      node, diag);
  }
  throw std::invalid_argument(
    "invalid longitudinal_controller_mode '" + mode + "' (expected \"pid\")");
}

template <typename T>
std::vector<T> resampleHorizonByZeroOrderHold(
  const std::vector<T> & original_horizon, const double original_time_step_ms,
  const double new_time_step_ms)
{
  std::vector<T> resampled_horizon{};
  const size_t step_factor = static_cast<size_t>(original_time_step_ms / new_time_step_ms);
  const size_t resampled_size = original_horizon.size() * step_factor;
  resampled_horizon.reserve(resampled_size);
  for (const auto & command : original_horizon) {
    for (size_t i = 0; i < step_factor; ++i) {
      resampled_horizon.push_back(command);
    }
  }
  return resampled_horizon;
}

// Copied from Controller::mergeLatLonHorizon (autoware_trajectory_follower_node,
// src/controller_node.cpp, fetched by scripts/fetch_upstream.py).
std::optional<autoware_control_msgs::msg::ControlHorizon> merge_lat_lon_horizon(
  const tf::LateralHorizon & lateral_horizon, const tf::LongitudinalHorizon & longitudinal_horizon,
  const rclcpp::Time & stamp)
{
  if (lateral_horizon.controls.empty() || longitudinal_horizon.controls.empty()) {
    return std::nullopt;
  }

  autoware_control_msgs::msg::ControlHorizon control_horizon{};
  control_horizon.stamp = stamp;

  if (lateral_horizon.controls.size() == 1) {
    control_horizon.time_step_ms = longitudinal_horizon.time_step_ms;
    control_horizon.controls.reserve(longitudinal_horizon.controls.size());
    const auto lateral = lateral_horizon.controls.front();
    for (const auto & longitudinal : longitudinal_horizon.controls) {
      autoware_control_msgs::msg::Control control;
      control.longitudinal = longitudinal;
      control.lateral = lateral;
      control.stamp = stamp;
      control_horizon.controls.push_back(control);
    }
    return control_horizon;
  }
  if (longitudinal_horizon.controls.size() == 1) {
    control_horizon.time_step_ms = lateral_horizon.time_step_ms;
    control_horizon.controls.reserve(lateral_horizon.controls.size());
    const auto longitudinal = longitudinal_horizon.controls.front();
    for (const auto & lateral : lateral_horizon.controls) {
      autoware_control_msgs::msg::Control control;
      control.longitudinal = longitudinal;
      control.lateral = lateral;
      control.stamp = stamp;
      control_horizon.controls.push_back(control);
    }
    return control_horizon;
  }

  const auto gcd_double = [](const double a, const double b) {
    const double precision = 1e9;
    const int int_a = static_cast<int>(round(a * precision));
    const int int_b = static_cast<int>(round(b * precision));
    return static_cast<double>(std::gcd(int_a, int_b)) / precision;
  };
  const double time_step_ms =
    gcd_double(lateral_horizon.time_step_ms, longitudinal_horizon.time_step_ms);
  control_horizon.time_step_ms = time_step_ms;

  const auto lateral_controls = resampleHorizonByZeroOrderHold(
    lateral_horizon.controls, lateral_horizon.time_step_ms, time_step_ms);
  const auto longitudinal_controls = resampleHorizonByZeroOrderHold(
    longitudinal_horizon.controls, longitudinal_horizon.time_step_ms, time_step_ms);

  if (lateral_controls.size() != longitudinal_controls.size()) {
    return std::nullopt;
  }

  control_horizon.controls.reserve(lateral_controls.size());
  for (size_t i = 0; i < lateral_controls.size(); ++i) {
    autoware_control_msgs::msg::Control control;
    control.lateral = lateral_controls[i];
    control.longitudinal = longitudinal_controls[i];
    control.stamp = stamp;
    control_horizon.controls.push_back(control);
  }
  return control_horizon;
}
}  // namespace

NodeHost::NodeHost(const std::string & name, const std::vector<rclcpp::Parameter> & parameters)
: node_(std::make_unique<rclcpp::Node>(
    name, rclcpp::NodeOptions().parameter_overrides(parameters))),
  diag_updater_(std::make_shared<diagnostic_updater::Updater>(node_.get()))
{
}

std::vector<diagnostic_msgs::msg::DiagnosticStatus> NodeHost::update_diagnostics()
{
  diag_updater_->force_update();
  return diag_updater_->get_statuses();
}

LateralController::LateralController(
  const std::string & mode, const std::vector<rclcpp::Parameter> & parameters)
: NodeHost("lateral_controller", parameters), mode_(mode)
{
  node_->declare_parameter<double>("ctrl_period");
  controller_ = make_lateral(mode_, *node_, diag_updater_);
}

LongitudinalController::LongitudinalController(
  const std::string & mode, const std::vector<rclcpp::Parameter> & parameters)
: NodeHost("longitudinal_controller", parameters), mode_(mode)
{
  node_->declare_parameter<double>("ctrl_period");
  controller_ = make_longitudinal(mode_, *node_, diag_updater_);
}

TrajectoryFollower::TrajectoryFollower(const std::vector<rclcpp::Parameter> & parameters)
: NodeHost("controller", parameters)
{
  // Same declarations as Controller::Controller (controller_node.cpp).
  ctrl_period_ = node_->declare_parameter<double>("ctrl_period");
  const auto trajectory_reference_mode =
    node_->declare_parameter<std::string>("trajectory_reference_mode", "spatial");
  if (trajectory_reference_mode != "spatial" && trajectory_reference_mode != "temporal") {
    throw std::invalid_argument(
      "Invalid trajectory_reference_mode. Expected \"spatial\" or \"temporal\".");
  }
  timeout_thr_sec_ = node_->declare_parameter<double>("timeout_thr_sec");
  node_->declare_parameter<double>("cyclic_message_timeout_thr_sec", 0.9);
  enable_control_cmd_horizon_pub_ =
    node_->declare_parameter<bool>("enable_control_cmd_horizon_pub", false);
  diag_updater_->setHardwareID("trajectory_follower_node");

  lateral_mode_ = node_->declare_parameter<std::string>("lateral_controller_mode");
  lateral_ = make_lateral(lateral_mode_, *node_, diag_updater_);
  longitudinal_mode_ = node_->declare_parameter<std::string>("longitudinal_controller_mode");
  longitudinal_ = make_longitudinal(longitudinal_mode_, *node_, diag_updater_);
}

ControlResult TrajectoryFollower::step(const tf::InputData & input)
{
  ControlResult result;
  result.control_cmd.stamp = node_->now();

  // 2. check if controllers are ready
  const bool is_lat_ready = lateral_->isReady(input);
  const bool is_lon_ready = longitudinal_->isReady(input);
  if (!is_lat_ready || !is_lon_ready) {
    result.skip_reason =
      "Control is skipped since lateral and/or longitudinal controllers are not ready to run.";
    return result;
  }

  // 3. run controllers
  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("lateral");
  auto lat_out = lateral_->run(input);
  result.lateral_processing_time_ms = stop_watch.toc("lateral");

  stop_watch.tic("longitudinal");
  auto lon_out = longitudinal_->run(input);
  result.longitudinal_processing_time_ms = stop_watch.toc("longitudinal");

  // 4. sync with each other controllers
  longitudinal_->sync(lat_out.sync_data);
  lateral_->sync(lon_out.sync_data);

  const auto now = node_->now();
  if ((now - lat_out.control_cmd.stamp).seconds() > timeout_thr_sec_) {
    result.skip_reason = "Lateral control command too old, control_cmd will not be published.";
  } else if ((now - lon_out.control_cmd.stamp).seconds() > timeout_thr_sec_) {
    result.skip_reason =
      "Longitudinal control command too old, control_cmd will not be published.";
  } else {
    // 5. control command
    result.control_cmd.lateral = lat_out.control_cmd;
    result.control_cmd.longitudinal = lon_out.control_cmd;
    result.is_published = true;

    // 7. experimental topic
    if (enable_control_cmd_horizon_pub_) {
      result.control_cmd_horizon =
        merge_lat_lon_horizon(lat_out.control_cmd_horizon, lon_out.control_cmd_horizon, now);
    }
  }
  result.lateral_output = std::move(lat_out);
  result.longitudinal_output = std::move(lon_out);
  return result;
}

}  // namespace autoware_trajectory_controller
