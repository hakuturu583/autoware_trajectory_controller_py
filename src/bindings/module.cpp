// pybind11 entry point of the ROS-free Autoware trajectory controller wrapper.
#include "messages.hpp"
#include "trajectory_follower.hpp"

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace py = pybind11;
namespace atc = autoware_trajectory_controller;
namespace tf = autoware::motion::control::trajectory_follower;

namespace
{
// ---- parameter conversion ---------------------------------------------------
bool is_py_bool(py::handle h) { return PyBool_Check(h.ptr()); }
bool is_py_int(py::handle h) { return !is_py_bool(h) && PyIndex_Check(h.ptr()); }
bool is_py_float(py::handle h)
{
  return !is_py_bool(h) && !is_py_int(h) && (PyFloat_Check(h.ptr()) || py::hasattr(h, "__float__"));
}

rclcpp::ParameterValue to_parameter_value(const std::string & name, py::handle h)
{
  if (is_py_bool(h)) return rclcpp::ParameterValue(h.cast<bool>());
  if (is_py_int(h)) return rclcpp::ParameterValue(h.cast<int64_t>());
  if (py::isinstance<py::str>(h)) return rclcpp::ParameterValue(h.cast<std::string>());
  if (is_py_float(h)) return rclcpp::ParameterValue(h.cast<double>());
  if (py::isinstance<py::sequence>(h) || py::hasattr(h, "tolist")) {
    py::object seq = py::hasattr(h, "tolist") ? h.attr("tolist")()
                                               : py::reinterpret_borrow<py::object>(h);
    py::list items(seq);
    bool all_bool = true, all_int = true, all_num = true, all_str = true;
    for (auto item : items) {
      all_bool &= is_py_bool(item);
      all_int &= is_py_int(item);
      all_num &= (is_py_int(item) || is_py_float(item));
      all_str &= py::isinstance<py::str>(item);
    }
    if (items.size() == 0) return rclcpp::ParameterValue(std::vector<double>{});
    if (all_bool) return rclcpp::ParameterValue(items.cast<std::vector<bool>>());
    if (all_int) return rclcpp::ParameterValue(items.cast<std::vector<int64_t>>());
    if (all_num) return rclcpp::ParameterValue(items.cast<std::vector<double>>());
    if (all_str) return rclcpp::ParameterValue(items.cast<std::vector<std::string>>());
  }
  throw py::type_error(
    "unsupported value for parameter '" + name + "': " + py::repr(h).cast<std::string>());
}

std::vector<rclcpp::Parameter> to_parameters(const py::dict & params)
{
  std::vector<rclcpp::Parameter> out;
  for (auto [k, v] : params) {
    const auto name = py::str(k).cast<std::string>();
    out.emplace_back(name, to_parameter_value(name, v));
  }
  return out;
}

py::object to_python(const rclcpp::ParameterValue & value)
{
  return std::visit(
    [](const auto & v) -> py::object {
      using V = std::decay_t<decltype(v)>;
      if constexpr (std::is_same_v<V, std::monostate>) {
        return py::none();
      } else {
        return py::cast(v);
      }
    },
    value.storage());
}

// ---- generic helpers shared by every host --------------------------------------
py::dict get_parameters(const atc::NodeHost & host)
{
  py::dict out;
  for (const auto & p : host.node().list_parameters()) {
    out[py::str(p.get_name())] = to_python(p.get_parameter_value());
  }
  return out;
}

py::object get_parameter(const atc::NodeHost & host, const std::string & name)
{
  return to_python(host.node().get_parameter(name).get_parameter_value());
}

void set_parameters(atc::NodeHost & host, const py::dict & params)
{
  const auto result = host.node().set_parameters_atomically(to_parameters(params));
  if (!result.successful) {
    throw py::value_error("failed to set parameters: " + result.reason);
  }
}

template <typename T>
bool try_debug(const rclcpp::Node & node, const std::string & topic, py::object & out)
{
  if (auto msg = node.get_last_message<T>(topic)) {
    out = py::cast(*msg);
    return true;
  }
  return false;
}

py::object get_debug(const atc::NodeHost & host, const std::string & topic)
{
  py::object out = py::none();
  const auto & node = host.node();
  try_debug<autoware_planning_msgs::msg::Trajectory>(node, topic, out) ||
    try_debug<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>(node, topic, out) ||
    try_debug<autoware_internal_debug_msgs::msg::Float32Stamped>(node, topic, out) ||
    try_debug<autoware_internal_debug_msgs::msg::Float64Stamped>(node, topic, out) ||
    try_debug<geometry_msgs::msg::PoseStamped>(node, topic, out) ||
    try_debug<visualization_msgs::msg::MarkerArray>(node, topic, out);
  return out;
}

py::dict get_debug_all(const atc::NodeHost & host)
{
  py::dict out;
  for (const auto & topic : host.node().get_topic_names()) {
    auto msg = get_debug(host, topic);
    if (!msg.is_none()) out[py::str(topic)] = msg;
  }
  return out;
}

template <typename HostT, typename PyClass>
void bind_host_common(PyClass & c)
{
  c.def_property(
     "time",
     [](const HostT & h) { return static_cast<double>(h.time_ns()) * 1e-9; },
     [](HostT & h, double t) { h.set_time_ns(static_cast<int64_t>(std::llround(t * 1e9))); },
     "Current time [s] of the controller clock. The clock does not advance by itself: set it "
     "before every control cycle (simulated time).")
    .def_property(
      "time_ns", &HostT::time_ns, &HostT::set_time_ns, "Current clock time in nanoseconds.")
    .def(
      "get_parameters", [](const HostT & h) { return get_parameters(h); },
      "All declared parameters as a dict.")
    .def(
      "get_parameter", [](const HostT & h, const std::string & n) { return get_parameter(h, n); },
      py::arg("name"))
    .def(
      "set_parameters", [](HostT & h, const py::dict & p) { set_parameters(h, p); },
      py::arg("parameters"),
      "Update parameters at runtime (goes through the controller's on-set-parameters callback, "
      "like `ros2 param set`).")
    .def(
      "update_diagnostics", &HostT::update_diagnostics,
      "Run the diagnostic tasks and return their DiagnosticStatus list.")
    .def(
      "debug_topics", [](const HostT & h) { return h.node().get_topic_names(); },
      "Names of the debug 'topics' the controller publishes to.")
    .def(
      "get_debug", [](const HostT & h, const std::string & t) { return get_debug(h, t); },
      py::arg("topic"), "Last message published on a debug topic (or None).")
    .def(
      "get_debug_all", [](const HostT & h) { return get_debug_all(h); },
      "Dict of every debug topic that has received a message.");
}

py::object g_log_handler = py::none();
}  // namespace

PYBIND11_MODULE(_core, m)
{
  m.doc() = "ROS-free Python bindings of the Autoware trajectory follower controllers.";

  atc::bindings::bind_messages(m);

  // ---- Time / Duration helpers --------------------------------------------------
  auto time_cls = py::reinterpret_borrow<py::class_<builtin_interfaces::msg::Time>>(
    py::type::of<builtin_interfaces::msg::Time>());
  time_cls
    .def_static(
      "from_seconds",
      [](double s) {
        return static_cast<builtin_interfaces::msg::Time>(
          rclcpp::Time(static_cast<int64_t>(std::llround(s * 1e9))));
      },
      py::arg("seconds"))
    .def("to_seconds", [](const builtin_interfaces::msg::Time & t) {
      return rclcpp::Time(t).seconds();
    });
  auto duration_cls = py::reinterpret_borrow<py::class_<builtin_interfaces::msg::Duration>>(
    py::type::of<builtin_interfaces::msg::Duration>());
  duration_cls
    .def_static(
      "from_seconds",
      [](double s) {
        return static_cast<builtin_interfaces::msg::Duration>(rclcpp::Duration::from_seconds(s));
      },
      py::arg("seconds"))
    .def("to_seconds", [](const builtin_interfaces::msg::Duration & d) {
      return rclcpp::Duration(d).seconds();
    });

  // ---- trajectory_follower_base data types ---------------------------------------
  py::class_<tf::InputData>(m, "InputData", "Inputs of one control cycle.")
    .def(py::init<>())
    .def(
      py::init([](
                 const autoware_planning_msgs::msg::Trajectory & trajectory,
                 const nav_msgs::msg::Odometry & odometry,
                 const autoware_vehicle_msgs::msg::SteeringReport & steering,
                 const geometry_msgs::msg::AccelWithCovarianceStamped & accel,
                 const autoware_adapi_v1_msgs::msg::OperationModeState & operation_mode) {
        tf::InputData d;
        d.current_trajectory = trajectory;
        d.current_odometry = odometry;
        d.current_steering = steering;
        d.current_accel = accel;
        d.current_operation_mode = operation_mode;
        return d;
      }),
      py::kw_only(), py::arg("trajectory"), py::arg("odometry"), py::arg("steering"),
      py::arg("accel"), py::arg("operation_mode"))
    .def_readwrite("current_trajectory", &tf::InputData::current_trajectory)
    .def_readwrite("current_odometry", &tf::InputData::current_odometry)
    .def_readwrite("current_steering", &tf::InputData::current_steering)
    .def_readwrite("current_accel", &tf::InputData::current_accel)
    .def_readwrite("current_operation_mode", &tf::InputData::current_operation_mode);

  py::class_<tf::LateralSyncData>(m, "LateralSyncData")
    .def(py::init<>())
    .def_readwrite("is_steer_converged", &tf::LateralSyncData::is_steer_converged)
    .def("__repr__", [](const tf::LateralSyncData & d) {
      return std::string("LateralSyncData(is_steer_converged=") +
             (d.is_steer_converged ? "True" : "False") + ")";
    });
  py::class_<tf::LongitudinalSyncData>(m, "LongitudinalSyncData").def(py::init<>());

  py::class_<tf::LateralHorizon>(m, "LateralHorizon")
    .def(py::init<>())
    .def_readwrite("time_step_ms", &tf::LateralHorizon::time_step_ms)
    .def_readwrite("controls", &tf::LateralHorizon::controls);
  py::class_<tf::LongitudinalHorizon>(m, "LongitudinalHorizon")
    .def(py::init<>())
    .def_readwrite("time_step_ms", &tf::LongitudinalHorizon::time_step_ms)
    .def_readwrite("controls", &tf::LongitudinalHorizon::controls);

  py::class_<tf::LateralOutput>(m, "LateralOutput")
    .def(py::init<>())
    .def_readwrite("control_cmd", &tf::LateralOutput::control_cmd)
    .def_readwrite("control_cmd_horizon", &tf::LateralOutput::control_cmd_horizon)
    .def_readwrite("sync_data", &tf::LateralOutput::sync_data);
  py::class_<tf::LongitudinalOutput>(m, "LongitudinalOutput")
    .def(py::init<>())
    .def_readwrite("control_cmd", &tf::LongitudinalOutput::control_cmd)
    .def_readwrite("control_cmd_horizon", &tf::LongitudinalOutput::control_cmd_horizon)
    .def_readwrite("sync_data", &tf::LongitudinalOutput::sync_data);

  py::class_<atc::ControlResult>(m, "ControlResult", "Result of TrajectoryFollower.step().")
    .def(py::init<>())
    .def_readwrite("is_published", &atc::ControlResult::is_published)
    .def_readwrite("skip_reason", &atc::ControlResult::skip_reason)
    .def_readwrite("control_cmd", &atc::ControlResult::control_cmd)
    .def_readwrite("control_cmd_horizon", &atc::ControlResult::control_cmd_horizon)
    .def_readwrite("lateral_output", &atc::ControlResult::lateral_output)
    .def_readwrite("longitudinal_output", &atc::ControlResult::longitudinal_output)
    .def_readwrite(
      "lateral_processing_time_ms", &atc::ControlResult::lateral_processing_time_ms)
    .def_readwrite(
      "longitudinal_processing_time_ms", &atc::ControlResult::longitudinal_processing_time_ms)
    .def("__bool__", [](const atc::ControlResult & r) { return r.is_published; })
    .def("__repr__", [](const atc::ControlResult & r) {
      if (!r.is_published) return "ControlResult(is_published=False, skip_reason=" +
                                   py::repr(py::str(r.skip_reason)).cast<std::string>() + ")";
      return "ControlResult(is_published=True, control_cmd=" +
             py::repr(py::cast(r.control_cmd)).cast<std::string>() + ")";
    });

  // ---- controller hosts ------------------------------------------------------------
  py::class_<atc::LateralController> lat(
    m, "LateralController",
    "Autoware lateral controller (MPC or pure pursuit) running on a ROS-free node.");
  lat.def(
       py::init([](const std::string & mode, const py::dict & params) {
         return std::make_unique<atc::LateralController>(mode, to_parameters(params));
       }),
       py::arg("mode"), py::arg("parameters"))
    .def_property_readonly("mode", &atc::LateralController::mode)
    .def("is_ready", &atc::LateralController::is_ready, py::arg("input"))
    .def("run", &atc::LateralController::run, py::arg("input"))
    .def("sync", &atc::LateralController::sync, py::arg("longitudinal_sync_data"))
    .def("set_steering_offset", &atc::LateralController::set_steering_offset, py::arg("offset"));
  bind_host_common<atc::LateralController>(lat);

  py::class_<atc::LongitudinalController> lon(
    m, "LongitudinalController",
    "Autoware longitudinal controller (PID) running on a ROS-free node.");
  lon.def(
       py::init([](const std::string & mode, const py::dict & params) {
         return std::make_unique<atc::LongitudinalController>(mode, to_parameters(params));
       }),
       py::arg("mode"), py::arg("parameters"))
    .def_property_readonly("mode", &atc::LongitudinalController::mode)
    .def("is_ready", &atc::LongitudinalController::is_ready, py::arg("input"))
    .def("run", &atc::LongitudinalController::run, py::arg("input"))
    .def("sync", &atc::LongitudinalController::sync, py::arg("lateral_sync_data"))
    .def("reset", &atc::LongitudinalController::reset);
  bind_host_common<atc::LongitudinalController>(lon);

  py::class_<atc::TrajectoryFollower> follower(
    m, "TrajectoryFollower",
    "ROS-free equivalent of autoware_trajectory_follower_node (lateral + longitudinal).");
  follower
    .def(
      py::init([](const py::dict & params) {
        return std::make_unique<atc::TrajectoryFollower>(to_parameters(params));
      }),
      py::arg("parameters"))
    .def("step", &atc::TrajectoryFollower::step, py::arg("input"))
    .def("set_steering_offset", &atc::TrajectoryFollower::set_steering_offset, py::arg("offset"))
    .def("reset_longitudinal", &atc::TrajectoryFollower::reset_longitudinal)
    .def_property_readonly("lateral_mode", &atc::TrajectoryFollower::lateral_mode)
    .def_property_readonly("longitudinal_mode", &atc::TrajectoryFollower::longitudinal_mode)
    .def_property_readonly("ctrl_period", &atc::TrajectoryFollower::ctrl_period);
  bind_host_common<atc::TrajectoryFollower>(follower);

  // ---- logging ---------------------------------------------------------------------
  m.def(
    "set_log_level",
    [](const std::string & level) {
      using L = rclcpp::logging::LogLevel;
      static const std::map<std::string, L> levels{
        {"debug", L::Debug}, {"info", L::Info}, {"warn", L::Warn},
        {"warning", L::Warn}, {"error", L::Error}, {"fatal", L::Fatal}};
      std::string key;
      for (char ch : level) key.push_back(static_cast<char>(std::tolower(ch)));
      const auto it = levels.find(key);
      if (it == levels.end()) throw py::value_error("unknown log level: " + level);
      rclcpp::logging::set_global_level(it->second);
    },
    py::arg("level"), "Minimum level of the controller logs (debug/info/warn/error/fatal).");
  m.def(
    "set_log_handler",
    [](py::object handler) {
      g_log_handler = handler;
      if (handler.is_none()) {
        rclcpp::logging::set_sink({});
        return;
      }
      rclcpp::logging::set_sink(
        [](rclcpp::logging::LogLevel level, const std::string & name, const std::string & msg) {
          py::gil_scoped_acquire gil;
          g_log_handler(static_cast<int>(level), name, msg);
        });
    },
    py::arg("handler"),
    "Route controller logs to `handler(level: int, logger_name: str, message: str)` "
    "(levels follow Python's logging module). None restores printing to stderr.");
  m.attr("__autoware_with_osqp__") =
#ifdef AUTOWARE_TC_WITH_OSQP
    true;
#else
    false;
#endif

  // Release the Python callable before interpreter shutdown.
  auto atexit = py::module_::import("atexit");
  atexit.attr("register")(py::cpp_function([]() {
    rclcpp::logging::set_sink({});
    g_log_handler = py::none();
  }));
}
