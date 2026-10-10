#!/usr/bin/env python3
"""Generate ROS-free C++ message structs into ``shim/include``.

The Autoware sources include ROS 2 generated message headers such as
``autoware_planning_msgs/msg/trajectory.hpp``.  This script emits plain C++
structs with the same names, namespaces, field names and include paths so
that those sources compile unchanged without ``rosidl``.
"""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "shim" / "include"

T = "builtin_interfaces::msg::Time"
D = "builtin_interfaces::msg::Duration"
H = "std_msgs::msg::Header"

# package -> message -> {"fields": [(type, name, default)], "consts": [(type, name, value)]}
MSGS: dict[str, dict[str, dict]] = {
    "builtin_interfaces": {
        "Time": {"fields": [("int32_t", "sec", "0"), ("uint32_t", "nanosec", "0")]},
        "Duration": {"fields": [("int32_t", "sec", "0"), ("uint32_t", "nanosec", "0")]},
    },
    "std_msgs": {
        "Header": {"fields": [(T, "stamp", ""), ("std::string", "frame_id", "")]},
        "ColorRGBA": {
            "fields": [("float", n, "0.0f") for n in ("r", "g", "b", "a")]
        },
    },
    "geometry_msgs": {
        "Point": {"fields": [("double", n, "0.0") for n in "xyz"]},
        "Point32": {"fields": [("float", n, "0.0f") for n in "xyz"]},
        "Vector3": {"fields": [("double", n, "0.0") for n in "xyz"]},
        "Quaternion": {
            "fields": [("double", "x", "0.0"), ("double", "y", "0.0"), ("double", "z", "0.0"),
                       ("double", "w", "1.0")]
        },
        "Pose": {"fields": [("geometry_msgs::msg::Point", "position", ""),
                            ("geometry_msgs::msg::Quaternion", "orientation", "")]},
        "PoseStamped": {"fields": [(H, "header", ""), ("geometry_msgs::msg::Pose", "pose", "")]},
        "PoseWithCovariance": {"fields": [("geometry_msgs::msg::Pose", "pose", ""),
                                          ("std::array<double, 36>", "covariance", "")]},
        "PoseWithCovarianceStamped": {"fields": [
            (H, "header", ""), ("geometry_msgs::msg::PoseWithCovariance", "pose", "")]},
        "Twist": {"fields": [("geometry_msgs::msg::Vector3", "linear", ""),
                             ("geometry_msgs::msg::Vector3", "angular", "")]},
        "TwistStamped": {"fields": [(H, "header", ""), ("geometry_msgs::msg::Twist", "twist", "")]},
        "TwistWithCovariance": {"fields": [("geometry_msgs::msg::Twist", "twist", ""),
                                           ("std::array<double, 36>", "covariance", "")]},
        "TwistWithCovarianceStamped": {"fields": [
            (H, "header", ""), ("geometry_msgs::msg::TwistWithCovariance", "twist", "")]},
        "Accel": {"fields": [("geometry_msgs::msg::Vector3", "linear", ""),
                             ("geometry_msgs::msg::Vector3", "angular", "")]},
        "AccelStamped": {"fields": [(H, "header", ""), ("geometry_msgs::msg::Accel", "accel", "")]},
        "AccelWithCovariance": {"fields": [("geometry_msgs::msg::Accel", "accel", ""),
                                           ("std::array<double, 36>", "covariance", "")]},
        "AccelWithCovarianceStamped": {"fields": [
            (H, "header", ""), ("geometry_msgs::msg::AccelWithCovariance", "accel", "")]},
        "Transform": {"fields": [("geometry_msgs::msg::Vector3", "translation", ""),
                                 ("geometry_msgs::msg::Quaternion", "rotation", "")]},
        "TransformStamped": {"fields": [(H, "header", ""), ("std::string", "child_frame_id", ""),
                                        ("geometry_msgs::msg::Transform", "transform", "")]},
        "Polygon": {"fields": [("std::vector<geometry_msgs::msg::Point32>", "points", "")]},
    },
    "nav_msgs": {
        "Odometry": {"fields": [
            (H, "header", ""), ("std::string", "child_frame_id", ""),
            ("geometry_msgs::msg::PoseWithCovariance", "pose", ""),
            ("geometry_msgs::msg::TwistWithCovariance", "twist", "")]},
    },
    "diagnostic_msgs": {
        "KeyValue": {"fields": [("std::string", "key", ""), ("std::string", "value", "")]},
        "DiagnosticStatus": {
            "fields": [("uint8_t", "level", "0"), ("std::string", "name", ""),
                       ("std::string", "message", ""), ("std::string", "hardware_id", ""),
                       ("std::vector<diagnostic_msgs::msg::KeyValue>", "values", "")],
            "consts": [("uint8_t", "OK", "0"), ("uint8_t", "WARN", "1"),
                       ("uint8_t", "ERROR", "2"), ("uint8_t", "STALE", "3")],
        },
    },
    "visualization_msgs": {
        "Marker": {
            "fields": [
                (H, "header", ""), ("std::string", "ns", ""), ("int32_t", "id", "0"),
                ("int32_t", "type", "0"), ("int32_t", "action", "0"),
                ("geometry_msgs::msg::Pose", "pose", ""),
                ("geometry_msgs::msg::Vector3", "scale", ""),
                ("std_msgs::msg::ColorRGBA", "color", ""),
                (D, "lifetime", ""), ("bool", "frame_locked", "false"),
                ("std::vector<geometry_msgs::msg::Point>", "points", ""),
                ("std::vector<std_msgs::msg::ColorRGBA>", "colors", ""),
                ("std::string", "texture_resource", ""), ("std::string", "text", ""),
                ("std::string", "mesh_resource", ""), ("bool", "mesh_use_embedded_materials", "false"),
            ],
            "consts": [(("int32_t"), n, str(v)) for n, v in [
                ("ARROW", 0), ("CUBE", 1), ("SPHERE", 2), ("CYLINDER", 3), ("LINE_STRIP", 4),
                ("LINE_LIST", 5), ("CUBE_LIST", 6), ("SPHERE_LIST", 7), ("POINTS", 8),
                ("TEXT_VIEW_FACING", 9), ("MESH_RESOURCE", 10), ("TRIANGLE_LIST", 11),
                ("ARROW_STRIP", 12), ("ADD", 0), ("MODIFY", 0), ("DELETE", 2), ("DELETEALL", 3)]],
        },
        "MarkerArray": {"fields": [("std::vector<visualization_msgs::msg::Marker>", "markers", "")]},
    },
    "autoware_planning_msgs": {
        "TrajectoryPoint": {"fields": [
            (D, "time_from_start", ""), ("geometry_msgs::msg::Pose", "pose", ""),
            ("float", "longitudinal_velocity_mps", "0.0f"), ("float", "lateral_velocity_mps", "0.0f"),
            ("float", "acceleration_mps2", "0.0f"), ("float", "heading_rate_rps", "0.0f"),
            ("float", "front_wheel_angle_rad", "0.0f"), ("float", "rear_wheel_angle_rad", "0.0f")]},
        "Trajectory": {"fields": [
            (H, "header", ""), ("std::vector<autoware_planning_msgs::msg::TrajectoryPoint>", "points", "")]},
        "PathPoint": {"fields": [
            ("geometry_msgs::msg::Pose", "pose", ""), ("float", "longitudinal_velocity_mps", "0.0f"),
            ("float", "lateral_velocity_mps", "0.0f"), ("float", "heading_rate_rps", "0.0f"),
            ("bool", "is_final", "false")]},
        "Path": {"fields": [
            (H, "header", ""), ("std::vector<autoware_planning_msgs::msg::PathPoint>", "points", ""),
            ("std::vector<geometry_msgs::msg::Point>", "left_bound", ""),
            ("std::vector<geometry_msgs::msg::Point>", "right_bound", "")]},
    },
    "autoware_internal_planning_msgs": {
        "PathPointWithLaneId": {"fields": [
            ("autoware_planning_msgs::msg::PathPoint", "point", ""),
            ("std::vector<int64_t>", "lane_ids", "")]},
        "PathWithLaneId": {"fields": [
            (H, "header", ""),
            ("std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>", "points", ""),
            ("std::vector<geometry_msgs::msg::Point>", "left_bound", ""),
            ("std::vector<geometry_msgs::msg::Point>", "right_bound", "")]},
    },
    "autoware_control_msgs": {
        "Lateral": {"fields": [
            (T, "stamp", ""), (T, "control_time", ""), ("float", "steering_tire_angle", "0.0f"),
            ("float", "steering_tire_rotation_rate", "0.0f"),
            ("bool", "is_defined_steering_tire_rotation_rate", "false")]},
        "Longitudinal": {"fields": [
            (T, "stamp", ""), (T, "control_time", ""), ("float", "velocity", "0.0f"),
            ("float", "acceleration", "0.0f"), ("float", "jerk", "0.0f"),
            ("bool", "is_defined_acceleration", "false"), ("bool", "is_defined_jerk", "false")]},
        "Control": {"fields": [
            (T, "stamp", ""), (T, "control_time", ""),
            ("autoware_control_msgs::msg::Lateral", "lateral", ""),
            ("autoware_control_msgs::msg::Longitudinal", "longitudinal", "")]},
        "ControlHorizon": {"fields": [
            (T, "stamp", ""), ("float", "time_step_ms", "0.0f"),
            ("std::vector<autoware_control_msgs::msg::Control>", "controls", "")]},
    },
    "autoware_vehicle_msgs": {
        "SteeringReport": {"fields": [(T, "stamp", ""), ("float", "steering_tire_angle", "0.0f")]},
    },
    "autoware_adapi_v1_msgs": {
        "OperationModeState": {
            "fields": [(T, "stamp", ""), ("uint8_t", "mode", "0")] + [
                ("bool", n, "false") for n in (
                    "is_autoware_control_enabled", "is_in_transition", "is_stop_mode_available",
                    "is_autonomous_mode_available", "is_local_mode_available",
                    "is_remote_mode_available")],
            "consts": [("uint8_t", n, str(v)) for n, v in [
                ("UNKNOWN", 0), ("STOP", 1), ("AUTONOMOUS", 2), ("LOCAL", 3), ("REMOTE", 4)]],
        },
    },
    "autoware_internal_debug_msgs": {
        "Float32Stamped": {"fields": [(T, "stamp", ""), ("float", "data", "0.0f")]},
        "Float64Stamped": {"fields": [(T, "stamp", ""), ("double", "data", "0.0")]},
        "Float32MultiArrayStamped": {"fields": [(T, "stamp", ""), ("std::vector<float>", "data", "")]},
    },
}


def snake(name: str) -> str:
    return re.sub(r"(?<=[a-z0-9])([A-Z])", r"_\1", name).lower()


def deps_of(spec: dict) -> list[str]:
    deps = set()
    for typ, _, _ in spec["fields"]:
        for m in re.finditer(r"([a-z_0-9]+)::msg::([A-Za-z0-9]+)", typ):
            deps.add(f"{m.group(1)}/msg/{snake(m.group(2))}.hpp")
    return sorted(deps)


def render(pkg: str, name: str, spec: dict) -> str:
    guard = f"SHIM__{pkg.upper()}__MSG__DETAIL__{snake(name).upper()}__STRUCT_HPP_"
    lines = [
        "// Generated by scripts/gen_msgs.py. Do not edit.",
        f"#ifndef {guard}",
        f"#define {guard}",
        "",
        "// The real rosidl headers pull these in transitively and upstream code relies on it.",
        "#include <algorithm>",
        "#include <array>",
        "#include <cmath>",
        "#include <math.h>",
        "#include <cstdint>",
        "#include <limits>",
        "#include <memory>",
        "#include <optional>",
        "#include <stdexcept>",
        "#include <string>",
        "#include <utility>",
        "#include <vector>",
        "",
    ]
    lines += [f'#include "{d}"' for d in deps_of(spec)]
    lines += ["", f"namespace {pkg}::msg", "{", f"struct {name}", "{"]
    lines.append(f"  using SharedPtr = std::shared_ptr<{name}>;")
    lines.append(f"  using ConstSharedPtr = std::shared_ptr<const {name}>;")
    lines.append(f"  using UniquePtr = std::unique_ptr<{name}>;")
    for typ, cname, val in spec.get("consts", []):
        lines.append(f"  static constexpr {typ} {cname} = {val};")
    for typ, fname, default in spec["fields"]:
        init = f"{{{default}}}" if default else "{}"
        lines.append(f"  {typ} {fname}{init};")
    eq = " && ".join(f"{f} == other.{f}" for _, f, _ in spec["fields"]) or "true"
    lines.append(f"  bool operator==(const {name} & other) const {{ return {eq}; }}")
    lines.append(f"  bool operator!=(const {name} & other) const {{ return !(*this == other); }}")
    lines += ["};", f"}}  // namespace {pkg}::msg", ""]
    lines += render_builder(pkg, name, spec)
    lines += [f"#endif  // {guard}", ""]
    return "\n".join(lines)


def render_builder(pkg: str, name: str, spec: dict) -> list[str]:
    """Emulate rosidl's ``<pkg>::build<Msg>().f1(..).f2(..)`` fluent builder."""
    fields = spec["fields"]
    msg = f"::{pkg}::msg::{name}"
    out = [f"namespace {pkg}::msg::builder", "{"]
    # Init_<Name>_<field> classes, generated from the last field backwards.
    for i in reversed(range(len(fields))):
        typ, fname, _ = fields[i]
        cls = f"Init_{name}_{fname}"
        nxt = msg if i == len(fields) - 1 else f"Init_{name}_{fields[i + 1][1]}"
        out += [
            f"class {cls}",
            "{",
            "public:",
            f"  explicit {cls}({msg} & msg) : msg_(msg) {{}}" if i > 0 else
            f"  {cls}() : msg_(storage_) {{}}",
            f"  {nxt} {fname}({typ} arg)",
            "  {",
            f"    msg_.{fname} = std::move(arg);",
            "    return " + ("std::move(msg_);" if i == len(fields) - 1 else f"{nxt}(msg_);"),
            "  }",
            "",
            "private:",
        ]
        if i == 0:
            out.append(f"  {msg} storage_;")
        out += [f"  {msg} & msg_;", "};", ""]
    out += [f"}}  // namespace {pkg}::msg::builder", ""]
    out += [f"namespace {pkg}", "{", "template <typename MessageType>", "auto build();", "template <>",
            f"inline auto build<{msg}>()", "{",
            f"  return {pkg}::msg::builder::Init_{name}_{fields[0][1]}();", "}",
            f"}}  // namespace {pkg}", ""]
    return out


def render_bindings() -> str:
    """pybind11 bindings for every message, emitted in dependency order."""
    out = [
        "// Generated by scripts/gen_msgs.py. Do not edit.",
        "#pragma once",
        "",
        "#include <pybind11/pybind11.h>",
        "#include <pybind11/stl.h>",
        "",
        "#include <string>",
        "",
    ]
    for pkg, msgs in MSGS.items():
        for name in msgs:
            out.append(f'#include "{pkg}/msg/{snake(name)}.hpp"')
    out += ["", "namespace autoware_trajectory_controller::bindings", "{",
            "inline void bind_messages(pybind11::module_ & m)", "{", "  namespace py = pybind11;"]
    for pkg, msgs in MSGS.items():
        for name, spec in msgs.items():
            cpp = f"{pkg}::msg::{name}"
            fields = spec["fields"]
            out.append(f'  {{')
            out.append(f'    py::class_<{cpp}> c(m, "{name}", "ROS-free counterpart of {pkg}/msg/{name}.");')
            # keyword constructor: every field optional
            params = ", ".join(f"py::object {f}" for _, f, _ in fields)
            out.append(f"    c.def(py::init<>());")
            out.append(f"    c.def(py::init([]({params}) {{")
            out.append(f"      {cpp} msg;")
            for typ, f, _ in fields:
                out.append(f"      if (!{f}.is_none()) msg.{f} = {f}.cast<{typ}>();")
            out.append("      return msg;")
            args = ", ".join(f'py::arg("{f}") = py::none()' for _, f, _ in fields)
            out.append(f"    }}), py::kw_only(), {args});")
            for typ, cname, val in spec.get("consts", []):
                out.append(f'    c.attr("{cname}") = static_cast<int>({cpp}::{cname});')
            for _, f, _ in fields:
                out.append(f'    c.def_readwrite("{f}", &{cpp}::{f});')
            out.append(f"    c.def(py::self == py::self);" if False else
                       f'    c.def("__eq__", [](const {cpp} & a, const {cpp} & b) {{ return a == b; }}, py::is_operator());')
            out.append(f'    c.def("__copy__", []({cpp} const & self) {{ return {cpp}(self); }});')
            out.append(f'    c.def("__deepcopy__", []({cpp} const & self, py::dict) {{ return {cpp}(self); }}, py::arg("memo"));')
            out.append(f'    c.def("__repr__", []({cpp} const & self) {{')
            out.append(f'      std::string s = "{name}(";')
            for i, (_, f, _) in enumerate(fields):
                sep = "" if i == 0 else ", "
                out.append(f'      s += "{sep}{f}=" + py::repr(py::cast(self.{f})).cast<std::string>();')
            out.append('      return s + ")";')
            out.append("    });")
            out.append("    c.attr(\"__hash__\") = py::none();")
            out.append("  }")
    out += ["}", "}  // namespace autoware_trajectory_controller::bindings", ""]
    return "\n".join(out)


def main() -> None:
    (ROOT / "src" / "bindings" / "messages.hpp").write_text(render_bindings())
    for pkg, msgs in MSGS.items():
        for name, spec in msgs.items():
            base = OUT / pkg / "msg"
            (base / "detail").mkdir(parents=True, exist_ok=True)
            s = snake(name)
            (base / "detail" / f"{s}__struct.hpp").write_text(render(pkg, name, spec))
            (base / f"{s}.hpp").write_text(
                "// Generated by scripts/gen_msgs.py. Do not edit.\n#pragma once\n"
                f'#include "{pkg}/msg/detail/{s}__struct.hpp"\n'
            )


if __name__ == "__main__":
    main()
