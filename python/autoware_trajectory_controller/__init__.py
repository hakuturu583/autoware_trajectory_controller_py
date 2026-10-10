"""ROS 2-free Python bindings of the Autoware trajectory follower controllers.

The upstream Autoware C++ sources (MPC / pure pursuit lateral controllers and the PID
longitudinal controller) are compiled unchanged against a small ROS 2 replacement
layer, so they run in any Python environment.
"""

from __future__ import annotations

import logging as _logging

from . import msg
from ._core import (
    ControlResult,
    InputData,
    LateralHorizon,
    LateralOutput,
    LateralSyncData,
    LongitudinalHorizon,
    LongitudinalOutput,
    LongitudinalSyncData,
    set_log_handler,
    set_log_level,
)
from .controllers import LateralController, LongitudinalController, TrajectoryFollower
from .params import default_parameters, load_param_file
from .simulator import SimpleVehicleSimulator
from .utils import (
    make_accel,
    make_odometry,
    make_operation_mode,
    make_pose,
    make_steering_report,
    make_trajectory,
    quaternion_to_yaw,
    trajectory_to_arrays,
    yaw_to_quaternion,
)

__version__ = "0.1.0"

__all__ = [
    "ControlResult",
    "InputData",
    "LateralController",
    "LateralHorizon",
    "LateralOutput",
    "LateralSyncData",
    "LongitudinalController",
    "LongitudinalHorizon",
    "LongitudinalOutput",
    "LongitudinalSyncData",
    "SimpleVehicleSimulator",
    "TrajectoryFollower",
    "default_parameters",
    "load_param_file",
    "make_accel",
    "make_odometry",
    "make_operation_mode",
    "make_pose",
    "make_steering_report",
    "make_trajectory",
    "msg",
    "quaternion_to_yaw",
    "route_logs_to_python_logging",
    "set_log_handler",
    "set_log_level",
    "trajectory_to_arrays",
    "yaw_to_quaternion",
]


def route_logs_to_python_logging(logger_name: str = "autoware") -> None:
    """Send the controllers' RCLCPP_* logs to Python's :mod:`logging`."""
    base = _logging.getLogger(logger_name)

    def _handler(level: int, name: str, message: str) -> None:
        base.getChild(name).log(level, message)

    set_log_handler(_handler)
