"""Pythonic front-ends of the Autoware controllers.

All classes accept ``parameters`` overriding the upstream defaults (see
:func:`autoware_trajectory_controller.params.default_parameters`) and are driven with
simulated time: pass ``time`` (seconds) to every ``control()`` call.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Optional, Union

from . import _core as m
from .params import default_parameters
from .utils import make_accel, make_operation_mode, make_steering_report

TimeLike = Union[float, m.Time, None]


def _input_data(
    trajectory: m.Trajectory,
    odometry: m.Odometry,
    steering: Union[m.SteeringReport, float, None],
    accel: Union[m.AccelWithCovarianceStamped, float, None],
    operation_mode: Union[m.OperationModeState, bool, None],
    stamp: TimeLike,
) -> m.InputData:
    if steering is None:
        steering = 0.0
    if not isinstance(steering, m.SteeringReport):
        steering = make_steering_report(float(steering), stamp)
    if accel is None:
        accel = 0.0
    if not isinstance(accel, m.AccelWithCovarianceStamped):
        accel = make_accel(float(accel), stamp)
    if operation_mode is None:
        operation_mode = True
    if not isinstance(operation_mode, m.OperationModeState):
        operation_mode = make_operation_mode(bool(operation_mode), stamp)
    return m.InputData(
        trajectory=trajectory,
        odometry=odometry,
        steering=steering,
        accel=accel,
        operation_mode=operation_mode,
    )


def _apply_time(host: Any, time: TimeLike) -> None:
    if time is None:
        return
    host.time = time.to_seconds() if isinstance(time, m.Time) else float(time)


class TrajectoryFollower(m.TrajectoryFollower):
    """Lateral + longitudinal controller, the equivalent of ``trajectory_follower_node``.

    Example::

        follower = TrajectoryFollower(lateral="mpc", longitudinal="pid")
        result = follower.control(time=t, trajectory=traj, odometry=odom, steering=0.0)
        if result:
            steer = result.control_cmd.lateral.steering_tire_angle
            acc = result.control_cmd.longitudinal.acceleration
    """

    def __init__(
        self,
        lateral: str = "mpc",
        longitudinal: str = "pid",
        parameters: Optional[Mapping[str, Any]] = None,
    ) -> None:
        super().__init__(default_parameters(lateral, longitudinal, parameters))

    def control(
        self,
        trajectory: m.Trajectory,
        odometry: m.Odometry,
        steering: Union[m.SteeringReport, float, None] = None,
        accel: Union[m.AccelWithCovarianceStamped, float, None] = None,
        operation_mode: Union[m.OperationModeState, bool, None] = None,
        time: TimeLike = None,
    ) -> m.ControlResult:
        """Run one control cycle.

        ``steering`` / ``accel`` may be messages or plain floats [rad] / [m/s^2];
        ``operation_mode`` may be a message or a bool (True = autonomous).
        ``time`` [s] sets the controller clock before the cycle.
        """
        _apply_time(self, time)
        return self.step(_input_data(trajectory, odometry, steering, accel, operation_mode, time))


class LateralController(m.LateralController):
    """A single lateral controller (``"mpc"`` or ``"pure_pursuit"``)."""

    def __init__(self, mode: str = "mpc", parameters: Optional[Mapping[str, Any]] = None) -> None:
        super().__init__(mode, default_parameters(mode, None, parameters))

    def control(
        self,
        trajectory: m.Trajectory,
        odometry: m.Odometry,
        steering: Union[m.SteeringReport, float, None] = None,
        operation_mode: Union[m.OperationModeState, bool, None] = None,
        time: TimeLike = None,
    ) -> Optional[m.LateralOutput]:
        """Run one cycle; returns None when the controller is not ready."""
        _apply_time(self, time)
        data = _input_data(trajectory, odometry, steering, None, operation_mode, time)
        if not self.is_ready(data):
            return None
        return self.run(data)


class LongitudinalController(m.LongitudinalController):
    """A single longitudinal controller (``"pid"``)."""

    def __init__(self, mode: str = "pid", parameters: Optional[Mapping[str, Any]] = None) -> None:
        super().__init__(mode, default_parameters(None, mode, parameters))

    def control(
        self,
        trajectory: m.Trajectory,
        odometry: m.Odometry,
        accel: Union[m.AccelWithCovarianceStamped, float, None] = None,
        operation_mode: Union[m.OperationModeState, bool, None] = None,
        steering: Union[m.SteeringReport, float, None] = None,
        time: TimeLike = None,
    ) -> Optional[m.LongitudinalOutput]:
        """Run one cycle; returns None when the controller is not ready."""
        _apply_time(self, time)
        data = _input_data(trajectory, odometry, steering, accel, operation_mode, time)
        if not self.is_ready(data):
            return None
        return self.run(data)
