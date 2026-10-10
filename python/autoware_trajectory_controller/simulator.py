"""A small vehicle simulator to close the loop around the controllers.

It follows the "DELAY_STEER_ACC" model of Autoware's ``simple_planning_simulator``
in spirit: kinematic bicycle model, first order steering and acceleration dynamics
and pure input dead time. It is meant for examples and tests, not for vehicle
dynamics studies.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

from . import _core as m
from .utils import make_accel, make_odometry, make_steering_report


@dataclass
class VehicleState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    velocity: float = 0.0
    acceleration: float = 0.0
    steering: float = 0.0


@dataclass
class SimpleVehicleSimulator:
    """Kinematic bicycle model with first-order steering / acceleration lag.

    Args:
        wheel_base: [m]
        steer_time_constant: first order lag of the steering [s]
        acc_time_constant: first order lag of the acceleration [s]
        steer_delay: steering dead time [s]
        acc_delay: acceleration dead time [s]
        max_steer: steering limit [rad]
        steer_bias: constant offset added to the actual steering angle [rad]
        dt: integration step [s]
    """

    wheel_base: float = 2.74
    steer_time_constant: float = 0.27
    acc_time_constant: float = 0.1
    steer_delay: float = 0.0
    acc_delay: float = 0.0
    max_steer: float = 0.7
    steer_bias: float = 0.0
    dt: float = 0.03
    state: VehicleState = field(default_factory=VehicleState)
    time: float = 0.0

    def __post_init__(self) -> None:
        self._reset_inputs()

    def _reset_inputs(self) -> None:
        self._steer_cmd = 0.0
        self._acc_cmd = 0.0
        self._steer_buffer: deque[float] = deque(
            [0.0] * max(int(round(self.steer_delay / self.dt)), 0)
        )
        self._acc_buffer: deque[float] = deque([0.0] * max(int(round(self.acc_delay / self.dt)), 0))

    def reset(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, velocity: float = 0.0) -> None:
        self.state = VehicleState(x=x, y=y, yaw=yaw, velocity=velocity)
        self._reset_inputs()

    @staticmethod
    def _delay(buffer: deque[float], value: float) -> float:
        if not buffer:
            return value
        buffer.append(value)
        return buffer.popleft()

    def step(self, control: m.Control | None, dt: float | None = None) -> VehicleState:
        """Advance the simulation by ``dt`` applying ``control`` (None = keep previous input)."""
        dt = self.dt if dt is None else dt
        s = self.state
        if control is not None:
            self._steer_cmd = float(control.lateral.steering_tire_angle)
            self._acc_cmd = float(control.longitudinal.acceleration)
        steer_cmd = self._delay(self._steer_buffer, self._steer_cmd)
        acc_cmd = self._delay(self._acc_buffer, self._acc_cmd)
        steer_cmd = max(-self.max_steer, min(self.max_steer, steer_cmd))

        s.steering += (steer_cmd - s.steering) * min(dt / self.steer_time_constant, 1.0)
        s.acceleration += (acc_cmd - s.acceleration) * min(dt / self.acc_time_constant, 1.0)

        v_prev = s.velocity
        s.velocity += s.acceleration * dt
        # The vehicle does not start reversing because of braking.
        if v_prev >= 0.0 and s.velocity < 0.0 and acc_cmd <= 0.0:
            s.velocity = 0.0
            s.acceleration = 0.0
        yaw_rate = self.yaw_rate
        s.x += s.velocity * math.cos(s.yaw) * dt
        s.y += s.velocity * math.sin(s.yaw) * dt
        s.yaw = math.atan2(math.sin(s.yaw + yaw_rate * dt), math.cos(s.yaw + yaw_rate * dt))
        self.time += dt
        return s

    @property
    def yaw_rate(self) -> float:
        s = self.state
        return s.velocity * math.tan(s.steering + self.steer_bias) / self.wheel_base

    def odometry(self) -> m.Odometry:
        s = self.state
        return make_odometry(s.x, s.y, s.yaw, s.velocity, yaw_rate=self.yaw_rate, stamp=self.time)

    def steering_report(self) -> m.SteeringReport:
        return make_steering_report(self.state.steering, stamp=self.time)

    def accel(self) -> m.AccelWithCovarianceStamped:
        return make_accel(self.state.acceleration, stamp=self.time)
