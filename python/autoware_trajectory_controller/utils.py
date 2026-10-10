"""Helpers to build controller inputs from plain numbers / numpy arrays."""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Union

import numpy as np

from . import _core as m

ArrayLike = Union[Sequence[float], np.ndarray]


def yaw_to_quaternion(yaw: float) -> m.Quaternion:
    return m.Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


def quaternion_to_yaw(q: m.Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def make_pose(x: float, y: float, yaw: float = 0.0, z: float = 0.0) -> m.Pose:
    return m.Pose(position=m.Point(x=x, y=y, z=z), orientation=yaw_to_quaternion(yaw))


def to_time(t: float | m.Time | None) -> m.Time:
    if t is None:
        return m.Time()
    if isinstance(t, m.Time):
        return t
    return m.Time.from_seconds(float(t))


def make_trajectory(
    x: ArrayLike,
    y: ArrayLike,
    velocity: ArrayLike | float,
    yaw: ArrayLike | None = None,
    acceleration: ArrayLike | float | None = None,
    z: ArrayLike | None = None,
    time_from_start: ArrayLike | None = None,
    stamp: float | m.Time | None = None,
    frame_id: str = "map",
) -> m.Trajectory:
    """Build an ``autoware_planning_msgs/Trajectory``.

    ``yaw`` defaults to the path tangent (``arctan2`` of the point differences) and
    ``time_from_start`` defaults to the time obtained by integrating ``velocity``.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    n = len(x)
    if len(y) != n:
        raise ValueError("x and y must have the same length")
    v = np.broadcast_to(np.asarray(velocity, dtype=float), (n,))
    acc = np.broadcast_to(np.asarray(0.0 if acceleration is None else acceleration, float), (n,))
    zz = np.zeros(n) if z is None else np.asarray(z, dtype=float)
    if yaw is None:
        if n < 2:
            yaw_arr = np.zeros(n)
        else:
            dx = np.gradient(x)
            dy = np.gradient(y)
            yaw_arr = np.arctan2(dy, dx)
    else:
        yaw_arr = np.asarray(yaw, dtype=float)
    if time_from_start is None:
        ds = np.hypot(np.diff(x), np.diff(y))
        v_mid = np.maximum(0.5 * (np.abs(v[1:]) + np.abs(v[:-1])), 1e-3)
        t_arr = np.concatenate([[0.0], np.cumsum(ds / v_mid)])
    else:
        t_arr = np.asarray(time_from_start, dtype=float)

    points = []
    for i in range(n):
        points.append(
            m.TrajectoryPoint(
                time_from_start=m.Duration.from_seconds(float(t_arr[i])),
                pose=make_pose(float(x[i]), float(y[i]), float(yaw_arr[i]), float(zz[i])),
                longitudinal_velocity_mps=float(v[i]),
                acceleration_mps2=float(acc[i]),
            )
        )
    traj = m.Trajectory()
    traj.header = m.Header(stamp=to_time(stamp), frame_id=frame_id)
    traj.points = points
    return traj


def trajectory_to_arrays(traj: m.Trajectory) -> dict[str, np.ndarray]:
    """Convert a Trajectory into a dict of numpy arrays (x, y, z, yaw, velocity, ...)."""
    pts = traj.points
    return {
        "x": np.array([p.pose.position.x for p in pts]),
        "y": np.array([p.pose.position.y for p in pts]),
        "z": np.array([p.pose.position.z for p in pts]),
        "yaw": np.array([quaternion_to_yaw(p.pose.orientation) for p in pts]),
        "velocity": np.array([p.longitudinal_velocity_mps for p in pts]),
        "acceleration": np.array([p.acceleration_mps2 for p in pts]),
        "front_wheel_angle": np.array([p.front_wheel_angle_rad for p in pts]),
        "time_from_start": np.array([p.time_from_start.to_seconds() for p in pts]),
    }


def make_odometry(
    x: float,
    y: float,
    yaw: float,
    velocity: float,
    yaw_rate: float = 0.0,
    lateral_velocity: float = 0.0,
    z: float = 0.0,
    stamp: float | m.Time | None = None,
    frame_id: str = "map",
) -> m.Odometry:
    odom = m.Odometry()
    odom.header = m.Header(stamp=to_time(stamp), frame_id=frame_id)
    odom.child_frame_id = "base_link"
    odom.pose = m.PoseWithCovariance(pose=make_pose(x, y, yaw, z))
    odom.twist = m.TwistWithCovariance(
        twist=m.Twist(
            linear=m.Vector3(x=velocity, y=lateral_velocity, z=0.0),
            angular=m.Vector3(x=0.0, y=0.0, z=yaw_rate),
        )
    )
    return odom


def make_steering_report(angle: float, stamp: float | m.Time | None = None) -> m.SteeringReport:
    return m.SteeringReport(stamp=to_time(stamp), steering_tire_angle=angle)


def make_accel(
    acceleration: float, stamp: float | m.Time | None = None, frame_id: str = "base_link"
) -> m.AccelWithCovarianceStamped:
    msg = m.AccelWithCovarianceStamped()
    msg.header = m.Header(stamp=to_time(stamp), frame_id=frame_id)
    msg.accel = m.AccelWithCovariance(
        accel=m.Accel(linear=m.Vector3(x=acceleration, y=0.0, z=0.0), angular=m.Vector3())
    )
    return msg


def make_operation_mode(
    autonomous: bool = True, stamp: float | m.Time | None = None
) -> m.OperationModeState:
    """Operation mode state. Controllers only act while in AUTONOMOUS + control enabled."""
    state = m.OperationModeState(
        stamp=to_time(stamp),
        mode=m.OperationModeState.AUTONOMOUS if autonomous else m.OperationModeState.STOP,
        is_autoware_control_enabled=autonomous,
        is_autonomous_mode_available=True,
        is_stop_mode_available=True,
    )
    return state
