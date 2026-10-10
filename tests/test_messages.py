import copy
import math

import pytest

import autoware_trajectory_controller as atc
from autoware_trajectory_controller import msg


def test_default_values():
    q = msg.Quaternion()
    assert (q.x, q.y, q.z, q.w) == (0.0, 0.0, 0.0, 1.0)
    p = msg.TrajectoryPoint()
    assert p.longitudinal_velocity_mps == 0.0
    assert p.pose.orientation.w == 1.0


def test_keyword_constructor_and_nested_assignment():
    pose = msg.Pose(position=msg.Point(x=1.0, y=2.0), orientation=atc.yaw_to_quaternion(0.5))
    assert pose.position.x == 1.0 and pose.position.z == 0.0
    assert math.isclose(atc.quaternion_to_yaw(pose.orientation), 0.5)
    odom = msg.Odometry()
    odom.pose.pose.position.x = 3.0  # nested attributes are references, like in C++
    assert odom.pose.pose.position.x == 3.0
    with pytest.raises(TypeError):
        msg.Point(1.0, 2.0)  # keyword-only


def test_equality_copy_and_repr():
    a = msg.Point(x=1.0, y=2.0, z=3.0)
    b = copy.deepcopy(a)
    assert a == b
    b.x = 0.0
    assert a != b
    assert repr(a) == "Point(x=1.0, y=2.0, z=3.0)"


def test_constants():
    assert msg.OperationModeState.AUTONOMOUS == 2
    assert msg.DiagnosticStatus.ERROR == 2


def test_time_conversion():
    t = msg.Time.from_seconds(12.25)
    assert (t.sec, t.nanosec) == (12, 250_000_000)
    assert t.to_seconds() == pytest.approx(12.25)
    d = msg.Duration.from_seconds(-0.5)
    assert d.to_seconds() == pytest.approx(-0.5)


def test_make_trajectory_roundtrip():
    traj = atc.make_trajectory([0, 1, 2, 3], [0, 0, 0, 0], 2.0, stamp=1.5)
    assert len(traj.points) == 4
    assert traj.header.stamp.to_seconds() == pytest.approx(1.5)
    arrays = atc.trajectory_to_arrays(traj)
    assert arrays["velocity"].tolist() == [2.0] * 4
    assert arrays["time_from_start"].tolist() == pytest.approx([0.0, 0.5, 1.0, 1.5])
    assert arrays["yaw"].tolist() == pytest.approx([0.0] * 4)
