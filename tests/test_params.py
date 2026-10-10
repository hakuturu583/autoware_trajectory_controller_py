import pytest

import autoware_trajectory_controller as atc
from autoware_trajectory_controller import params


def test_default_parameters_contain_every_package():
    p = atc.default_parameters("mpc", "pid")
    assert p["lateral_controller_mode"] == "mpc"
    assert p["longitudinal_controller_mode"] == "pid"
    assert p["ctrl_period"] == pytest.approx(0.03)
    assert p["wheel_base"] == pytest.approx(2.74)
    assert p["qp_solver_type"] == "osqp"
    assert p["kp"] == pytest.approx(1.0)
    assert p["steering_offset.enable_auto_steering_offset_removal"] is True


def test_overrides_are_flattened():
    p = atc.default_parameters(
        "pure_pursuit", "pid", {"steering_offset": {"max_update_th": 0.5}, "kp": 2.0}
    )
    assert p["steering_offset.max_update_th"] == 0.5
    assert p["kp"] == 2.0
    assert "mpc_prediction_horizon" not in p


def test_invalid_mode():
    with pytest.raises(ValueError):
        atc.default_parameters("stanley", "pid")


def test_load_ros_param_file(tmp_path):
    f = tmp_path / "x.param.yaml"
    f.write_text("/**:\n  ros__parameters:\n    a: 1\n    b:\n      c: [1.0, 2.0]\n")
    assert params.load_param_file(f) == {"a": 1, "b.c": [1.0, 2.0]}


def test_missing_parameter_is_reported():
    p = atc.default_parameters("mpc", "pid")
    del p["mpc_prediction_horizon"]
    with pytest.raises(RuntimeError, match="mpc_prediction_horizon"):
        atc._core.TrajectoryFollower(p)
