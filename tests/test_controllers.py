import numpy as np
import pytest
from conftest import sine_path

import autoware_trajectory_controller as atc
from autoware_trajectory_controller import msg


@pytest.fixture
def traj():
    x, y, v = sine_path()
    return atc.make_trajectory(x, y, v)


@pytest.mark.parametrize("mode", ["mpc", "pure_pursuit"])
def test_standalone_lateral_controller_steers_back_to_path(mode):
    straight = atc.make_trajectory(np.arange(0.0, 100.0), np.zeros(100), 5.0)
    lat = atc.LateralController(mode)
    odom = atc.make_odometry(10.0, 0.5, 0.0, 5.0)  # 0.5 m left of the path
    for k in range(20):
        out = lat.control(straight, odom, steering=0.0, time=0.03 * k)
    assert isinstance(out, atc.LateralOutput)
    assert out.control_cmd.steering_tire_angle < 0.0  # steer right
    assert out.control_cmd.stamp.to_seconds() == pytest.approx(0.03 * 19)


def test_standalone_longitudinal_controller_accelerates(traj):
    lon = atc.LongitudinalController("pid")
    # The PID controller stays stopped until the lateral controller reports a converged
    # steering (enable_keep_stopped_until_steer_convergence); emulate that report.
    sync = atc.LateralSyncData()
    sync.is_steer_converged = True
    lon.sync(sync)
    odom = atc.make_odometry(10.0, 5 * np.sin(0.5), np.arctan(0.25 * np.cos(0.5)), 0.0)
    for k in range(10):
        out = lon.control(traj, odom, accel=0.0, time=0.03 * k)
    assert isinstance(out, atc.LongitudinalOutput)
    assert out.control_cmd.velocity == pytest.approx(8.0)
    assert out.control_cmd.acceleration > 0.0


def test_not_ready_without_trajectory():
    lat = atc.LateralController("mpc")
    assert lat.control(msg.Trajectory(), atc.make_odometry(0, 0, 0, 0), time=0.0) is None


def test_parameters_roundtrip(traj):
    follower = atc.TrajectoryFollower(parameters={"kp": 1.5})
    assert follower.get_parameter("kp") == 1.5
    follower.set_parameters({"kp": 0.5, "mpc_weight_lat_error": 2.0})
    params = follower.get_parameters()
    assert params["kp"] == 0.5
    assert params["mpc_weight_lat_error"] == 2.0
    with pytest.raises(ValueError, match="not been declared"):
        follower.set_parameters({"does_not_exist": 1.0})


def test_debug_topics_and_diagnostics(traj):
    follower = atc.TrajectoryFollower()
    sim = atc.SimpleVehicleSimulator(dt=follower.ctrl_period)
    res = follower.control(traj, sim.odometry(), sim.steering_report(), sim.accel(), time=0.0)
    assert res.is_published
    predicted = follower.get_debug("~/output/predicted_trajectory")
    assert isinstance(predicted, msg.Trajectory) and len(predicted.points) > 0
    assert isinstance(follower.get_debug("~/output/lateral_diagnostic"), msg.Float32MultiArrayStamped)
    assert follower.get_debug("~/unknown") is None
    status = {d.name: d for d in follower.update_diagnostics()}
    assert status["MPC_solve_checker"].level == msg.DiagnosticStatus.OK


def test_log_handler_receives_messages(traj):
    records = []
    atc.set_log_level("debug")
    atc.set_log_handler(lambda level, name, text: records.append((level, name, text)))
    follower = atc.TrajectoryFollower()
    # An empty trajectory makes the controllers complain.
    follower.control(msg.Trajectory(), atc.make_odometry(0, 0, 0, 0), time=0.0)
    follower.control(traj, atc.make_odometry(0, 0, 0, 0), time=0.03)
    assert records
    assert all(isinstance(level, int) for level, _, _ in records)


def test_simulated_time_drives_stamps(traj):
    follower = atc.TrajectoryFollower()
    follower.time = 123.5
    assert follower.time_ns == 123_500_000_000
    res = follower.control(traj, atc.make_odometry(0, 0, 0, 0), time=200.0)
    assert res.control_cmd.stamp.to_seconds() == pytest.approx(200.0)
