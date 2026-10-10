import numpy as np
import pytest
from conftest import lateral_error, run_closed_loop, sine_path

import autoware_trajectory_controller as atc


@pytest.mark.parametrize(
    "lateral, overrides, max_error",
    [
        ("mpc", {}, 0.1),
        ("mpc", {"qp_solver_type": "unconstraint_fast"}, 0.1),
        ("mpc", {"vehicle_model_type": "kinematics_no_delay"}, 0.15),
        ("pure_pursuit", {}, 0.35),
    ],
)
def test_tracks_sine_path_and_stops(lateral, overrides, max_error):
    x, y, v = sine_path()
    traj = atc.make_trajectory(x, y, v)
    follower = atc.TrajectoryFollower(lateral=lateral, longitudinal="pid", parameters=overrides)
    sim = atc.SimpleVehicleSimulator(dt=follower.ctrl_period, steer_delay=0.24)
    sim.reset(x=0.0, y=0.3, yaw=0.0)

    log = run_closed_loop(follower, traj, sim, duration=45.0)

    assert log[:, 5].all(), "every control cycle should publish a command"
    # Lateral tracking once the initial offset has been absorbed.
    moving = (log[:, 0] > 8.0) & (log[:, 1] < x[-10])
    errors = [lateral_error(x, y, px, py) for px, py in log[moving][::10, 1:3]]
    assert max(errors) < max_error
    # Reaches the reference speed and stops near the end of the trajectory.
    assert log[:, 4].max() == pytest.approx(8.0, abs=0.5)
    assert sim.state.velocity == pytest.approx(0.0, abs=1e-3)
    stop_error = np.hypot(sim.state.x - x[-5], sim.state.y - y[-5])
    assert stop_error < 1.0


def test_does_not_move_when_not_autonomous():
    x, y, v = sine_path()
    traj = atc.make_trajectory(x, y, v)
    follower = atc.TrajectoryFollower()
    sim = atc.SimpleVehicleSimulator(dt=follower.ctrl_period)
    for _ in range(100):
        res = follower.control(
            trajectory=traj, odometry=sim.odometry(), steering=0.0, operation_mode=False,
            time=sim.time,
        )
        sim.step(res.control_cmd)
    assert sim.state.velocity == 0.0
    assert res.control_cmd.longitudinal.acceleration < 0.0  # stopped state brake


def test_command_is_deterministic():
    x, y, v = sine_path()
    traj = atc.make_trajectory(x, y, v)
    results = []
    for _ in range(2):
        follower = atc.TrajectoryFollower()
        sim = atc.SimpleVehicleSimulator(dt=follower.ctrl_period)
        sim.reset(y=0.5)
        log = run_closed_loop(follower, traj, sim, duration=5.0)
        results.append(log)
    np.testing.assert_array_equal(results[0], results[1])
