import numpy as np
import pytest

import autoware_trajectory_controller as atc


@pytest.fixture(autouse=True)
def quiet_logs():
    atc.set_log_level("fatal")
    yield
    atc.set_log_handler(None)
    atc.set_log_level("info")


def sine_path(length=200.0, amplitude=5.0, wavelength=20.0, speed=8.0, ds=1.0, decel=0.5):
    """Sine shaped path ending with a stop (constant deceleration ``decel`` [m/s^2])."""
    x = np.arange(0.0, length, ds)
    y = amplitude * np.sin(x / wavelength)
    stop_x = x[-5]
    v = np.minimum(speed, np.sqrt(np.maximum(0.0, 2.0 * decel * (stop_x - x))))
    return x, y, v


def lateral_error(x_ref, y_ref, px, py):
    """Distance from (px, py) to the reference polyline."""
    best = np.inf
    for i in range(len(x_ref) - 1):
        a = np.array([x_ref[i], y_ref[i]])
        b = np.array([x_ref[i + 1], y_ref[i + 1]])
        p = np.array([px, py])
        t = np.clip(np.dot(p - a, b - a) / np.dot(b - a, b - a), 0.0, 1.0)
        best = min(best, float(np.linalg.norm(a + t * (b - a) - p)))
    return best


def run_closed_loop(follower, traj, sim, duration):
    log = []
    for _ in range(int(duration / follower.ctrl_period)):
        res = follower.control(
            trajectory=traj,
            odometry=sim.odometry(),
            steering=sim.steering_report(),
            accel=sim.accel(),
            time=sim.time,
        )
        sim.step(res.control_cmd if res else None)
        s = sim.state
        log.append((sim.time, s.x, s.y, s.yaw, s.velocity, bool(res)))
    return np.array(log)
