"""Follow a sine shaped trajectory with the Autoware controllers (no ROS 2 required).

    uv run python examples/closed_loop_simulation.py --lateral mpc
    uv run --extra plot python examples/closed_loop_simulation.py --plot
"""

from __future__ import annotations

import argparse

import numpy as np

import autoware_trajectory_controller as atc


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lateral", choices=["mpc", "pure_pursuit"], default="mpc")
    parser.add_argument("--speed", type=float, default=8.0, help="reference speed [m/s]")
    parser.add_argument("--duration", type=float, default=45.0, help="simulated time [s]")
    parser.add_argument("--plot", action="store_true", help="plot the result (needs matplotlib)")
    args = parser.parse_args()

    atc.set_log_level("warn")

    # Reference trajectory: sine shaped path, constant speed and a smooth stop at the end.
    x = np.arange(0.0, 200.0, 1.0)
    y = 5.0 * np.sin(x / 20.0)
    v = np.minimum(args.speed, np.sqrt(np.maximum(0.0, 2.0 * 0.5 * (x[-5] - x))))
    trajectory = atc.make_trajectory(x, y, v)

    # Controllers with the default Autoware parameters (override any of them here).
    follower = atc.TrajectoryFollower(
        lateral=args.lateral,
        longitudinal="pid",
        parameters={"mpc_weight_lat_error": 1.0},
    )

    # Simple vehicle model to close the loop, starting 0.5 m beside the path.
    sim = atc.SimpleVehicleSimulator(dt=follower.ctrl_period, steer_delay=0.24)
    sim.reset(x=0.0, y=0.5, yaw=0.0)

    log = []
    for _ in range(int(args.duration / follower.ctrl_period)):
        result = follower.control(
            trajectory=trajectory,
            odometry=sim.odometry(),
            steering=sim.steering_report(),
            accel=sim.accel(),
            time=sim.time,  # the controllers run on simulated time
        )
        sim.step(result.control_cmd if result else None)
        cmd = result.control_cmd
        log.append(
            (
                sim.time,
                sim.state.x,
                sim.state.y,
                sim.state.velocity,
                cmd.lateral.steering_tire_angle,
                cmd.longitudinal.acceleration,
            )
        )
    log = np.array(log)

    # Lateral error: distance to the reference polyline (projection onto every segment).
    a = np.stack([x[:-1], y[:-1]], axis=1)
    d = np.diff(np.stack([x, y], axis=1), axis=0)
    p = log[:, None, 1:3]
    t = np.clip(np.sum((p - a) * d, axis=-1) / np.sum(d * d, axis=-1), 0.0, 1.0)
    err = np.min(np.linalg.norm(a + t[..., None] * d - p, axis=-1), axis=1)
    settled = log[:, 0] > 8.0
    print(f"lateral controller : {args.lateral}")
    print(f"max lateral error  : {err[settled].max():.3f} m (after 8 s)")
    print(f"rms lateral error  : {np.sqrt(np.mean(err[settled] ** 2)):.3f} m")
    print(f"max speed          : {log[:, 3].max():.2f} m/s")
    print(f"final position     : x={sim.state.x:.2f} y={sim.state.y:.2f} v={sim.state.velocity:.2f}")
    print("diagnostics        :", [(d.name, d.message) for d in follower.update_diagnostics()])

    if args.plot:
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(3, 1, figsize=(8, 9))
        ax[0].plot(x, y, "k--", label="reference")
        ax[0].plot(log[:, 1], log[:, 2], label="vehicle")
        ax[0].set_aspect("equal")
        ax[0].legend()
        ax[1].plot(log[:, 0], log[:, 3], label="velocity [m/s]")
        ax[1].plot(log[:, 0], log[:, 5], label="acc cmd [m/s^2]")
        ax[1].legend()
        ax[2].plot(log[:, 0], log[:, 4], label="steer cmd [rad]")
        ax[2].plot(log[:, 0], err, label="lateral error [m]")
        ax[2].set_xlabel("time [s]")
        ax[2].legend()
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
