"""Real-time visual demo.

Run from the project root (the dir containing this `examples/` folder)::

    python -m examples.run_visual_demo --links 2 --noise 0.05
    python -m examples.run_visual_demo --links 3 --mode actuated
    python -m examples.run_visual_demo --links 4 --noise 0.02 --duration 30

A Rerun window opens and the arm swings in real time, with angle/velocity time
series, a phase portrait, and (in red) the noisy "observed" arm overlaid on the
ground-truth arm.
"""

from __future__ import annotations

import time

import numpy as np
import typer

from pendulum_sim import (
    NLinkPendulum,
    PendulumConfig,
    SensorNoiseModel,
    NoiseConfig,
    RerunVisualizer,
)

app = typer.Typer(add_completion=False, help="Real-time pendulum visualization.")


def make_pd_controller(n_links: int, kp: float = 8.0, kd: float = 2.0):
    """A trivial PD controller that tries to hold every link upright (theta=pi).

    Demonstrates *actuated* mode. Holding a multi-link pendulum inverted is
    genuinely hard, so expect lively behavior — that is fine for a demo. The
    point is to show how to inject torques, not to solve the control problem.
    """
    target = np.full(n_links, np.pi)

    def controller(t: float, theta: np.ndarray, omega: np.ndarray) -> np.ndarray:
        # Wrap angle error into (-pi, pi] so the controller takes the short way.
        err = (target - theta + np.pi) % (2 * np.pi) - np.pi
        return kp * err - kd * omega

    return controller


@app.command()
def main(
    links: int = typer.Option(2, help="Number of links (>=1). Try 2, 3, or 4."),
    noise: float = typer.Option(0.05, help="Angle noise std [rad] (0 disables)."),
    mode: str = typer.Option("passive", help="'passive' or 'actuated'."),
    duration: float = typer.Option(20.0, help="Real-time seconds to run."),
    dt: float = typer.Option(0.01, help="Integration timestep [s]."),
    damping: float = typer.Option(0.02, help="Per-joint viscous friction."),
    realtime: bool = typer.Option(True, help="Sleep to play back at wall-clock speed."),
):
    cfg = PendulumConfig(
        n_links=links,
        masses=[1.0] * links,
        lengths=[1.0 / links] * links,  # keep total reach ~1 m as links grow
        damping=[damping] * links,
        dt=dt,
    )
    sim = NLinkPendulum(cfg)
    sim.reset()

    noise_model = SensorNoiseModel(
        NoiseConfig(angle_std=noise, velocity_std=noise * 2, seed=0),
        n_links=links,
    )
    viz = RerunVisualizer(sim, show_observation=noise > 0)
    controller = make_pd_controller(links) if mode == "actuated" else None

    typer.echo(
        f"Running {links}-link {mode} pendulum for {duration}s "
        f"(noise={noise}, dt={dt}). Close the Rerun window to stop early."
    )

    n_steps = int(duration / dt)
    for _ in range(n_steps):
        state = sim.step(controller=controller)
        obs = noise_model.observe(state.theta, state.omega, state.tau) if noise > 0 else None
        viz.log(state, obs)
        if realtime:
            time.sleep(dt)


if __name__ == "__main__":
    app()
