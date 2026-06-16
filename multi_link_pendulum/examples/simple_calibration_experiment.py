"""A minimal *calibration* experiment: recover hidden parameters from noisy data.

This is the conceptual core of the RuVector use case. We:

  1. Build a "true" pendulum with parameters the calibrator is NOT told.
  2. Generate a noisy observed trajectory (imperfect sensors).
  3. Ask an optimizer to recover a chosen parameter (link length OR joint
     friction) by minimizing the mismatch between the model's prediction and
     the observations — i.e. classic system identification.

Run from the project root::

    python -m examples.simple_calibration_experiment --target length
    python -m examples.simple_calibration_experiment --target damping --links 2

Where RuVector fits
-------------------
Here we use a plain SciPy optimizer over a single scalar. In the real workflow
RuVector's vector store + GNN memory replaces/augments this step:
  * The vector DB retrieves *past episodes with similar dynamics* to warm-start
    the estimate (so calibration converges from prior experience, not scratch).
  * The GNN consumes the link/joint graph (see ``state_to_graph``) and predicts
    a per-joint correction, generalizing across different link counts.
The residual signal we minimize below is exactly the supervision target such a
learned calibrator would be trained on.
"""

from __future__ import annotations

import numpy as np
import typer
from scipy.optimize import minimize_scalar

from pendulum_sim import (
    NLinkPendulum,
    PendulumConfig,
    SensorNoiseModel,
    NoiseConfig,
)

app = typer.Typer(add_completion=False, help="Recover hidden params from noisy data.")


def simulate_trajectory(cfg: PendulumConfig, theta0, omega0, steps: int) -> np.ndarray:
    """Roll out a passive trajectory and return the stacked angles, shape (T, n)."""
    sim = NLinkPendulum(cfg)
    sim.reset(theta0=theta0, omega0=omega0)
    traj = np.empty((steps, cfg.n_links))
    for t in range(steps):
        state = sim.step()
        traj[t] = state.theta
    return traj


@app.command()
def main(
    target: str = typer.Option("length", help="Parameter to recover: 'length' or 'damping'."),
    links: int = typer.Option(2, help="Number of links."),
    steps: int = typer.Option(400, help="Trajectory length in steps."),
    noise: float = typer.Option(0.01, help="Observation angle noise std [rad]."),
    seed: int = typer.Option(0, help="RNG seed."),
):
    rng = np.random.default_rng(seed)
    theta0 = rng.uniform(-1.0, 1.0, links)
    omega0 = np.zeros(links)

    # ---- 1. Ground truth (hidden from the calibrator) --------------------
    true_length = 1.0
    true_damping = 0.15
    true_cfg = PendulumConfig(
        n_links=links,
        masses=[1.0] * links,
        lengths=[true_length] * links,
        damping=[true_damping] * links,
        dt=0.01,
    )

    # ---- 2. Noisy observed trajectory ------------------------------------
    clean = simulate_trajectory(true_cfg, theta0, omega0, steps)
    noise_model = SensorNoiseModel(NoiseConfig(angle_std=noise, seed=seed), n_links=links)
    observed = np.array([noise_model.observe(th, np.zeros(links), np.zeros(links))["theta"]
                         for th in clean])

    # ---- 3. Define the loss: simulate with a guessed param, compare ------
    def make_cfg(guess: float) -> PendulumConfig:
        if target == "length":
            return PendulumConfig(n_links=links, masses=[1.0] * links,
                                  lengths=[guess] * links, damping=[true_damping] * links, dt=0.01)
        elif target == "damping":
            return PendulumConfig(n_links=links, masses=[1.0] * links,
                                  lengths=[true_length] * links, damping=[guess] * links, dt=0.01)
        raise typer.BadParameter("target must be 'length' or 'damping'")

    def loss(guess: float) -> float:
        # Mean-squared angle error between predicted and observed trajectories.
        # Chaotic systems diverge over long horizons, so a short window keeps the
        # loss informative — a real pipeline would use multiple short segments.
        pred = simulate_trajectory(make_cfg(float(guess)), theta0, omega0, steps)
        horizon = min(steps, 150)
        return float(np.mean((pred[:horizon] - observed[:horizon]) ** 2))

    truth = true_length if target == "length" else true_damping
    bounds = (0.3, 2.0) if target == "length" else (0.0, 1.0)
    typer.echo(f"Recovering '{target}' (true value = {truth}) from {links}-link noisy data...")

    result = minimize_scalar(loss, bounds=bounds, method="bounded")
    est = result.x
    err = abs(est - truth)
    typer.echo(
        f"  estimated {target} = {est:.4f}\n"
        f"  true      {target} = {truth:.4f}\n"
        f"  abs error           = {err:.4f}  ({100 * err / truth:.1f}% of true)\n"
        f"  final loss          = {result.fun:.2e}"
    )
    typer.echo(
        "This 1-D optimizer is the baseline RuVector's GNN-memory calibration "
        "would learn to beat by reusing experience across many such episodes."
    )


if __name__ == "__main__":
    app()
