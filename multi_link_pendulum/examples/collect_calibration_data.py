"""Collect a dataset of episodes and write RuVector-ready JSONL.

Run from the project root::

    python -m examples.collect_calibration_data --episodes 5 --links 2 --noise 0.03
    python -m examples.collect_calibration_data --config configs/default.yaml

Each episode is a fresh random drop. Every timestep becomes one JSONL record
containing ground truth, the noisy observation, the residual, a flat embedding
vector, and the link/joint graph — i.e. everything you need to push into a
vector index *and* a GNN. See ``pendulum_sim/data_logger.py`` for the schema.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
import typer

from pendulum_sim import (
    NLinkPendulum,
    PendulumConfig,
    SensorNoiseModel,
    NoiseConfig,
    JSONLLogger,
    EpisodeMetadata,
)

app = typer.Typer(add_completion=False, help="Generate RuVector-ready JSONL data.")


def _random_action(rng: np.random.Generator, n: int, scale: float) -> np.ndarray:
    """A simple exploratory policy: small random torque bursts per joint."""
    return rng.normal(0.0, scale, size=n)


@app.command()
def main(
    episodes: int = typer.Option(5, help="Number of episodes to collect."),
    links: int = typer.Option(2, help="Number of links."),
    steps: int = typer.Option(1000, help="Timesteps per episode."),
    noise: float = typer.Option(0.03, help="Angle noise std [rad]."),
    bias: float = typer.Option(0.03, help="Hidden encoder bias to embed [rad]."),
    mode: str = typer.Option("actuated", help="'passive' or 'actuated'."),
    action_scale: float = typer.Option(2.0, help="Torque std for random actions."),
    out: Path = typer.Option(Path("data/calibration.jsonl"), help="Output JSONL path."),
    seed: int = typer.Option(0, help="Base RNG seed."),
):
    out.parent.mkdir(parents=True, exist_ok=True)
    # We open one logger per file but write all episodes into it (append-style),
    # so re-open in append mode across episodes by reusing a single file handle.
    total_records = 0

    # Truncate/create the file once, then append per-episode via fresh loggers
    # that share the same path. (JSONL is line-oriented, so concatenation is safe.)
    if out.exists():
        out.unlink()

    for ep in range(episodes):
        rng = np.random.default_rng(seed + ep)
        cfg = PendulumConfig(
            n_links=links,
            masses=[1.0] * links,
            lengths=[1.0 / links] * links,
            damping=[0.05] * links,
            dt=0.01,
        )
        sim = NLinkPendulum(cfg)
        # Random initial condition per episode for dataset diversity.
        sim.reset(theta0=rng.uniform(-np.pi, np.pi, links), omega0=np.zeros(links))

        noise_model = SensorNoiseModel(
            NoiseConfig(
                angle_std=noise,
                velocity_std=noise * 2,
                angle_bias=bias,
                seed=seed + ep,
            ),
            n_links=links,
        )

        meta = EpisodeMetadata(
            episode_id=f"ep_{ep:04d}",
            n_links=links,
            mode=mode,
            masses=list(cfg.masses),
            lengths=list(cfg.lengths),
            damping=list(cfg.damping),
            gravity=cfg.gravity,
            dt=cfg.dt,
            noise_level=noise,
            extra={"true_angle_bias": noise_model.true_bias.tolist(), "seed": seed + ep},
        )

        # First episode truncates the file; later ones append to it.
        with JSONLLogger(out, meta, append=(ep > 0)) as logger:
            for _ in range(steps):
                tau = _random_action(rng, links, action_scale) if mode == "actuated" else None
                state = sim.step(tau=tau)
                obs = noise_model.observe(state.theta, state.omega, state.tau)
                logger.log_step(sim, state, obs)
                total_records += 1

    typer.echo(
        f"Wrote {total_records} step records across {episodes} episodes -> {out}\n"
        f"Ingest into RuVector by reading each line; use record['vector']['embedding'] "
        f"as the vector and record['graph'] for GNN message passing."
    )


if __name__ == "__main__":
    app()
