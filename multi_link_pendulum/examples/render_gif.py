"""Headless visual: render the pendulum swinging to an animated GIF.

Unlike ``run_visual_demo`` (which needs the interactive Rerun viewer), this
produces a shareable file with just matplotlib — handy for README art, remote
machines, or pasting into a chat. It draws the arm plus a fading trail of the
tip, which is the classic way to *see* a multi-link pendulum's chaos.

    python -m examples.render_gif --links 2 --duration 8 --out pendulum.gif

matplotlib is an optional extra:  uv pip install matplotlib
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import typer
import matplotlib

matplotlib.use("Agg")  # headless backend: render to file, no display needed
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

from pendulum_sim import NLinkPendulum, PendulumConfig

app = typer.Typer(add_completion=False, help="Render a pendulum swing to GIF.")


@app.command()
def main(
    links: int = typer.Option(2, help="Number of links."),
    duration: float = typer.Option(8.0, help="Simulated seconds to render."),
    fps: int = typer.Option(30, help="Output frames per second."),
    damping: float = typer.Option(0.0, help="Joint friction (0 = undamped chaos)."),
    out: Path = typer.Option(Path("pendulum.gif"), help="Output GIF path."),
):
    dt = 1.0 / fps
    cfg = PendulumConfig(
        n_links=links,
        masses=[1.0] * links,
        lengths=[1.0 / links] * links,  # total reach ~1 m
        damping=[damping] * links,
        dt=dt,
    )
    sim = NLinkPendulum(cfg)
    # Slightly asymmetric horizontal start -> lively, clearly chaotic motion.
    sim.reset(theta0=np.full(links, np.pi / 2) + np.linspace(0, 0.1, links))

    # Pre-roll the whole trajectory so the animation just replays cached frames.
    n_frames = int(duration * fps)
    frames = []
    for _ in range(n_frames):
        sim.step()
        frames.append(sim.link_positions().copy())

    reach = float(np.sum(cfg.lengths)) * 1.1
    fig, ax = plt.subplots(figsize=(5, 5))
    ax.set_xlim(-reach, reach)
    ax.set_ylim(-reach, reach)
    ax.set_aspect("equal")
    ax.set_title(f"{links}-link pendulum")
    ax.grid(alpha=0.2)

    (arm_line,) = ax.plot([], [], "o-", lw=2, color="#1e90ff", markersize=6)
    (trail_line,) = ax.plot([], [], "-", lw=1, color="#ff5050", alpha=0.6)
    tip_trail_x: list[float] = []
    tip_trail_y: list[float] = []
    trail_len = fps * 2  # ~2 seconds of fading tip history

    def update(i: int):
        pts = frames[i]
        arm_line.set_data(pts[:, 0], pts[:, 1])
        tip_trail_x.append(pts[-1, 0])
        tip_trail_y.append(pts[-1, 1])
        if len(tip_trail_x) > trail_len:
            del tip_trail_x[0], tip_trail_y[0]
        trail_line.set_data(tip_trail_x, tip_trail_y)
        return arm_line, trail_line

    anim = FuncAnimation(fig, update, frames=n_frames, interval=1000 / fps, blit=True)
    anim.save(out, writer=PillowWriter(fps=fps))
    plt.close(fig)
    typer.echo(f"Wrote {n_frames} frames -> {out}")


if __name__ == "__main__":
    app()
