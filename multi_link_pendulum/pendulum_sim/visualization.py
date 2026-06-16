"""Real-time visualization with Rerun.

Rerun is a logging-based viewer: you *log* primitives (line strips, points,
scalars) to named "entity paths" on a timeline, and the viewer renders and
scrubs them. That model fits a simulator perfectly — we just log the arm and a
few derived signals every step.

We log three things:
  * ``world/arm``      — the links as a 2D line strip + joint points (the swing).
  * ``plots/theta_*``  — per-joint angle time series.
  * ``phase/joint_*``  — phase portrait points (theta vs omega), the classic way
                         to *see* chaos / limit cycles in a pendulum.

If ``rerun`` is not installed the class degrades to a no-op so headless data
collection still runs. Install with ``uv pip install rerun-sdk``.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from .simulator import NLinkPendulum, PendulumState

try:
    import rerun as rr

    _HAVE_RERUN = True
except ImportError:  # pragma: no cover - optional dependency
    _HAVE_RERUN = False


class RerunVisualizer:
    """Stream a pendulum simulation to the Rerun viewer in real time."""

    def __init__(
        self,
        sim: NLinkPendulum,
        app_id: str = "multi_link_pendulum",
        spawn: bool = True,
        show_observation: bool = True,
    ):
        self.sim = sim
        self.enabled = _HAVE_RERUN
        self.show_observation = show_observation
        if not self.enabled:
            print("[viz] rerun-sdk not installed; visualization disabled.")
            return

        rr.init(app_id, spawn=spawn)
        # A static annotation so links render in a pleasant color.
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_UP, static=True)

    # ------------------------------------------------------------------ frame
    def log(self, state: PendulumState, obs: Optional[dict] = None) -> None:
        """Log one simulation frame at the state's own timestamp."""
        if not self.enabled:
            return

        # Put every logged value on a shared "sim_time" timeline so the viewer's
        # scrubber maps to physical seconds.
        rr.set_time_seconds("sim_time", state.t)

        pts = state.positions  # (n+1, 2)
        # The arm: a single connected polyline from anchor to tip.
        rr.log(
            "world/arm/links",
            rr.LineStrips2D([pts], colors=[(30, 144, 255)], radii=0.02),
        )
        rr.log(
            "world/arm/joints",
            rr.Points2D(pts, colors=[(255, 215, 0)], radii=0.05),
        )

        # Per-joint angle + velocity time series (one scalar entity each).
        for i in range(self.sim.cfg.n_links):
            rr.log(f"plots/theta/joint_{i}", rr.Scalar(float(state.theta[i])))
            rr.log(f"plots/omega/joint_{i}", rr.Scalar(float(state.omega[i])))
            # Phase portrait: a point per step at (theta, omega).
            rr.log(
                f"phase/joint_{i}",
                rr.Points2D(
                    [[float(state.theta[i]), float(state.omega[i])]],
                    radii=0.01,
                ),
            )

        # Overlay the noisy observed arm (semi-transparent) for a ground-truth
        # vs observation visual comparison.
        if self.show_observation and obs is not None:
            obs_pts = self.sim.link_positions(np.asarray(obs["theta"]))
            rr.log(
                "world/arm_observed/links",
                rr.LineStrips2D([obs_pts], colors=[(255, 80, 80, 140)], radii=0.012),
            )

        # Energy trace — should be ~flat for a passive pendulum; a good sanity
        # check that the integrator is behaving.
        rr.log("plots/energy", rr.Scalar(state.energy))
