"""Sensor noise models.

Calibration is the art of recovering true physical parameters from *imperfect*
measurements. To experiment with that, we need a principled way to corrupt the
clean ground-truth state into a realistic "observation". This module separates
the *true* state (what the simulator knows) from the *observed* state (what a
real encoder / IMU would report), which is exactly the split a calibration or
GNN-memory system must reason about.

Noise sources modeled here, all common on real robot joints:
  * Gaussian white noise   — encoder quantization + electrical noise.
  * Constant bias          — a miscalibrated zero-offset (the classic thing you
                             try to *recover* during calibration).
  * Dropout                — occasional missing/held samples (sensor glitch).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class NoiseConfig:
    """Per-channel noise parameters. Stds are in the channel's native units."""

    angle_std: float = 0.02  # [rad] ~1.1 deg of jitter
    velocity_std: float = 0.05  # [rad/s]
    torque_std: float = 0.0  # [N·m] noise on commanded torque readback
    # Constant per-joint bias on the angle channel (miscalibrated encoder zero).
    # Scalar => same bias on every joint; sequence => per-joint.
    angle_bias: float = 0.0
    dropout_prob: float = 0.0  # probability a given sample is "held" (stale)
    seed: Optional[int] = None


class SensorNoiseModel:
    """Turns ground-truth :class:`PendulumState` values into noisy observations.

    Holds its own RNG so experiments are reproducible given a seed, and keeps a
    fixed per-joint ``angle_bias`` vector so the bias is *consistent across time*
    (a real miscalibration does not resample every step) — that temporal
    consistency is what makes it recoverable.
    """

    def __init__(self, cfg: Optional[NoiseConfig] = None, n_links: int = 2):
        self.cfg = cfg or NoiseConfig()
        self.n_links = n_links
        self.rng = np.random.default_rng(self.cfg.seed)

        bias = self.cfg.angle_bias
        if np.isscalar(bias):
            self._bias = np.full(n_links, float(bias))
        else:
            self._bias = np.asarray(bias, dtype=float)

        self._last_obs: Optional[dict] = None  # for dropout "hold last value"

    def observe(
        self, theta: np.ndarray, omega: np.ndarray, tau: np.ndarray
    ) -> dict:
        """Return a dict of noisy observed channels matching the inputs' shapes."""
        c = self.cfg
        theta_obs = theta + self._bias + self.rng.normal(0, c.angle_std, theta.shape)
        omega_obs = omega + self.rng.normal(0, c.velocity_std, omega.shape)
        tau_obs = tau + self.rng.normal(0, c.torque_std, tau.shape)

        obs = {
            "theta": theta_obs,
            "omega": omega_obs,
            "tau": tau_obs,
        }

        # Dropout: with some probability, report the previous sample instead of
        # the fresh one (models a stuck/late sensor reading).
        if self._last_obs is not None and self.rng.random() < c.dropout_prob:
            obs = {k: self._last_obs[k].copy() for k in obs}
        self._last_obs = {k: v.copy() for k, v in obs.items()}
        return obs

    @property
    def true_bias(self) -> np.ndarray:
        """The hidden bias vector — the 'answer' a calibrator tries to recover."""
        return self._bias.copy()
