"""Core n-link pendulum simulator.

Physics
-------
We model a planar chain of ``n`` point masses ``m_i`` hanging off massless rigid
rods of length ``l_i``. Joint ``i`` connects link ``i-1`` to link ``i`` (joint 0
is anchored to the world). We use **absolute** angles ``theta_i`` measured from
the downward vertical, which yields a clean closed-form manipulator equation.

The equations of motion come straight from the Lagrangian ``L = T - V`` and are
the standard "manipulator equation" form:

    M(theta) @ theta_ddot = tau - C(theta) @ theta_dot**2 - G(theta) - b * theta_dot

with (deriving from T = 1/2 * sum_i m_i (x_dot_i^2 + y_dot_i^2)):

    mu[a, k] = sum_{i >= max(a, k)} m_i          # shared inertia of the chain tail
    M[a, k]  = mu[a, k] * l_a * l_k * cos(theta_a - theta_k)
    C[a, k]  = mu[a, k] * l_a * l_k * sin(theta_a - theta_k)
    G[a]     = g * (sum_{i >= a} m_i) * l_a * sin(theta_a)

``b`` is a simple viscous damping coefficient per joint (our stand-in for joint
friction). It acts on the *absolute* angular velocity, which is a deliberate
simplification — it keeps the model linear in ``b`` so the calibration examples
can recover it cleanly. See ``examples/simple_calibration_experiment.py``.

Why a custom model instead of MuJoCo?
    Every physical parameter (length, mass, friction) is a plain float you own,
    so you can corrupt it, log it, and try to recover it. That is the whole
    point of a *calibration* testbed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Optional, Sequence

import numpy as np

# A controller maps (time, theta, omega) -> torque vector of shape (n_links,).
Controller = Callable[[float, np.ndarray, np.ndarray], np.ndarray]


@dataclass
class PendulumConfig:
    """Physical + integration parameters for the pendulum.

    All per-link sequences must have length ``n_links``. Helper construction is
    available via :meth:`from_dict` so configs can come from YAML untouched.
    """

    n_links: int = 2
    masses: Sequence[float] = (1.0, 1.0)
    lengths: Sequence[float] = (1.0, 1.0)
    # Per-joint viscous damping (friction). Zero => ideal frictionless joints.
    damping: Sequence[float] = (0.0, 0.0)
    gravity: float = 9.81
    dt: float = 0.01  # integration timestep [s]

    def __post_init__(self) -> None:
        # Broadcast scalars to per-link lists for convenience, then validate.
        self.masses = self._broadcast(self.masses, "masses")
        self.lengths = self._broadcast(self.lengths, "lengths")
        self.damping = self._broadcast(self.damping, "damping")
        for name in ("masses", "lengths", "damping"):
            if len(getattr(self, name)) != self.n_links:
                raise ValueError(
                    f"{name} must have length n_links={self.n_links}, "
                    f"got {len(getattr(self, name))}"
                )

    def _broadcast(self, value, name: str) -> list[float]:
        if np.isscalar(value):
            return [float(value)] * self.n_links
        return [float(v) for v in value]

    @classmethod
    def from_dict(cls, d: dict) -> "PendulumConfig":
        """Build a config from a plain dict (e.g. parsed YAML)."""
        known = {f for f in cls.__dataclass_fields__}  # type: ignore[attr-defined]
        return cls(**{k: v for k, v in d.items() if k in known})


@dataclass
class PendulumState:
    """A single ground-truth snapshot of the system at one instant."""

    t: float
    theta: np.ndarray  # absolute joint angles [rad], shape (n,)
    omega: np.ndarray  # absolute joint angular velocities [rad/s], shape (n,)
    tau: np.ndarray  # applied joint torques [N·m], shape (n,)
    # Cartesian (x, y) of every joint including the fixed anchor, shape (n+1, 2).
    positions: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    energy: float = 0.0  # total mechanical energy [J] (useful sanity check)


class NLinkPendulum:
    """Configurable planar n-link pendulum with torque actuation.

    Typical loop::

        sim = NLinkPendulum(PendulumConfig(n_links=2))
        sim.reset(theta0=[np.pi/2, np.pi/2])
        for _ in range(1000):
            state = sim.step(tau=None)          # passive (free) swing
            # or sim.step(controller=my_pd_controller) for actuated mode
    """

    def __init__(self, config: Optional[PendulumConfig] = None):
        self.cfg = config or PendulumConfig()
        n = self.cfg.n_links
        self.m = np.asarray(self.cfg.masses, dtype=float)
        self.l = np.asarray(self.cfg.lengths, dtype=float)
        self.b = np.asarray(self.cfg.damping, dtype=float)
        self.g = float(self.cfg.gravity)
        self.dt = float(self.cfg.dt)

        # mu[a, k] = sum of masses from index max(a, k) to the tip. This "tail
        # mass" coupling matrix is constant, so we precompute it once.
        self.mu = np.zeros((n, n))
        cum_tail = np.cumsum(self.m[::-1])[::-1]  # cum_tail[i] = sum_{j>=i} m_j
        for a in range(n):
            for k in range(n):
                self.mu[a, k] = cum_tail[max(a, k)]

        self.reset()

    # ------------------------------------------------------------------ state
    def reset(
        self,
        theta0: Optional[Sequence[float]] = None,
        omega0: Optional[Sequence[float]] = None,
    ) -> PendulumState:
        """Reset to an initial condition. Defaults to a near-horizontal drop."""
        n = self.cfg.n_links
        if theta0 is None:
            # A classic chaotic-ish start: all links roughly horizontal.
            theta0 = np.full(n, np.pi / 2)
        if omega0 is None:
            omega0 = np.zeros(n)
        self.theta = np.asarray(theta0, dtype=float).copy()
        self.omega = np.asarray(omega0, dtype=float).copy()
        self.t = 0.0
        self.tau = np.zeros(n)
        return self.get_state()

    # --------------------------------------------------------------- dynamics
    def _mass_matrix(self, theta: np.ndarray) -> np.ndarray:
        d = theta[:, None] - theta[None, :]
        return self.mu * np.outer(self.l, self.l) * np.cos(d)

    def angular_acceleration(
        self, theta: np.ndarray, omega: np.ndarray, tau: np.ndarray
    ) -> np.ndarray:
        """Solve M @ theta_ddot = tau - C @ omega^2 - G - b*omega for theta_ddot.

        This is the heart of the simulator and the function a learned dynamics
        model (e.g. a GNN in RuVector) would ultimately try to approximate.
        """
        d = theta[:, None] - theta[None, :]
        M = self.mu * np.outer(self.l, self.l) * np.cos(d)
        C = self.mu * np.outer(self.l, self.l) * np.sin(d)

        cum_tail = np.cumsum(self.m[::-1])[::-1]
        G = self.g * cum_tail * self.l * np.sin(theta)

        rhs = tau - C @ (omega**2) - G - self.b * omega
        # solve is more stable than forming the explicit inverse.
        return np.linalg.solve(M, rhs)

    def _deriv(self, y: np.ndarray, tau: np.ndarray) -> np.ndarray:
        """State derivative for RK4. y = [theta, omega]."""
        n = self.cfg.n_links
        theta, omega = y[:n], y[n:]
        return np.concatenate([omega, self.angular_acceleration(theta, omega, tau)])

    def step(
        self,
        tau: Optional[Sequence[float]] = None,
        controller: Optional[Controller] = None,
    ) -> PendulumState:
        """Advance one timestep with a fixed-step RK4 integrator.

        Modes
        -----
        * **passive**:   ``step()``  -> zero torque, free swing under gravity.
        * **actuated**:  ``step(tau=[...])`` or ``step(controller=fn)``.

        RK4 (vs. Euler) keeps energy drift small enough that the passive
        pendulum looks physically believable over a demo's worth of steps.
        """
        n = self.cfg.n_links
        if controller is not None:
            tau_vec = np.asarray(
                controller(self.t, self.theta, self.omega), dtype=float
            )
        elif tau is not None:
            tau_vec = np.asarray(tau, dtype=float)
        else:
            tau_vec = np.zeros(n)
        self.tau = tau_vec

        y = np.concatenate([self.theta, self.omega])
        dt = self.dt
        k1 = self._deriv(y, tau_vec)
        k2 = self._deriv(y + 0.5 * dt * k1, tau_vec)
        k3 = self._deriv(y + 0.5 * dt * k2, tau_vec)
        k4 = self._deriv(y + dt * k3, tau_vec)
        y = y + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

        self.theta, self.omega = y[:n], y[n:]
        self.t += dt
        return self.get_state()

    # ----------------------------------------------------------- observables
    def link_positions(self, theta: Optional[np.ndarray] = None) -> np.ndarray:
        """Cartesian joint positions including the anchor, shape (n+1, 2).

        Row 0 is the fixed pivot at the origin; row i+1 is the tip of link i.
        Used both for rendering and as Cartesian node features for the GNN.
        """
        if theta is None:
            theta = self.theta
        x = np.concatenate([[0.0], np.cumsum(self.l * np.sin(theta))])
        y = np.concatenate([[0.0], -np.cumsum(self.l * np.cos(theta))])
        return np.column_stack([x, y])

    def total_energy(
        self, theta: Optional[np.ndarray] = None, omega: Optional[np.ndarray] = None
    ) -> float:
        """Total mechanical energy KE + PE. Constant for the passive system."""
        if theta is None:
            theta = self.theta
        if omega is None:
            omega = self.omega
        M = self._mass_matrix(theta)
        ke = 0.5 * omega @ M @ omega
        cum_tail = np.cumsum(self.m[::-1])[::-1]
        pe = -self.g * np.sum(cum_tail * self.l * np.cos(theta))
        return float(ke + pe)

    def get_state(self) -> PendulumState:
        return PendulumState(
            t=self.t,
            theta=self.theta.copy(),
            omega=self.omega.copy(),
            tau=self.tau.copy(),
            positions=self.link_positions(),
            energy=self.total_energy(),
        )
