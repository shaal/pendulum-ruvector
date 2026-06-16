"""Structured logging designed for ingestion into RuVector.

RuVector pairs a vector database with GNN memory, so it wants the data in two
complementary shapes, and this module produces both for every timestep:

1. A flat **embedding vector** + metadata  -> for vector similarity search.
   ("find moments in past episodes that look like *this* configuration").

2. A **graph** (nodes = links, edges = joints) -> for GNN message passing.
   The physical pendulum *is* a graph: each link is a node carrying its own
   state, and joints are edges carrying the coupling between neighbors. A GNN
   can therefore learn dynamics/calibration corrections that generalize across
   different numbers of links — train on 2, run on 4 — because the operator is
   shared across edges.

Output format is **JSONL** (one JSON object per line): append-friendly,
stream-friendly, and trivial to ingest from Rust or Python. Swap
``JSONLLogger`` for a direct RuVector client later without touching the
simulator — the record schema is the contract.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

from .simulator import NLinkPendulum, PendulumState


@dataclass
class EpisodeMetadata:
    """Static descriptors attached to every record in an episode.

    These become the *payload/metadata* stored alongside each vector in
    RuVector, so you can filter searches ("only actuated 3-link episodes").
    """

    episode_id: str
    n_links: int
    mode: str  # "passive" | "actuated"
    masses: list[float]
    lengths: list[float]
    damping: list[float]
    gravity: float
    dt: float
    noise_level: float = 0.0
    extra: dict = field(default_factory=dict)

    def as_dict(self) -> dict:
        return {
            "episode_id": self.episode_id,
            "n_links": self.n_links,
            "mode": self.mode,
            "masses": list(self.masses),
            "lengths": list(self.lengths),
            "damping": list(self.damping),
            "gravity": self.gravity,
            "dt": self.dt,
            "noise_level": self.noise_level,
            **self.extra,
        }


def state_to_graph(sim: NLinkPendulum, state: PendulumState, obs: Optional[dict]) -> dict:
    """Represent the pendulum as a GNN-ready graph at one instant.

    Node i (one per link) feature vector:
        [mass_i, length_i, theta_i, omega_i, tau_i, sin(theta_i), cos(theta_i),
         tip_x_i, tip_y_i]
    We include sin/cos so the GNN gets a continuous, wrap-around-free angle
    encoding, and the Cartesian tip so it has absolute spatial context.

    Edges: an undirected chain joint_{i-1 <-> i}. We emit both directions so a
    standard message-passing GNN sees a symmetric graph. Edge i->j feature:
        [relative_angle, relative_velocity, damping_at_shared_joint]

    The returned dict mirrors PyTorch-Geometric conventions (``edge_index`` is
    2 x E), so wiring it into a GNN is a copy-paste away.
    """
    n = sim.cfg.n_links
    tips = state.positions[1:]  # drop the anchor row; one tip per link

    node_features = []
    for i in range(n):
        node_features.append(
            [
                float(sim.m[i]),
                float(sim.l[i]),
                float(state.theta[i]),
                float(state.omega[i]),
                float(state.tau[i]),
                float(np.sin(state.theta[i])),
                float(np.cos(state.theta[i])),
                float(tips[i, 0]),
                float(tips[i, 1]),
            ]
        )

    edge_index: list[list[int]] = [[], []]
    edge_features: list[list[float]] = []
    for i in range(n - 1):
        rel_angle = float(state.theta[i + 1] - state.theta[i])
        rel_vel = float(state.omega[i + 1] - state.omega[i])
        damp = float(sim.b[i + 1])  # damping at the joint linking i and i+1
        for src, dst in ((i, i + 1), (i + 1, i)):
            edge_index[0].append(src)
            edge_index[1].append(dst)
            edge_features.append([rel_angle, rel_vel, damp])

    return {
        "node_feature_names": [
            "mass", "length", "theta", "omega", "tau",
            "sin_theta", "cos_theta", "tip_x", "tip_y",
        ],
        "node_features": node_features,
        "edge_index": edge_index,
        "edge_feature_names": ["rel_angle", "rel_velocity", "joint_damping"],
        "edge_features": edge_features,
    }


def state_to_vector(state: PendulumState, obs: Optional[dict]) -> dict:
    """Flatten a state into a fixed-order embedding vector + a named breakdown.

    The ``embedding`` is what you'd feed to a vector index. We use the redundant
    sin/cos angle encoding here too so that Euclidean/cosine distance in the
    index respects angle wrap-around (theta and theta+2pi map to nearby points).
    """
    theta = state.theta
    vec = np.concatenate(
        [
            np.sin(theta),
            np.cos(theta),
            state.omega,
            state.tau,
        ]
    ).astype(float)
    return {
        "embedding": vec.tolist(),
        "embedding_layout": ["sin_theta", "cos_theta", "omega", "tau"],
        "dim": int(vec.size),
    }


class JSONLLogger:
    """Append one rich JSON record per timestep to a ``.jsonl`` file.

    Each record bundles everything a downstream RuVector pipeline could want:
    ground truth, the noisy observation, the calibration residual, the flat
    embedding, and the graph. Downstream you pick the fields you need.
    """

    def __init__(self, path: str | Path, metadata: EpisodeMetadata, append: bool = False):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.metadata = metadata
        # "a" lets several episodes share one JSONL file (it is line-oriented,
        # so concatenation is always valid); "w" starts a fresh file.
        self._fh = self.path.open("a" if append else "w")
        self._count = 0
        # Episode header line (record_type lets a reader distinguish it).
        self._write({"record_type": "episode_header", **metadata.as_dict()})

    def _write(self, obj: dict) -> None:
        self._fh.write(json.dumps(obj) + "\n")

    def log_step(
        self,
        sim: NLinkPendulum,
        state: PendulumState,
        obs: Optional[dict] = None,
    ) -> None:
        """Log a single timestep with ground truth, observation, graph & vector."""
        record: dict = {
            "record_type": "step",
            "episode_id": self.metadata.episode_id,
            "step": self._count,
            "t": state.t,
            # --- Ground truth (what the simulator knows exactly) ---
            "ground_truth": {
                "theta": state.theta.tolist(),
                "omega": state.omega.tolist(),
                "tau": state.tau.tolist(),
                "energy": state.energy,
                "positions": state.positions.tolist(),
            },
            # --- RuVector-ready representations ---
            "vector": state_to_vector(state, obs),
            "graph": state_to_graph(sim, state, obs),
        }

        if obs is not None:
            record["observation"] = {
                "theta": _to_list(obs["theta"]),
                "omega": _to_list(obs["omega"]),
                "tau": _to_list(obs["tau"]),
            }
            # Calibration residual: observed - true. A learned calibration model
            # tries to predict/correct exactly this signal, so we precompute it.
            record["residual"] = {
                "theta": (_arr(obs["theta"]) - state.theta).tolist(),
                "omega": (_arr(obs["omega"]) - state.omega).tolist(),
            }

        self._write(record)
        self._count += 1

    def close(self) -> None:
        self._fh.flush()
        self._fh.close()

    # Context-manager sugar so callers can `with JSONLLogger(...) as log:`.
    def __enter__(self) -> "JSONLLogger":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def _arr(x) -> np.ndarray:
    return np.asarray(x, dtype=float)


def _to_list(x) -> list:
    return _arr(x).tolist()
