"""multi_link_pendulum — a configurable n-link pendulum testbed for RuVector.

Public API
----------
    from pendulum_sim import NLinkPendulum, PendulumConfig
    from pendulum_sim import SensorNoiseModel, NoiseConfig
    from pendulum_sim import JSONLLogger, EpisodeMetadata
    from pendulum_sim import RerunVisualizer

The package is deliberately split so each concern (physics, noise, logging,
viz) can be swapped independently — e.g. replace ``JSONLLogger`` with a direct
RuVector client without touching the simulator.
"""

from .simulator import NLinkPendulum, PendulumConfig, PendulumState
from .noise import SensorNoiseModel, NoiseConfig
from .data_logger import (
    JSONLLogger,
    EpisodeMetadata,
    state_to_graph,
    state_to_vector,
)
from .visualization import RerunVisualizer

__all__ = [
    "NLinkPendulum",
    "PendulumConfig",
    "PendulumState",
    "SensorNoiseModel",
    "NoiseConfig",
    "JSONLLogger",
    "EpisodeMetadata",
    "state_to_graph",
    "state_to_vector",
    "RerunVisualizer",
]

__version__ = "0.1.0"
