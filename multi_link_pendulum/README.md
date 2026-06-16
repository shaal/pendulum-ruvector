# multi_link_pendulum

![Double pendulum swinging](docs/double_pendulum.gif)

*A 2-link pendulum released from horizontal, rendered straight from `pendulum_sim`
([`examples/render_gif.py`](examples/render_gif.py)). The red trail is the tip's
chaotic path.*

A clean, self-contained **n-link pendulum simulator** built as a testbed for
experimenting with **[RuVector](https://github.com/ruvnet/RuVector)** (Rust
vector database + GNN memory) on **robotic-arm calibration** and agentic
workflows.

It gives you a fully-observable physical system whose *true* parameters you
control, *imperfect* sensors you configure, and a logging format that drops
straight into a vector index and a GNN.

---

## Why this helps with RuVector calibration experiments

Calibration = recovering a robot's true physical parameters (link lengths,
masses, joint friction, encoder offsets) from **noisy** measurements. To study
that with RuVector you need three things this project provides:

1. **Known ground truth.** Every physical parameter is a plain Python float you
   set, so you can corrupt it, log it, and measure how well it was recovered.
   (This is the main reason the sim is a transparent custom Lagrangian model
   rather than a MuJoCo black box.)

2. **A graph-shaped system.** A pendulum *is* a graph — links are nodes, joints
   are edges. `state_to_graph()` emits exactly the
   `node_features / edge_index / edge_features` triple a GNN consumes, so
   RuVector's GNN memory can message-pass over it and generalize across
   different link counts (train on 2, run on 4).

3. **Vector-ready episodes.** `state_to_vector()` produces a fixed embedding per
   timestep (with a wrap-around-safe sin/cos angle encoding) plus rich metadata.
   Store those in RuVector's vector DB to retrieve "moments that look like this"
   and warm-start calibration from prior experience.

The included `simple_calibration_experiment.py` solves the recovery problem with
a plain SciPy optimizer — that's the **baseline** a learned RuVector calibrator
aims to beat by reusing experience across episodes.

---

## Install (with `uv`)

```bash
cd multi_link_pendulum

# Option A: a managed venv from pyproject.toml
uv venv
uv pip install -e .

# Option B: straight from requirements.txt
uv pip install -r requirements.txt
```

Python 3.10+ required. `rerun-sdk` is only needed for the live visualization;
data collection and calibration run headless without it.

---

## Run

All commands run from the `multi_link_pendulum/` directory.

### 1. Watch it swing (real-time Rerun viewer)

```bash
# 2-link passive double pendulum with 0.05 rad sensor noise overlaid in red
python -m examples.run_visual_demo --links 2 --noise 0.05

# 3-link, torque-actuated (PD controller trying to stand it up)
python -m examples.run_visual_demo --links 3 --mode actuated

# 4-link, longer run
python -m examples.run_visual_demo --links 4 --duration 30
```

A Rerun window opens showing the arm, per-joint angle/velocity time series, a
phase portrait (θ vs ω), and the total-energy trace (≈flat ⇒ integrator healthy).

Prefer a shareable file with no GUI? Render a GIF instead (needs the
`viz-static` extra: `uv pip install -e '.[viz-static]'`):

```bash
python -m examples.render_gif --links 3 --duration 8 --out triple.gif
```

### 2. Collect a RuVector-ready dataset (JSONL)

```bash
python -m examples.collect_calibration_data --episodes 5 --links 2 --noise 0.03
# -> writes data/calibration.jsonl
```

### 3. Run a calibration experiment

```bash
python -m examples.simple_calibration_experiment --target length
python -m examples.simple_calibration_experiment --target damping --links 2
```

---

## Project layout

```
multi_link_pendulum/
├── README.md
├── requirements.txt / pyproject.toml
├── pendulum_sim/
│   ├── __init__.py
│   ├── simulator.py       # NLinkPendulum: Lagrangian dynamics + RK4 integrator
│   ├── visualization.py   # RerunVisualizer: real-time arm/plots/phase portrait
│   ├── data_logger.py     # JSONL logging + graph & vector representations
│   └── noise.py           # Gaussian/bias/dropout sensor noise models
├── examples/
│   ├── run_visual_demo.py
│   ├── collect_calibration_data.py
│   └── simple_calibration_experiment.py
└── configs/
    └── default.yaml
```

---

## The data schema (one JSONL line per timestep)

```jsonc
{
  "record_type": "step",
  "episode_id": "ep_0000",
  "step": 42,
  "t": 0.42,
  "ground_truth": { "theta": [...], "omega": [...], "tau": [...],
                    "energy": -1.23, "positions": [[x,y], ...] },
  "observation":  { "theta": [...], "omega": [...], "tau": [...] },  // noisy
  "residual":     { "theta": [...], "omega": [...] },                // obs - true
  "vector": {                       // -> RuVector vector DB
    "embedding": [...],             // [sin θ | cos θ | ω | τ]
    "embedding_layout": ["sin_theta","cos_theta","omega","tau"],
    "dim": 8
  },
  "graph": {                        // -> RuVector GNN memory
    "node_features": [[mass,length,theta,omega,tau,sinθ,cosθ,tip_x,tip_y], ...],
    "edge_index": [[src...],[dst...]],          // PyG-style, 2 x E
    "edge_features": [[rel_angle,rel_vel,joint_damping], ...]
  }
}
```

### Suggested RuVector ingestion sketch

```python
import json
for line in open("data/calibration.jsonl"):
    rec = json.loads(line)
    if rec["record_type"] != "step":
        continue
    # 1. vector index: rec["vector"]["embedding"] + metadata (episode_id, t, ...)
    # 2. GNN memory:   rec["graph"] node/edge tensors
    # 3. supervision:  rec["residual"] is the calibration error to predict
```

---

## Extending

- **More links:** pass `--links N`; the dynamics, graph, and vectors all scale
  automatically (the mass matrix is built for arbitrary `n`).
- **Custom controllers:** pass any `fn(t, theta, omega) -> torque` to
  `sim.step(controller=fn)`.
- **Direct RuVector calls:** swap `JSONLLogger` for a thin client that pushes
  `state_to_vector(...)` / `state_to_graph(...)` straight into RuVector — the
  record schema above is the contract.
