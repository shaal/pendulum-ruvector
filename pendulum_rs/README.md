# pendulum_rs

![Triple pendulum simulated in Rust](docs/rust_triple_pendulum.gif)

*A triple pendulum simulated by this crate. The same run inserted 108 state
vectors into a RuVector HNSW store and ran GNN message-passing over the link
graph — all in one Rust process.*

A **Rust-native** n-link pendulum: hand-derived Lagrangian dynamics → live
**Rerun** visualization, with an optional **in-process RuVector** loop
(vector-DB insert + GNN message passing). This is the Rust sibling of the Python
`multi_link_pendulum/` project — same physics, but everything stays in the Rust
ecosystem alongside RuVector, so there's **no JSONL bridge**: the simulator
calls RuVector directly in the same process.

Stack (matches the standard Rust robotics recommendation): custom dynamics +
[Rerun](https://rerun.io) Rust SDK, with RuVector's `ruvector-core` (HNSW vector
DB) and `ruvector-gnn` (graph attention layer) linked as path dependencies into
the `../RuVector` submodule.

## Play it: You vs RuVector (`play` binary)

An interactive duel. You drive the left arm's base motor with **A / D** and try to
keep it balanced straight up; the right arm balances itself (and recalibrates on
disturbance). Press **SPACE** to fire a disturbance, **R** to reset.

```bash
cargo run --release --features game --bin play
```

Balancing an underactuated double pendulum through one joint by hand is *brutal*
— that's the point. (Uses [macroquad](https://macroquad.rs) for the live window;
only built with `--features game`.)

## Underactuated arm balance (`arm` binary) — the main demo

A 2-link arm with **only joint 0 motorized** balances straight up (an unstable
equilibrium) using an LQR computed in-Rust. Two arms run side by side; partway
through, the second link **changes length** on both:

```bash
cargo run --release --bin arm -- --duration 8 --newlen 2.0 --out arm.rrd
rerun arm.rrd
```

- **naive** arm keeps its old gain → topples.
- **adaptive** arm recomputes its balance gain for the new arm → stays upright.

This is the control core for the RuVector calibration story: the balance gain is
derived from the arm's mass/length model, so when the model changes the gain
must change too. In Phase 1 the adaptive arm is told the new length by an oracle;
**Phase 2 replaces that oracle with RuVector** estimating the change from motion.

`cargo run --bin check` is a diagnostic that sweeps configurations to find which
ones are stabilizable and where adaptation actually decides survival.

## Phase 2: RuVector *is* the estimator (`estimate` binary)

Phase 1's adaptive arm is handed its new parameters by an oracle. Phase 2 makes
it **recall** them — replacing the oracle with a real RuVector lookup.

```bash
cargo run --release --features vectordb --bin estimate
rerun estimate.rrd
```

How it works:

1. **Seed (offline).** Sweep a grid of arms `(link-2 length × mass × friction)`.
   For each, fingerprint its dynamics at upright and store it in RuVector:
   `embedding = dynamics signature`, `payload = {params, gain K, e_up}`
   (`src/memory.rs`).
2. **Signature.** The fingerprint is the *closed-loop* linearization `A − b·K`
   (acceleration-per-state-error and input rows) under a fixed probe gain — ten
   numbers that determine the balance gain (`src/estimator.rs`). It is matched in
   closed-loop space because the online regression measures exactly those
   coefficients; recovering open-loop `A` would amplify the input-term noise.
3. **Recognize (online).** When the arm is disturbed it runs a short **dithered
   probe**: it keeps its stale gain, injects a small exogenous multi-sine torque,
   and regresses *measured* accelerations against state and dither. The dither is
   the instrument that makes the input column identifiable (a `u = −K·x`
   stabilizer alone is collinear with the state). The result is the live
   signature → `VectorDB::search` for the nearest seeded arm → adopt its gain.
4. **The win.** Side by side, the naive arm keeps its stale gain and **topples**;
   the adaptive arm recovers via recall after an honest **recognition lag**
   (~0.38 s), printed every run.
5. **Self-learning.** After a successful catch, the *measured* signature is
   inserted back into RuVector tagged as a verified ("learned") config. The same
   disturbance thrown again is recognized from a rougher, earlier estimate
   (~0.15 s) — a **~60% lag shrink**. A config felt once is shrugged off faster.

**Operating envelope (honest).** Recognition keys on *structural* (link-length)
change and works for extensions up to ~2.2 m; beyond that the arm topples faster
than the probe can identify it — that regime is Phase-3 swing-up. Mass/payload
changes shift the same gravity-stiffness terms and are confounded with length
under measurement noise; generalizing across the full config space is what
Phase-3's GNN interpolation is for. Tests: `cargo test --features vectordb`.

## Phase 3: swing-up + GNN generalization

### Swing-up — recover from a full knockdown (`check` binary)

Phase 1's controller only catches small pokes. Phase 3 adds a **collocated
partial-feedback-linearization (PFL)** swing-up (`control::swingup_pfl`): the
passive-joint equation lets us solve `q̈₁` from `q̈₀`, so the actuated row becomes
`u = M̄·q̈₀ + h̄` and commanding `q̈₀ = v` *feedback-linearizes* joint 0 regardless
of configuration. The outer loop is the classic energy pump `v = k_e·(E_up−E)·ω₀`
(aggressive, near-bang-bang); the LQR catches at the top (`recover_torque`).

```bash
cargo run --release --bin check        # recovery harness over 10 knockdown starts
```

**Honest result:** this lifts recovery from **2/4** (the naive direct-torque
pump) to **7/10** diverse knockdowns — *including a dead vertical hang*. A few
worst-case starts (hard-sideways, both-folded) still defeat it: full swing-up of
a double pendulum from *any* state is genuinely research-grade and remains
unsolved here. The improved swing-up is live in the `play` game, so the RuVector
arm now hoists itself back up from most knockdowns. Test:
`swings_up_from_a_dead_hang`.

**Goal-conditioned (the target can change).** The controller is parameterized by
a *goal* equilibrium (`recover_to(sim, goal, …)`, `balance_gain_for(goal)`,
`energy_at(goal)`) — the upright functions are now thin wrappers. The *same*
controller reaches different targets: `[π,π]` (both up, 7/10) and `[π,0]` (link-1
up, link-2 dangling, 4/10). Reachable goals are exactly `{θ₀ free} × {θ₁∈{0,π}}`
— with joint 1 passive, link 2 must hang gravity-balanced. `cargo run --bin check`
prints both. (`[π,0]` is less reliable because the energy pump aims at *energy*,
not the goal *posture*.) Tests: `reaches_link1_up_link2_down_goal`.

### GNN interpolation — generalize between seeded arms

Nearest-neighbour recall *snaps* an unseen arm to one seeded config.
`ConfigMemory::recall_interpolated` (feature `gnn`) instead treats the seeds as a
graph and message-passes the query over its k nearest neighbours with a real
`ruvector-gnn::RuvectorLayer`, then adopts the **attention-weighted blend** of
their gains — interpolating to arms it never saw.

```bash
cargo run --release --features ruvector --bin estimate   # prints the GNN blend
```

**Honest note:** the layer ships *untrained*, so the gain is the graph-attention
blend (`K = Σ wᵢ·Kᵢ`, `wᵢ = softmax(−distance)`), not a learned regression — the
message-pass contextualizes the neighbourhood, the graph weights do the
interpolating. For a between-seed arm the blend lands measurably closer to the
true gain than any single neighbour (test: `gnn_interpolation_beats_snapping`).

## Stage 1: evolutionary swing-up — *discover* a better controller (`evolve` binary)

So far the arm *adapts* (recall a known controller). Here it *discovers* one. A
population of hundreds of candidate swing-up policies competes each generation;
a gradient-free **cross-entropy search** marches the distribution toward policies
that recover more knockdowns. The LQR catch stays untouched (hybrid) — only the
swing-up `v` (the commanded actuated acceleration the PFL inversion realizes) is
learned, as a linear combination of physical features (`learn::EnergyShapingPolicy`).

```bash
cargo run --release --bin evolve     # ~10s on a multicore box; no GPU, no ML deps
```

The hand-tuned baseline recovers **7/10**; the evolved champion (default seed)
recovers **10/10**, using features the hand-tuning never touched (passive-joint
pump, posture-sin term, velocity damping). It's honest generalization — trained
on randomized knockdowns, judged on the held-out `check` harness.

**Honest note:** the search is stochastic on a chaotic landscape, but reliably
strong — seeds 0–7 recover **7–10/10 (median ~9.5), and 7 of 8 beat the 7/10
baseline** (an outlier seed gave 6). A given seed reproduces exactly (the RNG is
seeded). Implementation is a few hundred lines of dependency-light Rust
(`std::thread::scope` for the parallel population, a splitmix64 RNG). Test
`evolved_champion_beats_baseline` pins the champion. Design + roadmap:
[`../docs/plans/`](../docs/plans/).

### Stage 2: domain randomization + the live recall consumer

Two pieces. First, **the controller now uses what it stored**: `recover` recalls
the nearest learned swing-up policy from RuVector by the arm's config signature
and runs it (LQR catch + recalled swing-up), closing the loop discover → store →
recall → run (`learn::rollout_recalling_policy`; test
`controller_recalls_and_runs_a_stored_policy`).

Second, **domain-randomized evolution** — each candidate is scored over randomized
*arm configs* (not just the nominal arm), so the champion must generalize:

```bash
cargo run --release --bin evolve                     # nominal-arm search (Stage 1)
RANDOMIZE_ARM=1 cargo run --release --bin evolve     # cross-arm search (Stage 2)
```

**Honest result — and why one policy can't win.** On a held-out set of arms it
never trained on (80 arm×knockdown trials), the domain-randomized champion
recovers **28/80**, edging the nominal-only champion's **27/80** and clearly
beating the baseline's **19/80**. A closest-approach fitness (`Rollout.min_tip` —
reward swinging *nearer* upright even on a miss) made DR markedly more
consistent across seeds (25–28 vs an earlier 19–24). But the edge over the
nominal champion stays marginal — for a structural reason the report makes plain:
the **union ceiling, what *any* of {baseline, nominal, DR} recovers, is 42/80**,
far above any single policy. No one universal controller generalizes decisively;
different arms are rescued by different policies. **The path to the ceiling is
per-arm policy *recall* (the RuVector store above), not one domain-randomized
policy.** (Side-finding: *short* links are the hard case — less leverage to pump
energy.) `evolve RANDOMIZE_ARM=1` prints the per-policy, per-link-length, and
ceiling breakdown. Tests: `domain_randomized_champion_generalizes`,
`policy_union_exceeds_any_single`.

**Per-arm library (`evolve LIBRARY=1`).** Following the ceiling finding, this
evolves a champion *per anchor config*, stores the library in RuVector, and
recovers held-out arms by **recalling the nearest champion per arm**. The honest
decomposition (80 held-out trials): best single **28** → per-arm recall **29** →
per-arm *oracle* (best policy/arm) **38** → union ceiling **42**. Per-arm
*selection* has real headroom (38 ≫ 28), but signature-keyed *recall* captures
little of it — the dynamics-nearest anchor isn't the best-policy anchor. Test:
`per_arm_library_beats_single_policies`. *Stage 2.7 then tested keying policies by
**performance** instead of training origin — and it failed conclusively (23/80,
worse than 29), because per-arm-best champions overfit to a few chaotic
knockdowns. Across single policy, domain randomization, and both recall schemes,
cross-arm generalization plateaus at ~28–29/80 — the oracle's 38 needs test-arm-
specific knowledge no recall can access.* Full write-up in
[`../docs/plans/`](../docs/plans/).

## Build & run (the logging/visualization demo)

The base build is self-contained (just the Rerun SDK):

```bash
cd pendulum_rs

# 1. Sim + visualization only (fastest). Writes pendulum_rs.rrd:
cargo run --release -- --links 2 --duration 12
rerun pendulum_rs.rrd            # open the recording in the viewer

# ...or stream live into the viewer instead of a file:
cargo run --release -- --links 3 --spawn
```

Add the RuVector loop with cargo features:

```bash
cargo run --release --features gnn       -- --links 3   # GNN over the link graph
cargo run --release --features vectordb  -- --links 2   # index state vectors
cargo run --release --features ruvector  -- --links 3   # the whole unified loop
```

CLI flags: `--links N`, `--duration SECS`, `--fps N`, `--damping D`, `--spawn`,
`--out FILE.rrd`.

## What gets logged where

| Sink | What | Code |
|------|------|------|
| Rerun `world/arm` | links (line strip) + joints (points), swinging | `LineStrips2D` / `Points2D` |
| Rerun `plots/*` | per-joint angle time series + total energy | `Scalars` |
| RuVector vector DB | `[sinθ \| cosθ \| ω \| τ]` per step + `{t, step}` metadata | `VectorDB::insert(VectorEntry)` |
| RuVector GNN | each link's features message-passed with chain neighbors | `RuvectorLayer::forward(...)` |

## Why in-process matters

Because the sim and RuVector are both Rust, a state vector goes
`sim → VectorEntry → HNSW index` with zero serialization, and the link graph
goes `sim → node/edge features → GNN forward` in the same loop. That's the tight
calibration loop the Python project can only approximate by writing JSONL and
shelling out to the `ruvector` CLI / REST server.

## RuVector APIs used (real, from the submodule)

- `ruvector_core::VectorDB::new(DbOptions{ dimensions, distance_metric, storage_path, .. })`
  then `.insert(VectorEntry{ id, vector, metadata })` — synchronous, HNSW-backed.
- `ruvector_gnn::RuvectorLayer::new(input_dim, hidden_dim, heads, dropout)`
  then `.forward(node_embedding, neighbor_embeddings, edge_weights) -> Vec<f32>`.

RuVector's default features (`simd-avx512`, `simsimd`) are x86/C-toolchain
specific, so this crate links `ruvector-core` with
`default-features = false, features = ["storage", "hnsw", "parallel"]` for clean
builds on Apple Silicon.
