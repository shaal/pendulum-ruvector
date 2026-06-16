# Learning roadmap — from adaptation to discovery

Where the project is, and the staged plan for the "learns / competes toward the
goal" vision. Each stage must **beat the previous baseline on the `check`
harness or it isn't worth shipping**.

## Where we are (shipped)

| Capability | How | Status |
|---|---|---|
| Balance (hold upright) | in-Rust LQR from linearized dynamics | ✅ optimal, instant |
| Recalibrate on change | RuVector recognition: probe → dynamics signature → recall gain | ✅ Phase 2 (length, ~2.0–2.2m envelope) |
| Recover from knockdown | collocated-PFL energy swing-up + LQR catch | ✅ Phase 3, **7/10** harness |
| Generalize between configs | `ruvector-gnn` attention blend over the config graph | ✅ Phase 3 (untrained layer) |
| Live recognition in the game | `play` with `--features "game vectordb"` | ✅ Stage 0 |

The system today **adapts** (recognize → recall a *known* controller). It does
not yet **discover** (find controllers nobody designed) or **learn from reward**.

## The vision (user's words, mapped to method)

- "rewards when it finds ways toward the goal" → reward shaping (potential-based).
- "sometimes tries random things" → exploration (population noise in ES; entropy
  in RL). On a chaotic system, structured > naive random.
- "hundreds of versions sent to learn/predict, competing to reach the goal
  faster" → **population-based / evolutionary search** (+ RuVector as the shared
  memory the population learns through).
- "goal can change in the future" → **goal-conditioned** policy/reward.

## Stages

### Stage 1 — Evolutionary swing-up search *(next; see `evolutionary-swingup.md`)*
Population of candidate swing-up policies, gradient-free ES, domain-randomized
arms, scored on the harness; winners stored in RuVector as retrievable skills.
The jump from adaptation to discovery. **Target: beat 7/10 + show a learning
curve.** Use ship-task.

### Stage 2 — Generalization via domain randomization
Evolve a *single* policy over randomized arm configs so it recovers across arms
it never saw — a stronger, learned version of the GNN interpolation. Compose:
recall nearest learned policy, then run it. **Target: recover across a held-out
config set.**

### Stage 3 — Goal-conditioning (changeable goal)
Make the target a parameter (posture / energy / even "spin at rate ω"). The LQR
setpoint and the reward become functions of a goal vector; the policy takes the
goal as input. **Target: same machinery hits a second goal without re-coding.**

### Stage 4 — Live competing population (optional, ambitious)
Many agents run concurrently, each on randomized conditions, **sharing
discoveries through RuVector mid-run** (write good trajectories; retrieve peers'
successes to warm-start). This is the literal "hundreds competing, learning from
each other." Measure: does shared memory reach a target fitness in fewer total
rollouts than independent search?

### Stage 5 — Residual RL (optional)
If ES plateaus, add a small learned residual *on top of* the model-based
controller (learn the correction, not the whole policy) — sample-efficient and
keeps interpretability.

## Principles (carry across stages)

- **Hybrid, not replacement.** Keep LQR for balance; learn only the hard regime.
- **Every stage measured against `check`.** No learning ships unless it beats the
  prior baseline; report honest numbers including failures.
- **Dependency-light.** Prefer a few hundred lines of Rust (CMA-ES/CEM, tiny MLP)
  over a heavy ML framework.
- **Determinism / reproducibility.** Seed RNGs explicitly; pass seeds in.
- **RuVector is the memory spine.** Recognition, recall, interpolation, and
  (Stage 4) cross-agent transfer all flow through it — that's the throughline
  that makes this a RuVector showcase, not just a control demo.
