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

### Stage 1 — Evolutionary swing-up search *(core ✅ shipped; see `evolutionary-swingup.md`)*
Population of candidate swing-up policies, gradient-free CEM, scored on the
harness; the jump from adaptation to discovery. **Done:** `learn.rs` policy/
fitness + `evolve` binary; champion beats baseline (7/10 → 10/10 default seed,
6–10 across seeds). **Remaining:** store winners in RuVector, domain
randomization (→ Stage 2), game payoff. Use ship-task.

### Stage 2.6 — Per-arm policy library + recall: a precise decomposition *(shipped)*
Evolved a champion **per anchor config** (`evolve LIBRARY=1`), stored each in
RuVector, and recovered held-out arms (between anchors) by recalling the nearest
champion. **Honest decomposition of the held-out ceiling (80 trials):**
- best single policy: **28** · per-arm **recall**: **29** · per-arm **oracle**
  (best policy/arm): **38** · union ceiling (best per arm×knockdown): **42**.

So per-arm *selection* has real headroom (oracle 38 ≫ 28), but signature-keyed
*recall* captures almost none of it (29) — the arm whose *dynamics* are nearest
isn't the one whose *champion* transfers best; and 38→42 is per-knockdown
variation no arm-keyed scheme can reach. **The lever is the recall→policy
mapping, not more domain randomization.** Test: `per_arm_library_beats_single_policies`.

### Stage 2.7 — Performance-keyed recall: a definitive negative *(shipped)*
Tried to close the 29→38 gap by keying policies on **performance** instead of
training origin: profile the library over a grid (off the held-out set) and store
each grid arm's best-*performing* champion. **It failed — and conclusively.**
Performance-keyed recall recovered **23/80, *worse* than training-keyed recall
(29)**: "best on a profile arm" is chosen over a few chaotic knockdowns, so it
overfits to that arm and transfers worse than a champion *trained* to be robust
around its anchor. The oracle (38) needs the *test arm's own* best champion,
which recall can't access. **Conclusion: cross-arm generalization for this system
is exhausted — single policy, domain randomization, training-keyed recall, and
performance-keyed recall all plateau at ~28–29/80.** `evolve LIBRARY=1` prints
the full comparison; helper `best_library_champion_for` (test
`best_library_champion_is_the_per_arm_max`). **The generalization thread is
closed; the next frontier is Stage 3 (goal-conditioning), a different axis.**

### Stage 2.5 — Why one policy can't win, and what to do instead *(shipped)*
Tried to make domain randomization *decisively* beat the nominal champion.
**Finding: it can't — structurally.** Closest-approach fitness shaping
(`Rollout.min_tip`, reward the nearest swing to upright even on misses) made DR
*more consistent* (25–28/80 across seeds vs the old 19–24), and the best DR
champion edges nominal (28 vs 27/80). But the **union ceiling — what *any* of
{baseline, nominal, DR} recovers — is 42/80**, far above any single policy. No
one universal controller generalizes decisively; different arms favour different
policies. **So the path to the ceiling is per-arm policy *recall* (Stage 1.4),
not one domain-randomized policy.** Side-finding: *short* links are the hard case
(less leverage to pump energy), not long/heavy ones. Tests:
`policy_union_exceeds_any_single`, `domain_randomized_champion_generalizes`.
**Next:** evolve a *library* of per-config champions, store each in RuVector, and
recover by recalling the best per arm — measure how close that gets to 42/80.

### Stage 2 — Generalization via domain randomization *(shipped; marginal win)*
Evolve a *single* policy over randomized arm configs so it recovers across arms
it never saw. **Done:** `rollout_config` (any arm), `evolve RANDOMIZE_ARM=1`,
held-out eval, and the **live recall consumer** (`rollout_recalling_policy` —
controller recalls a stored policy by signature and runs it; closes the deferred
Stage-1 gap). **Honest result:** the DR champion edges the nominal champion on
held-out arms (29/80 vs 27/80, baseline 19/80) but only marginally and only with
warm-start + the best seed (cold-start DR loses). The nominal champion transfers
surprisingly well and the heavy/long held-out arms are hard for everything.
**Remaining (Stage 2.5):** make the generalization *decisive* — curriculum
(easy→hard arms), fitness reweighting so hard arms aren't drowned, broader seed
statistics.

### Stage 3 — Goal-conditioning (changeable goal) *(shipped)*
Made the target a parameter: `linearize_about(goal)`, `balance_gain_for(goal)`,
`energy_at(goal)`, `equilibrium_torque(goal)`, and goal-conditioned `recover_to`
— with the upright functions now thin wrappers (no regression; `[π,π]` still
7/10). The **same** controller, given a different goal, reaches a different
equilibrium: `[π,π]` (both up, 7/10) and `[π,0]` (link-1 up, link-2 dangling,
4/10; the dead-hang case is solid). Reachable goals are exactly `{θ₀ free} ×
{θ₁ ∈ {0,π}}` (link 2 must be gravity-balanced — joint 1 is passive). Tests:
`reaches_link1_up_link2_down_goal`, `swings_up_from_a_dead_hang`. `check` prints
both goals. Posture handled in Stage 3.5 (below); goal-conditioned *energy*/spin targets
remain open.

### Stage 3.5 — Posture-aware swing-up *(shipped; clean win)*
The energy pump reached the goal *energy* but not its *posture*, so `[π,0]` only
caught 4/10. Diagnosis: 3 of 6 failures never reached the basin, 3 reached it but
arrived too fast to catch — so the fix needs *both* posture and damping.
`swingup_to(goal)` blends `v = k_e·(E_goal−E)·ω₀ − k_p·(θ₀−goal₀) − k_d·ω₀`. The
crux is a **goal-dependent** posture gain `k_p = 3·(1 + cos θ₁_goal)` — **0 when
link 2's target is up** (energy already pins posture; posture only fights — this
is the Stage-1 finding) **and 6 when it's down** (energy is ambiguous, posture is
essential). Result: **`[π,0]` 4/10 → 7/10 and `[π,π]` 7/10 → 8/10** — both
improved, no regression. Tests: `goal_conditioned_recovery_rates`. `swingup_pfl`
(the evolved-policy rollout pump) is left untouched.

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
