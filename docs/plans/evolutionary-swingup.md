# Plan — Evolutionary swing-up search (Stage 1 of the learning roadmap)

> Status: **planned**. This doc is written so a future session can resume cold.
> Use the **ship-task** skill when implementing this (the user asked for it).

## Goal

Discover swing-up controllers the hand-designed PFL controller can't, by running
a **population of hundreds of candidate policies competing toward the goal**
(keep the arm upright), scored on the recovery harness. Beat the current
baseline of **7/10** knockdowns (`cargo run --bin check`), and — the real point —
go from a system that *adapts* (recognize → recall a known controller) to one
that *discovers* (finds controllers nobody designed).

This is the user's "hundreds of versions sent to learn/predict, competing to
reach the goal faster" idea, scoped to the part where it actually pays off.

## Why evolutionary search (ES), not deep RL

- The sim is **deterministic Rust at microseconds/step** → thousands of full
  rollouts per second on one CPU, no GPU. Population search is cheap here.
- **Gradient-free**: no autodiff/ML stack to bolt on; fits the dependency-light
  crate. CMA-ES or a simple (μ, λ)-ES / cross-entropy method is enough.
- **Embarrassingly parallel** = literally "hundreds competing." Maps 1:1 to the
  vision and to `std::thread`/`rayon`.
- RL from scratch would fight the already-optimal LQR (see below) and is
  sample-hungry. **Keep model-based control for what it wins; learn only the
  hard regime.**

## Hybrid architecture (do NOT replace the LQR)

```
state ── tip_err < basin ? ──> LQR balance (unchanged, optimal, instant)
                            └─> LEARNED swing-up policy  (this project)
```

The LQR catch stays. We only learn the swing-up policy that runs when the arm is
knocked outside the LQR basin. The handoff is the existing `recover_torque`
switch (`tip_err < 1.0`); the learned policy replaces `swingup_pfl` in the
`else` branch (keep `swingup_pfl` as a fallback / baseline to beat).

## Policy parameterization (start simple)

Two candidate forms, in increasing power — start with (A):

- **(A) Parameterized energy-shaping law.** Evolve a small vector of gains/shape
  params around the current pump, e.g.
  `u = clamp( M̄·(kE·(E_up−E)·ω₀ + kEE·(E_up−E)·sin θ₀ + kω·ω₀ + …) + h̄ )`.
  ~4–8 parameters. Cheap, interpretable, a strict superset of today's hand-tuned
  `k_e=20`. Almost certainly beats 7/10 and is low-risk. **Do this first.**
- **(B) Tiny neural policy.** A 2-layer MLP (input: `[sinθ, cosθ, ω, E−E_up]`,
  output: joint-0 torque), ~50–200 weights, evolved by ES. More expressive,
  still gradient-free. Only attempt if (A) plateaus.

Keep the policy `dt`-agnostic and clamp to `U_MAX`.

## Fitness (reward) — physics-grounded, not clever

Per candidate, average over a **batch of randomized knockdown starts** (and
randomized arm configs — domain randomization, see below):

```
fitness = mean over rollouts of:
    + bonus if caught upright (tip_err < 0.2 sustained for last 1s)
    − time_to_catch            (faster is better)
    − ∫ tip_err dt             (potential-based shaping: reward reducing
                                distance-to-upright; provably doesn't move the
                                optimum)
    − small effort penalty ∫ u² dt   (discourage frantic bang-bang if unneeded)
```

Use **potential-based shaping** (reward the *decrease* in tip-error/energy
deficit), not raw progress, so the optimum is preserved. Avoid reward hacking:
require *sustained* upright, not a single frame passing through it.

## Domain randomization (where generalization comes from)

Each rollout (or each candidate's batch) uses a **randomized arm config**
sampled from / around the seeded grid (`l1, m1, b1`) and a randomized knockdown
start. A policy that survives the distribution generalizes to arms it never saw
— a stronger generalization than the Phase-3 GNN interpolation, and it composes
with it (recall the nearest learned policy, then run it).

## RuVector's role — the shared/collective memory

This is what makes "hundreds competing" more than N independent searches:

- After evolution, store each **winning policy keyed by the arm/start signature**
  in RuVector (extend `ConfigMemory`: payload = evolved policy params, not just
  the LQR gain). 
- At runtime, an arm recalls the **nearest learned swing-up policy** to
  warm-start — exactly the Phase-2/3 recall pipeline, now over discovered skills.
- Optional later: agents in a live population write good trajectories to a shared
  store mid-search and retrieve peers' successes (cross-agent transfer).

## Suggested implementation steps (each = one ship-task pass)

1. **Harness + fitness as a library fn.** Factor `recovery_test`-style rollout
   into `pendulum_rs::lib` (not just the `check` bin) so ES can call it. Add a
   `Policy` trait (`fn torque(&self, sim) -> f64`) with the energy-shaping form
   (A) behind it; today's `swingup_pfl` becomes one `Policy` impl (the baseline).
2. **ES driver.** A new bin `evolve` (feature-gated, `rayon` optional): sample a
   population of param vectors, evaluate fitness in parallel over randomized
   starts, select/recombine (CMA-ES or cross-entropy), iterate. Log best fitness
   per generation and the final recovery count on the fixed `check` harness.
3. **Beat the baseline.** Target > 7/10 on the harness; report honestly. Plot/log
   the learning curve (fitness vs generation) — that curve is the deliverable.
4. **Store winners in RuVector** keyed by config signature; add a recall path so
   `recover_torque` can adopt a learned policy. Test: a recalled learned policy
   stabilizes an arm the hand-tuned one fails on.
5. **Domain randomization + generalization test.** Evolve over randomized arms;
   assert the single evolved policy recovers across a held-out set of configs.
6. **(Optional) Game payoff.** Surface "the RuVector arm is *learning*" — e.g.
   the auto arm visibly improves across rounds, or loads an evolved policy.

## Honest expectations / risks

- ES will very likely push past 7/10 and gives a clean "population learning"
  curve, but **full 10/10 mastery of double-pendulum swing-up from any state is
  not guaranteed** — it's a known-hard, chaotic, underactuated problem. Frame
  Stage 1 as "beat 7/10 + show learning," not "solve it."
- Watch for **reward hacking** (passing through upright, frantic spinning that
  scores progress) — the sustained-upright bonus + effort penalty + potential
  shaping guard against it.
- Keep determinism: the sim forbids `Math.random`-style nondeterminism in some
  contexts; seed the ES RNG explicitly and pass seeds in, so runs reproduce.
- **Scope discipline**: keep the crate dependency-light. CMA-ES/CEM in a few
  hundred lines of Rust beats pulling a heavy framework.

## Definition of done (Stage 1)

- `evolve` bin produces a learning curve and a champion policy.
- Champion beats `swingup_pfl` on the `check` harness (report the number).
- Champion stored in / recalled from RuVector and stabilizes a held-out arm.
- Honest write-up of where it still fails.

See [`learning-roadmap.md`](learning-roadmap.md) for how this fits the larger
vision (goal-conditioning, residual RL, live populations).
