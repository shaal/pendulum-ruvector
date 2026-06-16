# Web experience — one page, all the demos, running in the browser

A single static page (hosted on **Cloudflare Pages**) where every pendulum ×
RuVector demo runs **entirely client-side in WebAssembly** — the physics, the
control, the learning, and RuVector's vector recall all happen in the visitor's
tab. No backend. The page is a friendly, explorable exhibit: a newcomer
understands what they're looking at from the label alone, and the curious can
expand any panel down to the real math and the exact technique.

This plan is the build spec. It assumes the WASM-feasibility work already
checked: RuVector ships browser builds (`ruvector-core` has a `memory-only`
feature; `ruvector-wasm` / `ruvector-gnn-wasm` exist), `RuVector/examples/vectorvroom`
is a static-WASM browser app we can crib from, and the only browser-hostile bits
in `pendulum_rs` are threads, file/DB I/O, and the Rerun viewer.

## Decisions (locked with the user)

| Question | Decision |
|---|---|
| Architecture | **HTML/CSS/TS UI shell + Rust→WASM "brain" + Canvas2D rendering.** HTML owns layout, text, controls; WASM owns physics/control/learning/RuVector; the (trivial) 2D arm drawing is rewritten in Canvas2D. |
| Aesthetic | **Playful science exhibit** — light, warm, rounded, big friendly labels, bouncy transitions, cartoony arms, tooltips. Museum-kiosk feel. |
| Explanation depth | **Two layers, skip the middle.** Top layer: dead-simple, zero jargon. Expanded layer: the *deepest* technical version (math, technique names, links to code + tests). Nothing in between. |
| Devices | **Desktop + mobile/touch.** Keyboard A/D on desktop; on-screen hold buttons on mobile; fully responsive layouts for every station. |

Out of scope: the Python testbed (`multi_link_pendulum/`) — the web build is the
Rust path only. Rerun is not used in the browser; we render with Canvas2D.

## Product shape — the exhibit

One page, a sticky top nav of **stations**, ordered as a narrative arc that
mirrors the project's own roadmap (chaos → control → memory → recovery →
discovery → competition → you-vs-it). Scroll or click a station; each is a
section with its live canvas, its controls, and its layered explanation.

Desktop layout per station: **canvas left, explanation+controls right**
(the chosen mock). Mobile: stacked, canvas on top, controls below, explanation
under that; touch controls overlay the canvas where needed.

### Stations (with build priority)

| # | Station | One-line label (top layer) | Priority |
|---|---|---|---|
| 0 | **The toy** (free swing) | "A chain of sticks that swings. Tiny changes blow up fast — that's chaos." | P1 |
| 1 | **Balance it** (`arm`) | "Hold the whole thing straight up by wiggling only the bottom joint." | P1 |
| 2 | **Recognize the change** (`estimate`) | "The arm changes. One of them *remembers* what to do. That memory is RuVector." | **P0** |
| 3 | **Get back up** (swing-up / `check`) | "Knock it down — watch it hoist itself back upright." | P1 |
| 4 | **Discover a trick** (`evolve`) | "Hundreds of attempts compete; the best way to swing up is *found*, not coded." | P1 |
| 5 | **A crowd that shares** (`popviz`) | "Many learners racing — and when one finds something good, RuVector hands it to the rest." | **P0** |
| 6 | **You vs RuVector** (`play`) | "You drive one arm with A/D. The other fixes itself. Try to keep up." | **P0** |

P0 = the three that *show off RuVector* and the manual challenge; ship these
first as the minimum compelling page. P1 fills out the arc.

## The two-layer explanation model

Every station renders the same content scaffold so the page feels coherent:

1. **Label** — one playful sentence (the table above).
2. **"What you're seeing"** — 2–3 plain sentences: what's on screen and what to
   watch for. No acronyms.
3. **"Where's the memory?" / "Where's the learning?"** — one plain sentence, only
   on stations where RuVector or learning is the point (2, 4, 5, 6). This is the
   thread that ties the whole page to RuVector.
4. **`▸ How it really works`** (collapsed) — the deepest layer: the actual
   technique, the math, why it's done that way, and links to the source symbol +
   the test that pins it. This is where we *don't* hold back.

Implementation: a reusable `<Station>` component takes `{ label, seeing, memory,
deepDive }`. The deep-dive is authored markdown (rendered to HTML at build time)
so it can carry equations (KaTeX), code spans, and links.

## Per-station content map (the substance)

For each station: **what the visitor sees**, **how it uses RuVector**, **the
learning / special technique** (the deep layer), and the **tunable parameters**.
Pulled from the real implementation so the copy stays honest.

### Station 0 — The toy (free swing)
- **See:** an n-link pendulum released and swinging; two near-identical starts
  drift apart within seconds.
- **RuVector:** none (this is the warm-up).
- **Deep:** hand-derived Lagrangian dynamics (`simulator::Pendulum`), semi-implicit
  integration at `dt`; sensitivity-to-initial-conditions as the motivation for why
  *memory of past dynamics* is valuable later. Show the divergence quantitatively
  (two traces).
- **Params:** links (2–4, re-init), per-link length & mass (re-init), damping
  (hot), gravity (hot), "release angle" + a "nudge by 0.001 rad" button to
  demonstrate chaos.

### Station 1 — Balance it (`arm`, adaptive vs naive)
- **See:** two underactuated 2-link arms holding straight up via only joint 0.
  Partway, link 2 changes length on both. The **naive** arm (stale gain) topples;
  the **adaptive** arm (recomputed gain) stays up.
- **RuVector:** not yet — here the adaptive arm is handed the new length by an
  oracle. This station sets up *why* recognition matters (next station replaces
  the oracle with RuVector).
- **Deep:** LQR from the linearized dynamics (`control::balance_gain`), why the
  balance gain is model-dependent, the unstable-equilibrium framing, what
  "underactuated" means and why one motor for two joints is hard.
- **Params:** new link-2 length (`--newlen`, hot via re-trigger), which arm is
  adaptive, disturbance time.

### Station 2 — Recognize the change (`estimate`) **[RuVector showcase]**
- **See:** the arm is disturbed; one arm runs a brief "probe" wiggle, then
  recognizes itself and recovers. A HUD shows: probe → nearest stored config →
  adopted gain → recognition lag (~0.38 s cold). Throw the *same* disturbance
  again and the lag drops (~0.15 s) — it learned.
- **RuVector:** **this is the core RuVector loop.** A grid of arm "signatures" is
  seeded into an in-memory RuVector store (`memory::ConfigMemory`). On disturbance
  the live signature is `search`ed against the store; the nearest entry's gain is
  adopted. After a successful catch the *measured* signature is `insert`ed back —
  self-learning that shrinks the next lag ~60%.
- **Deep:** the dynamics signature = closed-loop linearization `A − b·K` under a
  fixed probe gain (`estimator`); why closed-loop (the online regression measures
  exactly those coefficients); the **dithered probe** (multi-sine exogenous
  torque) that makes the input column identifiable; honest operating envelope
  (structural/length change up to ~2.2 m; mass/length confound under noise). Link
  `estimator.rs`, `memory.rs`, tests.
- **Params:** new length (`NEW_L1`, hot), sharing/self-learning on/off, "show the
  probe math" toggle (the `PROBE_DEBUG` view: distance, matched cfg, consensus),
  reset memory.

### Station 3 — Get back up (collocated-PFL swing-up, `check`)
- **See:** pick a knockdown (small poke … dead hang … both folded); the arm pumps
  energy and swings up, then the LQR catches it at the top. Some hard starts fail
  — shown honestly.
- **RuVector:** optional — recall the swing-up *policy* for this arm
  (`learn::rollout_recalling_policy`); off by default at this station.
- **Deep:** collocated partial-feedback linearization (`control::swingup_pfl`):
  solve the passive-joint row for `q̈₁` from `q̈₀`, command `q̈₀ = v` to
  feedback-linearize joint 0; outer energy pump `v = k_e·(E_up−E)·ω₀`; goal-
  conditioning (`recover_to`, reachable goals `{θ₀ free}×{θ₁∈{0,π}}`); posture-
  aware gain `k_p = 3·(1+cos θ₁_goal)`. Honest result: 7/10 baseline harness.
- **Params:** knockdown type, goal posture ([π,π] both up vs [π,0] elbow-down),
  energy-pump gain, run the full 10-start harness and show the scoreboard.

### Station 4 — Discover a trick (`evolve`)
- **See:** a live fitness curve climbing over generations; the current champion
  arm swinging up better each round. Baseline 7/10 → evolved champion up to 10/10.
- **RuVector:** the winning policies are stored in RuVector (per-arm library,
  `LIBRARY` path) so later stations/arms can recall them.
- **Deep:** gradient-free **cross-entropy method** over a linear energy-shaping
  policy (`learn::EnergyShapingPolicy`); features it learns that hand-tuning
  missed (passive-joint pump, posture-sin, velocity damping); domain randomization
  (`RANDOMIZE_ARM`) and the **honest ceiling finding** — no single policy
  generalizes; union ceiling 42/80 ≫ any single 28/80, so the lever is per-arm
  *recall*, not one universal policy. Link `evolutionary-swingup.md`, tests.
- **Params:** seed (`SEED`, deterministic + shareable), domain-randomize on/off,
  fitness target, population size, speed (generations/sec budget).

### Station 5 — A crowd that shares (`popviz`) **[RuVector showcase]**
- **See:** a grid of live arms, one per "island," each driven by that island's
  current best. Arms tint red→green by skill; the overall best is haloed. Toggle
  **sharing**: on, a weak island visibly inherits the global best the instant a
  migration fires (a flash); off, stragglers keep flailing. A counter shows total
  practice attempts — sharing reaches competence in far fewer.
- **RuVector:** **the population learns *through* RuVector.** Each island writes
  its champion into a shared in-memory RuVector store every few generations and
  migrates the global best back (`learn::PopulationSim`, `set_sharing`). Sharing
  vs not is the only difference at equal seed → up to ~80% fewer total rollouts.
- **Deep:** island-model evolution, migration topology, why deliberately-weak
  searchers + shared memory beats independent search; the rollout-count metric;
  determinism via splitmix64. Link `PopulationSim`, the
  `ruvector_sharing_accelerates_the_population` test.
- **Params:** #islands, candidates/island, knockdowns/eval, migrate interval,
  sharing on/off (hot), seed, speed budget, restart.

### Station 6 — You vs RuVector (`play`) **[manual challenge]**
- **See:** two arms. **You** drive the left arm's base motor (A/D on desktop,
  on-screen ◀ ▶ on mobile) and try to balance it. The right arm balances itself
  and recalibrates on disturbance. Buttons throw disturbances (poke, wind,
  payload). A scoreline tracks who's holding upright. Balancing it by hand is
  brutal — that's the point.
- **RuVector:** the auto arm uses the Station-2 recognition pipeline live (real
  RuVector recall on disturbance) when built with the vectordb path; otherwise an
  oracle. The web build uses the real RuVector path.
- **Deep:** same control + recognition stack as Stations 1–2; what makes manual
  control of an underactuated arm so hard (you command joint 0, physics couples
  it to a passive joint 1).
- **Params:** disturbance buttons (poke ←/→, wind W, payload M), reset R,
  difficulty (torque limit / disturbance strength).

## Architecture

```
┌──────────────────────── index.html (one page) ────────────────────────┐
│  HTML + CSS (playful exhibit)   ·   TS app shell   ·   <Station> ×7     │
│        │ sliders / buttons / A-D / touch          ▲ flat state arrays   │
│        ▼ set_param / step / input                 │ (positions, fitness)│
│  ┌───────────────────────── WASM module (one .wasm) ─────────────────┐ │
│  │  pendulum_web (cdylib, wasm-bindgen)                               │ │
│  │    wraps pendulum_rs lib  +  ruvector-core (memory-only)  + (gnn)  │ │
│  │    per-station handles: FreeSwing, BalanceDuel, Recalibrator,      │ │
│  │    Recover, Evolver, Population, Duel                              │ │
│  └───────────────────────────────────────────────────────────────────┘ │
│  Canvas2D renderer (TS) draws arms from the flat position arrays         │
└─────────────────────────────────────────────────────────────────────────┘
```

### WASM core API (wasm-bindgen sketch)

One stateful handle per station; JS steps it and reads flat arrays. Illustrative:

```rust
#[wasm_bindgen]
pub struct Population { /* owns PopulationSim + display arms */ }

#[wasm_bindgen]
impl Population {
    #[wasm_bindgen(constructor)]
    pub fn new(seed: u64, islands: usize, pop: usize, cases: usize, migrate: usize) -> Population;
    pub fn set_sharing(&mut self, on: bool);
    /// advance physics of the live arms by `steps` fixed dt
    pub fn tick(&mut self, steps: usize);
    /// do a bounded slice of evolution work; returns true if a generation completed
    pub fn evolve_slice(&mut self, budget_rollouts: usize) -> bool;
    /// flat [x0,y0,x1,y1,...] for all arms, for Canvas2D
    pub fn positions(&self) -> Box<[f64]>;
    pub fn fitnesses(&self) -> Box<[f64]>;
    pub fn best_island(&self) -> usize;
    pub fn migrated_pulse(&mut self) -> bool;
    pub fn restart(&mut self);
}
```

Same pattern for the others (`Recalibrator.disturb(new_l1)` → `recognition()`
returns lag + nearest cfg + learned flag; `Duel.input(dir)` / `disturb(kind)` /
`scores()`; `Evolver.evolve_slice(budget)` / `champion()` / `fitness_curve()`).

Physics and RNG stay **in Rust** so runs are deterministic and reproducible
(splitmix64, seeded). JS only renders and forwards input.

### Cooperative scheduler (the key to staying at 60 fps single-threaded)

The expensive work is evolution (CEM rollouts), not the live physics. Native code
parallelizes it across cores with `thread::scope`; the browser is single-threaded.
So we **time-slice**: each `requestAnimationFrame`, run live physics (cheap) to
real-time, then call `evolve_slice(budget)` with a budget tuned so the frame stays
under ~8 ms. Evolution advances a little each frame; the UI never blocks. A
"speed" slider trades evolution pace for smoothness.

### RuVector in the browser

- Link `ruvector-core` with **`memory-only`** (no `redb`/`memmap2`/`std::fs`).
- The pendulum stores are tiny (≤30 configs, ≤8 islands), so an in-memory **flat
  / brute-force search is exact and instant** — and avoids `hnsw_rs`'s `memmap2`
  dependency. Decision: v1 uses `memory-only` core with flat search (or
  `micro-hnsw-wasm` if we want the HNSW path); either way it is *genuinely
  RuVector* doing insert/search, which keeps the "it uses RuVector" claim honest.
- `memory::ConfigMemory` is refactored to take an in-memory store instead of a
  file `storage_path` (see Porting).
- **GNN interpolation (Station 4/2 deep extra) is staged later** — `ruvector-gnn`
  has a hard `rayon` dependency; use `ruvector-gnn-wasm` (built with its `wasm`
  feature) or defer the attention-blend feature past v1. Flat vector recall does
  not need it.

### State, determinism, share links

Every station's config (params + seed) serializes to the URL query string, so a
visitor can tweak sliders and **share an exact reproducible run**. Deterministic
RNG makes this real, and it doubles as our manual-test fixture.

## Porting work from the native crate

`pendulum_rs` core logic already lives in the library (`lib.rs`, `control`,
`learn`, `memory`, `estimator`, `simulator`); the binaries are thin. The web crate
calls the same functions. What must change:

| Concern | Where (native) | Browser fix |
|---|---|---|
| Background thread | `popviz.rs:55` `thread::spawn`, `:83` `sleep` | Drive evolution from rAF via `evolve_slice` budget; no thread, no sleep |
| Parallel rollouts | `evolve` `thread::scope` | Single-threaded slice loop in v1; `wasm-bindgen-rayon` later (see Perf) |
| File-backed RuVector | `memory.rs` `storage_path` `.db`, `std::fs::remove_file` | In-memory store; `ConfigMemory::new_in_memory()` |
| Rerun viewer | `main.rs`/`arm.rs`/`estimate.rs` `.spawn()/.save()` | Not used; Canvas2D renders from position arrays |
| CSV writes | `main.rs`, `arm.rs` `--csv` | Drop (or return arrays to JS for a "download data" button) |
| `rand::gen_range` | `popviz.rs` knockdown | Reuse the splitmix64 in `learn.rs`; add `getrandom` `js` feature only if needed |
| Time | `std::time` / sleeps | Frame timing from `performance.now()` via rAF; no `Instant` in core |

Net: a new `pendulum_web` crate + small refactors to make `ConfigMemory` and the
evolution loop storage-agnostic and incremental. No physics/control/learning
rewrite.

## Performance plan

- **v1 single-threaded** with the cooperative scheduler. Tune budgets so live
  stations hit 60 fps and evolution stations advance visibly without jank.
  Shrink popviz defaults if needed (fewer islands/cases) for a smooth phone.
- **wasm size:** `ruvector-core` + crate could be large. Build with
  `opt-level="s"`/`"z"`, `lto=true`, `codegen-units=1`, run `wasm-opt -Oz`. Lazy-
  load the module before first canvas paint; show a friendly loader. Budget a
  target (<~2–3 MB gz) and measure.
- **Threads later (optional):** recover real parallelism for evolution with
  `wasm-bindgen-rayon` + `SharedArrayBuffer`. That needs `Cross-Origin-Opener-
  Policy: same-origin` + `Cross-Origin-Embedder-Policy: require-corp`, which
  Cloudflare Pages can serve via a `_headers` file. Tradeoff: COEP can block
  third-party embeds; only enable if the single-threaded pace proves too slow.

## Tech stack & project layout

- **Rust:** new workspace crate `pendulum_web/` (`crate-type = ["cdylib"]`),
  `wasm-bindgen`, depends on `pendulum_rs` (lib) + `ruvector-core` (`memory-only`).
  Build with `wasm-pack build --target web`.
- **Frontend:** **Vite + TypeScript + Svelte** (light, reactive, great for the
  station components and sliders; minimal bundle). Vanilla TS is a fine fallback.
  KaTeX for the deep-layer math; a tiny motion lib (e.g. `motion`/`@motionone`)
  for the bouncy transitions. Markdown deep-dives compiled at build time.
- **Rendering:** Canvas2D per station (one `<canvas>` each, drawn from the WASM
  position arrays); fitness curves on a small Canvas2D chart (no heavy chart dep).

```
pendulum-ruvector/
├── pendulum_rs/            # existing crate (lib refactors for in-memory store)
├── pendulum_web/           # NEW: wasm-bindgen cdylib wrapping the lib + ruvector
│   ├── Cargo.toml
│   └── src/lib.rs          # per-station handles
└── web/                    # NEW: Vite + Svelte + TS frontend
    ├── index.html
    ├── src/
    │   ├── stations/       # one component per station + its markdown deep-dive
    │   ├── render/         # Canvas2D arm + chart renderers
    │   ├── wasm/           # generated pkg/ from wasm-pack
    │   └── app.ts
    ├── public/_headers     # (only if/when threads are enabled)
    └── package.json        # build: wasm-pack build && vite build  →  dist/
```

## Build & deploy (Cloudflare Pages)

1. `wasm-pack build pendulum_web --target web --release` → `pkg/` (.wasm + JS glue).
2. `vite build` (frontend imports `pkg/`) → `web/dist/`.
3. Cloudflare Pages: connect the repo; **build command** `npm run build` (wraps
   steps 1–2, needs Rust + wasm-pack in the build image, or commit a prebuilt
   `pkg/` like `vectorvroom` does to avoid a Rust toolchain in CI), **output
   directory** `web/dist`. Pure static — no Workers, no Functions. (Optional KV/D1
   + a Worker only if we later want a shared leaderboard.)

## Milestones

1. **M0 — Spike (de-risk the whole idea).** `pendulum_web` crate that exposes one
   trivial handle (FreeSwing); Canvas2D renders it; deploy a one-canvas page to
   Cloudflare Pages. Proves Rust→wasm-bindgen→Canvas2D→Pages end-to-end and
   measures wasm size. *Smallest thing that proves the pipeline.*
   **✅ Done (2026-06-16).** Built end-to-end: `rerun` made optional in
   `pendulum_rs` (lib now wasm-clean), `pendulum_web` cdylib wraps the physics +
   in-memory `ruvector-core` (memory-only), Svelte+Vite shell renders the live arm
   on Canvas2D, RuVector insert/search runs in-tab (verified in a browser).
   Energy-conservation tests added as the physics/parity guard. **Bundle: wasm
   ≈ 59 KB gz, whole page ≈ 83 KB gz** — far under budget, so the remaining
   stations are unconstrained by size. The libm-for-transcendentals change was
   *deferred* (it perturbs the seeded chaotic trajectories and would need the
   pinned champion tests re-pinned — do it deliberately with the share-links work,
   not in the spike).
2. **M1 — RuVector in the tab.** Refactor `ConfigMemory` to in-memory; ship
   **Station 2 (Recognize)** with the real seed→probe→recall→self-learn loop and
   the layered explanation. This is the proof the "brain runs in the browser."
3. **M2 — The crowd + the duel.** **Station 5 (popviz)** with the cooperative
   scheduler + sharing toggle, and **Station 6 (You vs RuVector)** with keyboard +
   touch A/D. Now all three P0 stations are live.
4. **M3 — Fill the arc.** Stations 0, 1, 3, 4; the fitness chart; share-links.
5. **M4 — Exhibit polish.** Playful theme, transitions, mobile passes, loader,
   wasm-opt size pass, accessibility (keyboard nav, reduced-motion, alt text),
   copy edit of every deep-dive.
6. **M5 — (optional) Threads** if evolution pace needs it (`_headers` + rayon).

## Risks & mitigations (critical thinking)

- **wasm-bindgen vs the macroquad code.** We deliberately *don't* reuse macroquad
  for the web (it has its own non-bindgen JS bootstrap that fights an HTML shell).
  Rendering ~30 lines of line/circle drawing in Canvas2D is cheaper than bridging
  macroquad. The native `play`/`popviz` binaries stay as-is for desktop.
- **`ruvector-gnn` + rayon on wasm.** Real risk. Mitigation: v1 uses flat vector
  recall (no GNN), which covers Stations 2/5/6 fully; GNN interpolation is a
  later, optional deep-layer extra via `ruvector-gnn-wasm`.
- **HNSW's `memmap2`.** Avoid by using `memory-only` + flat search at our tiny N,
  or `micro-hnsw-wasm`. Either is honestly "RuVector."
- **Single-thread evolution feels slow.** Mitigated by time-slicing + tunable
  speed + smaller defaults; threads as the escape hatch (M5).
- **wasm bundle too big / slow first paint.** Size pass (opt-z, lto, wasm-opt),
  lazy load, friendly loader; measure at M0 before committing to all 7 stations.
- **"Uses RuVector" must be real, not decorative.** Stations 2/5/6 genuinely call
  RuVector insert/search every run; the copy says exactly where. No fake memory.
- **Mobile A/D.** On-screen hold buttons + larger hit targets; test the duel on a
  real phone early (M2), since touch latency could make it unfair/unfun.
- **Cloudflare build image lacks Rust.** Either add the toolchain to the build, or
  commit a prebuilt `pkg/` (the `vectorvroom` approach) so Pages only runs Vite.

## Open questions (for the user, lower-stakes — sensible defaults assumed)

1. **Frontend framework:** Svelte (my default) vs vanilla TS vs React? (Affects
   nothing architectural; Svelte keeps it small and fast to build.)
2. **CI toolchain:** OK to commit a prebuilt `pkg/` so Cloudflare needn't run
   Rust (simplest, mirrors `vectorvroom`), or add Rust to the Pages build?
3. **Branding:** page name/title, link back to the GitHub repo, any logo? Analytics?
4. **Persistence/leaderboard** for the duel high scores — keep it pure-static
   (no), or add a tiny Worker + KV later? (Default: no, stays backend-free.)
5. **Sound** — subtle SFX/clicks fit the playful exhibit; want them, or silent?
6. **Fidelity vs phone smoothness** — if we must choose, do we shrink popviz on
   mobile (fewer islands) or just lower its speed? (Default: shrink on mobile.)
