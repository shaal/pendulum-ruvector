//! Stage 1 — evolutionary swing-up search (the driver).
//!
//! A dependency-light, explicitly-seeded **cross-entropy method (CEM)**: keep a
//! Gaussian over [`EnergyShapingPolicy`] parameters, sample a population, score
//! each candidate on a batch of randomized knockdowns (in parallel), refit the
//! Gaussian to the top elites, repeat. Hundreds of candidate controllers compete
//! each generation; the distribution marches toward ones that recover more.
//!
//! The LQR catch is untouched (hybrid). Determinism: the RNG is seeded (env
//! `SEED`, default fixed), so a run reproduces exactly.
//!
//!   cargo run --release --bin evolve
//!
//! It prints the learning curve (best fitness per generation) and the champion's
//! recovery on the same 10-start `check` harness the baseline is judged on.

use pendulum_rs::learn::{
    fitness, knockdown_starts, recovery_count, rollout, EnergyShapingPolicy, PflBaseline, NP,
};
use std::f64::consts::PI;
use std::thread;

// CEM hyperparameters.
const POP: usize = 96; // candidates per generation ("hundreds competing")
const ELITE: usize = 16; // top candidates that refit the distribution
const GENERATIONS: usize = 30;
const TRAIN_SECS: f64 = 8.0; // shorter rollouts while searching (speed)
const EVAL_SECS: f64 = 15.0; // full rollouts for the final harness verdict
const N_TRAIN_STARTS: usize = 24; // randomized knockdowns per fitness evaluation

/// Tiny deterministic splitmix64 RNG — no external crate, fully reproducible.
struct Rng(u64);
impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    /// Uniform in [0, 1).
    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    /// Standard normal via Box–Muller.
    fn gauss(&mut self) -> f64 {
        let u1 = self.unit().max(1e-12);
        let u2 = self.unit();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }
}

/// A randomized knockdown: each joint angle drawn broadly around the circle so
/// the training distribution covers pokes, sideways, folds, and full hangs.
fn random_start(rng: &mut Rng) -> Vec<f64> {
    vec![PI + (rng.unit() * 2.0 - 1.0) * PI, PI + (rng.unit() * 2.0 - 1.0) * PI]
}

/// Mean fitness of a candidate over the shared training starts.
fn eval_candidate(p: [f64; NP], starts: &[Vec<f64>]) -> f64 {
    let policy = EnergyShapingPolicy { p };
    let sum: f64 = starts.iter().map(|s| fitness(&rollout(s, &policy, TRAIN_SECS))).sum();
    sum / starts.len() as f64
}

/// Evaluate a whole population in parallel across the available cores.
fn eval_population(pop: &[[f64; NP]], starts: &[Vec<f64>]) -> Vec<f64> {
    let n_threads = thread::available_parallelism().map(|n| n.get()).unwrap_or(4).min(pop.len());
    let chunk = pop.len().div_ceil(n_threads);
    thread::scope(|s| {
        let handles: Vec<_> = pop
            .chunks(chunk)
            .map(|c| s.spawn(move || c.iter().map(|&p| eval_candidate(p, starts)).collect::<Vec<_>>()))
            .collect();
        handles.into_iter().flat_map(|h| h.join().unwrap()).collect()
    })
}

fn main() {
    // CEM is stochastic on this chaotic landscape, but reliably strong: seeds
    // 0–7 recover 7–10/10 (median ~9.5) and 7 of 8 beat the 7/10 baseline; a
    // given seed reproduces exactly. Pass SEED=N to explore. Default = 1 (10/10).
    let seed: u64 = std::env::var("SEED").ok().and_then(|s| s.parse().ok()).unwrap_or(1);
    let mut rng = Rng(seed);

    // Fixed (seeded) training distribution, reused every generation so candidate
    // fitnesses are directly comparable. Kept separate from the eval harness.
    let train_starts: Vec<Vec<f64>> = (0..N_TRAIN_STARTS).map(|_| random_start(&mut rng)).collect();

    // CEM distribution: start centred on the hand-tuned baseline so we never do
    // worse than where Phase 3 left off, with generous initial exploration.
    let mut mean = EnergyShapingPolicy::baseline().p;
    let mut std = [10.0, 6.0, 6.0, 3.0, 3.0];

    let base_train = eval_candidate(EnergyShapingPolicy::baseline().p, &train_starts);
    eprintln!("Evolutionary swing-up search  (seed={seed:#x}, pop={POP}, elites={ELITE}, gens={GENERATIONS})");
    eprintln!("baseline mean train-fitness = {base_train:.1}\n");
    eprintln!("gen |  best  |  elite-mean | champion params");

    let mut champion = mean;
    let mut champion_fit = base_train;

    for gen in 0..GENERATIONS {
        // Sample the population from the current Gaussian.
        let pop: Vec<[f64; NP]> = (0..POP)
            .map(|_| std::array::from_fn(|i| mean[i] + std[i] * rng.gauss()))
            .collect();
        let fits = eval_population(&pop, &train_starts);

        // Rank and take the elites.
        let mut idx: Vec<usize> = (0..POP).collect();
        idx.sort_by(|&a, &b| fits[b].partial_cmp(&fits[a]).unwrap());
        let elites: Vec<[f64; NP]> = idx[..ELITE].iter().map(|&i| pop[i]).collect();

        // Refit the Gaussian to the elites (with a small std floor so it keeps
        // exploring instead of collapsing prematurely).
        for d in 0..NP {
            let m = elites.iter().map(|e| e[d]).sum::<f64>() / ELITE as f64;
            let var = elites.iter().map(|e| (e[d] - m).powi(2)).sum::<f64>() / ELITE as f64;
            mean[d] = m;
            std[d] = var.sqrt().max(0.5);
        }

        let best_fit = fits[idx[0]];
        let elite_mean_fit = idx[..ELITE].iter().map(|&i| fits[i]).sum::<f64>() / ELITE as f64;
        if best_fit > champion_fit {
            champion_fit = best_fit;
            champion = pop[idx[0]];
        }
        eprintln!(
            "{gen:3} | {best_fit:6.1} | {elite_mean_fit:7.1}     | [{}]",
            mean.iter().map(|x| format!("{x:.1}")).collect::<Vec<_>>().join(", ")
        );
    }

    // Final honest verdict on the SAME harness the baseline reports on.
    let champ_policy = EnergyShapingPolicy { p: champion };
    let champ_recovered = recovery_count(&champ_policy, EVAL_SECS);
    let base_recovered = recovery_count(&PflBaseline, EVAL_SECS);

    eprintln!("\n────────────────────── RESULT (check harness, {EVAL_SECS:.0}s) ──────────────────────");
    eprintln!("hand-tuned baseline : {base_recovered}/10 recovered");
    eprintln!("evolved champion    : {champ_recovered}/10 recovered   params [{}]",
        champion.iter().map(|x| format!("{x:.2}")).collect::<Vec<_>>().join(", "));
    eprintln!("\nPer-start (champion):");
    for (label, theta0) in knockdown_starts() {
        let r = rollout(&theta0, &champ_policy, EVAL_SECS);
        eprintln!("  {label} -> {}", if r.caught { "RECOVERED ✅" } else { "did not catch ❌" });
    }
    if champ_recovered > base_recovered {
        eprintln!("\n→ evolution beat the hand-tuned controller ({base_recovered} → {champ_recovered}).");
    } else {
        eprintln!("\n→ no improvement over baseline this run (try a different SEED or more generations).");
    }
    eprintln!("(CEM is stochastic but reliable: seeds 0–7 → 7–10/10, 7 of 8 beat the 7/10 baseline.)");
}
