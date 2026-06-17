//! Where does RuVector plan-memory actually help the predictive planner?
//!
//! A good initial guess matters most when the search is too small to find a
//! plan on its own. So we sweep the CEM population size (the planner's budget)
//! at a fixed single refinement iteration, and at each budget compare:
//!
//!   * `cold` — no memory: the planner is on its own.
//!   * `warm` — every controller shares one growing RuVector store of past
//!              plans; each re-plan injects the nearest remembered plan as an
//!              extra candidate (strictly safe — it can only win or be ignored).
//!
//! The honest expectation: at a **tiny budget** the cold search is weak (slow,
//! flaily, may miss) and recall rescues it; at a **large budget** the search
//! already finds good plans, so memory converges to no effect. That contrast is
//! the point — RuVector buys the most where compute is scarce, which is exactly
//! the regime a browser / embedded controller lives in.
//!
//!   cargo run --release --features vectordb --bin mpc_warmstart

use std::cell::RefCell;
use std::rc::Rc;

use pendulum_rs::learn::knockdown_starts;
use pendulum_rs::mpc::{rollout_metrics, Metrics, MpcConfig, MpcSwingUp, PlanMemory};

const SECS: f64 = 15.0;

/// Run every knockdown at a given planner budget, optionally sharing a plan
/// memory. Returns the per-start metrics and total candidate rollouts.
fn run(pop: usize, elite: usize, iters: usize, mem: Option<Rc<RefCell<PlanMemory>>>) -> (Vec<Metrics>, u64) {
    let mut total = 0u64;
    let ms = knockdown_starts()
        .iter()
        .map(|(_, t)| {
            let cfg = MpcConfig { pop, elite, iters, ..MpcConfig::default() };
            let mpc = match &mem {
                Some(m) => MpcSwingUp::with_memory(cfg, m.clone()),
                None => MpcSwingUp::new(cfg),
            };
            let m = rollout_metrics(1.0, 1.0, 0.05, t, &mpc, SECS);
            total += mpc.planning_rollouts();
            m
        })
        .collect();
    (ms, total)
}

/// `(recovered, avg time-to-catch, avg reversals)` over the caught runs.
fn stats(ms: &[Metrics]) -> (usize, f64, f64) {
    let caught = ms.iter().filter(|m| m.caught).count();
    let n = caught.max(1) as f64;
    let avg_t = ms.iter().filter(|m| m.caught).map(|m| m.time_to_catch).sum::<f64>() / n;
    let avg_r = ms.iter().filter(|m| m.caught).map(|m| m.reversals as f64).sum::<f64>() / n;
    (caught, avg_t, avg_r)
}

fn cell(ms: &[Metrics]) -> String {
    let (c, t, r) = stats(ms);
    format!("{c}/{} / {t:.1}s / {r:.0}rev", ms.len())
}

fn main() {
    println!("Where does RuVector plan-memory help? Planner-budget sweep (CEM iters=1).\n");
    println!("  {:<6}{:>26}{:>26}", "pop", "cold (no memory)", "warm (shared memory)");
    println!("  {:<6}{:>26}{:>26}", "", "rec / time / reversals", "rec / time / reversals");

    for &pop in &[8usize, 16, 32, 64] {
        let elite = (pop / 4).max(2);
        let (cold, _rc) = run(pop, elite, 1, None);

        let path = std::env::temp_dir().join(format!("mpc_mem_{pop}.db")).to_string_lossy().into_owned();
        let horizon = MpcConfig::default().horizon;
        let mem = Rc::new(RefCell::new(PlanMemory::new(&path, horizon, 0.4).expect("open plan memory")));
        let (warm, _rw) = run(pop, elite, 1, Some(mem));

        println!("  {:<6}{:>26}{:>26}", pop, cell(&cold), cell(&warm));
    }

    println!("\n  Read across each row: where warm beats cold (more recovered, faster,");
    println!("  fewer reversals), RuVector recall is supplying plan quality the small");
    println!("  search could not find on its own. Where they match, the search already");
    println!("  suffices and memory is free insurance.");
}
