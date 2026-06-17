//! Reactive vs. predictive swing-up — the head-to-head the MPC prototype exists
//! to settle.
//!
//! Three swing-up brains run the *identical* recovery harness (LQR catch + same
//! basin threshold + same canonical knockdown starts); only the out-of-basin
//! swing-up differs:
//!
//!   * `PFL baseline`   — hand-tuned collocated-PFL energy pump (reactive).
//!   * `energy-shaping`  — the evolved 5-param champion (reactive, learned).
//!   * `MPC (predict)`   — receding-horizon CEM that rolls the real dynamics
//!                         forward and plans (predictive).
//!
//! For each we report: how many of the 10 knockdowns it recovers, and — on the
//! starts *all three* solve, so it's apples-to-apples — the average time to
//! catch, the control effort `∫|u|`, and the number of torque reversals (the
//! literal "how many back-and-forth moves" count).

use pendulum_rs::learn::{knockdown_starts, EnergyShapingPolicy, PflBaseline, SwingUpPolicy, NOMINAL_CHAMPION};
use pendulum_rs::mpc::{rollout_metrics, Metrics, MpcConfig, MpcSwingUp};

const SECS: f64 = 15.0;

/// Run every knockdown start under one swing-up policy. A fresh policy instance
/// per start keeps each recovery independent (the MPC planner carries warm-start
/// state, which must not leak across scenarios).
fn run_all<F: Fn() -> P, P: SwingUpPolicy>(make: F) -> Vec<Metrics> {
    knockdown_starts()
        .iter()
        .map(|(_, theta0)| rollout_metrics(1.0, 1.0, 0.05, theta0, &make(), SECS))
        .collect()
}

fn main() {
    let starts = knockdown_starts();

    let pfl = run_all(|| PflBaseline);
    let evo = run_all(|| EnergyShapingPolicy { p: NOMINAL_CHAMPION });
    let mpc = run_all(|| MpcSwingUp::new(MpcConfig::default()));

    let controllers: [(&str, &Vec<Metrics>); 3] =
        [("PFL baseline ", &pfl), ("energy-shaping", &evo), ("MPC (predict) ", &mpc)];

    // Per-start outcome grid.
    println!("Per-start outcome (✅ caught / ❌ missed):");
    print!("  {:<16}", "start");
    for (name, _) in &controllers {
        print!("{:>16}", name.trim());
    }
    println!();
    for (i, (label, _)) in starts.iter().enumerate() {
        print!("  {:<16}", label.trim());
        for (_, ms) in &controllers {
            let m = ms[i];
            let cell = if m.caught { format!("✅ {:>4.1}s", m.time_to_catch) } else { "❌  --".to_string() };
            print!("{:>16}", cell);
        }
        println!();
    }

    // Recovered counts.
    println!("\nRecovered:");
    for (name, ms) in &controllers {
        let c = ms.iter().filter(|m| m.caught).count();
        println!("  {name}: {c}/{}", ms.len());
    }

    // Apples-to-apples averages over the starts ALL three solve.
    let common: Vec<usize> = (0..starts.len()).filter(|&i| controllers.iter().all(|(_, ms)| ms[i].caught)).collect();
    println!("\nOn the {} start(s) all three recover — averages:", common.len());
    println!("  {:<16}{:>14}{:>14}{:>14}", "controller", "time-to-catch", "effort ∫|u|", "reversals");
    for (name, ms) in &controllers {
        if common.is_empty() {
            continue;
        }
        let n = common.len() as f64;
        let t = common.iter().map(|&i| ms[i].time_to_catch).sum::<f64>() / n;
        let e = common.iter().map(|&i| ms[i].effort).sum::<f64>() / n;
        let r = common.iter().map(|&i| ms[i].reversals as f64).sum::<f64>() / n;
        println!("  {name}{t:>12.2}s{e:>14.0}{r:>14.1}");
    }
    println!("\n(lower effort + fewer reversals on the same catches = the predictive");
    println!(" controller spends less flailing to reach upright.)");
}
