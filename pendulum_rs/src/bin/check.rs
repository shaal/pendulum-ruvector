//! Diagnostic: which arm configurations can the LQR actually balance, and how
//! big are the gains? Helps pick a demonstrable adaptive scenario instead of
//! guessing. Not part of the demo — a control-tuning aid.

use pendulum_rs::control::balance_gain;
use pendulum_rs::simulator::Pendulum;
use std::f64::consts::PI;

const DT: f64 = 0.005;
const U_MAX: f64 = 150.0;

fn settle_error(m1: f64, l1: f64, gain_from: Option<(f64, f64)>) -> (f64, f64) {
    // Build the *true* arm.
    let mut sim = Pendulum::new(vec![1.0, m1], vec![1.0, l1], vec![0.05, 0.05], 9.81, DT);
    // Gain computed either for this arm (adaptive) or for a reference arm (naive).
    let gain_sim = match gain_from {
        Some((gm, gl)) => Pendulum::new(vec![1.0, gm], vec![1.0, gl], vec![0.05, 0.05], 9.81, DT),
        None => Pendulum::new(vec![1.0, m1], vec![1.0, l1], vec![0.05, 0.05], 9.81, DT),
    };
    let k = balance_gain(&gain_sim, DT);
    let kmag = (k[0] * k[0] + k[1] * k[1] + k[2] * k[2] + k[3] * k[3]).sqrt();

    sim.reset(vec![PI - 0.2, PI + 0.15], vec![0.0, 0.0]);
    let wrap = |a: f64| (a + PI).rem_euclid(2.0 * PI) - PI;
    let mut last = 0.0f64;
    for step in 0..(5.0 / DT) as usize {
        let e0 = wrap(sim.theta[0] - PI);
        let e1 = wrap(sim.theta[1] - PI);
        let u = -(k[0] * e0 + k[1] * e1 + k[2] * sim.omega[0] + k[3] * sim.omega[1]);
        sim.step(&[u.clamp(-U_MAX, U_MAX), 0.0]);
        if step as f64 * DT > 4.0 {
            last = last.max(wrap(sim.theta[0] - PI).abs() + wrap(sim.theta[1] - PI).abs());
        }
    }
    (last, kmag)
}

fn main() {
    println!("config (m1,l1) | own-gain settle / |K|  | naive(light-gain) settle");
    for &(m1, l1) in &[
        (1.0, 1.0),
        (2.0, 1.0),
        (3.0, 1.0),
        (5.0, 1.0),
        (1.0, 1.5),
        (1.0, 2.0),
        (1.0, 0.6),
        (2.0, 1.5),
    ] {
        let (own, kmag) = settle_error(m1, l1, None);
        // "naive" = gain computed for the reference light arm (1,1).
        let (naive, _) = settle_error(m1, l1, Some((1.0, 1.0)));
        println!(
            "  ({:.1},{:.1})        | {:6.3} rad / {:7.1}   | {:6.3} rad  {}",
            m1,
            l1,
            own,
            kmag,
            naive,
            if naive > 0.5 && own < 0.2 { "<-- CONTRAST" } else { "" }
        );
    }

    // Recovery harness — shared with the `evolve` search via `learn` so the
    // hand-tuned baseline and the evolved champion are judged identically.
    println!("\nRecovery (collocated-PFL swing-up + LQR catch) from knocked-down starts, 15s:");
    let starts = pendulum_rs::learn::knockdown_starts();
    let mut recovered = 0;
    for (label, theta0) in &starts {
        let r = pendulum_rs::learn::rollout(theta0, &pendulum_rs::learn::PflBaseline, 15.0);
        let ok = if r.caught {
            recovered += 1;
            "RECOVERED ✅"
        } else {
            "did not catch ❌"
        };
        println!("  {label} -> final tip error {:.2} rad  {}", r.final_tip, ok);
    }
    println!("\n  recovered {recovered}/{} knockdown starts", starts.len());
}
